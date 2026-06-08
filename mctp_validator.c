/*
 * MCTP Control protocol validator over MCTP-over-I2C against the Zephyr target.
 *
 * Unlike mctp_i2c_send.c (which sends a throwaway "ping"), this issues real
 * MCTP control-protocol requests (DSP0236) and validates each response. It
 * runs a small suite, prints a clear reason for every failure, and exits
 * nonzero if any command fails.
 *
 * The Zephyr slave is being developed alongside this: until it implements a
 * given control command, that test is expected to FAIL. The printed reason
 * (e.g. "not an MCTP control message", "EID=0 expected 11") tells you exactly
 * what the target still owes.
 *
 * Same wiring assumptions as mctp_i2c_send.c: SDA / SCL / GND only, no notify
 * GPIO, so we read the response with a single master read after a settle delay.
 */

#define _GNU_SOURCE
#include "libmctp.h"
#include "libmctp-i2c.h"
#include "libmctp-sizes.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define I2C_DEV          "/dev/i2c-1"
#define OUR_I2C_ADDR     0x10
#define OUR_EID          0x08      /* matches BUS_OWNER_ID hardcoded in slave app */
#define PEER_I2C_ADDR    0x50    /* matches Zephyr DT i2c-addr */
#define PEER_EID         0x12      /* matches Zephyr DT endpoint-id */
#define RESP_SETTLE_MS   100     /* give the slave time to queue the reply */

/* MCTP control protocol (DSP0236) */
#define MCTP_MSG_TYPE_CONTROL 0x00
#define MCTP_MSG_TYPE_PLDM    0x01
#define MCTP_CTRL_RQ_BIT      0x80 /* byte 1: Request bit */
#define MCTP_CTRL_IID_MASK    0x1f /* byte 1: Instance ID */

#define CMD_GET_ENDPOINT_ID      0x02
#define CMD_GET_MCTP_VERSION     0x04
#define CMD_GET_MSG_TYPE_SUPPORT 0x05

#define CC_SUCCESS               0x00

struct tx_ctx {
	int fd;
};

struct rx_state {
	bool got_response;
	uint8_t src_eid;
	bool tag_owner;
	uint8_t msg_tag;
	size_t len;
	uint8_t buf[128]; /* full MCTP message, buf[0] = message type */
};

static int my_i2c_tx(const void *buf, size_t len, void *ctx_)
{
	struct tx_ctx *ctx = ctx_;
	const uint8_t *p = buf;

	if (len < 2) {
		return -EINVAL;
	}
	uint8_t dest_addr = p[0] >> 1;

	if (ioctl(ctx->fd, I2C_SLAVE, dest_addr) < 0) {
		fprintf(stderr, "  [tx] ioctl(I2C_SLAVE, 0x%02x): %s\n",
			dest_addr, strerror(errno));
		return -errno;
	}
	/* Skip buf[0]; i2c-dev prepends it from the ioctl. */
	ssize_t n = write(ctx->fd, p + 1, len - 1);
	if (n != (ssize_t)(len - 1)) {
		fprintf(stderr, "  [tx] write: %s\n", strerror(errno));
		return -errno;
	}
	return 0;
}

static void on_mctp_msg(uint8_t src_eid, bool tag_owner, uint8_t msg_tag,
			void *data, void *msg, size_t len)
{
	struct rx_state *st = data;

	st->src_eid = src_eid;
	st->tag_owner = tag_owner;
	st->msg_tag = msg_tag;
	st->len = len > sizeof(st->buf) ? sizeof(st->buf) : len;
	memcpy(st->buf, msg, st->len);
	st->got_response = true;
}

/* Send one MCTP message and harvest the single response via a master read.
 * Returns true if a response reached on_mctp_msg(). */
static bool round_trip(struct mctp *mctp, struct mctp_binding_i2c *i2c, int fd,
		       uint8_t tag, uint8_t *msg, size_t msg_len,
		       struct rx_state *st)
{
	memset(st, 0, sizeof(*st));

	int rc = mctp_message_tx(mctp, PEER_EID, /*tag_owner=*/true, tag, msg,
				 msg_len);
	if (rc < 0) {
		printf("    mctp_message_tx rc=%d (%s)\n", rc, strerror(-rc));
		return false;
	}
	mctp_i2c_tx_poll(i2c);

	usleep(RESP_SETTLE_MS * 1000);

	if (ioctl(fd, I2C_SLAVE, PEER_I2C_ADDR) < 0) {
		printf("    ioctl(I2C_SLAVE, 0x%02x): %s\n", PEER_I2C_ADDR,
		       strerror(errno));
		return false;
	}
	uint8_t rxbuf[128];
	ssize_t n = read(fd, rxbuf, sizeof(rxbuf));
	if (n <= 0) {
		printf("    no bytes from target (read=%zd%s%s)\n", n,
		       n < 0 ? ": " : "", n < 0 ? strerror(errno) : "");
		return false;
	}

	/* A plain i2c-dev master read clocks out a fixed number of bytes and the
	 * slave zero-pads the tail. The real frame length comes from the SMBus
	 * byte count: rxbuf = [cmd|bytecount|source|...], where bytecount counts
	 * everything after it (source + MCTP payload). So the on-wire frame is
	 * cmd + bytecount-field + bytecount bytes. Trim to that before parsing,
	 * otherwise libmctp's "bytecount == len - 3" check rejects the padding. */
	if (n < 2) {
		printf("    short read (%zd bytes, no byte-count field)\n", n);
		return false;
	}
	size_t frame_len = 2 + (size_t)rxbuf[1];
	if (frame_len > (size_t)n) {
		printf("    byte count %u exceeds %zd bytes read\n", rxbuf[1], n);
		return false;
	}

	printf("    [rx] %zu-byte frame:", frame_len);
	for (size_t i = 0; i < frame_len && i < 24; i++) {
		printf(" %02x", rxbuf[i]);
	}
	printf("%s\n", frame_len > 24 ? " ..." : "");

	/* Prepend our own address byte so libmctp's i2c parser sees a full
	 * mctp_i2c_hdr (dest first); the dest byte isn't on the wire for reads. */
	uint8_t framed[1 + sizeof(rxbuf)];
	framed[0] = OUR_I2C_ADDR << 1;
	memcpy(framed + 1, rxbuf, frame_len);
	mctp_i2c_rx(i2c, framed, frame_len + 1);

	if (!st->got_response) {
		printf("    libmctp dropped the %zu-byte frame (bad cmd/bytecount/dest-EID?)\n",
		       frame_len + 1);
	}
	return st->got_response;
}

/* Validate the MCTP control response envelope common to every command. On
 * success, hands back the command-specific data (after the completion code). */
static bool check_ctrl_resp(const struct rx_state *st, uint8_t iid, uint8_t tag,
			    uint8_t cmd, const uint8_t **cdata, size_t *clen)
{
	if (st->src_eid != PEER_EID) {
		printf("    src EID %u, expected %u\n", st->src_eid, PEER_EID);
		return false;
	}
	if (st->tag_owner) {
		printf("    tag_owner set on a response\n");
		return false;
	}
	if (st->msg_tag != tag) {
		printf("    msg tag %u, expected %u\n", st->msg_tag, tag);
		return false;
	}
	if (st->len < 4) {
		printf("    response too short (%zu bytes, need >=4)\n",
		       st->len);
		return false;
	}
	if (st->buf[0] != MCTP_MSG_TYPE_CONTROL) {
		printf("    not an MCTP control message (type 0x%02x)\n",
		       st->buf[0]);
		return false;
	}
	if (st->buf[1] & MCTP_CTRL_RQ_BIT) {
		printf("    Rq bit set -- not a response\n");
		return false;
	}
	if ((st->buf[1] & MCTP_CTRL_IID_MASK) != iid) {
		printf("    instance ID %u, expected %u\n",
		       st->buf[1] & MCTP_CTRL_IID_MASK, iid);
		return false;
	}
	if (st->buf[2] != cmd) {
		printf("    command 0x%02x, expected 0x%02x\n", st->buf[2], cmd);
		return false;
	}
	if (st->buf[3] != CC_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", st->buf[3]);
		return false;
	}
	*cdata = st->buf + 4;
	*clen = st->len - 4;
	return true;
}

/* Get Endpoint ID (0x02): no request data; response = [EID][type][medium]. */
static bool test_get_endpoint_id(struct mctp *mctp,
				 struct mctp_binding_i2c *i2c, int fd,
				 uint8_t iid, uint8_t tag, struct rx_state *st)
{
	uint8_t req[3] = { MCTP_MSG_TYPE_CONTROL,
			   MCTP_CTRL_RQ_BIT | (iid & MCTP_CTRL_IID_MASK),
			   CMD_GET_ENDPOINT_ID };

	if (!round_trip(mctp, i2c, fd, tag, req, sizeof(req), st)) {
		return false;
	}

	const uint8_t *d;
	size_t dlen;
	if (!check_ctrl_resp(st, iid, tag, CMD_GET_ENDPOINT_ID, &d, &dlen)) {
		return false;
	}
	if (dlen < 1) {
		printf("    missing EID field\n");
		return false;
	}
	if (d[0] != PEER_EID) {
		printf("    EID=%u, expected %u\n", d[0], PEER_EID);
		return false;
	}
	printf("    EID=%u%s\n", d[0],
	       dlen >= 2 ? "" : " (no endpoint-type byte)");
	return true;
}

/* Get MCTP Version Support (0x04): request 1 byte (0xFF = base spec);
 * response = [count][count * 4-byte version entries]. */
static bool test_get_mctp_version(struct mctp *mctp,
				  struct mctp_binding_i2c *i2c, int fd,
				  uint8_t iid, uint8_t tag, struct rx_state *st)
{
	uint8_t req[4] = { MCTP_MSG_TYPE_CONTROL,
			   MCTP_CTRL_RQ_BIT | (iid & MCTP_CTRL_IID_MASK),
			   CMD_GET_MCTP_VERSION,
			   0xff /* base MCTP spec versions */ };

	if (!round_trip(mctp, i2c, fd, tag, req, sizeof(req), st)) {
		return false;
	}

	const uint8_t *d;
	size_t dlen;
	if (!check_ctrl_resp(st, iid, tag, CMD_GET_MCTP_VERSION, &d, &dlen)) {
		return false;
	}
	if (dlen < 1) {
		printf("    missing version-count byte\n");
		return false;
	}
	uint8_t count = d[0];
	if (count == 0) {
		printf("    target reports 0 supported versions\n");
		return false;
	}
	if (dlen < (size_t)1 + 4 * count) {
		printf("    truncated version list (count=%u, %zu data bytes)\n",
		       count, dlen);
		return false;
	}
	printf("    %u version(s):", count);
	for (uint8_t i = 0; i < count; i++) {
		const uint8_t *v = d + 1 + 4 * i;
		printf(" %02x.%02x.%02x.%02x", v[0], v[1], v[2], v[3]);
	}
	printf("\n");
	return true;
}

/* Get Message Type Support (0x05): no request data;
 * response = [count][count message-type bytes]. PLDM presence is reported but
 * not required -- this is a pure MCTP control check. */
static bool test_get_msg_type_support(struct mctp *mctp,
				      struct mctp_binding_i2c *i2c, int fd,
				      uint8_t iid, uint8_t tag,
				      struct rx_state *st)
{
	uint8_t req[3] = { MCTP_MSG_TYPE_CONTROL,
			   MCTP_CTRL_RQ_BIT | (iid & MCTP_CTRL_IID_MASK),
			   CMD_GET_MSG_TYPE_SUPPORT };

	if (!round_trip(mctp, i2c, fd, tag, req, sizeof(req), st)) {
		return false;
	}

	const uint8_t *d;
	size_t dlen;
	if (!check_ctrl_resp(st, iid, tag, CMD_GET_MSG_TYPE_SUPPORT, &d,
			     &dlen)) {
		return false;
	}
	if (dlen < 1) {
		printf("    missing type-count byte\n");
		return false;
	}
	uint8_t count = d[0];
	if (dlen < (size_t)1 + count) {
		printf("    truncated type list (count=%u, %zu data bytes)\n",
		       count, dlen);
		return false;
	}
	bool has_pldm = false;
	printf("    %u type(s):", count);
	for (uint8_t i = 0; i < count; i++) {
		uint8_t t = d[1 + i];
		printf(" 0x%02x", t);
		if (t == MCTP_MSG_TYPE_PLDM) {
			has_pldm = true;
		}
	}
	printf("\n");
	/* PLDM is an upper-layer concern, not an MCTP requirement: a target that
	 * advertises only Control (0x00) is still MCTP-conformant. Note its
	 * absence for the PLDM bring-up that follows, but don't fail the MCTP
	 * check on it. */
	if (!has_pldm) {
		printf("    note: PLDM (0x01) not advertised yet\n");
	}
	return true;
}

typedef bool (*test_fn)(struct mctp *, struct mctp_binding_i2c *, int,
			uint8_t iid, uint8_t tag, struct rx_state *st);

struct test_case {
	const char *name;
	test_fn fn;
};

int main(void)
{
	struct tx_ctx tctx = { .fd = -1 };

	tctx.fd = open(I2C_DEV, O_RDWR);
	if (tctx.fd < 0) {
		fprintf(stderr, "open %s: %s\n", I2C_DEV, strerror(errno));
		return 2;
	}

	struct mctp *mctp = mctp_init();
	if (!mctp) {
		fprintf(stderr, "mctp_init failed\n");
		return 2;
	}

	struct mctp_binding_i2c *i2c = calloc(1, MCTP_SIZEOF_BINDING_I2C);
	if (!i2c) {
		fprintf(stderr, "calloc binding failed\n");
		return 2;
	}

	if (mctp_i2c_setup(i2c, OUR_I2C_ADDR, my_i2c_tx, &tctx) < 0) {
		fprintf(stderr, "mctp_i2c_setup failed\n");
		return 2;
	}

	struct rx_state st;
	mctp_set_rx_all(mctp, on_mctp_msg, &st);

	if (mctp_register_bus(mctp, mctp_binding_i2c_core(i2c), OUR_EID) < 0) {
		fprintf(stderr, "mctp_register_bus failed\n");
		return 2;
	}
	if (mctp_i2c_set_neighbour(i2c, PEER_EID, PEER_I2C_ADDR) < 0) {
		fprintf(stderr, "mctp_i2c_set_neighbour failed\n");
		return 2;
	}

	fprintf(stderr,
		"[validator] %s  us=EID%u@0x%02x  peer=EID%u@0x%02x  settle=%dms\n",
		I2C_DEV, OUR_EID, OUR_I2C_ADDR, PEER_EID, PEER_I2C_ADDR,
		RESP_SETTLE_MS);

	struct test_case tests[] = {
		{ "Get Endpoint ID", test_get_endpoint_id },
		{ "Get MCTP Version Support", test_get_mctp_version },
		{ "Get Message Type Support", test_get_msg_type_support },
	};
	size_t ntests = sizeof(tests) / sizeof(tests[0]);
	int failures = 0;

	for (size_t i = 0; i < ntests; i++) {
		printf("\n=== [%zu/%zu] %s ===\n", i + 1, ntests,
		       tests[i].name);
		/* MCTP instance IDs are 5-bit; tags are 3-bit. */
		bool ok = tests[i].fn(mctp, i2c, tctx.fd,
				      (uint8_t)(i & MCTP_CTRL_IID_MASK),
				      (uint8_t)(i & 0x7), &st);
		printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", tests[i].name);
		if (!ok) {
			failures++;
		}
	}

	printf("\nvalidator: %zu/%zu passed\n", ntests - (size_t)failures,
	       ntests);

	mctp_destroy(mctp);
	free(i2c);
	close(tctx.fd);
	return failures ? 1 : 0;
}
