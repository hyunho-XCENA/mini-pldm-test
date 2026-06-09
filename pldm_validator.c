/*
 * PLDM base-protocol validator over MCTP-over-I2C against the Zephyr target.
 *
 * This is the PLDM-layer counterpart to mctp_validator.c. Where that exercises
 * the MCTP control protocol (DSP0236, message type 0x00), this exercises PLDM
 * (DSP0240, MCTP message type 0x01): a PLDM message rides as the MCTP payload
 * with byte 0 = 0x01 followed by a pldm_msg (3-byte header + payload).
 *
 * It issues the four PLDM base-type commands every responder must implement and
 * validates each response with libpldm's decoders:
 *   GetTID (0x02), GetPLDMVersion (0x03), GetPLDMTypes (0x04),
 *   GetPLDMCommands (0x05).
 *
 * The Zephyr slave is being developed alongside this: until it answers a given
 * PLDM command, that test is expected to FAIL. The printed reason (e.g. "no
 * PLDM response -- target advertises only MCTP control?", "completion code
 * 0x05 ERROR_UNSUPPORTED_PLDM_CMD") tells you exactly what the target still
 * owes.
 *
 * Same wiring assumptions as mctp_validator.c: SDA / SCL / GND only, no notify
 * GPIO, so we read the response with a single master read after a settle delay.
 */

#define _GNU_SOURCE
#include "libmctp.h"
#include "libmctp-i2c.h"
#include "libmctp-sizes.h"

#include "libpldm/base.h"
#include "libpldm/pldm_types.h"

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

#define MCTP_MSG_TYPE_PLDM 0x01

#define PLDM_HDR_LEN     ((size_t)sizeof(struct pldm_msg_hdr)) /* 3 bytes */

/* The base PLDM version, 1.0.0, BCD-encoded as 0xF1F1F000. ver32_t stores it
 * little-end-first: {alpha, update, minor, major}. Used as the version operand
 * for GetPLDMCommands. */
static const ver32_t PLDM_BASE_VERSION = {
	.alpha = 0x00, .update = 0xf0, .minor = 0xf1, .major = 0xf1
};

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

/* Validate the MCTP+PLDM response envelope common to every command. On success,
 * hands back the inner pldm_msg and its payload length (bytes after the 3-byte
 * PLDM header) so the caller can run the command-specific libpldm decoder. */
static bool check_pldm_resp(const struct rx_state *st, uint8_t iid, uint8_t tag,
			    uint8_t cmd, const struct pldm_msg **pmsg,
			    size_t *payload_len)
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
	if (st->buf[0] != MCTP_MSG_TYPE_PLDM) {
		printf("    not a PLDM message (MCTP type 0x%02x)\n",
		       st->buf[0]);
		return false;
	}
	/* MCTP type byte + PLDM header must be present before the payload. */
	if (st->len < 1 + PLDM_HDR_LEN) {
		printf("    response too short (%zu bytes, need >=%zu)\n",
		       st->len, 1 + PLDM_HDR_LEN);
		return false;
	}

	const struct pldm_msg *m = (const struct pldm_msg *)(st->buf + 1);
	if (m->hdr.request != PLDM_RESPONSE) {
		printf("    request bit set -- not a PLDM response\n");
		return false;
	}
	if (m->hdr.instance_id != iid) {
		printf("    instance ID %u, expected %u\n",
		       m->hdr.instance_id, iid);
		return false;
	}
	if (m->hdr.type != PLDM_BASE) {
		printf("    PLDM type %u, expected %u (base)\n",
		       m->hdr.type, PLDM_BASE);
		return false;
	}
	if (m->hdr.command != cmd) {
		printf("    command 0x%02x, expected 0x%02x\n",
		       m->hdr.command, cmd);
		return false;
	}

	*pmsg = m;
	*payload_len = st->len - 1 - PLDM_HDR_LEN;
	return true;
}

/* Build the MCTP message for a PLDM request: byte 0 = PLDM message type, then
 * the pldm_msg the libpldm encoder wrote. Returns the total MCTP message
 * length, or 0 on encode failure. The pldm_msg is encoded into *out + 1. */
static size_t frame_pldm_req(uint8_t *out, struct pldm_msg **req)
{
	out[0] = MCTP_MSG_TYPE_PLDM;
	*req = (struct pldm_msg *)(out + 1);
	return 1 + PLDM_HDR_LEN; /* payload bytes added by the caller */
}

/* GetTID (0x02): no request payload; response = [cc][tid]. */
static bool test_get_tid(struct mctp *mctp, struct mctp_binding_i2c *i2c,
			 int fd, uint8_t iid, uint8_t tag, struct rx_state *st)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req);

	int rc = encode_get_tid_req(iid, req);
	if (rc) {
		printf("    encode_get_tid_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_pldm_resp(st, iid, tag, PLDM_GET_TID, &m, &plen)) {
		return false;
	}

	uint8_t cc, tid;
	rc = decode_get_tid_resp(m, plen, &cc, &tid);
	if (rc) {
		printf("    decode_get_tid_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return false;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return false;
	}
	printf("    TID=%u\n", tid);
	return true;
}

/* GetPLDMVersion (0x03): request transfer_handle + opflag + type;
 * response = [cc][next_handle][flag][ver32]. */
static bool test_get_version(struct mctp *mctp, struct mctp_binding_i2c *i2c,
			     int fd, uint8_t iid, uint8_t tag,
			     struct rx_state *st)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_VERSION_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) + PLDM_GET_VERSION_REQ_BYTES;

	int rc = encode_get_version_req(iid, /*transfer_handle=*/0,
					PLDM_GET_FIRSTPART, PLDM_BASE, req);
	if (rc) {
		printf("    encode_get_version_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_pldm_resp(st, iid, tag, PLDM_GET_PLDM_VERSION, &m, &plen)) {
		return false;
	}

	uint8_t cc, flag;
	uint32_t next_handle;
	ver32_t ver;
	rc = decode_get_version_resp(m, plen, &cc, &next_handle, &flag, &ver);
	if (rc) {
		printf("    decode_get_version_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return false;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return false;
	}
	printf("    base version %02x.%02x.%02x.%02x (transfer flag 0x%02x)\n",
	       ver.major, ver.minor, ver.update, ver.alpha, flag);
	return true;
}

/* GetPLDMTypes (0x04): no request payload; response = [cc][8-byte type mask]. */
static bool test_get_types(struct mctp *mctp, struct mctp_binding_i2c *i2c,
			   int fd, uint8_t iid, uint8_t tag, struct rx_state *st)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req);

	int rc = encode_get_types_req(iid, req);
	if (rc) {
		printf("    encode_get_types_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_pldm_resp(st, iid, tag, PLDM_GET_PLDM_TYPES, &m, &plen)) {
		return false;
	}

	uint8_t cc;
	bitfield8_t types[8];
	rc = decode_get_types_resp(m, plen, &cc, types);
	if (rc) {
		printf("    decode_get_types_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return false;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return false;
	}

	bool has_base = false;
	printf("    supported PLDM types:");
	for (int t = 0; t < PLDM_MAX_TYPES; t++) {
		if (types[t / 8].byte & (1u << (t % 8))) {
			printf(" %d", t);
			if (t == PLDM_BASE) {
				has_base = true;
			}
		}
	}
	printf("\n");
	if (!has_base) {
		printf("    base type (0) not advertised -- required by DSP0240\n");
		return false;
	}
	return true;
}

/* GetPLDMCommands (0x05): request type + version;
 * response = [cc][32-byte command mask]. */
static bool test_get_commands(struct mctp *mctp, struct mctp_binding_i2c *i2c,
			      int fd, uint8_t iid, uint8_t tag,
			      struct rx_state *st)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_COMMANDS_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) + PLDM_GET_COMMANDS_REQ_BYTES;

	int rc = encode_get_commands_req(iid, PLDM_BASE, PLDM_BASE_VERSION, req);
	if (rc) {
		printf("    encode_get_commands_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_pldm_resp(st, iid, tag, PLDM_GET_PLDM_COMMANDS, &m, &plen)) {
		return false;
	}

	uint8_t cc;
	bitfield8_t cmds[32];
	rc = decode_get_commands_resp(m, plen, &cc, cmds);
	if (rc) {
		printf("    decode_get_commands_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return false;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return false;
	}

	printf("    base commands:");
	for (int c = 0; c < PLDM_MAX_CMDS_PER_TYPE; c++) {
		if (cmds[c / 8].byte & (1u << (c % 8))) {
			printf(" 0x%02x", c);
		}
	}
	printf("\n");
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
		"[pldm-validator] %s  us=EID%u@0x%02x  peer=EID%u@0x%02x  settle=%dms\n",
		I2C_DEV, OUR_EID, OUR_I2C_ADDR, PEER_EID, PEER_I2C_ADDR,
		RESP_SETTLE_MS);

	struct test_case tests[] = {
		{ "GetTID", test_get_tid },
		{ "GetPLDMVersion", test_get_version },
		{ "GetPLDMTypes", test_get_types },
		{ "GetPLDMCommands", test_get_commands },
	};
	size_t ntests = sizeof(tests) / sizeof(tests[0]);
	int failures = 0;

	for (size_t i = 0; i < ntests; i++) {
		printf("\n=== [%zu/%zu] %s ===\n", i + 1, ntests,
		       tests[i].name);
		/* PLDM instance IDs are 5-bit; MCTP tags are 3-bit. */
		bool ok = tests[i].fn(mctp, i2c, tctx.fd,
				      (uint8_t)(i & PLDM_INSTANCE_MAX),
				      (uint8_t)(i & 0x7), &st);
		printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", tests[i].name);
		if (!ok) {
			failures++;
		}
	}

	printf("\npldm-validator: %zu/%zu passed\n", ntests - (size_t)failures,
	       ntests);

	mctp_destroy(mctp);
	free(i2c);
	close(tctx.fd);
	return failures ? 1 : 0;
}
