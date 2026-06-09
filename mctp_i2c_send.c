/*
 * PLDM-over-MCTP-over-I2C round-trip against a Zephyr MCTP target.
 *
 * Counterpart device tree on the STM32H573I-DK:
 *   compatible = "zephyr,mctp-i2c-gpio-target";
 *   i2c-addr = <0x46>;
 *   endpoint-id = <11>;
 *
 * Wiring on this Pi: only SDA / SCL / GND. The sideband notify GPIO is NOT
 * connected, so we poll for the response by issuing master reads on /dev/i2c-1
 * until we either get bytes back or hit a timeout.
 */

#define _GNU_SOURCE
#include <libpldm/base.h>
#include <libpldm/pldm.h>

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
#define OUR_EID          20      /* matches BUS_OWNER_ID hardcoded in slave app */
#define PEER_I2C_ADDR    0x50    /* matches Zephyr DT i2c-addr */
#define PEER_EID         0x12    /* matches Zephyr DT endpoint-id */
#define MCTP_TYPE_PLDM   0x01
#define RESP_SETTLE_MS   100     /* give the slave time to queue "pong" before the single read */

struct tx_ctx {
	int fd;
};

struct rx_state {
	bool got_response;
	size_t resp_len;
	uint8_t resp_buf[128];
};

static int my_i2c_tx(const void *buf, size_t len, void *ctx_)
{
	struct tx_ctx *ctx = ctx_;
	const uint8_t *p = buf;

	if (len < 2) {
		return -EINVAL;
	}
	uint8_t dest_addr = p[0] >> 1;

	fprintf(stderr, "[tx_fn] write to 0x%02x, %zu bytes:", dest_addr, len);
	for (size_t i = 0; i < len; i++) {
		fprintf(stderr, " %02x", p[i]);
	}
	fprintf(stderr, "\n");

	if (ioctl(ctx->fd, I2C_SLAVE, dest_addr) < 0) {
		fprintf(stderr, "[tx_fn] ioctl(I2C_SLAVE, 0x%02x): %s\n",
			dest_addr, strerror(errno));
		return -errno;
	}
	/* Skip buf[0]; i2c-dev prepends it from the ioctl. */
	ssize_t n = write(ctx->fd, p + 1, len - 1);
	if (n != (ssize_t)(len - 1)) {
		fprintf(stderr, "[tx_fn] write: %s\n", strerror(errno));
		return -errno;
	}
	return 0;
}

static void on_mctp_msg(uint8_t src_eid, bool tag_owner, uint8_t msg_tag,
			void *data, void *msg, size_t len)
{
	struct rx_state *st = data;
	const uint8_t *buf = msg;

	fprintf(stderr,
		"[rx_cb] MCTP from EID=%u tag_owner=%d tag=%u len=%zu type=0x%02x\n",
		src_eid, tag_owner, msg_tag, len, len > 0 ? buf[0] : 0);

	fprintf(stderr, "[rx_cb] payload bytes:");
	for (size_t i = 0; i < len && i < 32; i++) {
		fprintf(stderr, " %02x", buf[i]);
	}
	fprintf(stderr, "%s\n", len > 32 ? " ..." : "");

	/* If the payload after MCTP type byte looks printable, also show as ASCII. */
	if (len > 1) {
		fprintf(stderr, "[rx_cb] payload as ASCII (after type byte): \"");
		for (size_t i = 1; i < len; i++) {
			fputc(buf[i] >= 0x20 && buf[i] < 0x7f ? buf[i] : '.',
			      stderr);
		}
		fprintf(stderr, "\"\n");
	}

	st->resp_len = len > sizeof(st->resp_buf) ? sizeof(st->resp_buf) : len;
	memcpy(st->resp_buf, buf, st->resp_len);
	st->got_response = true;
}

int main(void)
{
	struct tx_ctx tctx = { .fd = -1 };
	struct rx_state st = { 0 };

	tctx.fd = open(I2C_DEV, O_RDWR);
	if (tctx.fd < 0) {
		fprintf(stderr, "open %s: %s\n", I2C_DEV, strerror(errno));
		return 1;
	}

	struct mctp *mctp = mctp_init();
	if (!mctp) {
		fprintf(stderr, "mctp_init failed\n");
		return 1;
	}

	struct mctp_binding_i2c *i2c = calloc(1, MCTP_SIZEOF_BINDING_I2C);
	if (!i2c) {
		fprintf(stderr, "calloc binding failed\n");
		return 1;
	}

	if (mctp_i2c_setup(i2c, OUR_I2C_ADDR, my_i2c_tx, &tctx) < 0) {
		fprintf(stderr, "mctp_i2c_setup failed\n");
		return 1;
	}

	mctp_set_rx_all(mctp, on_mctp_msg, &st);

	if (mctp_register_bus(mctp, mctp_binding_i2c_core(i2c), OUR_EID) < 0) {
		fprintf(stderr, "mctp_register_bus failed\n");
		return 1;
	}
	if (mctp_i2c_set_neighbour(i2c, PEER_EID, PEER_I2C_ADDR) < 0) {
		fprintf(stderr, "mctp_i2c_set_neighbour failed\n");
		return 1;
	}

	fprintf(stderr,
		"[main] %s  us=EID%u@0x%02x  peer=EID%u@0x%02x  settle=%dms\n",
		I2C_DEV, OUR_EID, OUR_I2C_ADDR, PEER_EID, PEER_I2C_ADDR,
		RESP_SETTLE_MS);

	/* MCTP Control: Get Endpoint ID — type=0x00, Rq=1, inst=0, cmd=0x02 */
	uint8_t msg[] = { 0x00, 0x80, 0x02 };

	fprintf(stderr,
		"[main] sending MCTP \"ping\" (%zu bytes, type=0x%02x)\n",
		sizeof(msg), msg[0]);

	int rc = mctp_message_tx(mctp, PEER_EID, /*tag_owner=*/true, /*tag=*/0,
				 msg, sizeof(msg));
	if (rc < 0) {
		fprintf(stderr, "mctp_message_tx rc=%d (%s)\n", rc,
			strerror(-rc));
		return 1;
	}
	mctp_i2c_tx_poll(i2c);

	/* The slave needs a brief moment to process the request and queue its
	 * "pong" reply. Wait once, then issue a single master read -- the
	 * response is reliably ready by then, so there's no need to poll. */
	usleep(RESP_SETTLE_MS * 1000);

	if (ioctl(tctx.fd, I2C_SLAVE, PEER_I2C_ADDR) < 0) {
		fprintf(stderr, "ioctl(I2C_SLAVE, 0x%02x): %s\n",
			PEER_I2C_ADDR, strerror(errno));
	} else {
		uint8_t rxbuf[128];
		ssize_t n = read(tctx.fd, rxbuf, sizeof(rxbuf));
		if (n < 0) {
		    fprintf(stderr, "read: %s\n", strerror(errno));
		} else if (n >= 2 && rxbuf[0] == 0x0f) {
		    /* STM32 pads with 0x00; real frame = cmd + bytecount + (source + MCTP). */
		    size_t used = 2 + rxbuf[1];
		    if ((size_t)n < used) {
		        fprintf(stderr, "[rx] short read: got %zd, need %zu\n", n, used);
		    } else {
		        fprintf(stderr, "[rx] frame %zu bytes:", used);
			    for (size_t i = 0; i < used && i < 16; i++) {
		    		fprintf(stderr, " %02x", rxbuf[i]);
				}
				fprintf(stderr, "%s\n", used > 16 ? " ..." : "");
				/* libmctp wants [dest][cmd][bytecount][source][MCTP...]. Prepend dest. */
				uint8_t framed[1 + sizeof(rxbuf)];
				framed[0] = OUR_I2C_ADDR << 1;
				memcpy(framed + 1, rxbuf, used);
				mctp_i2c_rx(i2c, framed, 1 + used);
			}
		} else {
    		fprintf(stderr, "[rx] no staged reply (first byte 0x%02x)\n",
            	n > 0 ? rxbuf[0] : 0);
		}
	}

	if (st.got_response) {
		printf("[main] MCTP round-trip OK: got %zu bytes back from peer\n",
		       st.resp_len);
	} else {
		fprintf(stderr,
			"[main] no response. check wiring/STM32 logs.\n");
	}

	mctp_destroy(mctp);
	free(i2c);
	close(tctx.fd);
	return st.got_response ? 0 : 1;
}
