/*
 * TX-only PLDM-over-MCTP-over-I2C demo.
 *
 * Opens /dev/i2c-1, drives libmctp's i2c binding manually (without
 * mctp-demux-daemon), and emits a single PLDM GetTID request to the
 * configured peer I2C address.
 *
 * Without a real MCTP slave on the bus you should expect the kernel write
 * to fail with -ENXIO or -EREMOTEIO (NACK). The point of this demo is to
 * exercise libmctp's I2C framing path; replace PEER_I2C_ADDR with the real
 * slave to actually talk to a device.
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define I2C_DEV         "/dev/i2c-1"
#define OUR_I2C_ADDR    0x10
#define OUR_EID         8
#define PEER_I2C_ADDR   0x50
#define PEER_EID        9
#define MCTP_TYPE_PLDM  0x01

struct tx_ctx {
	int fd;
};

static int my_i2c_tx(const void *buf, size_t len, void *ctx_)
{
	struct tx_ctx *ctx = ctx_;
	const uint8_t *p = buf;

	if (len < 2) {
		return -EINVAL;
	}

	uint8_t dest_addr_7bit = p[0] >> 1;

	fprintf(stderr,
		"[tx_fn] I2C frame len=%zu to 7bit-addr=0x%02x: ", len,
		dest_addr_7bit);
	for (size_t i = 0; i < len; i++) {
		fprintf(stderr, "%02x ", p[i]);
	}
	fprintf(stderr, "\n");

	if (ioctl(ctx->fd, I2C_SLAVE, dest_addr_7bit) < 0) {
		fprintf(stderr, "[tx_fn] ioctl(I2C_SLAVE, 0x%02x): %s\n",
			dest_addr_7bit, strerror(errno));
		return -errno;
	}

	/* Skip buf[0] (the I2C address); i2c-dev prepends it from the ioctl. */
	ssize_t n = write(ctx->fd, p + 1, len - 1);
	if (n != (ssize_t)(len - 1)) {
		fprintf(stderr, "[tx_fn] write: %s\n", strerror(errno));
		return -errno;
	}
	fprintf(stderr, "[tx_fn] wrote %zd bytes\n", n);
	return 0;
}

int main(void)
{
	struct tx_ctx ctx = { .fd = -1 };

	ctx.fd = open(I2C_DEV, O_RDWR);
	if (ctx.fd < 0) {
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
		fprintf(stderr, "calloc i2c binding failed\n");
		return 1;
	}

	int rc = mctp_i2c_setup(i2c, OUR_I2C_ADDR, my_i2c_tx, &ctx);
	if (rc) {
		fprintf(stderr, "mctp_i2c_setup rc=%d\n", rc);
		return 1;
	}

	rc = mctp_register_bus(mctp, mctp_binding_i2c_core(i2c), OUR_EID);
	if (rc) {
		fprintf(stderr, "mctp_register_bus rc=%d\n", rc);
		return 1;
	}

	rc = mctp_i2c_set_neighbour(i2c, PEER_EID, PEER_I2C_ADDR);
	if (rc) {
		fprintf(stderr, "mctp_i2c_set_neighbour rc=%d\n", rc);
		return 1;
	}

	fprintf(stderr,
		"[mctp_i2c_send] %s: our EID=%u (I2C 0x%02x), peer EID=%u (I2C 0x%02x)\n",
		I2C_DEV, OUR_EID, OUR_I2C_ADDR, PEER_EID, PEER_I2C_ADDR);

	/* MCTP message body: [msg_type=PLDM][PLDM hdr][GetTID req body] */
	uint8_t msg[1 + sizeof(struct pldm_msg_hdr) + PLDM_GET_TID_REQ_BYTES];
	msg[0] = MCTP_TYPE_PLDM;
	struct pldm_msg *pldm = (struct pldm_msg *)(msg + 1);
	rc = encode_get_tid_req(/*instance_id=*/7, pldm);
	if (rc != PLDM_SUCCESS) {
		fprintf(stderr, "encode_get_tid_req rc=%d\n", rc);
		return 1;
	}

	fprintf(stderr,
		"[mctp_i2c_send] sending PLDM GetTID (iid=7, %zu MCTP message bytes)\n",
		sizeof(msg));

	rc = mctp_message_tx(mctp, PEER_EID, /*tag_owner=*/true, /*tag=*/0, msg,
			     sizeof(msg));
	if (rc < 0) {
		fprintf(stderr, "mctp_message_tx rc=%d (%s)\n", rc,
			strerror(-rc));
	} else {
		fprintf(stderr, "[mctp_i2c_send] mctp_message_tx returned %d\n",
			rc);
	}

	/* Flush any queued packets (i2c binding may defer). */
	mctp_i2c_tx_poll(i2c);

	mctp_destroy(mctp);
	free(i2c);
	close(ctx.fd);
	return 0;
}
