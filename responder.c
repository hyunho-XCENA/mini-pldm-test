/*
 * PLDM loopback responder.
 *
 * Pretends to be the mctp-demux-daemon: listens on the abstract Unix socket
 * \0mctp-mux, accepts one libpldm client, and answers GetTID requests.
 *
 * Wire format on the socket (per libpldm src/transport/mctp-demux.c):
 *   first packet from client:  [type:1]                          (we eat it)
 *   request from client:       [dst_eid:1][type:1][pldm msg...]
 *   response we send back:     [src_eid:1][type:1][pldm msg...]
 *
 * The client (requester) maps TID=1 <-> EID=9, so we reflect EID 9.
 */

#define _GNU_SOURCE
#include <libpldm/base.h>
#include <libpldm/pldm.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define MY_EID         9
#define MCTP_TYPE_PLDM 0x01
#define REPORTED_TID   42

static int bind_abstract_mctp_mux(void)
{
	int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (fd < 0) {
		perror("socket");
		return -1;
	}

	struct sockaddr_un addr = { 0 };
	addr.sun_family = AF_UNIX;
	memcpy(addr.sun_path, "\0mctp-mux", 9);
	socklen_t addrlen = offsetof(struct sockaddr_un, sun_path) + 9;

	if (bind(fd, (struct sockaddr *)&addr, addrlen) < 0) {
		perror("bind");
		close(fd);
		return -1;
	}
	if (listen(fd, 1) < 0) {
		perror("listen");
		close(fd);
		return -1;
	}
	return fd;
}

int main(void)
{
	int lfd = bind_abstract_mctp_mux();
	if (lfd < 0) {
		return 1;
	}
	fprintf(stderr,
		"[responder] listening on \\0mctp-mux as EID=%d, reporting TID=%d\n",
		MY_EID, REPORTED_TID);

	int cfd = accept(lfd, NULL, NULL);
	if (cfd < 0) {
		perror("accept");
		return 1;
	}
	fprintf(stderr, "[responder] requester connected (fd=%d)\n", cfd);

	uint8_t type_reg = 0;
	ssize_t n = recv(cfd, &type_reg, 1, 0);
	if (n != 1 || type_reg != MCTP_TYPE_PLDM) {
		fprintf(stderr,
			"[responder] bad type-byte registration: n=%zd byte=0x%02x\n",
			n, type_reg);
		return 1;
	}
	fprintf(stderr, "[responder] type byte registered: 0x%02x (PLDM)\n",
		type_reg);

	uint8_t in[512];
	n = recv(cfd, in, sizeof(in), 0);
	if (n < (ssize_t)(2 + sizeof(struct pldm_msg_hdr))) {
		fprintf(stderr, "[responder] short request (n=%zd)\n", n);
		return 1;
	}
	uint8_t dst_eid = in[0];
	uint8_t mtype = in[1];
	struct pldm_msg *req = (struct pldm_msg *)(in + 2);
	size_t req_payload_len = (size_t)n - 2 - sizeof(struct pldm_msg_hdr);

	fprintf(stderr,
		"[responder] recv %zd-byte frame: dst_eid=%u mctp_type=0x%02x\n"
		"            pldm hdr: req=%u dgram=%u iid=%u type=%u cmd=0x%02x payload=%zu\n",
		n, dst_eid, mtype, req->hdr.request, req->hdr.datagram,
		req->hdr.instance_id, req->hdr.type, req->hdr.command,
		req_payload_len);

	if (mtype != MCTP_TYPE_PLDM || req->hdr.type != PLDM_BASE ||
	    req->hdr.command != PLDM_GET_TID || req->hdr.request != 1) {
		fprintf(stderr, "[responder] not a GetTID request, refusing\n");
		return 1;
	}

	uint8_t resp_buf[PLDM_MSG_SIZE(PLDM_GET_TID_RESP_BYTES)];
	struct pldm_msg *resp = (struct pldm_msg *)resp_buf;
	int rc = encode_get_tid_resp(req->hdr.instance_id, PLDM_SUCCESS,
				     REPORTED_TID, resp);
	if (rc != PLDM_SUCCESS) {
		fprintf(stderr, "[responder] encode_get_tid_resp rc=%d\n", rc);
		return 1;
	}
	size_t resp_msg_len =
		sizeof(struct pldm_msg_hdr) + PLDM_GET_TID_RESP_BYTES;

	uint8_t out[2 + sizeof(resp_buf)];
	out[0] = MY_EID;
	out[1] = MCTP_TYPE_PLDM;
	memcpy(out + 2, resp, resp_msg_len);

	ssize_t s = send(cfd, out, 2 + resp_msg_len, 0);
	if (s < 0) {
		perror("send");
		return 1;
	}
	fprintf(stderr,
		"[responder] sent %zd-byte response with TID=%d, completion=0x00\n",
		s, REPORTED_TID);

	close(cfd);
	close(lfd);
	return 0;
}
