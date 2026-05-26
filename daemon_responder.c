/*
 * Daemon-attached PLDM responder.
 *
 * Connects to the real mctp-demux-daemon as a PLDM client and answers GetTID
 * requests reflected by the daemon. Run order:
 *
 *   1. mctp-demux-daemon null    (or with another binding)
 *   2. ./daemon_responder         (this program, stays alive)
 *   3. ./requester                (sends GetTID; receives our response)
 *
 * Reflection logic in libmctp's demux: when a client sends to dst_eid ==
 * local_eid (default 8), the daemon broadcasts the message to every connected
 * client of the same MCTP type. Both this responder and the requester see all
 * reflected frames, so we ignore anything that isn't a GetTID *request*.
 */

#define _GNU_SOURCE
#include <libpldm/base.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define MCTP_TYPE_PLDM 0x01
#define REPORTED_TID   42

static int connect_to_demux(void)
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

	if (connect(fd, (struct sockaddr *)&addr, addrlen) < 0) {
		perror("connect to \\0mctp-mux");
		close(fd);
		return -1;
	}

	uint8_t type = MCTP_TYPE_PLDM;
	if (write(fd, &type, 1) != 1) {
		perror("write type byte");
		close(fd);
		return -1;
	}
	return fd;
}

int main(void)
{
	int fd = connect_to_demux();
	if (fd < 0) {
		return 1;
	}
	fprintf(stderr,
		"[daemon-responder] connected to mctp-demux-daemon, will report TID=%d\n",
		REPORTED_TID);

	for (;;) {
		uint8_t in[512];
		ssize_t n = recv(fd, in, sizeof(in), 0);
		if (n < 0) {
			perror("recv");
			break;
		}
		if (n == 0) {
			fprintf(stderr, "[daemon-responder] daemon closed\n");
			break;
		}
		if (n < (ssize_t)(2 + sizeof(struct pldm_msg_hdr))) {
			fprintf(stderr,
				"[daemon-responder] ignoring short frame (n=%zd)\n",
				n);
			continue;
		}

		uint8_t src_eid = in[0];
		uint8_t mtype = in[1];
		struct pldm_msg *req = (struct pldm_msg *)(in + 2);

		if (mtype != MCTP_TYPE_PLDM) {
			continue;
		}
		if (req->hdr.request != 1) {
			/* Response (or our own reflected response). Skip. */
			continue;
		}
		if (req->hdr.type != PLDM_BASE ||
		    req->hdr.command != PLDM_GET_TID) {
			fprintf(stderr,
				"[daemon-responder] non-GetTID request, skipping (type=%u cmd=0x%02x)\n",
				req->hdr.type, req->hdr.command);
			continue;
		}

		fprintf(stderr,
			"[daemon-responder] GetTID request from src_eid=%u iid=%u\n",
			src_eid, req->hdr.instance_id);

		uint8_t resp_buf[PLDM_MSG_SIZE(PLDM_GET_TID_RESP_BYTES)];
		struct pldm_msg *resp = (struct pldm_msg *)resp_buf;
		int rc = encode_get_tid_resp(req->hdr.instance_id, PLDM_SUCCESS,
					     REPORTED_TID, resp);
		if (rc != PLDM_SUCCESS) {
			fprintf(stderr,
				"[daemon-responder] encode_get_tid_resp rc=%d\n",
				rc);
			continue;
		}
		size_t resp_msg_len =
			sizeof(struct pldm_msg_hdr) + PLDM_GET_TID_RESP_BYTES;

		uint8_t out[2 + sizeof(resp_buf)];
		out[0] = src_eid; /* send to daemon's local_eid -> reflects */
		out[1] = MCTP_TYPE_PLDM;
		memcpy(out + 2, resp, resp_msg_len);

		ssize_t s = send(fd, out, 2 + resp_msg_len, 0);
		if (s < 0) {
			perror("send");
			break;
		}
		fprintf(stderr,
			"[daemon-responder] sent %zd-byte response (TID=%d)\n",
			s, REPORTED_TID);
	}

	close(fd);
	return 0;
}
