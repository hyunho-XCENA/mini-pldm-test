/*
 * PLDM loopback requester.
 *
 * Uses libpldm's mctp-demux transport unchanged. Sends a GetTID request to
 * TID=1 (mapped to EID=9) and decodes the response.
 *
 * Run AFTER the responder is already listening on \0mctp-mux.
 */

#include <libpldm/base.h>
#include <libpldm/pldm.h>
#include <libpldm/transport.h>
#include <libpldm/transport/mctp-demux.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define PEER_EID 8
#define PEER_TID 1

int main(void)
{
	struct pldm_transport_mctp_demux *demux = NULL;
	int rc = pldm_transport_mctp_demux_init(&demux);
	if (rc) {
		fprintf(stderr,
			"[requester] pldm_transport_mctp_demux_init rc=%d\n",
			rc);
		return 1;
	}
	fprintf(stderr, "[requester] connected to \\0mctp-mux\n");

	rc = pldm_transport_mctp_demux_map_tid(demux, PEER_TID, PEER_EID);
	if (rc) {
		fprintf(stderr, "[requester] map_tid rc=%d\n", rc);
		return 1;
	}

	struct pldm_transport *t = pldm_transport_mctp_demux_core(demux);

	uint8_t req_buf[PLDM_MSG_SIZE(PLDM_GET_TID_REQ_BYTES)];
	struct pldm_msg *req = (struct pldm_msg *)req_buf;
	rc = encode_get_tid_req(/*instance_id=*/7, req);
	if (rc != PLDM_SUCCESS) {
		fprintf(stderr, "[requester] encode_get_tid_req rc=%d\n", rc);
		return 1;
	}
	size_t req_msg_len =
		sizeof(struct pldm_msg_hdr) + PLDM_GET_TID_REQ_BYTES;
	fprintf(stderr,
		"[requester] sending GetTID (iid=7) to TID=%d (EID=%d), %zu bytes\n",
		PEER_TID, PEER_EID, req_msg_len);

	void *resp = NULL;
	size_t resp_len = 0;
	pldm_requester_rc_t prc = pldm_transport_send_recv_msg(
		t, PEER_TID, req, req_msg_len, &resp, &resp_len);
	if (prc != PLDM_REQUESTER_SUCCESS) {
		fprintf(stderr, "[requester] send_recv_msg failed rc=%d\n",
			(int)prc);
		return 1;
	}
	fprintf(stderr, "[requester] received %zu-byte response\n", resp_len);

	uint8_t cc = 0xff;
	uint8_t tid = 0;
	rc = decode_get_tid_resp((struct pldm_msg *)resp,
				 resp_len - sizeof(struct pldm_msg_hdr), &cc,
				 &tid);
	if (rc != PLDM_SUCCESS) {
		fprintf(stderr, "[requester] decode_get_tid_resp rc=%d\n", rc);
		free(resp);
		return 1;
	}
	printf("[requester] PLDM GetTID round-trip OK: completion=0x%02x TID=%u\n",
	       cc, tid);

	free(resp);
	pldm_transport_mctp_demux_destroy(demux);
	return 0;
}
