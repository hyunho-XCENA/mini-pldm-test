/*
 * PLDM Type 2 (Platform Monitoring & Control, DSP0248) validator over
 * MCTP-over-I2C against the Zephyr target.
 *
 * This is the Type-2 counterpart to pldm_validator.c (which exercises the PLDM
 * base type, DSP0240). The transport plumbing is identical -- a PLDM message
 * rides as the MCTP payload with byte 0 = 0x01 followed by a pldm_msg (3-byte
 * header + payload) -- only the PLDM type in the header is PLDM_PLATFORM (2) and
 * the commands are the platform discovery/read commands.
 *
 * It issues, in order:
 *   GetPDRRepositoryInfo (0x50): repository metadata.
 *   GetPDR              (0x51): walks the PDR repository from record handle 0,
 *                               printing each PDR's type/handle and recording
 *                               the first numeric- and state-sensor IDs it sees.
 *   GetSensorReading    (0x11): reads the numeric sensor discovered above.
 *   GetStateSensorReadings (0x21): reads the state sensor discovered above.
 *
 * The two read tests are skipped (not failed) when the PDR walk found no
 * matching sensor -- a target with an empty repository can still pass.
 *
 * The Zephyr slave is being developed alongside this: until it answers a given
 * command, that test FAILs, and the printed reason (e.g. "no PLDM response",
 * "completion code 0x05 ERROR_UNSUPPORTED_PLDM_CMD", "PLDM type 0, expected 2")
 * tells you what the target still owes.
 *
 * Same wiring assumptions as the other validators: SDA / SCL / GND only, no
 * notify GPIO, so we read the response with a single master read after a settle
 * delay.
 */

#define _GNU_SOURCE
#include "libmctp.h"
#include "libmctp-i2c.h"
#include "libmctp-sizes.h"

#include "libpldm/base.h"
#include "libpldm/platform.h"
#include "libpldm/pldm_types.h"
#include "libpldm/state_set.h"

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

/* Bounds for the PDR walk: how many records to follow, and the largest single
 * PDR record this validator will pull in one GetPDR (also the request_cnt we
 * advertise). Kept under the 128-byte MCTP rx buffer. */
#define PDR_WALK_MAX_RECORDS 32
#define PDR_RECORD_BUF       96

/* The single-read transport occasionally hands back the previous response that
 * is still sitting in the slave's TX buffer (a settle-delay race), so a GetPDR
 * for handle N can return record N-1. We detect that by checking the returned
 * record_handle and re-issue the request up to this many times. */
#define PDR_READ_RETRIES     3

/* Sensor / effecter IDs discovered while walking the PDR repository, consumed by
 * the read tests (GetSensorReading, GetStateSensorReadings,
 * GetSensorThresholds, GetStateEffecterStates). -1 means "not found". */
static int g_numeric_sensor_id = -1;
static int g_state_sensor_id = -1;
static int g_state_effecter_id = -1;
/* The LED: the state effecter whose PDR advertises the Identify state set.
 * Discovered during the walk so neither its id nor its "on" value is hardcoded. */
static int g_led_effecter_id = -1;
/* TID advertised by the Terminus Locator PDR, captured during the walk, so the
 * TID-consistency test can cross-check it against GetTID. -1 = no such PDR. */
static int g_terminus_tid = -1;

/* Tri-state test result: a read test with no sensor to target is SKIPPED, not
 * failed. */
enum tres { T_FAIL = 0, T_PASS = 1, T_SKIP = 2 };

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

/* Validate the MCTP+PLDM response envelope and hand back the inner pldm_msg and
 * its payload length (bytes after the 3-byte PLDM header) so the caller can run
 * the command-specific libpldm decoder. expected_type is the PLDM type the
 * response header must carry -- PLDM_PLATFORM for type-2 commands, PLDM_BASE for
 * base commands like GetPLDMCommands. */
static bool check_pldm_resp(const struct rx_state *st, uint8_t iid, uint8_t tag,
			    uint8_t expected_type, uint8_t cmd,
			    const struct pldm_msg **pmsg, size_t *payload_len)
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
	if (m->hdr.type != expected_type) {
		printf("    PLDM type %u, expected %u\n", m->hdr.type,
		       expected_type);
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

/* Convenience wrapper: most tests expect a PLDM_PLATFORM response. */
static bool check_platform_resp(const struct rx_state *st, uint8_t iid,
				uint8_t tag, uint8_t cmd,
				const struct pldm_msg **pmsg, size_t *payload_len)
{
	return check_pldm_resp(st, iid, tag, PLDM_PLATFORM, cmd, pmsg,
			       payload_len);
}

/* Build the MCTP message for a PLDM request: byte 0 = PLDM message type, then
 * the pldm_msg the libpldm encoder wrote. Returns the base MCTP message length
 * (just the PLDM header); the caller adds payload bytes. The pldm_msg is encoded
 * into *out + 1. */
static size_t frame_pldm_req(uint8_t *out, struct pldm_msg **req)
{
	out[0] = MCTP_MSG_TYPE_PLDM;
	*req = (struct pldm_msg *)(out + 1);
	return 1 + PLDM_HDR_LEN; /* payload bytes added by the caller */
}

/* Human-readable name for a platform (type 2) command code, or NULL if we don't
 * have a label for it. Used to annotate the GetPLDMCommands bitmap. */
static const char *platform_cmd_name(int c)
{
	switch (c) {
	case PLDM_SET_NUMERIC_SENSOR_ENABLE:	   return "SetNumericSensorEnable";
	case PLDM_GET_SENSOR_READING:		   return "GetSensorReading";
	case PLDM_GET_SENSOR_THRESHOLDS:	   return "GetSensorThresholds";
	case PLDM_SET_SENSOR_THRESHOLDS:	   return "SetSensorThresholds";
	case PLDM_SET_STATE_SENSOR_ENABLES:	   return "SetStateSensorEnables";
	case PLDM_GET_STATE_SENSOR_READINGS:	   return "GetStateSensorReadings";
	case PLDM_SET_NUMERIC_EFFECTER_ENABLE:	   return "SetNumericEffecterEnable";
	case PLDM_SET_NUMERIC_EFFECTER_VALUE:	   return "SetNumericEffecterValue";
	case PLDM_GET_NUMERIC_EFFECTER_VALUE:	   return "GetNumericEffecterValue";
	case PLDM_SET_STATE_EFFECTER_STATES:	   return "SetStateEffecterStates";
	case PLDM_GET_STATE_EFFECTER_STATES:	   return "GetStateEffecterStates";
	case PLDM_GET_PDR_REPOSITORY_INFO:	   return "GetPDRRepositoryInfo";
	case PLDM_GET_PDR:			   return "GetPDR";
	case PLDM_PLATFORM_EVENT_MESSAGE:	   return "PlatformEventMessage";
	case PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE: return "PollForPlatformEventMessage";
	default:				   return NULL;
	}
}

/* GetPLDMCommands for the platform type: a PLDM *base* command (0x05) that asks
 * "which type-2 commands do you implement?". GetPLDMCommands needs the version
 * of the type being queried, so we first fetch it with GetPLDMVersion(platform).
 * Both responses are PLDM_BASE-typed. Prints the advertised command bitmap so a
 * single run shows which of this validator's tests the slave should support. */
static enum tres test_get_platform_commands(struct mctp *mctp,
					    struct mctp_binding_i2c *i2c, int fd,
					    uint8_t iid, uint8_t tag,
					    struct rx_state *st)
{
	/* 1) GetPLDMVersion(type=platform) -> ver32_t */
	uint8_t vbuf[1 + PLDM_HDR_LEN + PLDM_GET_VERSION_REQ_BYTES];
	struct pldm_msg *vreq;
	size_t vlen = frame_pldm_req(vbuf, &vreq) + PLDM_GET_VERSION_REQ_BYTES;

	int rc = encode_get_version_req(iid, /*transfer_handle=*/0,
					PLDM_GET_FIRSTPART, PLDM_PLATFORM, vreq);
	if (rc) {
		printf("    encode_get_version_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, vbuf, vlen, st)) {
		return T_FAIL;
	}
	const struct pldm_msg *vm;
	size_t vplen;
	if (!check_pldm_resp(st, iid, tag, PLDM_BASE, PLDM_GET_PLDM_VERSION, &vm,
			     &vplen)) {
		return T_FAIL;
	}
	uint8_t vcc, vflag;
	uint32_t vnext;
	ver32_t ver;
	rc = decode_get_version_resp(vm, vplen, &vcc, &vnext, &vflag, &ver);
	if (rc) {
		printf("    decode_get_version_resp rc=%d (payload %zu bytes)\n",
		       rc, vplen);
		return T_FAIL;
	}
	if (vcc != PLDM_SUCCESS) {
		printf("    GetPLDMVersion(platform) completion code 0x%02x\n",
		       vcc);
		return T_FAIL;
	}
	printf("    platform version %02x.%02x.%02x.%02x\n", ver.major,
	       ver.minor, ver.update, ver.alpha);

	/* 2) GetPLDMCommands(type=platform, ver) -> 32-byte command mask */
	uint8_t cbuf[1 + PLDM_HDR_LEN + PLDM_GET_COMMANDS_REQ_BYTES];
	struct pldm_msg *creq;
	size_t clen = frame_pldm_req(cbuf, &creq) + PLDM_GET_COMMANDS_REQ_BYTES;

	rc = encode_get_commands_req(iid, PLDM_PLATFORM, ver, creq);
	if (rc) {
		printf("    encode_get_commands_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, cbuf, clen, st)) {
		return T_FAIL;
	}
	const struct pldm_msg *cm;
	size_t cplen;
	if (!check_pldm_resp(st, iid, tag, PLDM_BASE, PLDM_GET_PLDM_COMMANDS,
			     &cm, &cplen)) {
		return T_FAIL;
	}
	uint8_t ccc;
	bitfield8_t cmds[32];
	rc = decode_get_commands_resp(cm, cplen, &ccc, cmds);
	if (rc) {
		printf("    decode_get_commands_resp rc=%d (payload %zu bytes)\n",
		       rc, cplen);
		return T_FAIL;
	}
	if (ccc != PLDM_SUCCESS) {
		printf("    GetPLDMCommands(platform) completion code 0x%02x\n",
		       ccc);
		return T_FAIL;
	}

	printf("    platform commands advertised:\n");
	for (int c = 0; c < PLDM_MAX_CMDS_PER_TYPE; c++) {
		if (cmds[c / 8].byte & (1u << (c % 8))) {
			const char *nm = platform_cmd_name(c);
			printf("      0x%02x%s%s\n", c, nm ? "  " : "",
			       nm ? nm : "");
		}
	}
	return T_PASS;
}

/* GetPDRRepositoryInfo (0x50): no request payload;
 * response = [cc][state][update_time 13][oem_update_time 13][record_count 4]
 *            [repo_size 4][largest_record_size 4][xfer_timeout]. */
static enum tres test_get_pdr_repo_info(struct mctp *mctp,
					struct mctp_binding_i2c *i2c, int fd,
					uint8_t iid, uint8_t tag,
					struct rx_state *st)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req);

	int rc = encode_get_pdr_repository_info_req(iid, req, /*payload_length=*/0);
	if (rc) {
		printf("    encode_get_pdr_repository_info_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_GET_PDR_REPOSITORY_INFO, &m,
				 &plen)) {
		return T_FAIL;
	}

	uint8_t cc, repo_state, xfer_timeout;
	uint8_t update_time[PLDM_TIMESTAMP104_SIZE];
	uint8_t oem_update_time[PLDM_TIMESTAMP104_SIZE];
	uint32_t record_count, repo_size, largest_record_size;
	rc = decode_get_pdr_repository_info_resp(
		m, plen, &cc, &repo_state, update_time, oem_update_time,
		&record_count, &repo_size, &largest_record_size, &xfer_timeout);
	if (rc) {
		printf("    decode_get_pdr_repository_info_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return T_FAIL;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return T_FAIL;
	}

	printf("    repo state=%u  records=%u  size=%u B  largest=%u B  xfer-timeout=%u\n",
	       repo_state, record_count, repo_size, largest_record_size,
	       xfer_timeout);
	return T_PASS;
}

/* Pull a sensor_id / effecter_id out of the PDR types the read tests can target:
 * numeric- (2), compact-numeric- (21), state-sensor- (4) and state-effecter-
 * (11) PDRs. In all of them the 16-bit ID is a little-endian uint16 right after
 * the common header and the 2-byte terminus handle. Numeric and compact-numeric
 * sensors are both read with GetSensorReading, so they feed the same
 * numeric-sensor slot. Records the first of each kind into the file-scope
 * globals consumed by the read tests. */
static void note_pdr_ids(uint8_t type, const uint8_t *rec, uint16_t cnt)
{
	const size_t ID_OFF = sizeof(struct pldm_pdr_hdr) + 2;
	bool numeric = type == PLDM_NUMERIC_SENSOR_PDR ||
		       type == PLDM_COMPACT_NUMERIC_SENSOR_PDR;

	if (!numeric && type != PLDM_STATE_SENSOR_PDR &&
	    type != PLDM_STATE_EFFECTER_PDR) {
		return;
	}
	if (cnt < ID_OFF + 2) {
		return; /* record was truncated before the ID */
	}
	uint16_t id = (uint16_t)(rec[ID_OFF] | (rec[ID_OFF + 1] << 8));
	if (numeric && g_numeric_sensor_id < 0) {
		g_numeric_sensor_id = id;
	} else if (type == PLDM_STATE_SENSOR_PDR && g_state_sensor_id < 0) {
		g_state_sensor_id = id;
	} else if (type == PLDM_STATE_EFFECTER_PDR && g_state_effecter_id < 0) {
		g_state_effecter_id = id;
	}
}

/* Issue one GetPDR (0x51) for record_handle and decode the envelope. Returns
 * true and fills *out on success. The record bytes land in rec[] (capacity
 * PDR_RECORD_BUF). */
static bool get_one_pdr(struct mctp *mctp, struct mctp_binding_i2c *i2c, int fd,
			uint8_t iid, uint8_t tag, struct rx_state *st,
			uint32_t record_handle, uint8_t *rec, uint16_t *resp_cnt,
			uint32_t *next_record, uint8_t *transfer_flag)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_PDR_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) + PLDM_GET_PDR_REQ_BYTES;

	int rc = encode_get_pdr_req(iid, record_handle, /*data_transfer_hndl=*/0,
				    PLDM_GET_FIRSTPART, /*request_cnt=*/PDR_RECORD_BUF,
				    /*record_chg_num=*/0, req,
				    PLDM_GET_PDR_REQ_BYTES);
	if (rc) {
		printf("    encode_get_pdr_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_GET_PDR, &m, &plen)) {
		return false;
	}

	uint8_t cc, transfer_crc;
	uint32_t next_data_hndl;
	rc = decode_get_pdr_resp(m, plen, &cc, next_record, &next_data_hndl,
				 transfer_flag, resp_cnt, rec, PDR_RECORD_BUF,
				 &transfer_crc);
	if (rc) {
		printf("    decode_get_pdr_resp rc=%d (payload %zu bytes)\n", rc,
		       plen);
		return false;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return false;
	}
	return true;
}

static uint16_t rd16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

/* Print a Terminus Locator PDR (type 1) and capture its TID for the
 * TID-consistency cross-check. Field offsets follow struct
 * pldm_terminus_locator_pdr, counted from the end of the 10-byte common
 * header: terminus_handle(2), validity(1), tid(1). */
static void scan_terminus_locator_pdr(const uint8_t *rec, uint16_t cnt)
{
	const size_t H = sizeof(struct pldm_pdr_hdr); /* common header = 10 */

	if (cnt < H + 4) {
		return; /* truncated before the tid byte */
	}
	uint16_t terminus_handle = rd16(rec + H);
	uint8_t validity = rec[H + 2];
	uint8_t tid = rec[H + 3];

	if (g_terminus_tid < 0) {
		g_terminus_tid = tid;
	}
	printf("        terminus_handle=%u validity=%u tid=%u\n",
	       terminus_handle, validity, tid);
}

/* Print the identifying fields of a StateEffecter PDR (type 11) and, while we
 * have them parsed, capture the LED: the effecter whose first composite set uses
 * the Identify state set. Both the effecter_id and the later "on" value thus
 * come from the PDR + state set definition rather than hardcoded constants.
 * Field offsets follow struct pldm_state_effecter_pdr, counted from the end of
 * the 10-byte common header. */
static void scan_state_effecter_pdr(const uint8_t *rec, uint16_t cnt)
{
	const size_t H = sizeof(struct pldm_pdr_hdr); /* common header = 10 */

	/* Need through composite_effecter_count + first state_set_id + size. */
	if (cnt < H + 18) {
		return;
	}
	uint16_t effecter_id = rd16(rec + H + 2);
	uint16_t entity_type = rd16(rec + H + 4);
	uint16_t semantic_id = rd16(rec + H + 10);
	uint8_t comp_count = rec[H + 14];
	uint16_t state_set_id = rd16(rec + H + 15);
	uint8_t pss_size = rec[H + 17];

	if (state_set_id == PLDM_STATE_SET_IDENTIFY_STATE &&
	    g_led_effecter_id < 0) {
		g_led_effecter_id = effecter_id;
	}

	printf("        effecter_id=0x%04x entity_type=%u semantic_id=%u comp=%u state_set=%u%s\n",
	       effecter_id, entity_type, semantic_id, comp_count, state_set_id,
	       state_set_id == PLDM_STATE_SET_IDENTIFY_STATE ? " (IdentifyState)" :
								"");

	if (pss_size > 0 && cnt >= H + 18 + pss_size) {
		printf("        supported states:");
		for (uint8_t b = 0; b < pss_size; b++) {
			uint8_t bits = rec[H + 18 + b];
			for (int i = 0; i < 8; i++) {
				if (bits & (1u << i)) {
					printf(" %d", b * 8 + i);
				}
			}
		}
		printf("\n");
	}
}

static const char *pdr_type_name(uint8_t t)
{
	switch (t) {
	case PLDM_TERMINUS_LOCATOR_PDR:   return "TerminusLocator";
	case PLDM_NUMERIC_SENSOR_PDR:     return "NumericSensor";
	case PLDM_COMPACT_NUMERIC_SENSOR_PDR: return "CompactNumericSensor";
	case PLDM_STATE_SENSOR_PDR:       return "StateSensor";
	case PLDM_NUMERIC_EFFECTER_PDR:   return "NumericEffecter";
	case PLDM_STATE_EFFECTER_PDR:     return "StateEffecter";
	case PLDM_PDR_ENTITY_ASSOCIATION: return "EntityAssociation";
	case PLDM_PDR_FRU_RECORD_SET:     return "FruRecordSet";
	default:                          return "other";
	}
}

/* GetPDR (0x51): walk the repository from record handle 0, following
 * nextRecordHandle until it returns 0. PASSes if the first GetPDR succeeds; an
 * empty repository (first handle's next == 0 with zero records) still passes. */
static enum tres test_get_pdr_walk(struct mctp *mctp,
				   struct mctp_binding_i2c *i2c, int fd,
				   uint8_t iid, uint8_t tag, struct rx_state *st)
{
	uint32_t handle = 0; /* 0 = first record */
	int count = 0;

	for (; count < PDR_WALK_MAX_RECORDS; count++) {
		uint8_t rec[PDR_RECORD_BUF];
		uint16_t resp_cnt = 0;
		uint32_t next = 0;
		uint8_t transfer_flag = 0;
		struct pldm_pdr_hdr hdr;
		bool have_hdr = false;

		/* Re-issue until the returned record matches what we asked for,
		 * to ride out the stale-buffer race. A request for handle 0
		 * means "the first record", whose handle we can't predict, so
		 * that one is accepted as-is. */
		int attempt;
		for (attempt = 0; attempt < PDR_READ_RETRIES; attempt++) {
			if (!get_one_pdr(mctp, i2c, fd, iid, tag, st, handle,
					 rec, &resp_cnt, &next,
					 &transfer_flag)) {
				/* A hard transport/decode failure -- the first
				 * one is the real failure; mid-walk it still
				 * fails so the operator sees an incomplete walk. */
				printf("    GetPDR failed at record handle %u\n",
				       handle);
				return T_FAIL;
			}

			have_hdr = resp_cnt >= sizeof(struct pldm_pdr_hdr);
			if (have_hdr) {
				memcpy(&hdr, rec, sizeof(hdr));
			}
			if (handle == 0 || !have_hdr ||
			    hdr.record_handle == handle) {
				break; /* fresh, matching response */
			}
			printf("    stale read: asked handle %u, got %u -- retry %d\n",
			       handle, hdr.record_handle, attempt + 1);
		}
		if (attempt == PDR_READ_RETRIES) {
			printf("    handle %u kept returning a stale record -- giving up\n",
			       handle);
			return T_FAIL;
		}

		if (have_hdr) {
			printf("    PDR handle=%u type=%u (%s) len=%u bytes=%u%s\n",
			       hdr.record_handle, hdr.type,
			       pdr_type_name(hdr.type), hdr.length, resp_cnt,
			       transfer_flag == PLDM_START_AND_END ? "" :
			       " [partial]");
			note_pdr_ids(hdr.type, rec, resp_cnt);
			if (hdr.type == PLDM_STATE_EFFECTER_PDR) {
				scan_state_effecter_pdr(rec, resp_cnt);
			} else if (hdr.type == PLDM_TERMINUS_LOCATOR_PDR) {
				scan_terminus_locator_pdr(rec, resp_cnt);
			}
		} else {
			printf("    PDR handle=%u: %u bytes, shorter than a PDR header\n",
			       handle, resp_cnt);
		}

		if (next == 0) {
			count++;
			break; /* end of repository */
		}
		handle = next;
	}

	printf("    walked %d PDR%s", count, count == 1 ? "" : "s");
	if (g_numeric_sensor_id >= 0) {
		printf(", numeric sensor ID %d", g_numeric_sensor_id);
	}
	if (g_state_sensor_id >= 0) {
		printf(", state sensor ID %d", g_state_sensor_id);
	}
	if (g_state_effecter_id >= 0) {
		printf(", state effecter ID %d", g_state_effecter_id);
	}
	if (g_led_effecter_id >= 0) {
		printf(", LED effecter ID %d", g_led_effecter_id);
	}
	printf("\n");
	return T_PASS;
}

/* GetSensorReading (0x11) on the numeric sensor discovered during the walk. */
static enum tres test_get_sensor_reading(struct mctp *mctp,
					 struct mctp_binding_i2c *i2c, int fd,
					 uint8_t iid, uint8_t tag,
					 struct rx_state *st)
{
	if (g_numeric_sensor_id < 0) {
		printf("    no numeric sensor PDR found in the walk -- nothing to read\n");
		return T_SKIP;
	}
	uint16_t sensor_id = (uint16_t)g_numeric_sensor_id;

	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_SENSOR_READING_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) +
			 PLDM_GET_SENSOR_READING_REQ_BYTES;

	int rc = encode_get_sensor_reading_req(iid, sensor_id,
					       /*rearm_event_state=*/false, req);
	if (rc) {
		printf("    encode_get_sensor_reading_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_GET_SENSOR_READING, &m,
				 &plen)) {
		return T_FAIL;
	}

	uint8_t cc, data_size, op_state, event_enable, present_state;
	uint8_t previous_state, event_state, present_reading;
	rc = decode_get_sensor_reading_resp(m, plen, &cc, &data_size, &op_state,
					    &event_enable, &present_state,
					    &previous_state, &event_state,
					    &present_reading);
	if (rc) {
		printf("    decode_get_sensor_reading_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return T_FAIL;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return T_FAIL;
	}

	printf("    sensor %u: op_state=%u present_state=%u reading=%u (data_size=%u)\n",
	       sensor_id, op_state, present_state, present_reading, data_size);
	return T_PASS;
}

/* GetStateSensorReadings (0x21) on the state sensor discovered during the walk. */
static enum tres test_get_state_sensor_readings(struct mctp *mctp,
						struct mctp_binding_i2c *i2c,
						int fd, uint8_t iid,
						uint8_t tag, struct rx_state *st)
{
	if (g_state_sensor_id < 0) {
		printf("    no state sensor PDR found in the walk -- nothing to read\n");
		return T_SKIP;
	}
	uint16_t sensor_id = (uint16_t)g_state_sensor_id;

	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_STATE_SENSOR_READINGS_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) +
			 PLDM_GET_STATE_SENSOR_READINGS_REQ_BYTES;

	/* sensor_rearm bit 0 set => rearm the first composite sensor; reserved 0. */
	bitfield8_t sensor_rearm = { .byte = 0 };
	int rc = encode_get_state_sensor_readings_req(iid, sensor_id,
						      sensor_rearm,
						      /*reserved=*/0, req);
	if (rc) {
		printf("    encode_get_state_sensor_readings_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_GET_STATE_SENSOR_READINGS,
				 &m, &plen)) {
		return T_FAIL;
	}

	uint8_t cc, comp_count = 8; /* in/out: max composite sets we accept */
	get_sensor_state_field field[8];
	rc = decode_get_state_sensor_readings_resp(m, plen, &cc, &comp_count,
						   field);
	if (rc) {
		printf("    decode_get_state_sensor_readings_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return T_FAIL;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n", cc);
		return T_FAIL;
	}

	printf("    sensor %u: %u composite set%s\n", sensor_id, comp_count,
	       comp_count == 1 ? "" : "s");
	for (uint8_t i = 0; i < comp_count && i < 8; i++) {
		printf("      [%u] op_state=%u present=%u previous=%u event=%u\n",
		       i, field[i].sensor_op_state, field[i].present_state,
		       field[i].previous_state, field[i].event_state);
	}
	return T_PASS;
}

/* GetStateEffecterStates (0x3a) on the state effecter discovered during the
 * walk. Read-only: it reports the effecter's present state, it does not change
 * it. */
static enum tres test_get_state_effecter_states(struct mctp *mctp,
						struct mctp_binding_i2c *i2c,
						int fd, uint8_t iid,
						uint8_t tag, struct rx_state *st)
{
	if (g_state_effecter_id < 0) {
		printf("    no state effecter PDR found in the walk -- nothing to read\n");
		return T_SKIP;
	}
	uint16_t effecter_id = (uint16_t)g_state_effecter_id;

	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) +
			 PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES;

	int rc = encode_get_state_effecter_states_req(
		iid, effecter_id, req, PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES);
	if (rc) {
		printf("    encode_get_state_effecter_states_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_GET_STATE_EFFECTER_STATES,
				 &m, &plen)) {
		return T_FAIL;
	}

	struct pldm_get_state_effecter_states_resp resp;
	memset(&resp, 0, sizeof(resp));
	rc = decode_get_state_effecter_states_resp(m, plen, &resp);
	if (rc) {
		printf("    decode_get_state_effecter_states_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return T_FAIL;
	}
	if (resp.completion_code != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n",
		       resp.completion_code);
		return T_FAIL;
	}

	printf("    effecter %u: %u composite set%s\n", effecter_id,
	       resp.comp_effecter_count,
	       resp.comp_effecter_count == 1 ? "" : "s");
	for (uint8_t i = 0;
	     i < resp.comp_effecter_count &&
	     i < PLDM_GET_EFFECTER_STATE_FIELD_COUNT_MAX;
	     i++) {
		printf("      [%u] op_state=%u pending=%u present=%u\n", i,
		       resp.field[i].effecter_op_state,
		       resp.field[i].pending_state, resp.field[i].present_state);
	}
	return T_PASS;
}

/* One GetStateEffecterStates for effecter_id; on success hands back the
 * present_state of the first composite set. Used by the SetStateEffecterStates
 * test to read the state before/after driving it. */
static bool effecter_present_state(struct mctp *mctp,
				   struct mctp_binding_i2c *i2c, int fd,
				   uint8_t iid, uint8_t tag, struct rx_state *st,
				   uint16_t effecter_id, uint8_t *present)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) +
			 PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES;

	int rc = encode_get_state_effecter_states_req(
		iid, effecter_id, req, PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES);
	if (rc) {
		printf("    encode_get_state_effecter_states_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_GET_STATE_EFFECTER_STATES,
				 &m, &plen)) {
		return false;
	}

	struct pldm_get_state_effecter_states_resp resp;
	memset(&resp, 0, sizeof(resp));
	rc = decode_get_state_effecter_states_resp(m, plen, &resp);
	if (rc) {
		printf("    decode_get_state_effecter_states_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return false;
	}
	if (resp.completion_code != PLDM_SUCCESS) {
		printf("    completion code 0x%02x (not SUCCESS)\n",
		       resp.completion_code);
		return false;
	}
	if (resp.comp_effecter_count < 1) {
		printf("    effecter %u returned no state fields\n", effecter_id);
		return false;
	}
	*present = resp.field[0].present_state;
	return true;
}

/* SetStateEffecterStates (0x39) for one composite effecter set. Drives
 * effecter_id's first set to `state`. Returns true only on a SUCCESS response. */
static bool set_one_effecter_state(struct mctp *mctp,
				   struct mctp_binding_i2c *i2c, int fd,
				   uint8_t iid, uint8_t tag, struct rx_state *st,
				   uint16_t effecter_id, uint8_t state)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN + PLDM_SET_STATE_EFFECTER_STATES_REQ_BYTES];
	struct pldm_msg *req;
	/* Payload for one set: effecter_id(2) + comp_count(1) + 1 field(2). */
	size_t msg_len = frame_pldm_req(reqbuf, &req) + 3 + 2;

	set_effecter_state_field field = {
		.set_request = PLDM_REQUEST_SET,
		.effecter_state = state,
	};
	int rc = encode_set_state_effecter_states_req(iid, effecter_id,
						      /*comp_effecter_count=*/1,
						      &field, req);
	if (rc) {
		printf("    encode_set_state_effecter_states_req rc=%d\n", rc);
		return false;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return false;
	}

	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, PLDM_SET_STATE_EFFECTER_STATES,
				 &m, &plen)) {
		return false;
	}

	uint8_t cc;
	rc = decode_set_state_effecter_states_resp(m, plen, &cc);
	if (rc) {
		printf("    decode_set_state_effecter_states_resp rc=%d (payload %zu bytes)\n",
		       rc, plen);
		return false;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    set completion code 0x%02x (not SUCCESS)\n", cc);
		return false;
	}
	return true;
}

/* SetStateEffecterStates (0x39): drive the LED effecter on, confirm via
 * read-back, then restore the original state. The effecter and the "on" value
 * are taken from the walk's PDR scan (the Identify-state effecter) and the
 * Identify state set definition, not hardcoded. This is the one test that
 * mutates the target -- it briefly changes the LED and puts it back. */
static enum tres test_set_state_effecter_states(struct mctp *mctp,
						struct mctp_binding_i2c *i2c,
						int fd, uint8_t iid,
						uint8_t tag, struct rx_state *st)
{
	if (g_led_effecter_id < 0) {
		printf("    no Identify-state effecter found in the walk -- no LED to drive\n");
		return T_SKIP;
	}
	uint16_t effecter_id = (uint16_t)g_led_effecter_id;
	const uint8_t on_state = PLDM_STATE_SET_IDENTIFY_STATE_ASSERTED;
	uint8_t original = 0;

	if (!effecter_present_state(mctp, i2c, fd, iid, tag, st, effecter_id,
				    &original)) {
		printf("    could not read effecter %u before set\n",
		       effecter_id);
		return T_FAIL;
	}
	printf("    effecter %u present_state=%u (before)\n", effecter_id,
	       original);

	if (!set_one_effecter_state(mctp, i2c, fd, iid, tag, st, effecter_id,
				    on_state)) {
		return T_FAIL;
	}
	printf("    set effecter %u -> state %u (ON/ASSERTED)\n", effecter_id,
	       on_state);

	uint8_t after = 0;
	if (!effecter_present_state(mctp, i2c, fd, iid, tag, st, effecter_id,
				    &after)) {
		printf("    read-back after set failed\n");
		return T_FAIL;
	}
	printf("    effecter %u present_state=%u (after set)\n", effecter_id,
	       after);

	bool applied = after == on_state;
	if (!applied) {
		printf("    present_state %u != requested %u -- effecter did not apply\n",
		       after, on_state);
	}

	/* Restore the original state whatever the outcome above. */
	if (set_one_effecter_state(mctp, i2c, fd, iid, tag, st, effecter_id,
				   original)) {
		printf("    restored effecter %u -> state %u\n", effecter_id,
		       original);
	} else {
		printf("    WARNING: failed to restore effecter %u to state %u\n",
		       effecter_id, original);
	}

	return applied ? T_PASS : T_FAIL;
}

/* For commands we only want to confirm the responder *handles* (not necessarily
 * supports): a well-formed PLDM response carrying the matching command code is a
 * PASS regardless of completion code -- even 0x05 UNSUPPORTED_PLDM_CMD proves the
 * request crossed the wire and was parsed. Only a missing/garbled response or a
 * mismatched envelope fails. */
static enum tres roundtrip_result(const struct rx_state *st, uint8_t iid,
				  uint8_t tag, uint8_t cmd)
{
	const struct pldm_msg *m;
	size_t plen;
	if (!check_platform_resp(st, iid, tag, cmd, &m, &plen)) {
		return T_FAIL;
	}
	if (plen < 1) {
		printf("    response carried no completion code byte\n");
		return T_FAIL;
	}
	uint8_t cc = m->payload[0];
	const char *meaning =
		cc == PLDM_SUCCESS ? "SUCCESS" :
		cc == PLDM_ERROR_UNSUPPORTED_PLDM_CMD ? "UNSUPPORTED_PLDM_CMD" :
		cc == PLDM_ERROR_INVALID_LENGTH ? "INVALID_LENGTH" :
						  "other";
	printf("    round-trip OK -- completion code 0x%02x (%s)\n", cc,
	       meaning);
	return T_PASS;
}

/* GetSensorThresholds (0x12) reachability check. libpldm has no requester codec,
 * so the 2-byte request (sensorID) is packed by hand. We only confirm a PLDM
 * response returns -- the slave may legitimately answer UNSUPPORTED. */
static enum tres test_rt_get_sensor_thresholds(struct mctp *mctp,
					       struct mctp_binding_i2c *i2c,
					       int fd, uint8_t iid, uint8_t tag,
					       struct rx_state *st)
{
	if (g_numeric_sensor_id < 0) {
		printf("    no numeric sensor found in the walk -- nothing to query\n");
		return T_SKIP;
	}
	uint16_t sensor_id = (uint16_t)g_numeric_sensor_id;

	uint8_t reqbuf[1 + PLDM_HDR_LEN + 2];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) + 2;

	struct pldm_header_info hi = {
		.msg_type = PLDM_REQUEST,
		.instance = iid,
		.pldm_type = PLDM_PLATFORM,
		.command = PLDM_GET_SENSOR_THRESHOLDS,
	};
	if (pack_pldm_header(&hi, &req->hdr)) {
		printf("    pack_pldm_header failed\n");
		return T_FAIL;
	}
	req->payload[0] = (uint8_t)(sensor_id & 0xff);
	req->payload[1] = (uint8_t)(sensor_id >> 8);

	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}
	return roundtrip_result(st, iid, tag, PLDM_GET_SENSOR_THRESHOLDS);
}

/* SetSensorThresholds (0x13) reachability check -- packed by hand (no libpldm
 * requester codec). NOTE: this is a state-MUTATING command. If the slave ever
 * implements it, this request would overwrite the sensor's six thresholds with
 * zero. It is safe here only as long as the slave answers UNSUPPORTED; the test
 * prints a warning to keep that explicit. */
static enum tres test_rt_set_sensor_thresholds(struct mctp *mctp,
					       struct mctp_binding_i2c *i2c,
					       int fd, uint8_t iid, uint8_t tag,
					       struct rx_state *st)
{
	if (g_numeric_sensor_id < 0) {
		printf("    no numeric sensor found in the walk -- nothing to query\n");
		return T_SKIP;
	}
	uint16_t sensor_id = (uint16_t)g_numeric_sensor_id;

	/* sensorID(2) + sensorDataSize(1) + 6 uint8 thresholds(6) = 9 bytes. */
	uint8_t reqbuf[1 + PLDM_HDR_LEN + 9];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) + 9;

	struct pldm_header_info hi = {
		.msg_type = PLDM_REQUEST,
		.instance = iid,
		.pldm_type = PLDM_PLATFORM,
		.command = PLDM_SET_SENSOR_THRESHOLDS,
	};
	if (pack_pldm_header(&hi, &req->hdr)) {
		printf("    pack_pldm_header failed\n");
		return T_FAIL;
	}
	req->payload[0] = (uint8_t)(sensor_id & 0xff);
	req->payload[1] = (uint8_t)(sensor_id >> 8);
	req->payload[2] = PLDM_SENSOR_DATA_SIZE_UINT8;
	memset(&req->payload[3], 0, 6); /* six thresholds, all zero */

	printf("    NOTE: mutating command; safe only while slave reports UNSUPPORTED\n");
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}
	return roundtrip_result(st, iid, tag, PLDM_SET_SENSOR_THRESHOLDS);
}

/* PollForPlatformEventMessage (0x0b) reachability check (BMC->terminus, the
 * correct direction). Uses the libpldm request encoder; we deliberately skip the
 * full response decoder (it rejects a short error response) -- a matching
 * envelope plus any completion code is enough to prove the command round-trips. */
static enum tres test_rt_poll_for_event(struct mctp *mctp,
					struct mctp_binding_i2c *i2c, int fd,
					uint8_t iid, uint8_t tag,
					struct rx_state *st)
{
	uint8_t reqbuf[1 + PLDM_HDR_LEN +
		       PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES];
	struct pldm_msg *req;
	size_t msg_len = frame_pldm_req(reqbuf, &req) +
			 PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES;

	int rc = encode_poll_for_platform_event_message_req(
		iid, PLDM_PLATFORM_EVENT_MESSAGE_FORMAT_VERSION,
		PLDM_GET_FIRSTPART, /*data_transfer_handle=*/0,
		/*event_id_to_acknowledge=*/0, req,
		PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES);
	if (rc) {
		printf("    encode_poll_for_platform_event_message_req rc=%d\n",
		       rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, reqbuf, msg_len, st)) {
		return T_FAIL;
	}
	return roundtrip_result(st, iid, tag,
				PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE);
}

/* TID consistency: the slave's TID (GetTID, a PLDM base command) should match
 * the TID its Terminus Locator PDR advertises. The walk captures the PDR's TID
 * into g_terminus_tid; here we read the live TID and compare. The slave does not
 * expose a Terminus Locator PDR yet, so this SKIPs for now -- but it is wired up
 * so the cross-check runs automatically once that PDR appears. */
static enum tres test_tid_consistency(struct mctp *mctp,
				      struct mctp_binding_i2c *i2c, int fd,
				      uint8_t iid, uint8_t tag,
				      struct rx_state *st)
{
	uint8_t buf[1 + PLDM_HDR_LEN];
	struct pldm_msg *req;
	size_t len = frame_pldm_req(buf, &req);

	int rc = encode_get_tid_req(iid, req);
	if (rc) {
		printf("    encode_get_tid_req rc=%d\n", rc);
		return T_FAIL;
	}
	if (!round_trip(mctp, i2c, fd, tag, buf, len, st)) {
		return T_FAIL;
	}
	const struct pldm_msg *m;
	size_t plen;
	if (!check_pldm_resp(st, iid, tag, PLDM_BASE, PLDM_GET_TID, &m, &plen)) {
		return T_FAIL;
	}
	uint8_t cc, tid;
	rc = decode_get_tid_resp(m, plen, &cc, &tid);
	if (rc) {
		printf("    decode_get_tid_resp rc=%d (payload %zu bytes)\n", rc,
		       plen);
		return T_FAIL;
	}
	if (cc != PLDM_SUCCESS) {
		printf("    GetTID completion code 0x%02x (not SUCCESS)\n", cc);
		return T_FAIL;
	}
	printf("    GetTID reports TID=%u\n", tid);

	if (g_terminus_tid < 0) {
		printf("    no Terminus Locator PDR in the repository -- nothing to cross-check\n");
		return T_SKIP;
	}
	if ((int)tid != g_terminus_tid) {
		printf("    MISMATCH: Terminus Locator PDR TID=%d != GetTID %u\n",
		       g_terminus_tid, tid);
		return T_FAIL;
	}
	printf("    matches Terminus Locator PDR TID=%d\n", g_terminus_tid);
	return T_PASS;
}

typedef enum tres (*test_fn)(struct mctp *, struct mctp_binding_i2c *, int,
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
		"[pldm-type2-validator] %s  us=EID%u@0x%02x  peer=EID%u@0x%02x  settle=%dms\n",
		I2C_DEV, OUR_EID, OUR_I2C_ADDR, PEER_EID, PEER_I2C_ADDR,
		RESP_SETTLE_MS);

	struct test_case tests[] = {
		{ "GetPLDMCommands(platform)", test_get_platform_commands },
		{ "GetPDRRepositoryInfo", test_get_pdr_repo_info },
		{ "GetPDR (walk)", test_get_pdr_walk },
		{ "GetSensorReading", test_get_sensor_reading },
		{ "GetStateSensorReadings", test_get_state_sensor_readings },
		{ "GetStateEffecterStates", test_get_state_effecter_states },
		{ "SetStateEffecterStates (LED)", test_set_state_effecter_states },
		{ "GetSensorThresholds (round-trip)",
		  test_rt_get_sensor_thresholds },
		{ "SetSensorThresholds (round-trip)",
		  test_rt_set_sensor_thresholds },
		{ "PollForPlatformEventMessage (round-trip)",
		  test_rt_poll_for_event },
		{ "TID consistency", test_tid_consistency },
	};
	size_t ntests = sizeof(tests) / sizeof(tests[0]);
	int failures = 0, passed = 0, skipped = 0;

	for (size_t i = 0; i < ntests; i++) {
		printf("\n=== [%zu/%zu] %s ===\n", i + 1, ntests,
		       tests[i].name);
		/* PLDM instance IDs are 5-bit; MCTP tags are 3-bit. */
		enum tres r = tests[i].fn(mctp, i2c, tctx.fd,
					  (uint8_t)(i & PLDM_INSTANCE_MAX),
					  (uint8_t)(i & 0x7), &st);
		const char *tag = r == T_PASS ? "[PASS]" :
				  r == T_SKIP ? "[SKIP]" : "[FAIL]";
		printf("%s %s\n", tag, tests[i].name);
		if (r == T_PASS) {
			passed++;
		} else if (r == T_SKIP) {
			skipped++;
		} else {
			failures++;
		}
	}

	printf("\npldm-type2-validator: %d passed, %d failed, %d skipped (of %zu)\n",
	       passed, failures, skipped, ntests);

	mctp_destroy(mctp);
	free(i2c);
	close(tctx.fd);
	return failures ? 1 : 0;
}
