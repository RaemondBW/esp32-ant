/*
 * ant_message.h - ANT serial message protocol (framing layer).
 *
 * Implements the ANT Message Protocol as used over the serial interface of an
 * ANT network processor (nRF24AP2 / D52 / softdevice). This is the lowest
 * portable layer: it builds and parses framed messages of the form
 *
 *     SYNC | LENGTH | MSG_ID | DATA[LENGTH] | CHECKSUM
 *
 * where SYNC = 0xA4, LENGTH is the number of DATA bytes, and CHECKSUM is the
 * XOR of every preceding byte (SYNC included).
 *
 * Reference: "ANT Message Protocol and Usage", Dynastream/Garmin.
 *
 * This file contains no hardware dependencies and is unit-tested on the host.
 */
#ifndef ANT_MESSAGE_H
#define ANT_MESSAGE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Framing constants. */
#define ANT_SYNC_TX          0xA4u  /* host -> ANT chip */
#define ANT_SYNC_RX          0xA4u  /* ANT chip -> host (0xA4); 0xA5 legacy */
#define ANT_MAX_DATA_LEN     8u     /* standard ANT payload is 8 bytes */
#define ANT_MAX_MSG_DATA     32u    /* extended / burst framing headroom */
/* Fully framed buffer: sync + len + id + data + checksum. */
#define ANT_MAX_FRAME_LEN    (3u + ANT_MAX_MSG_DATA + 1u)

/* Message IDs - control (host -> ANT). */
#define ANT_MSG_UNASSIGN_CHANNEL     0x41u
#define ANT_MSG_ASSIGN_CHANNEL       0x42u
#define ANT_MSG_SET_CHANNEL_PERIOD   0x43u
#define ANT_MSG_SET_CHANNEL_RF_FREQ  0x45u
#define ANT_MSG_SET_NETWORK_KEY      0x46u
#define ANT_MSG_RESET_SYSTEM         0x4Au
#define ANT_MSG_OPEN_CHANNEL         0x4Bu
#define ANT_MSG_CLOSE_CHANNEL        0x4Cu
#define ANT_MSG_REQUEST              0x4Du
#define ANT_MSG_SET_CHANNEL_ID       0x51u
#define ANT_MSG_SET_CHANNEL_TX_POWER 0x60u
#define ANT_MSG_SET_SEARCH_TIMEOUT   0x44u

/* Message IDs - data (both directions). */
#define ANT_MSG_BROADCAST_DATA       0x4Eu
#define ANT_MSG_ACKNOWLEDGED_DATA    0x4Fu
#define ANT_MSG_BURST_DATA           0x50u

/* Message IDs - notifications (ANT -> host). */
#define ANT_MSG_CHANNEL_RESPONSE     0x40u  /* channel event / response to cmd */
#define ANT_MSG_CHANNEL_STATUS       0x52u
#define ANT_MSG_CHANNEL_ID           0x51u  /* also a response */
#define ANT_MSG_CAPABILITIES         0x54u
#define ANT_MSG_STARTUP              0x6Fu  /* sent after reset */
#define ANT_MSG_SERIAL_ERROR         0xAEu

/* Channel types (assign channel). */
/* Bit 7 of the device type: a master asking to be paired. A slave searching
 * without it accepts either; a slave searching with it set accepts only
 * masters that set it. */
#define ANT_DEVICE_TYPE_PAIRING_BIT  0x80u
#define ANT_DEVICE_TYPE_OF(t)        ((uint8_t)((t) & 0x7Fu))

#define ANT_CHANNEL_TYPE_SLAVE_RX    0x00u  /* bidirectional receive */
#define ANT_CHANNEL_TYPE_MASTER_TX   0x10u  /* bidirectional transmit */
#define ANT_CHANNEL_TYPE_SHARED_SLAVE 0x20u
#define ANT_CHANNEL_TYPE_SHARED_MASTER 0x30u
#define ANT_CHANNEL_TYPE_SLAVE_RX_ONLY 0x40u

/* Channel event codes (payload of ANT_MSG_CHANNEL_RESPONSE when msg_id==1). */
#define ANT_EVENT_RX_SEARCH_TIMEOUT  0x01u
#define ANT_EVENT_RX_FAIL            0x02u
#define ANT_EVENT_TX                 0x03u
#define ANT_EVENT_TRANSFER_RX_FAILED 0x04u
#define ANT_EVENT_TRANSFER_TX_COMPLETED 0x05u
#define ANT_EVENT_TRANSFER_TX_FAILED 0x06u
#define ANT_EVENT_CHANNEL_CLOSED     0x07u
#define ANT_EVENT_RX_FAIL_GO_TO_SEARCH 0x08u
#define ANT_EVENT_CHANNEL_COLLISION  0x09u
#define ANT_RESPONSE_NO_ERROR        0x00u  /* success for a command response */

/* Command response / error codes (same payload slot as the event codes). */
#define ANT_RESPONSE_CHANNEL_IN_WRONG_STATE      0x15u
#define ANT_RESPONSE_CHANNEL_NOT_OPENED          0x16u
#define ANT_RESPONSE_CHANNEL_ID_NOT_SET          0x18u
#define ANT_RESPONSE_TRANSFER_IN_PROGRESS        0x1Fu
#define ANT_RESPONSE_TRANSFER_SEQUENCE_ERROR     0x20u
#define ANT_RESPONSE_INVALID_MESSAGE             0x28u
#define ANT_RESPONSE_INVALID_NETWORK_NUMBER      0x29u
#define ANT_RESPONSE_INVALID_PARAMETER           0x33u

/* Channel status byte (ANT_MSG_CHANNEL_STATUS data[1]). */
#define ANT_STATUS_UNASSIGNED        0x00u
#define ANT_STATUS_ASSIGNED          0x01u
#define ANT_STATUS_SEARCHING         0x02u
#define ANT_STATUS_TRACKING          0x03u
#define ANT_STATUS_STATE_MASK        0x03u

/* Extra channel types / message ids used by the embedded MAC. */
#define ANT_CHANNEL_TYPE_MASTER_TX_ONLY 0x50u
#define ANT_MSG_VERSION              0x3Eu

/*
 * A parsed ANT message. `data` points into the caller-supplied frame buffer for
 * zero-copy parsing, or into `own` when constructed. `data_len` is LENGTH.
 */
typedef struct {
    uint8_t  msg_id;
    uint8_t  data_len;
    uint8_t  data[ANT_MAX_MSG_DATA];
} ant_message_t;

/* Compute the ANT checksum (XOR of all bytes in `buf`). */
uint8_t ant_checksum(const uint8_t *buf, size_t len);

/*
 * Serialize a message into `out` (must be >= ANT_MAX_FRAME_LEN).
 * Returns total framed length, or 0 on invalid input.
 */
size_t ant_message_encode(uint8_t msg_id, const uint8_t *data, uint8_t data_len,
                          uint8_t *out, size_t out_cap);

/*
 * Parse one complete frame from `buf`. On success fills `msg`, sets
 * *consumed to the number of bytes used, returns true. Returns false if the
 * buffer does not yet contain a full valid frame (or checksum mismatch).
 */
bool ant_message_decode(const uint8_t *buf, size_t len,
                        ant_message_t *msg, size_t *consumed);

/*
 * Incremental byte-stream parser (for UART reception). Feed bytes one at a
 * time; when a full, checksum-valid frame is assembled it is written to
 * `out_msg` and the function returns true. Resets automatically on framing or
 * checksum errors so the stream re-synchronizes on the next SYNC.
 */
typedef struct {
    uint8_t  state;
    uint8_t  length;
    uint8_t  idx;
    uint8_t  running_xor;
    uint8_t  msg_id;
    uint8_t  buf[ANT_MAX_MSG_DATA];
} ant_parser_t;

void ant_parser_init(ant_parser_t *p);
bool ant_parser_push(ant_parser_t *p, uint8_t byte, ant_message_t *out_msg);

/* ---- Convenience builders for standard control messages. ----
 * Each writes a full framed message into `out` and returns its length. */
size_t ant_build_reset(uint8_t *out, size_t cap);
size_t ant_build_set_network_key(uint8_t net, const uint8_t key[8],
                                 uint8_t *out, size_t cap);
size_t ant_build_assign_channel(uint8_t chan, uint8_t type, uint8_t net,
                                uint8_t *out, size_t cap);
size_t ant_build_set_channel_id(uint8_t chan, uint16_t dev_num,
                                uint8_t dev_type, uint8_t trans_type,
                                uint8_t *out, size_t cap);
size_t ant_build_set_rf_freq(uint8_t chan, uint8_t freq, uint8_t *out, size_t cap);
size_t ant_build_set_period(uint8_t chan, uint16_t period, uint8_t *out, size_t cap);
size_t ant_build_set_search_timeout(uint8_t chan, uint8_t timeout,
                                    uint8_t *out, size_t cap);
size_t ant_build_open_channel(uint8_t chan, uint8_t *out, size_t cap);
size_t ant_build_close_channel(uint8_t chan, uint8_t *out, size_t cap);
size_t ant_build_broadcast_data(uint8_t chan, const uint8_t payload[8],
                                uint8_t *out, size_t cap);
size_t ant_build_acknowledged_data(uint8_t chan, const uint8_t payload[8],
                                   uint8_t *out, size_t cap);
size_t ant_build_request(uint8_t chan, uint8_t requested_msg_id,
                         uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* ANT_MESSAGE_H */
