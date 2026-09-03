/*
 * ant_radio.h - Radio transport abstraction (HAL) for the ANT stack.
 *
 * The ANT serial-message state machine (ant_channel) speaks the byte-oriented
 * ANT protocol to *some* transport behind this interface. In the pure-ESP32
 * build the over-the-air radio is NOT a serial ANT chip - it is the internal
 * radio driven directly as a PHY (see ant_espphy.*), which does not use this
 * transport. What remains here is:
 *
 *   - ant_radio_loopback: a pure-software transport (and simulated ANT device)
 *                        that exercises the ANT serial protocol and the channel
 *                        state machine with no hardware. Used by the host tests
 *                        and available for on-target self-test. It is also the
 *                        seam where a future "internal radio as a serial ANT
 *                        endpoint" backend would plug in.
 *
 * A transport is byte-oriented: the caller writes framed ANT messages with
 * write(), and pumps received bytes out with read(). Implementations must be
 * non-blocking; read() returns however many bytes are currently available.
 */
#ifndef ANT_RADIO_H
#define ANT_RADIO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ant_radio ant_radio_t;

struct ant_radio {
    /* Write `len` bytes to the transport. Returns bytes written (or <0 err). */
    int  (*write)(ant_radio_t *self, const uint8_t *data, size_t len);
    /* Read up to `cap` bytes into `buf`. Returns count (0 if none, <0 err). */
    int  (*read)(ant_radio_t *self, uint8_t *buf, size_t cap);
    /* Optional: called periodically so a backend can service its own I/O. */
    void (*poll)(ant_radio_t *self);
    /* Optional: release resources. */
    void (*close)(ant_radio_t *self);
    void *ctx; /* backend private state */
};

static inline int ant_radio_write(ant_radio_t *r, const uint8_t *d, size_t n)
{
    return (r && r->write) ? r->write(r, d, n) : -1;
}
static inline int ant_radio_read(ant_radio_t *r, uint8_t *b, size_t n)
{
    return (r && r->read) ? r->read(r, b, n) : -1;
}
static inline void ant_radio_poll(ant_radio_t *r)
{
    if (r && r->poll) r->poll(r);
}
static inline void ant_radio_close(ant_radio_t *r)
{
    if (r && r->close) r->close(r);
}

#ifdef __cplusplus
}
#endif

#endif /* ANT_RADIO_H */
