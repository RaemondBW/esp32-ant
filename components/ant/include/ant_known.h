/*
 * ant_known.h - the paired-device table: which master a slave channel should
 * reconnect to for each ANT+ device type.
 *
 * ANT pairing is nothing more than remembering a channel id. A display opens
 * a slave channel with wildcards, the first master heard fills them in, and
 * from then on the display opens with that id so it only ever tracks that
 * sensor. This table holds one such id per device type (one heart-rate
 * strap, one power meter, ...) as a plain array that a store can persist as
 * a blob. It is portable and host-tested; ant_node drives it.
 */
#ifndef ANT_KNOWN_H
#define ANT_KNOWN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_KNOWN_MAX   8u

/* A paired master. device_type carries no pairing bit. 4 bytes, no padding,
 * so an array of them is the persisted blob. */
typedef struct {
    uint16_t device_num;
    uint8_t  device_type;
    uint8_t  trans_type;
} ant_known_device_t;

typedef struct {
    ant_known_device_t dev[ANT_KNOWN_MAX];
    uint8_t count;
} ant_known_t;

void ant_known_init(ant_known_t *k);
/* The device paired for `device_type` (pairing bit ignored), or NULL. */
const ant_known_device_t *ant_known_find(const ant_known_t *k, uint8_t device_type);
/* Pair `dev` for its type, replacing any previous device of that type. Returns
 * false if the table is full or the device is not a real identity (device
 * number 0). No change (and true) if it is already the paired device. */
bool ant_known_put(ant_known_t *k, const ant_known_device_t *dev);
/* Unpair the device of that type. Returns whether there was one. */
bool ant_known_remove(ant_known_t *k, uint8_t device_type);
void ant_known_clear(ant_known_t *k);

/* Blob form: `count` * 4 bytes, little-endian device number. Returns bytes
 * written / devices loaded; load drops entries that fail validation. */
size_t ant_known_to_blob(const ant_known_t *k, uint8_t *blob, size_t cap);
size_t ant_known_from_blob(ant_known_t *k, const uint8_t *blob, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* ANT_KNOWN_H */
