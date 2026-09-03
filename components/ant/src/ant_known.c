/*
 * ant_known.c - paired-device table. See ant_known.h.
 */
#include "ant_known.h"
#include <string.h>

#define TYPE_OF(t) ((uint8_t)((t) & 0x7Fu))

void ant_known_init(ant_known_t *k)
{
    memset(k, 0, sizeof(*k));
}

static int index_of(const ant_known_t *k, uint8_t device_type)
{
    uint8_t t = TYPE_OF(device_type);
    for (unsigned i = 0; i < k->count; i++) {
        if (k->dev[i].device_type == t) return (int)i;
    }
    return -1;
}

const ant_known_device_t *ant_known_find(const ant_known_t *k, uint8_t device_type)
{
    int i = index_of(k, device_type);
    return i >= 0 ? &k->dev[i] : NULL;
}

bool ant_known_put(ant_known_t *k, const ant_known_device_t *dev)
{
    if (!dev || dev->device_num == 0 || TYPE_OF(dev->device_type) == 0) return false;
    ant_known_device_t d = { dev->device_num, TYPE_OF(dev->device_type), dev->trans_type };
    int i = index_of(k, d.device_type);
    if (i < 0) {
        if (k->count >= ANT_KNOWN_MAX) return false;
        i = k->count++;
    }
    k->dev[i] = d;
    return true;
}

bool ant_known_remove(ant_known_t *k, uint8_t device_type)
{
    int i = index_of(k, device_type);
    if (i < 0) return false;
    for (unsigned j = (unsigned)i + 1; j < k->count; j++) k->dev[j - 1] = k->dev[j];
    k->count--;
    memset(&k->dev[k->count], 0, sizeof(k->dev[0]));
    return true;
}

void ant_known_clear(ant_known_t *k)
{
    ant_known_init(k);
}

size_t ant_known_to_blob(const ant_known_t *k, uint8_t *blob, size_t cap)
{
    size_t n = 0;
    for (unsigned i = 0; i < k->count && n + 4u <= cap; i++, n += 4u) {
        blob[n]     = (uint8_t)(k->dev[i].device_num & 0xFFu);
        blob[n + 1] = (uint8_t)(k->dev[i].device_num >> 8);
        blob[n + 2] = k->dev[i].device_type;
        blob[n + 3] = k->dev[i].trans_type;
    }
    return n;
}

size_t ant_known_from_blob(ant_known_t *k, const uint8_t *blob, size_t len)
{
    ant_known_init(k);
    if (!blob) return 0;
    for (size_t off = 0; off + 4u <= len; off += 4u) {
        ant_known_device_t d = {
            (uint16_t)(blob[off] | ((uint16_t)blob[off + 1] << 8)),
            blob[off + 2], blob[off + 3]
        };
        ant_known_put(k, &d);
    }
    return k->count;
}
