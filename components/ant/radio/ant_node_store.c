/*
 * ant_node_store.c - built-in stores for the known-device table: NVS and a
 * process-lifetime RAM copy. See ant_node.h.
 */
#include "ant_node.h"

#ifdef ESP_PLATFORM

#include <string.h>
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "ant_node";

#define STORE_NS   "ant"
#define STORE_KEY  "known"

/* --------------------------------- NVS ------------------------------------ */

static bool nvs_load(const ant_node_store_t *s, ant_known_t *k)
{
    (void)s;
    nvs_handle_t h;
    if (nvs_open(STORE_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t blob[ANT_KNOWN_MAX * 4];
    size_t len = sizeof(blob);
    esp_err_t e = nvs_get_blob(h, STORE_KEY, blob, &len);
    nvs_close(h);
    if (e != ESP_OK) return false;
    ant_known_from_blob(k, blob, len);
    return true;
}

static bool nvs_save(const ant_node_store_t *s, const ant_known_t *k)
{
    (void)s;
    nvs_handle_t h;
    esp_err_t e = nvs_open(STORE_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open: %s (nvs_flash_init() called?)", esp_err_to_name(e));
        return false;
    }
    uint8_t blob[ANT_KNOWN_MAX * 4];
    size_t len = ant_known_to_blob(k, blob, sizeof(blob));
    e = len ? nvs_set_blob(h, STORE_KEY, blob, len) : nvs_erase_key(h, STORE_KEY);
    if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;      /* erasing nothing */
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) ESP_LOGW(TAG, "nvs save: %s", esp_err_to_name(e));
    return e == ESP_OK;
}

const ant_node_store_t ant_node_store_nvs = { nvs_load, nvs_save, NULL };

/* --------------------------------- RAM ------------------------------------ */

static ant_known_t s_ram;
static bool        s_ram_valid;

static bool ram_load(const ant_node_store_t *s, ant_known_t *k)
{
    (void)s;
    if (!s_ram_valid) return false;
    *k = s_ram;
    return true;
}

static bool ram_save(const ant_node_store_t *s, const ant_known_t *k)
{
    (void)s;
    s_ram = *k;
    s_ram_valid = true;
    return true;
}

const ant_node_store_t ant_node_store_ram = { ram_load, ram_save, NULL };

#endif /* ESP_PLATFORM */
