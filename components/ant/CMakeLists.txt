# ESP-IDF component: complete ANT/ANT+ stack on the ESP32's own radio.
#
# src/  is portable C99 with no ESP dependencies (also built by test/host):
#   ant_message/ant_channel/antplus_profiles - host-side ANT serial stack
#   ant_phy_shockburst/ant_sb_link           - ANT on-air framing
#   ant_mac                                  - the ANT protocol engine (MAC)
#   ant_known                                - paired-device table
#   ant_air                                  - virtual air PHY (tests/self-test)
#   ant_embedded                             - ANT serial bridge onto ant_mac
#   ant_selftest                             - boot-time protocol self-test
# radio/ ant_espphy drives the ESP32-S3/C3 BLE core as a raw 1 Mbit/s GFSK
# ANT modem (ESPwn32 technique) as the ant_phy_t; ant_node is radio + engine +
# FreeRTOS task behind the application API (ant_node_store: its NVS/RAM
# stores for the paired-device table); ant_radio_loopback is the
# legacy simulated-chip transport used by the host tests.

idf_component_register(
    SRCS
        "src/ant_message.c"
        "src/ant_channel.c"
        "src/antplus_profiles.c"
        "src/ant_phy_shockburst.c"
        "src/ant_sb_link.c"
        "src/ant_mac.c"
        "src/ant_known.c"
        "src/ant_air.c"
        "src/ant_embedded.c"
        "src/ant_selftest.c"
        "radio/ant_radio_loopback.c"
        "radio/ant_espphy.c"
        "radio/ant_node.c"
        "radio/ant_node_store.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_timer
    PRIV_REQUIRES
        bt
        esp_phy
        nvs_flash
)
