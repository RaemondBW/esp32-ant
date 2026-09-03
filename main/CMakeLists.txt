idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES ant nvs_flash
)
# Role: display (default, receives) or sensor (transmits).
#   idf.py -DANT_ROLE=sensor build
if(ANT_ROLE STREQUAL "sensor")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ANT_APP_ROLE_SENSOR)
endif()
# Forget the paired devices at boot (one-off): idf.py -DANT_FORGET_KNOWN=1 build
if(ANT_FORGET_KNOWN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ANT_FORGET_KNOWN)
endif()
# Only pair with devices received at >= this RSSI (dBm): idf.py -DANT_PROXIMITY_RSSI=-60 build
if(ANT_PROXIMITY_RSSI)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ANT_PROXIMITY_RSSI=${ANT_PROXIMITY_RSSI})
endif()
# Log every packet the radio hook decodes: idf.py -DANT_LOG_FRAMES=1 build
if(ANT_LOG_FRAMES)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ANT_LOG_FRAMES)
endif()
