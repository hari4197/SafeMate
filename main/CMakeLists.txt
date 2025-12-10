idf_component_register(
    SRCS "Project_SafeMate.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_rainmaker
        nvs_flash
        esp_netif
        esp_wifi
        rmaker_common
        rmaker_app_network
        bt
        led_strip
)

    set(COMPONENT_ADD_LDFLAGS "-u esp_ble_gap_start_advertising")
