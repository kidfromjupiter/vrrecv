idf_component_register(
    SRCS
        "vrrecv.c"
        "receiver_protocol.c"
        "magnetometer_protocol.c"
    INCLUDE_DIRS "."
        REQUIRES
        esp_driver_gpio
        esp_driver_usb_serial_jtag
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
)
