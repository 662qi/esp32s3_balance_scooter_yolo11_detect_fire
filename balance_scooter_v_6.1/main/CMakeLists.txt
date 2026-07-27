set(srcs)

idf_component_register(SRCS "main.c" "ble_spp_server.c" "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES
                        components
                        nvs_flash
                        bt
                        driver
                        esp_timer
                        nimble_peripheral_utils
)
