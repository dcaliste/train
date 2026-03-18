set(requires nvs_flash esp_netif esp_driver_gpio esp_wifi esp_driver_gptimer mdns esp_adc esp_driver_mcpwm bt)

idf_component_register(SRCS power.c system.c io.c track.c wifi.c
                       PRIV_REQUIRES ${requires}
                       INCLUDE_DIRS ".")
