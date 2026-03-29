set(requires nvs_flash esp_netif esp_driver_gpio esp_wifi esp_driver_gptimer mdns esp_adc esp_driver_mcpwm bt esp_http_server app_update)

idf_component_register(SRCS power.c system.c io.c track.c wifi.c webserver.c ota.c
                       PRIV_REQUIRES ${requires}
                       INCLUDE_DIRS "."
                       EMBED_FILES favicon.png
                       EMBED_TXTFILES index.html)
