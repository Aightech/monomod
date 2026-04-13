idf_component_register(
    SRCS
        "src/wifi_manager.cpp"
        "src/wifi_provisioning.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif nvs_flash esp_event esp_http_client
)
