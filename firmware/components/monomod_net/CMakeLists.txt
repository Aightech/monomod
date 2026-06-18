idf_component_register(
    SRCS "src/network_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        monomod_protocol    # packet_types.h, crc16.h
        mdns
        lwip
        esp_timer
        esp_hw_support      # esp_mac.h (esp_read_mac)
        esp_netif
)
