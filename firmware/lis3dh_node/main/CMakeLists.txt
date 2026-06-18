idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        monomod_board       # D-label -> GPIO pin map (XIAO / Super Mini)
        lis3dh              # accel + ADC1 over SPI
        monomod_protocol    # packet_types.h, crc16.h
        monomod_core        # timestamp.hpp
        monomod_drivers     # WiFi provisioning
        monomod_net         # NetworkManager (UDP/TCP/mDNS)
        led_strip           # NeoPixel status (managed dep, see idf_component.yml)
        nvs_flash
        driver
        esp_timer
        esp_system          # esp_task_wdt
        esp_driver_usb_serial_jtag
)
