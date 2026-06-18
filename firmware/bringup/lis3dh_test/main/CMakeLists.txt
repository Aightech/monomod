idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        monomod_board   # D-label -> GPIO pin map (XIAO / Super Mini)
        lis3dh          # repo driver: SPI bus + WHO_AM_I probe
        led_strip       # managed dep (see idf_component.yml) — drives the NeoPixel
        driver
)
