idf_component_register(SRCS "drone_low_level.c"
                    INCLUDE_DIRS "."
                    REQUIRES encoder usb_jtag mcpwm)
