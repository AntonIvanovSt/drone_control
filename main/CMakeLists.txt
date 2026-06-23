idf_component_register(SRCS "drone_control.c"
                    INCLUDE_DIRS "."
                    REQUIRES encoder usb_jtag mcpwm kinematics robot_state)
