idf_component_register(SRCS "dd_board.c" "tca6408.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver led_strip)