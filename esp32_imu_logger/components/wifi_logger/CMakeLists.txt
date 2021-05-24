set(srcs "wifi_logger.cpp" "utils.cpp" "udp_handler.cpp")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")                    