set(srcs "src/wifi_logger.cpp" "src/utils.cpp" "src/udp_handler.cpp")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")                    