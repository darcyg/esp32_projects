idf_component_register(SRCS "src/MPU.cpp"
                       INCLUDE_DIRS "include" "../SPIbus/include"
                       REQUIRES "SPIbus")