idf_component_register(SRCS "vl53l1x.c" 
                    "zranger2.c"
                    "core/src/VL53L1X_calibration.c"
                    "core/src/VL53L1X_api.c"
                       INCLUDE_DIRS "." "include" "core/inc"
                     REQUIRES i2c_bus crazyflie platform config)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-overflow")