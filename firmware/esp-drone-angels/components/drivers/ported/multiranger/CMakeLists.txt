idf_component_register(SRCS "multiranger.c"
                       INCLUDE_DIRS "." "include"
                       PRIV_REQUIRES crazyflie config i2c_bus deck)