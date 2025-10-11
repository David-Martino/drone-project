idf_component_register(SRCS "mmc5983ma.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus 
                       PRIV_REQUIRES crazyflie platform)