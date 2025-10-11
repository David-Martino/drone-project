idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc)

set(CMAKE_OBJECT_PATH_MAX 512)
