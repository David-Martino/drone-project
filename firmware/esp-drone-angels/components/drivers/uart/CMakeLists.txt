idf_component_register(
  SRCS "uart_sync_isr.c" "uart_esp32.c"
  INCLUDE_DIRS "include"        # public header lives in include/
  REQUIRES driver                # uart_esp32.h uses uart_port_t from driver/uart.h
  PRIV_REQUIRES esp_timer       # only used inside .c for esp_timer_get_time()
)
