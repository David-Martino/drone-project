# Codebase Notes

These notes are intended to assist in understanding how the `esp-drone` codebase is structured and how it functions. It is based on Commit `527ee2e` on [GitHub](https://github.com/espressif/esp-drone) and assumes you are using ESP-IDF v5.0 installed via the VS Code ESP-IDF extension. It also assumes you are using an ESP32-S3 for your drone.

## Getting Started

To start exploring and modifying the `esp-drone` codebase, follow this guide.

### Configuring VS Code

First configure VS Code for the `esp-drone` project by following these steps:

1. Clone the project from [GitHub](https://github.com/espressif/esp-drone) and open the project folder in VS Code.
1. Install the following extensions in VS Code:
    - C/C++
    - C/C++ Extension Pack
    - C/C++ Themes
    - CMake Tools
    - ESP-IDF
1. In VS Code, use `ctrl+shift+p`, search `ESP-IDF: Configure ESP-IDF Extension`, and follow the Express setup mode to install ESP-IDF v5.0. 
1. Once installation is complete, follow the VS Code prompt to generate `compile_commands.json`.
1. Once `.\build\compile_commands.json` has been generated, point the C/C++ Intellisense to the compile commands by using `ctrl+shift+p` and searching `C/C++: Select Intellisense Configuration...`, selecting the newly-generated file from the presented options. 

### Testing Build

Then do a test build of the `esp-drone` project by following these steps:

1. Set the device target by using `ctrl+shift+p` and searching `ESP-IDF: Set Target`, selecting `esp32s3` from the list of options, and then `ESP32-S3 chip (via builtin USB-JTAG)`. 
1. Build the project by using `ctrl+shift+p` and searching `ESP-IDF: Build Your Project`. 


## Reset / Power-Up

This outlines what happens when the `esp-drone` project has been flashed to an ESP32-S3 and then resets or powers up. 

### ROM Bootloader

The ESP32-S3 has an immutable bootloader that has been burned into ROM, which runs after the ESP32-S3 has been powered up or reset (including the reset that occurs after flashing your code). 

**What it does:**
- Initialises the clocks, the CPU, and a minimal amount of RAM so that basic start up code can be executed.
- Configures the SPI (serial peripheral interface) for flash memory so that flashed code can be accessed.
- Reads the partition table from a fixed flash address so that it can locate the second-stage bootloader.
- Loads the second-stage bootloader from flash into RAM and jumps to it to begin executing it.

### Second-Stage Bootloader

When you build and flash a project using ESP-IDF, the second-stage bootloader will be generated for you at runtime when you build the project before flashing it to your device. The ROM bootloader will load it into RAM and jump to it after powering up or resetting.

**What it does:**
- Reads the flash's partition table to locate the available app partitions.
- Chooses which app partition to boot (your app is usually located in the default `factory` partition).
- Finishes initialising RAM (including external PSRAM if you have it) so it can all be used by your app.
- Further initialises the clocks to perform a full boot.
- Configures and starts the MMU (memory management unit), which maps sections of your app code's instructions and constants to flash memory addresses. The instructions and constants are loaded into CPU cache in small chunks, and if the next instruction is in the cache then it will execute immediately. If not, the MMU will load it on demand from flash into cache for subsequent execution (called execute-in-place, or XIP).
- Jumps to your app's "reset vector", which is the very first instruction in your app.

### App Startup Code

Part of the build process involves including some startup code (part of the ESP-IDF and ESP's FreeRTOS port) that links the boot process to your app code. This is usually your app's "reset vector" and runs on CPU0 after the second-stage bootloader completes. 

**What it does on CPU0:**
- `esp\v5.0\esp-idf\components\esp_system\port\cpu_start.c` contains the ESP-IDF entrypoint function `call_start_cpu0()` that acts as the app's "reset vector". It does a bunch of low-level bring-up and then calls the `SYS_STARTUP_FN()` macro. This ultimately resolves to calling `start_cpu0_default()` on CPU0.
- `esp\v5.0\esp-idf\components\esp_system\startup.c` contains `start_cpu0_default()`, which does more initialisation and calls the FreeRTOS function `esp_startup_start_app()`.
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\xtensa\port.c` contains `esp_startup_start_app()`, which calls `esp_startup_start_app_common()` and then starts the FreeRTOS scheduler on CPU0 (which also starts the FreeRTOS tick timer). 
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\port_common.c` contains `esp_startup_start_app_common()`, which initialises cross-core interrupts on CPU0 and creates the `main` task in the `Ready` state from the `main_task()` function.
- The running scheduler on CPU0 sees the `main` task in its list of ready tasks and begins executing its `main_task()` function. 
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\port_common.c` contains the `main_task()` function, which waits for the second core to finish initialising (if there is one), frees the RAM of FreeRTOS startup stacks, and then calls the `app_main()` function (in your project's directory). Usually, the `app_main()` function (indirectly) creates your app's tasks that run periodically via the scheduler and then returns.
- If `app_main()` has this "bootstrap-and-return" pattern and returns after completion, the `main_task()` function will delete the `main` task from the scheduler's list of tasks, freeing its stack in RAM.

**What it does on CPU1:**
- If there is a second core (like on the ESP32-S3), then the `call_start_cpu0()` function also sets the boot address of CPU1 to the `call_start_cpu1()` function via `start_other_core()` and resets CPU1, which causes CPU1 to start executing `call_start_cpu1()`. 
- Like `call_start_cpu0()`, the `call_start_cpu1()` function also does a bunch of low-level bring-up and then calls the `SYS_STARTUP_FN()` macro. This ultimately resolves to calling `start_cpu_other_cores_default()` on CPU1.
- `esp\v5.0\esp-idf\components\esp_system\startup.c` contains `start_cpu_other_cores_default()`, which does a bunch of initialisation and calls `esp_startup_start_app_other_cores()`.
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\xtensa\port.c`  contains `esp_startup_start_app_other_cores()`, which waits for the FreeRTOS scheduler to start on CPU0, initialises cross-core interrupts on CPU1, and starts the FreeRTOS scheduler on CPU1 (note that the ESP32-S3 does not have symmetric multi-processing, or SMP, and so each CPU runs its own scheduler and interrupts keep the cores synchronised instead of a global scheduler). 


## ESP Drone App Code

### Platform Initialisation

- After initialising non-volatile storage (NVS) flash memory for wifi communications, `app_main()` calls `platformInit()`. 
- `esp-drone\components\platform\platform.c` contains the `platformInit()` function, which is seemingly just scaffolding for supporting different hardware platforms. Currently, it defaults to the `EP20` device type, which causes the `active_config` instance of the `platformConfig_t` struct to be set to a default. The variables in that default are read by subsequent initialisation functions, and the default struct looks like this:

```
static const platformConfig_t *active_config;

active_config -> (platformConfig_t){
  .deviceType = "EP20",
  .deviceTypeName = "ESPlane 2.0 ",
  .sensorImplementation = SensorImplementation_mpu6050_HMC5883L_MS5611,
  .physicalLayoutAntennasAreClose = false,
  .motorMap = motorMapDefaultBrushed,
};
```

- Here, `SensorImplementation_mpu6050_HMC5883L_MS5611` is just an enum value, but `esp-drone\components\drivers\general\motors\motors_def_cf2.c` ultimately defines `motorMapDefaultBrushed` as follows: 

```
static const MotorPerifDef CONN_M1 = { .drvType = BRUSHED };
static const MotorPerifDef CONN_M2 = { .drvType = BRUSHED };
static const MotorPerifDef CONN_M3 = { .drvType = BRUSHED };
static const MotorPerifDef CONN_M4 = { .drvType = BRUSHED };

const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS] = {
    &CONN_M1, &CONN_M2, &CONN_M3, &CONN_M4
};
```

### The SYSTEM Task

- After `app_main()` calls `platformInit()`, it then calls `systemLaunch()`. 
- `esp-drone\components\core\crazyflie\modules\src\system.c` contains the `systemLaunch()` function, which contains the `STATIC_MEM_TASK_CREATE()` macro. This resolves to calling the `xTaskCreateStatic()` function to create the `SYSTEM` task using the stack that was initialised in memory by the file scope line `STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);` in `system.c`. The `SYSTEM` task runs the `systemTask()` function, which is also defined in `system.c`.
- After creating the `SYSTEM` task, the `systemLaunch()` function returns, which causes the `app_main()` function to return, which causes the `main_task()` function from ESP-IDF's FreeRTOS port to delete the `main` task that FreeRTOS created to launch the app code (i.e., the task that ran the `main_task()` function). 
- At this point, the app startup code provided by ESP-IDF has fully completed and the ESP Drone app code is entirely managing the FreeRTOS tasks. 
- The `SYSTEM` task is then the only ready task available and so the `systemTask()` function begins executing, starting with a bunch of initialisation.  

#### LED Initialisation

- First, `systemTask()` calls `ledInit()`, which is defined in `esp-drone\components\drivers\general\led\led_esp32.c`.
- `ledInit()` initialises the GPIO for three LEDs (Blue, Red, and Green) by first instantiating a `gpio_config_t` struct and then passing it to the `gpio_config()` function from ESP-IDF. 
- Then `systemTask()` calls `ledSet(CHG_LED, 1)`, which is aslo defined in `esp-drone\components\drivers\general\led\led_esp32.c`.
- `ledSet(CHG_LED, 1)` causes the Red LED to be turned on, indicating that the ESP Drone app code is starting up. 

#### Wifi Initialisation

- Then `systemTask()` calls `wifiInit()`, which is defined in `esp-drone\components\drivers\general\wifi\wifi_esp32.c`.
- `wifi_esp32.c` globally defines several values, including the following:

```
#define UDP_SERVER_PORT         2390  // UDP socket listens here
#define UDP_SERVER_BUFSIZE      64  // 64B for temp Rx buffer
static char WIFI_SSID[32] = "";
static char WIFI_PWD[64] = CONFIG_WIFI_PASSWORD;  // = 12345678
static uint8_t WIFI_CH = CONFIG_WIFI_CHANNEL;  // = 6
#define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN  // = 3
```

- And `wifi_esp32.h` also defines: 

```
// 32 bytes is enough for CRTP packets (30+1) + checksum 1
#define WIFI_RX_TX_PACKET_SIZE   (32)

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[WIFI_RX_TX_PACKET_SIZE];
} UDPPacket;
```

- `wifiInit()` first initialises two queues of length 16*33B called `udpDataRx` and `udpDataTx` using the FreeRTOS function `xQueueCreate()`. These are essentially thread-safe FIFO buffers in heap memory that can hold 16 `UDPPacket` structs.
- `wifiInit()` then calls `espnow_storage_init()`, which essentially ensures NVS flash was correctly initialised for wifi communication by `app_main()`. If erasing and reflashing isn't working to configure NVS, it will flag that the ESP32 should select the next app partition for the next reboot if it exists (i.e., boot the application firmware from the `ota_1` app slot instead of the `factory` one). 
- `wifiInit()` then initialises an empty pointer called `ap_netif` that will store the address of an `esp_netif_t` network interface object, which will be opaquely created and managed by ESP-IDF.
- `wifiInit()` then calls the `esp_netif_init()` function from ESP-IDF, which initialises the LwIP TCP/IP stack. This indirectly creates the LwIP core task, which handles all of the internet protocol aspects of wifi communication such as UDP/TCP, IP, ARP/ND, DHCP, DNS. It also prepares the LwIP core task to connect with an `esp_netif` network interface. 
- `wifiInit()` then calls `esp_event_loop_create_default()` to create the default event loop, which includes creating the `sys_evt` task and a corresponding queue. This task accepts network events from various parts of the wifi stack and serialises them so the events can run their callbacks safely. 
- `wifiInit()` then calls the `esp_netif_create_default_wifi_ap()` function, which loads a default configuration for a SoftAP `esp_netif_t` network interface object, uses that configuration to create an `esp_netif_t` network interface object (which also creates the actual `esp_netif` interface), connects the LwIP and `esp_wifi` layers together with the `esp_netif` bridge, and registers default handlers with the default event loop (i.e., tells the `sys_evt` task what function it should run for each of the default event types that can be posted to the default event loop's queue). This function also returns a pointer to the newly created `esp_netif_t` network interface object, which `wifiInit()` assigns to `ap_netif` pointer variable. 
- `wifiInit()` then loads a default wifi driver configuration (shown below) and uses it to call the `esp_wifi_init()` function, which ultimately allocates resources for the `esp_wifi` driver (including Rx/Tx buffers and the wifi NVS structure) and starts the `wifi` task. 

```
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT() = { .osi_funcs = &g_wifi_osi_funcs, .wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs, .static_rx_buf_num = 10, .dynamic_rx_buf_num = 32, .tx_buf_type = 1, .static_tx_buf_num = 0, .dynamic_tx_buf_num = 32, .cache_tx_buf_num = 0, .csi_enable = 0, .ampdu_rx_enable = 1, .ampdu_tx_enable = 1, .amsdu_tx_enable = 0, .nvs_enable = 1, .nano_enable = 0, .rx_ba_win = 6, .wifi_task_core_id = 0, .beacon_max_len = 752, .mgmt_sbuf_num = 32, .feature_caps = g_wifi_feature_caps, .sta_disconnected_pm = 0, .magic = 0x1F2F3F4F}
```
- `wifiInit()` then registers another handler with the default event loop using `esp_event_handler_instance_register()`, which essentially just logs when a station(device) joins or leaves the ESP32's SoftAP. 
- `wifiInit()` then gets the ESP32's AP MAC address with `esp_wifi_get_mac()` and uses it to create an SSID (i.e., the name of its wifi network) with `sprintf()` in the form `ESP-DRONE_00112233445566`.
- `wifiInit()` then defines a default configuration for the wifi radio that expands to:
```
wifi_config_t wifi_config = {
  .ap = {
    .channel = 6,  // sets 2.4GHz wifi channel
    .max_connection = 3,  // sets max number of clients
    .authmode = WIFI_AUTH_WPA_WPA2_PSK,  // requires password
  },
};
```
- `wifiInit()` then saves the SSID and password (default is `12345678`) with `memcpy()`, tells the `esp_wifi` driver to operate a SoftAP with `esp_wifi_set_mode()`, passes the `wifi_config` to the `esp_wifi` with `esp_wifi_set_config()`, and starts the SoftAP radio with `esp_wifi_start()`.
- `wifiInit()` then disables the second wifi channel with `esp_wifi_set_channel()` and loads a default configuration for ESP-NOW into `espnow_config` (shown below). 

```
espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT() = { .pmk = "ESP_NOW", .forward_enable = 1, .forward_switch_channel = 0, .sec_enable = 0, .reserved1 = 0, .qsize = 32, .send_retry_num = 10, .send_max_timeout = ( ( TickType_t ) ( ( ( TickType_t ) ( 3000 ) * ( TickType_t ) 1000 ) / ( TickType_t ) 1000U ) ), .receive_enable = { .ack = 1, .forward = 1, .group = 1, .provisoning = 0, .control_bind = 0, .control_data = 0, .ota_status = 0, .ota_data = 0, .debug_log = 0, .debug_command = 0, .data = 0, .sec_status = 0, .sec = 0, .sec_data = 0, .reserved2 = 0, }, }
```

- `wifiInit()` then uses that configuration with `espnow_init()` to register a handler with the default event loop with `esp_event_handler_instance_register()` (just logs when a station joins or leaves the ESP32's SoftAP), register send/recv callback functions with the `esp_wifi` driver, and create the `espnow_main` task with `xTaskCreate()` using the `espnow_main_task()` function.

- `wifiInit()` then registers a handler with the default event loop that logs when another device using ESP-NOW connects to the drone as an ESP-NOW controller using `esp_event_handler_register()`, and then opens a "bind window" for 30 seconds (during which controllers can connect and be remembered as trusted controllers).  
- `wifiInit()` then registers the `espnow_ctrl_data_cb()` callback function that the ESP-NOW srecv callback function can call when it detects an ESP-NOW controller has sent a CONTROL_DATA frame. The `espnow_ctrl_data_cb()` callback function formats the parsed values from the CONTROL_DATA frame into a message and adds that message to the same `udpDataRx` queue that the UDP receiver uses (so the flight controller can read all control commands from one stream). 
- `wifiInit()` then defines a static IP (shown below), uses `esp_netif_dhcps_stop()` to stop the DHCP server that was automatically started by `esp_wifi_start()`, updates the IP info in the `esp_netif_t` network interface object that `ap_netif` points to with the new static IP using `esp_netif_set_ip_info()`, and then restarts the DHCP server with `esp_netif_dhcps_start()`.
```
esp_netif_ip_info_t ip_info = {
    .ip.addr = ipaddr_addr("192.168.43.42"),
    .netmask.addr = ipaddr_addr("255.255.255.0"),
    .gw.addr      = ipaddr_addr("192.168.43.42"),
};
```
- `wifiInit()` then uses `udp_server_create()` to tell LwIP that it needs to create a socket (which includes a queue and some other state), return the socket's handle to the `sock` variable, and bind the socket to `port 2390`. Subsequently, when LwIP receives a UDP packet destined for `port 2390`, it will post the processed UDP frame to the socket's queue. When `wifiInit()` eventually creates the `UDP_RX` task, it will loop the ESP-IDF `recvfrom()` function with the `sock` handle to ask LwIP for the next UDP frame to process. LwIP will then check the socket's queue and return the next UDP frame if there is one. If the socket's queue is empty, LwIP will block the `UDP_RX` task, and the unblocking condition is LwIP adding a new UDP frame to the socket's queue. 

- Finally, `wifiInit()` creates two tasks called `UDP_RX` and `UDP_TX` using the FreeRTOS function `xTaskCreate()`.

- The `UDP_RX` task executs the `udp_server_rx_task()` function, which is defined in `wifi_esp32.c`.
- The `udp_server_rx_task()` function loops calling the ESP-IDF `recvfrom()` function to ask LwIP for the next UDP frame to process using the socket handle `sock`. If it's length is between 0B and 32B inclusive, it computes a checksum for the message using `calculate_cksum()` and compares it to the one in the UDP packet, adding the data frame to the `udpDataRx` queue with `xQueueSend()` if the checksums match. If the socket's queue is empty, LwIP will block the `UDP_RX` task, and the unblocking condition is LwIP adding a new UDP frame to the socket's queue. 

- The `UDP_TX` task executs the `udp_server_tx_task()` function, which is defined in `wifi_esp32.c`.
- The `udp_server_tx_task()` function loops calling the FreeRTOS `xQueueReceive()` function to read the next UDP frame the app code wants to send from the `udpDataTx` queue. If the queue is empty, this will cause the `UDP_TX` task to block until a UDP frame is added to the `udpDataTx` queue. Once a UDP fame is ready, it will compute a checksum using `calculate_cksum()` and append it to the frame. It then uses the ESP-IDF `sendto()` function along with the `sock` handle to hand the UDP frame to LwIP, which will send the UDP frame to the last IP address that transmitted to `port 2390` via `esp_netif` and `esp_wifi`.  

#### Wifi Link Initialisation

- `systemTask()` then calls `systemInit()`, also defined in `system.c`. 
- The first initialisation function in `systemInit()` is `wifiLinkInit()` (defined in `wifilink.c`), which creates the `crtpPacketDelivery` queue and creates the `WIFILINK` task (with `PRIORITY=2`) that runs the `wifilinkTask()` function. 
- The `wifilinkTask()` function bridges the `udpDataRx` queue with the rest of the application logic by forwarding complete CRTP messages from the `udpDataRx` queue to the `crtpPacketDelivery` queue, reformatting any ESP-NOW messages in the `udpDataRx` queue into full CRTP messages before forwarding them. 
- The `wifilinkTask()` function retrieves messages from the `udpDataRx` queue with `wifiGetDataBlocking()`, which is defined in `wifi_esp32.c` and uses `xQueueReceive()` to block the `WIFILINK` task until a message can be received from the queue. 
- Note that `commInit()` in `comm.c` uses `crtpSetLink(wifilinkGetLink())` to point the CRTP link struct to the `wifilink.c` functions, enabling the `CRTP-TX` and `CRTP-RX` tasks to send and receive CRTP packets over UDP via the `wifilink.c` functions.

#### Task Dump Initialisation

- `systemInit()` then calls `sysLoadInit()` (defined in `sysload.c`), which creates and starts the 5-second auto-reloading `sysLoadMonitorTimer` FreeRTOS software timer. 
- Every 5 seconds, the `sysLoadMonitorTimer` will call the `timerHandler()` callback function, which logs the task dump in the terminal if `triggerDump=1` and there are less than 32 tasks (which is true initially, since `triggerDump` is preset and then `timerHandler()` resets `triggerDump=0` after completing the first task dump after startup).

#### Debug Initialisation
- `systemInit()` then calls `debugInit()`, which currently does nothing because `DEBUG_PRINT_ON_SEGGER_RTT` is not defined.

#### CRTP Initialisation
- `systemInit()` then calls `crtpInit()` (defined in `crtp.c`), which creates the `txQueue` queue and the `CRTP-TX` and `CRTP-RX` tasks (with `PRIORITY=2`) that run the `crtpTxTask()` and `crtpRxTask()` functions respectively. 
- Note that `commInit()` in `comm.c` uses `crtpSetLink(wifilinkGetLink())` to point the CRTP link struct to the `wifilink.c` functions, enabling the `CRTP-TX` and `CRTP-RX` tasks to send and receive CRTP packets over UDP via the `wifilink.c` functions 
- `crtpTxTask()` is defined in `crtp.c` and essentially uses function pointers to run `wifilinkSendPacket()` (defined in `wifilink.c`), which makes sure the CRTP packet is not too long, blinks the blue LED, and then uses `wifiSendData()` (defined in `wifi_esp32.c`) to add the packet to the `udpDataTx` queue with `xQueueSend()`. This runs in a loop until `txQueue` is empty, after which the `CRTP-TX` task blocks until another packet is added to `txQueue`. 
- `crtpRxTask()` is defined in `crtp.` and essentially uses function pointers to run `wifilinkReceiveCRTPPacket()` (defined in `wifilink.c`), which receives a CRTP packet from the `crtpPacketDelivery` queue with `xQueueReceive()` (blocking for up to 100ms if the queue is empty) and blinks the green LED. The `crtpRxTask()` function then adds the CRTP packet to the queue specified within the CRTP packet (see below for list of `CRTPPort` options), and then calls the callback function associated with that port (if one has been registered).
  ```
  typedef enum {
    CRTP_PORT_CONSOLE          = 0x00,
    CRTP_PORT_PARAM            = 0x02,
    CRTP_PORT_SETPOINT         = 0x03,
    CRTP_PORT_MEM              = 0x04,
    CRTP_PORT_LOG              = 0x05,
    CRTP_PORT_LOCALIZATION     = 0x06,
    CRTP_PORT_SETPOINT_GENERIC = 0x07,
    CRTP_PORT_SETPOINT_HL      = 0x08,
    CRTP_PORT_PLATFORM         = 0x0D,
    CRTP_PORT_LINK             = 0x0F,
  } CRTPPort;
  ```
- Note that `crtp.c` contains a function called `crtpInitTaskQueue()`, which can be called by other tasks to create a queue that is attached to one of the `CRTPPort` types, which will then be registered in the `queues[]` array that is private to `crtp.c`. Then if the `CRTP-RX` task receives a packet addressed to that port, it will be put into that task's queue. 

#### Console Initialisation
- `systemInit()` then calls `consoleInit()` (defined in `console.c`), which initialises an empty CRTP packet configured to be transmitted to a `CRTP_PORT_CONSOLE` port, creates a binary semaphore for sending one message at a time, and initialises the `messageSendingIsPending` flag as `false`. Effectively, this process allows the drone to send CRTP packets to the ground station's console.

#### Configuration Initialisation
- `systemInit()` then calls `configblockInit()`, which is made for reading a radio communication configuration from EEPROM over I2C, but since neither ESP-Drone or Dell's Angel has an external EEPROM, it will load the following `configblockDefault` configuration into RAM:
  ```
  static configblock_t configblockDefault =
  {
      .magic = MAGIC,  // 0x43427830
      .version = VERSION,  // 1
      .radioChannel = RADIO_CHANNEL,  // 80
      .radioSpeed = RADIO_DATARATE,  // 2
      .calibPitch = 0.0,
      .calibRoll = 0.0,
      .radioAddress_upper = ((uint64_t)RADIO_ADDRESS >> 32),  // 0xE7E7E7E7E7ULL
      .radioAddress_lower = (RADIO_ADDRESS & 0xFFFFFFFFULL),
  };
  ``` 
- However, since the CrazyRadio is not being used, this is probably redundant.

#### Worker Queue Initialisation
- `systemInit()` then calls `workerInit()` (defined in `worker.c`), which just creates the `workerQueue` FreeRTOS queue. 

#### ADC Initialisation
- `systemInit()` then calls `adcInit()` (defined in `adc_esp32.c`), which sets the ADC1 bit resolution to 12 bits (fixed default for ESP32-S3 anyway), sets the ADC1 attenuation factor to `attn=3` or 11 dB so that the ADC can measure pin voltages over the full 0-3.3V range, and then attempts to calibrate ADC1 with that particular attenuation factor to correct for ADC inaccuracies due to process variations.

#### LED Sequence Initialisation
- `systemInit()` then calls `ledseqInit()` (defined in `ledseq.c`), which redundantly calls `ledInit()` again, registers all of the predefined system LED sequences with a priority implied by their registration order, initialises an array of state flags as all `0` (one for each sequence), creates an array of one-shot FreeRTOS software timers (one for each sequence), creates the `ledseqMutex` mutex for changing LED sequences, creates the `ledseqCmdQueue` queue for processing LED sequences, and then finally creates the `LEDSEQCMD` task (with `PRIORITY=1`), which runs the `lesdeqCmdTask()` function.
- The `lesdeqCmdTask()` function is defined in `ledseq.c` and it runs in a loop, blocking until an LED sequence command is put in `ledseqCmdQueue` via `ledseqRun()` or `ledseqStop()`. When either a run or stop command is received from the queue, it immediately starts that sequence, which is then handled asynchronously by the previously initialised software timers. 

#### Power Management Initialisation
- `systemInit()` then calls `pmInit()` (defined in `pm_esplane.c`), which configures battery voltage measurement to occur with a 2x multiplier (to correct for the voltage divider used on the PCB), initialises both the min and max battery voltages as 3.7V, and creates the `PWRMGNT` task (with `PRIORITY=1`) that runs the `pmTask()` function. 
- The `pmTask()` function is defined in `pm_esplane.c` and it runs a 100 ms loop that samples VBAT, tracks how long VBAT has been below low/critical thresholds, decides charging/charged/normal/low-power using charger flags and those timers, then drives LEDs/sounds, toggles flight permission, and can auto-shutdown on sustained critical voltage or long inactivity (although the auto-shutdown has not been implemented).

#### Buzzer Initialisation
- Finally, `systemInit()` calls `buzzerInit()` (defined in `buzzer.c`), which calls `buzzDeckInit()` (defined in `buzzdeck.c`) since `CONFIG_BUZZER_ON=1` in `sdkconfig.h` for some reason (potentially redundant since no buzz deck is being used).
- `buzzDeckInit()` calls `piezzoInit()` (defined in `piezzo.c`), which configures a timer with the ESP32's LED Control peripheral to set the tone of the buzzer, configures a channel with the LED Control peripheral to be able to drive the buzzer pin with the timer, and exposes a control interface so that higher level code can play sounds or patterns.

#### Communications Initialisation
- `systemTask()` then moves on from `systemInit()` to call `commInit()` (defined in `comm.c`), which first calls `crtpSetLink(wifilinkGetLink())` to retrieve the wifi-CRTP function pointers defined in `wifilink.c` and then set the CRTP link struct to point to those `wifilink.c` functions, enabling the `CRTP-TX` and `CRTP-RX` tasks to send and receive CRTP packets over UDP via the `wifilink.c` functions. Here, `crtpSetLink()` is defined in `crtp.c`, whilst `wifilinkGetLink()` is defined in `wifilink.c`.
- Then `commInit()` calls `crtpserviceInit()` (defined in `crtpservice.c`), which uses `crtpRegisterPortCB()` (defined in `crtp.c`) to register the `crtpserviceHandler()` callback function (defined in `crtpservice.c`) with the `CRTP_PORT_LINK` port in the `callbacks[]` array that is defined in `crtp.c`. This allows `crtpserviceHandler()` to automatically process CRTP packets that are addressed to the `CRTP_PORT_LINK` port and get delivered to that port's queue by the `CRTP-RX` task.
- `crtpserviceHandler()` processes all CRTP packets delivered to the `CRTP_PORT_LINK` port according to which channel is specified in the CRTP packet: packets addressed to the `linkEcho` channel get immediately retransmitted back to the sender with `crtpSendPacket()` (defined in `crtp.c`) for loopback testing, packets addressed to the `linkSink` channel get ignored, and packets addressed to the `linkSource` channel trigger an immediate reply to the sender with a maximum-length CRTP packet that contains the message `Bitcraze Crazyflie` followed by zeros for device identification. 
- Then `commInit()` calls `platformserviceInit()` (defined in `platformservice.c`), which first calls `appchannelInit()` (defined in `app_channel.c`) to create both the `sendMutex` mutex and `rxQueue` queue (of length 10) and initialise the `overflow` flag as `false`. `platformserviceInit()` then calls `crtpRegisterPortCB()` (defined in `crtp.c`) to register the `platformserviceHandler()` callback function (defined in `platformservice.c`) with the `CRTP_PORT_PLATFORM` port in the `callbacks[]` array that is defined in `crtp.c`. This allows `platformserviceHandler()` to automatically process CRTP packets that are addressed to the `CRTP_PORT_PLATFORM` port and get delivered to that port's queue by the `CRTP-RX` task. 
- `platformserviceHandler()` processes all CRTP packets delivered to the `CRTP_PORT_PLATFORM` port according to which channel is specified in the CRTP packet: packets addressed to the `platformCommand` channel are currently just echoed back to the sender (implementation incomplete), packets addressed to the `versionCommand` channel trigger an immediate reply to the sender with either the protocol version (`4`), firmware version (`{tag}`?), or platform name (`ESPlane 2.0`) depending on what information was requested in the packet's data field (`getProtocolVersion`, `getFirmwareVersion`, or `getDeviceTypeName`). 
- Then `commInit()` calls `logInit()` (defined in `log.c`), which scans/validates the compiled-in log table, computes a CRC for versioning, sets up locks and runtime slots, clears any old state, and starts the `LOG` task (with `PRIORITY=2`) that runs the `logTask()` function.
- The `logTask()` function is defined in `log.c` and it creates the CRTP queue for the `CRTP_PORT_LOG` port, registers it with the `queues[]` array defined in `crtp.c`, and then receives packets from its queue, blocking until a message addressed to `CRTP_PORT_LOG` is delivered to the queue by the `CRTP-RX` task. When a CRTP log packet is received, `logTask()` will process it according to which channel is specified in the CRTP packet: packets addressed to the `TOC_CH` channel trigger the `logTOCProcess()` function to be called, whereas packets addressed to the `CONTROL_CH` channel trigger the `logControlProcess()` function to be be called. The `logTOCProcess()` function triggers an immediate reply to the sender with either information about the log implementation or information about a particular logging variable depending on what information was requested in the packet's data field. The `logControlProcess()` function allows the ground station to group logging variables into blocks and start/stop sampling them at a specified rate for communication to the ground station via CRTP.
- Then `commInit()` finally calls `paramInit()` (defined in `param.c`), which discovers the compiled-in parameter table (via linker symbols), computes a CRC and sanity-checks names, counts the non-group params, then starts the `PARAM` task (with `PRIORITY=2`) that runs the `paramTask()` function (similar to `logInit()`). 
- The `paramTask()` function is defined in `param.c` and it creates the CRTP queue for the `CRTP_PORT_PARAM` port, registers it with the `queues[]` array defined in `crtp.c`, and then receives packets from its queue, blocking until a message addressed to `CRTP_PORT_PARAM` is delivered to the queue by the `CRTP-RX` task. When a CRTP log packet is received, `paramTask()` will process it according to which channnel is specified in the CRTP packet: packets addressed to the `TOC_CH` channel trigger an immediate reply to the sender (using `paramTOCProcess()`) with information about either the param implementation or informaiton about a particular param variable depending on which is requested in the packet's data field, packets addressed to the `READ_CH` channel trigger an immediate reply to the sender (using `paramReadProcess()`) with the value of a particular parameter, packets addressed to the `WRITE_CH` channel trigger an immediate reply to the sender (using `paramWriteProcess()`) echoing back the request after updating the param value in RAM, and packets addressed to the `MISC_CH` channel are for changing a parameter identified by group and name rather than by ID (using `paramWriteByNameProcess()`), triggering an immediate echo of the CRTP packet to the sender with the type field being overwritten by the error status returned by `paramWriteByNameProcess()` (note the param must be writeable and must match name, group and data type). 

#### Commander Initialisation
- `systemTask()` then calls `commanderInit()` (defined in `commander.c`), which creates the `setpointQueue` queue, sends the `nullSetpoint` setpoint to the `setpointQueue`, creates the `priorityQueue` queue, and sends the `priorityDisable` (which is set to `COMMANDER_PRIORITY_DISABLE=0`) to the `priorityQueue` queue. 
- Then `commanderInit()` calls `crtpCommanderInit()` (defined in `crtp_commander.c`), which redundantly calls `crtpInit()` again and then uses `crtpRegisterPortCB()` (defined in `crtp.c`) to register the `commanderCrtpCB()` callback function (defined in `crtp_commander.c`) with both the `CRTP_PORT_SETPOINT` and `CRTP_PORT_SETPOINT_GENERIC` ports in the `callbacks[]` array (defined in `crtp.c`).
- The `commanderCrtpCB()` callback function is called whenever the `CRTP-RX` task adds a CRTP packet to the queues associated with either the `CRTP_PORT_SETPOINT` or `CRTP_PORT_SETPOINT_GENERIC` ports, and it first differentiates which port and channel is being targeted by reading the CRTP packet's address. 
- If the CRTP packet was addressed to channel 0 of the `CRTP_PORT_SETPOINT` port, then `crtpCommanderRpytDecodeSetpoint()` (defined in `crtp_commander_rpyt.c`) is called to parse the CRTP data (currently in the legacy `rpyt` format) into the `setpoint` variable (of type `setpoint_t`) that the stabilizer can then use. In this case, the CRTP packet's data field consists of a `float` for `roll`, `pitch`, and `yaw`, along with a `uint16_t` for `thrust`. The system initialises with `thrustLocked=true`, which keeps the setpoint thrust at zero until a CRTP packet with `thrust=0` is received (after which `thrustLocked=false` and the `thrust` specified in the CRTP packet becomes the updated setpoint thrust after potential clipping to min/max bounds). Then the currently enabled flight modes (`altHoldMode`, `posHoldMode`, `posSetMode`) are used to conditionally parse the `roll`, `pitch`, and `yaw` values into `setpoint` and set the stabilizer `mode` (`modeDisable`, `modeAbs`, `modeVelocity`) for each stabilizer state variable (`x`, `y`, `z`, `roll`, `pitch`, `yaw`, `quat`).
- If `altHoldMode=true`, then the setpoint's `thrust` is set to zero, `mode.z` is set to `modeVelocity`, and `velocity.z` is set to a value between -1 and +1 depending on the value of the CRTP packet's `thrust`. Hence, if the CRTP packet's `thrust` is at the midpoint of its range, the setpoint for the stabilizer will be to maintain whatever the current altitude is by keeping the vertical velocity zero. By temporarily increasing/decreasing the CRTP packet's `thrust` from its midpoint, the stabilizer will try achieve a 
positive/negative vertical velocity, allowing the user to change the current altitude before returning `thrust` to its midpoint and holding the new altitude. In other words, the setpoint becomes to hold the current `z` position, which can be changed with the `thrust` control. If `altHoldMode=false`, then the setpoint's `mode.z` is set to `modeDisable`. 
- If `posHoldMode=true`, then a similar parsing occurs where the setpoint's `mode.x` and `mode.y` are set to `modeVelocity`, `mode.roll` and `mode.pitch` are set to `modeDisable`, `attitude.roll` and `attitude.pitch` are set to zero, `velocity_body` is set to `true` (so that velocity setpoints are interpreted with reference to the drone's frame of reference), and `velocity.x` and `velocity.y` are set to scaled versions of the `roll` and `pitch` values in the CRTP packet respectively. Hence, similarly to if `altHoldMode=true`, the setpoint becomes to hold the current `x` and `y` positions, which can be changed with the `pitch` and `roll` controls respectively.
- If `posSetMode=true` and the CRTP packet's `thrust` is non-zero, then a similar parsing occurs where `x`, `y`, `z`, and `yaw` are specified in absolute terms by the CRTP packet's `pitch`, `roll`, `thrust` and `yaw` values, so the CRTP packet is repurposed as a pose command. 
- If all three flight modes are disabled, then the CRTP packet's values are directly parsed into the setpoint's `roll`, `pitch` and `yaw` values either as rates or angles depending on the stabilization mode. The default initialisation is that the setpoint's `roll` and `pitch` values are interpreted as angles, whereas the `yaw` value is interpreted as a rate. This represents the setpoints used in the default manual flight mode. 
- Once this setpoint has been determined, `commanderCrtpCB()` calls `commanderSetSetpoint()` (defined in `commander.c`) to apply a timestamp to the setpoint, overwrite the current setpoint in the `setpointQueue` queue, and overwrite the current priority in the `priorityQueue` queue with `COMMANDER_PRIORITY_CRTP`. `commanderSetSetpoint()` also calls `crtpCommanderHighLevelStop()`, which ensures that the high-level commander is set to an idle status and any following of a trajectory is aborted so that the newly requested low-level setpoint can be implemented.
- If the CRTP packet was addressed to the `SET_SETPOINT_CHANNEL` channel of the `CRTP_PORT_SETPOINT_GENERIC` port, then `commanderCrtpCB()` calls `crtpCommanderGenericDecodeSetpoint()` (defined in `crtp_commander_generic.c`) to parse the data in the CRTP packet into `setpoint`. 
- `crtpCommanderGenericDecodeSetpoint()` uses a sort of switch case where it reads the CRTP packet's type to see what kind of setpoint is being requested, then it wipes the current setpoint variable by resetting it to all zeros, and then it calls the decoder function for that setpoint type to parse the subsequent data in the CRTP packet into `setpoint`. The `packetDecoders[]` array that maps the setpoint types to their decoder functions is shown below:

  ```
  const static packetDecoder_t packetDecoders[] = {
    [stopType]          = stopDecoder,
    [velocityWorldType] = velocityDecoder,
    [zDistanceType]     = zDistanceDecoder,
    [cppmEmuType]       = cppmEmuDecoder,
    [altHoldType]       = altHoldDecoder,
    [hoverType]         = hoverDecoder,
    [fullStateType]     = fullStateDecoder,
    [positionType]      = positionDecoder,
  };
  ```
- `stopDecoder()` just leaves `setpoint` as all zeros, causing the motors to stop and the drone to fall to the ground. 
- `velocityDecoder()` sets the x/y/z velocities and the yaw rate.
- `zDistanceDecoder()` sets the absolute z value, the roll and pitch angles, and the yaw rate.
- `cppmEmuDecoder()` allows you to emulate a radio receiving combined pulse position modulated (CPPM) signals for roll/pitch/yaw/thrust. With PPM, each of a controller's stick values (like the roll stick) is mapped to a value between 1000 and 2000, with the neutral midpoint being 1500. Then these can be combined or "time multiplexed" train of fixed width pulses, where the pulses separated by a number of microseconds equal to the stick values. On the receiver end, these pulse trains would be decoded back into a set of stick values between 1000 and 2000. The decoder function parses such values for roll/pitch.yaw/thrust into `setpoint` by normalising them.
- `altHoldDecoder()` sets the vertical velocity (set to zero for holding current altitutde), as well as the roll and pitch.
- `hoverDecoder()` sets the absolute height, x/y velocities in the drone's frame of reference, and the yaw rate.
- `fullStateDecoder()` sets every element of `setpoint` to specify an entire state.
- `positionDeocder()` sets the absolute x/y/z positions and yaw angle.
- Again, once this setpoint has been determined, `commanderCrtpCB()` calls `commanderSetSetpoint()` (defined in `commander.c`) to apply a timestamp to the setpoint, overwrite the current setpoint in the `setpointQueue` queue, and overwrite the current priority in the `priorityQueue` queue with `COMMANDER_PRIORITY_CRTP`. `commanderSetSetpoint()` also calls `crtpCommanderHighLevelStop()`, which ensures that the high-level commander is set to an idle status and any following of a trajectory is aborted so that the newly requested low-level setpoint can be implemented.
- If the CRTP packet was addressed to the `META_COMMAND_CHANNEL` channel of the `CRTP_PORT_SETPOINT_GENERIC` port, then `commanderCrtpCB()` uses a similar sort of switch case where it reads the CRTP packet's type to see what kind of metacommand is being requested and then uses a decoder function to implement that metacommand. Currently there is only one type of metacommand that can be requested via CRTP (the `metaNotifySetpointsStop` type), which executes the `notifySetpointsStopDecoder()` function. This function modifies the current setpoint's timestamp to extend its validity by the number of milliseconds specified in the CRTP packet's data, and this function is used if the ground station wants to prevent the high-level commander from timing out before it's ready to send another setpoint. 

- Then `commanderInit()` calls `crtpCommanderHighLevelInit()` (defined in `crtp_commander_high_level.c`), which first registers the `memDef` memory handler (a collection of pointers to memory management functions) with the `handlers[]` array defined in `mem.c`. 
- `crtpCommanderHighLevelInit()` then calls `plan_init()` (defined in `planner.c`), which initialises the previously declared `planner` as configured for a one-piece piecewise trajectory, although with no current trajectory and an idle status.
- `crtpCommanderHighLevelInit()` then creates the `CMDHL` task (with `PRIORITY=3`), which runs the `crtpCommanderHighLevelTask()` function (also defined in `crtp_commander_high_level.c`).
- After this, `crtpCommanderHighLevelInit()` creates the `lockTraj` mutex to prevent trajectories from being modified or updated whilst they are being executed, and then initialises the float for `yaw` and vectors for `pos` and `vel` as all zeros (these are used by `crtp_commander_high_level.c` to remember the last setpoint for creating new setpoints from high-level commands).
- `crtpCommanderHighLevelTask()` first creates a CRTP queue for the `CRTP_PORT_SETPOINT_HL` port and then reads from the queue in a loop, blocking until the `CRTP-RX` receives a CRTP packet addressed for that port and adds it to the queue. When a CRTP packet is received, it reads the packet to see which high-level command is being requested, and then calls the appropriate function with a switch case to plan the execution of the requested command with the requested parameters, replying to the sender of the CRTP packet with an echo that also contains the return value of the planning. The available high-level commands are shown below:

  ```
  enum TrajectoryCommand_e {
    COMMAND_SET_GROUP_MASK          = 0,
    COMMAND_TAKEOFF                 = 1, // Deprecated, use COMMAND_TAKEOFF_2
    COMMAND_LAND                    = 2, // Deprecated, use COMMAND_LAND_2
    COMMAND_STOP                    = 3,
    COMMAND_GO_TO                   = 4,
    COMMAND_START_TRAJECTORY        = 5,
    COMMAND_DEFINE_TRAJECTORY       = 6,
    COMMAND_TAKEOFF_2               = 7,
    COMMAND_LAND_2                  = 8,
    COMMAND_TAKEOFF_WITH_VELOCITY   = 9,
    COMMAND_LAND_WITH_VELOCITY      = 10,
  };
  ```
- Note that this is the general structure of a setpoint:

  ```
  typedef struct setpoint_s {
    uint32_t timestamp;

    attitude_t attitude;      // deg
    attitude_t attitudeRate;  // deg/s
    quaternion_t attitudeQuaternion;
    float thrust;
    point_t position;         // m
    velocity_t velocity;      // m/s
    acc_t acceleration;       // m/s^2
    bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

    struct {
      stab_mode_t x;
      stab_mode_t y;
      stab_mode_t z;
      stab_mode_t roll;
      stab_mode_t pitch;
      stab_mode_t yaw;
      stab_mode_t quat;
    } mode;
  } setpoint_t;
  ```


#### Kalman Filter Initialisation
- Note that these are the possible state estimator types (i.e., complementary or kalman):
  ```
  typedef enum {
    anyEstimator = 0,
    complementaryEstimator,
    kalmanEstimator,
    StateEstimatorTypeCount,
  } StateEstimatorType;
  ```
- `systemTask()` then initialises the state estimator type as `estimator = anyEstimator` and calls `estimatorKalmanTaskInit()` (defined in `estimator_kalman.c`), which initialises the data queues shown below, creates the `runTaskSemaphore` semaphore (taken/given at the start/end of each loop of `kalmanTask()`), creates the `dataMutex` mutex (used for brief operations when copying sensor data or updating state estimations to maintain atomicity between the kalman estimator and the stabilizer), and starts the `KALMAN` task (with `PRIORITY=4`), which runs the `kalmanTask()` function (also defined in `estimator_kalman.c`).
  ```
  distDataQueue  // Distance-to-point measurements
  posDataQueue  // Direct measurements of Crazyflie position
  poseDataQueue  // Direct measurements of Crazyflie pose
  tdoaDataQueue  // Measurements of a UWB Tx/Rx
  flowDataQueue  // Measurements of flow (dnx, dny)
  tofDataQueue  // Measurements of TOF from laser sensor
  heightDataQueue  // Absolute height along room's Z
  yawErrorDataQueue  // Yaw error?
  ```
- Before entering the main while loop, the `kalmanTask()` function first calls `systemWaitStart()`, which causes the `KALMAN` task to wait until `systemInit()` has completed and `systemStart()` (both defined in `system.c`) has been called, initialises four time stamps with the current time, and calls `rateSupervisorInit()` (defined in `rateSupervisor.c`) to initialise the rate supervisor context. 
- In the main while loop, the `runTaskSemaphore` semaphore is taken, the kalman estimator is reset if the `resetEstimation` parameter has been set to `true` by the user, the `doneUpdate` flag is initialised as `false`, and the current time is saved in `osTick` (measured in milliseconds). 
- Before running the system dynamics to predict the state forward, the `kalmanTask()` checks if the `osTick` has reached `nextPrediction`. To predict the state forward, the `predictStateForward()` function (also defined in `estimator_kalman.c`) is called with the current time `osTick` and the time interval since the last prediction `dt=osTick-lastPrediction`, setting `lastPrediction-osTick` and `doneUpdate=true` upon successfully returning.
- The `predictStateForward()` function first checks if `gyroAccumulatorCount`, `accAccumulatorCount`, or `thrustAccumulatorCount` are equal to zero (i.e., if any of those state variables haven't been sampled once), returning `false` if there is insufficient data to predict the state forward. 
- It should be noted that the only way these variables get incremented is after `estimatorKalman()` (defined in `estimator_kalman.c`) is called, which can only be called by `estimatorFunctions[currentEstimator].update()` (defined in `estimator.c`) if `currentEstimator=kalmanEstimator`. The call `estimatorFunctions[currentEstimator].update()` can only be made by `stateEstimator()` (defined in `estimator.c`), and that is only called by the `stabilizerTask()` main loop, which is triggered every time the SENSORS task responds to the IMU ISR by reading and processing the IMU data, which occurs whenever the IMU raises its INT pin. 
- As for setting `currentEstimator`, it is initialised as `anyEstimator` in `estimator.c` (and `estimator` is initialised as `anyEstimator` in `system.c` too). The only way `currentEstimator` can be updated is by calling `stateEstimatorSwitchTo()` (defined in `estimator.c`), which can update the value of a `StateEstimatorType` variable (such as `currentEstimator`) to the value passed to `stateEstimatorSwitchTo()` after ensuring that the requested estimator has been initialised. Note that if `stateEstimatorSwitchTo()` is passed a `StateEstimatorType` variable with the "default" value of `anyEstimator`, then it will switch its value to `complementaryEstimator`. Also note that `stateEstimatorSwitchTo()` will override the requested estimator type with `forcedEstimator` if `forcedEstimator` is something other than `anyEstimator`, although `forcedEstimator` is set by the `ESTIMATOR_NAME` macro, which is currently set to `anyEstimator`. The only time `stateEstimatorSwitchTo()` is explicitly called is in the main loop of the `stabilizerTask()` function in `stabilizer.c`, where it checks if its `StateEstimatorType` variable `estimatorType` is the same as `currentEstimator` in `estimator.c` by calling `getStateEstimator()` (defined in `estimator.c`), which just returns the value of `currentEstimator`. If there is a mismatch, then the main loop of `stabilizerTask()` calls `stateEstimatorSwitchTo()` to update `currentEstimator` to match `estimatorType`. `estimatorType` is zero initialised by default, and so its value starts as `anyEstimator`, but it can be changed in two ways. It is registered as a parameter and so one way it can be changed is by the user via CRTP. The other way is when `stabilizerInit()` is called, which only happens once during startup when `systemTask()` calls `stabilizerInit()` with the `estimator` variable that was initialised as `anyEstimator`. When this happens, the value of `estimator` inside the function gets updated with the result of `deckGetRequiredEstimator()` (defined in `estimator.c`), which just returns the value of `requiredEstimator`, before calling `stateEstimatorInit()` with the updated `estimator` (and all `stateEstimatorInit()` does is call `stateEstimatorSwitchTo()` with the estimator type it's been passed - this is the only time `stateEstimatorSwitchTo()` is called indirectly). The value of `requiredEstimator` is also initialised as `anyEstimator`, and the only way that `requiredEstimator` can be updated is by calling `registerRequiredEstimator()` (defined in `estimator.c`) with the requested estimator type. Note that `registerRequiredEstimator()` will only override  `requiredEstimator` with the requested estimator if `requiredEstimator` is currently set to `anyEstimator`. The only way `registerRequiredEstimator()` is called is by `setCommandermode()` (defined in `crtp_commander_rpyt.c`), which is only called once during `sensorsMpu6050Hmc5883lMs5611Init() -> sensorsDeviceInit()` if `flowdeck2Test()` passes (i.e., if the flow deck is connected and initialises successfully). The `sensorsMpu6050Hmc5883lMs5611Init()` function is only called once when `sensorsInit()` identifies that the active sensor implementation is set to `SENSOR_INCLUDED_MPU6050_HMC5883L_MS5611` using `platformConfigGetSensorImplementation()`, finds the group of functions associated with that sensor implementation using `findImplementation()`, sets the `activeImplementation` struct of function pointes to those functions, and then calls `activeImplementation->init()`. 
- Hence, if the flow deck is connected and initialises properly, then `currentEstimator` will get updated to `kalmanEstimator`, resulting in the kalman filter's accumulator and count variables being used each time the stabilizer loop calls `estimatorFunctions[currentEstimator].update()` in response to the sensors loop having completed reading and processing sensor data (which happens whenever the MPU6050 raises its INT pin with new data, triggering an ISR that unblocks the sensors loop), and so `predictStateForward()` will pass its check for sufficient data.
- Then `predictStateForward()` takes the `dataMutex` mutex and computes average accel, gyro, and thrust values over the `dt` period by dividing the accumulated measurements during that interval by the number of measurements (and also converting units). Then the accumulator variables and count variables are reset to zero before returning the `dataMutex` mutex.
- If the newly computed average thrust value is above `IN_FLIGHT_THRUST_THRESHOLD`, the drone decides it is still flying and calls `kalmanCorePredict()`, which updates the value of the kalman filter's state using a linearised state-transition Jacobian and rigid body kinematics to advance the mean state and associated uncertainty. 
- Once this prediction step is completed, the `nextPrediction` timestamp is set to 10ms after the current `osTick` time, the rate supervisor is called with `rateSupervisorValidate()` (which increments a counter and evaluates the frequency of kalman estimator loop if the current evaluation period has elapsed), and then `kalmanCoreAddProcessNoise()` is called with a `dt` equal to the time since the last process noise update. This adds noise to the state uncertainty to reflect how uncertainty in the state increases as the state evolves.
- Then `kalmanTask()` takes the `dataMutex` mutex, copies the gyro data from `gyroSnapshot` into a local `gyro` variable, returns `dataMutex`, and calls `updateQueuedMeasurements()` (defined in `estimator_kalman.c`) with the copied `gyro` data. 
- The `updateQueuedMeasurements()` function sequentially processes each data queue listed above until empty by running a while loop conditioned on `stateEstimatorHas{DataType}Packet()` (which receives a measurement from the data queue into a local variable - all defined in `estimator_kalman.c`), running `kalmanCoreUpdateWith{DataType}()` within the loop (which adjusts/corrects the predicted kalman state using measurement models and the measured data - all defined in `kalman_core.c`).
- Once all of the sensor data has been consumed, `kalmanTask()` calls `kalmanCoreFinalize()` (defined in `kalman_corec.`) to fold the attitude error it computed whilst updating into the state's attitude quaternion, which also necessitates transforming the model's rotation matrix and state uncertainty. Doing this regularly allows the computed attitude errors to remain small, ensuring the validity of a linearised model around the current attitude.
- Then `kalmanTask()` ensures the state is sensible (not more than 100m away and flying slower than 10m/s) before taking the `dataMutex` mutex, calling `kalmanCoreExternalizeState()` (defined in `kalman_core`), and returning `dataMutex`.
- The `kalmanCoreExternalizeState()` function translates the kalman filter's state variable representations into the same `state_t` type used by the rest of the firmware, which includes transforming the accelerations and velocities to the world frame, using "gravity-free" acceleration, and converting units. This translated state is stored into the `taskEstimatorState` state variable used within `estimator_kalman.c`, which gets copied into the `state` variable each time the main loop in `stabilizerTask()` calls `stateEstimator()` (which calls `estimatorKalman()` via function pointers). 
- Hence, each time the `STABILIZER` task gets IMU data from the `SENSORS` task and calls on `estimatorKalman()` to accumulate it, it gets an updated `state` that includes any corrections from the drone's other sensors and a potentially evolved state if the `KALMAN` task reached its update time since the last call of `estimatorKalman()`. 

#### Stabiliser Initialisation
- `systemTask()` then calls `stabilizerInit(estimator)`(defined in `stabilizer.c`), which

#### Sound Initialisation
- `systemTask()` then calls `soundInit()` (defined in `sound_cf2.c`), which

#### Memory Initialisation
- `systemTask()` then calls `memInit()` (defined in `mem.c`), which

#### Pre-Start Checks
- `systemTask()` then does a chain of checks to confirm that each initialisation completed successfully, mostly by checking if each module's `isInit` variable is `true`, although this also includes performing the one-by-one 150ms motor test pulse, enabling the LED sequences (just allows them to start), and checking if any previous errors had caused the system to restart to get to this point. 

#### Startup
- If all of the pre-start checks pass, then `systemTask()` calls `systemStart()` to give the `canStartMutex` mutex (which seemingy isn't used anywhere), logs that the system passed its pre-start checks and is starting, tells the sound subsystem to play a startup sound with `soundSetEffect()`, blinks the green LED 7 times to indicate the system passsed its pre-start checks and is starting (using `ledseqRun()`), and starts blinking the blue LED every 2 seconds as a heartbeat signal (also using `ledseqRun()`).
- If the `systemTest()` check passes but something else fails, then the drone rapidly blinks the blue LED in a loop and can still be forced to start by setting the `selftestPassed` param to `1` via CF Client.
- If the `systemTest()` fails, then the drone will set the blue LED to be on constantly. 
- Regardless of the startup test outcomes, the drone will then call `workerLoop()` (if the worker loop erroneously exits for some reason, the system will get stuck in an infinite while loop guard after the call of `workerLoop()`).



## ESP32-Pi UART Communication 

### What data is being sent?

- The ROS 2 Orb SLAM application running on the ground station requires IMU data fusion to improve the accuracy of its Orb localisaiton.
- Hence, the IMU data (along with the timestamp for when it was created) needs to be transmitted to the Raspberry Pi Zero 2 W over UART so that it can be timestamped with the same timebase as the Pi Camera video feed and transmitted over wifi to the ground station.
- Since the ESP32 does several stages of IMU data processing, there are three options for what data can be transmitted to the Pi over UART:
  1. The raw IMU data straight from the MPU6050 after being read by the `SENSORS` task.
  1. The IMU data after it has been processed by the `SENSORS` task.
  1. The Kalman-filtered IMU data, or state estimation of the current accel/gyro values.

#### Raw data
- This data is read by `sensorsTask()` --> `i2cdevReadReg8()` in `sensors_mpu6050_hm5883L_ms5611.c`.
- It is stored in `buffer`, which is currently defined as a static global variable in `sensors_mpu6050_hm5883L_ms5611.c`, meaning it exists for the full program duration but is only visible to the functions defined in `sensors_mpu6050_hm5883L_ms5611.c`. 
- The structure of `buffer` is a `uint8_t` array of length 28, although the IMU only uses the first 14 elements, with the remaining 14 elements being reserved in case a magnetometer and barometer are being read over I2C as well.
- This raw data is not immediately useful to ROS since it hasn't been axes-corrected, scale-corrected, bias-corrected, or filtered.

#### Processed data
- This data is created by `sensorsTask()` --> `processAccGyroMeasurements()` in `sensors_mpu6050_hm5883L_ms5611.c`.
- It processes the data in `buffer` and then updates the data in `sensorData.acc` and `sensorData.gyro` with the new values. 
- `sensorData` is currently defined as a static global variable in `sensors_mpu6050_hm5883L_ms5611.c`, meaning it exists for the full program duration but is only visible to the functions defined in `sensors_mpu6050_hm5883L_ms5611.c`. 
- Then `sensorsTask()` uses the new `sensorData` values to overwrite the contents of two queues with handles `accelerometerDataQueue` and `gyroDataQueue` by calling `xQueueOverwrite()`. 
- Given these handles, any FreeRTOS task (such as a UART task) can receive or peek these data queues to obtain the latest processed IMU data.

!!! I think the Kalman task consumes these Queue values by receiving from them, so a UART task might not be able to peek these queue values. Might need to read directly from sensorData by requesting a pointer. !!!

#### Kalman-filtered data


#### Current plan



### IMU Data

- `stabilizerInit()` in `stabilizer.c` creates the `STABILIZER` task (with `PRIORITY=7`) which runs the `stabilizerTask()` function. 
- `stabilizerTask()` in `stabilizer.c` has a main loop, and the first function call in the loop is `sensorsWaitDataReady()`, which blocks the task until data is ready, causing the loop to run at a frequency of 1kHz or 500Hz if the kalman filter is being used.
- `sensorsWaitDataReady()` in `sensors.c` points to `sensorsMpu6050Hmc5883lMs5611WaitDataReady()` in `sensors_mpu6050_hm5883L_ms5611.c`, which takes the `dataReady` semaphore, blocking until it becomes available. `sensorsTask()` also in `sensors_mpu6050_hm5883L_ms5611.c` is what gives the `dataReady` semaphore after it has read and processed the IMU data.
- `sensorsTaskInit()` in `sensors_mpu6050_hm5883L_ms5611.c` creates the `SENSORS` task (with `PRIORITY=6`), which calls the `sensorsTask()` function.
- `sensorsTask()` in `sensors_mpu6050_hm5883L_ms5611.c` does the following:
  1. Takes the `sensorsDataReady` semaphore when it becomes available (given by `sensors_inta_isr_handler()` when the MPU6050 raises the INT pin or GPIO12).
  1. Saves the `imuIntTimestamp` timestamp to `sensorData.interruptTimestamp`, measured in microseconds since startup (`uint64_t` created by `sensors_inta_isr_handler()` when the MPU6050 raises the INT pin or GPIO12).
  1. Reads the MPU6050's data buffer into a sensor data buffer (and magnetometer data, if enabled) with `i2cdevReadReg8()`.
  1. Processes the data with `processAccGyroMeasurements()` in `sensors_mpu6050_hm5883L_ms5611.c` by correcting for the MPU6050's mounting direction as it reads the buffer data into the `accelRaw` and `gyroRaw` structs, compensating the gyro measurements for any bias that has been detected by `processGyroBias()`, scaling the accel and gyro raw data based on the selected user-programmable scales, compensating the accel measurements with the misalignment-correcting trim values set by the user, and then low-pass filtering the accel and gyro data to suppress noise.
  1. Value-copies the processed accel and gyro data to `accelerometerDataQueue` and `gyroDataQueue` using `xQueueOverwrite()`, so the latest values are always available in the queue.
  1. Gives the `dataReady` semaphore to unblock the `STABILIZER` task. 

- `compressState()` in `stabilizer.c` flattens the `state` struct of type `state_t` into `stateCompressed` and also converts units.

- Before being "compressed", `state` looks like this:
  
  ```
  {
    "state": {
      "position": {
        "x": "32-bit float measured in m",
        "y": "32-bit float measured in m",
        "z": "32-bit float measured in m"
      },
      "velocity": {
        "x": "32-bit float measured in m s^-1",
        "y": "32-bit float measured in m s^-1",
        "z": "32-bit float measured in m s^-1"
      },
      "acc": {
        "x": "32-bit float measured in g",
        "y": "32-bit float measured in g",
        "z": "32-bit float measured in g-1 (gravity-free)"
      },
      "attitudeQuaternion": {
        "x": "32-bit float component of a normalised quaternion",
        "y": "32-bit float component of a normalised quaternion",
        "z": "32-bit float component of a normalised quaternion",
        "w": "32-bit float component of a normalised quaternion"
      },
      "gyro": {
        "x": "32-bit float measured in deg s^-2",
        "y": "32-bit float measured in deg s^-2",
        "z": "32-bit float measured in deg s^-2"
      }
    }
  }
  ```
- After being "compressed", `stateCompressed` looks like this: 
  ```
  {
    "stateCompressed": {
      "x": "int16_t measured in mm",
      "y": "int16_t measured in mm",
      "z": "int16_t measured in mm",
      "vx": "int16_t measured in mm s^-1", 
      "vy": "int16_t measured in mm s^-1",
      "vz": "int16_t measured in mm s^-1",
      "ax": "int16_t measured in mm s^-2",
      "ay": "int16_t measured in mm s^-2",
      "az": "int16_t measured in mm s^-2",
      "quat": "int32_T compressed quaternion",
      "rateRoll": "int16_t measured in millirad s^-2",
      "ratePitch": "int16_t measured in millirad s^-2",
      "rateYaw": "int16_t measured in millirad s^-2"
    }
  }
  ```
- The `quatcompress()` function in `quatcompress.h` is used by `compressState()` to (lossily) compress the 128-bit quaternion to a single 32-bit value by:
  1. Identifying the quaternion component with the largest magnitude and forcing it to be positive by inverting the quaternion's components (since this saves the need to transmit or store its sign bit - the receiver knows it is always positive).
  1. Discarding the largest quaternion (since it can be reconstructed from the others using the normalised length condition).
  1. Quantising each of the remaining three components to a 9-bit magnitude plus a sign bit (10 + 10 + 10 = 30 bits).
  1. Indicating which of the four quaternion components was removed due to being the largest (either x/y/z/w - requires 2 bits).
  1. Packaging the data into a single `int32_t` value.  


Data types:
- Raw IMU data is generated at 500 Hz by the IMU and read at 500 Hz by `sensorsTask()`, it is stored in `buffer`, defined as a static global in `sensors_mpu6050_hm5883L_ms5611.c`, overwritten after each IMU read when the IMU raises the INT pin. It is also copied into `accelRaw` and `gyroRaw` during processing, so extracting from the buffer may not be necessary.  
- Processed IMU data is generated at 500 Hz by `sensorsTask()`, stored in `sensorData`, defined as a static global in `sensors_mpu6050_hm5883L_ms5611.c`, overwritten after processing each IMU read when the IMU raises the INT pin. The data in `accelerometerDataQueue` and `gyroDataQueue` is overwritten with `sensorData.acc` and `sensorData.gyro` each time processing is completed. These are FreeRTOS queues with global accessibility if the queue handles are known. The data is received (consumed) from these queues when `sensorsMpu6050Hmc5883lMs5611ReadAcc()` and `sensorsMpu6050Hmc5883lMs5611ReadGyro()` are called, which happens when `estimatorKalman()` calls `sensorsReadAcc()` and `sensorsReadGyro()`, which happens when the stabilizer main loop calls `stateEstimator()`. This effectively copies the accel and gyro data from `sensorData` in `sensors_mpu6050_hm5883L_ms5611.c` to `sensorData` in `stabilizer.c` (also defined as static global). It is also copied into `accSnapshot` and `gyroSnapshot`, which are static globals in `estimator_kalman.c`.
- Kalman-filtered IMU data is generated into the `taskEstimatorState` state variable in `estimator_kalman.c` each time the kalman main loop processes sensor data or advances its state prediction, and this is copied to `state` periodically each time `estimatorKalman()` is called (500 Hz). The `state.acc` values are in the world-frame (gravity-free) and their values will be slightly impacted by the sensor fusion altering the attitude, which is used to transform body-frame accs to world-frame accs. The `state.attitude` values are directly impacted by fusion, but it is in deg, not deg/s. 

### How Does the ESP32-S3 Handle UART?

- It has three dedicated UART controllers (UART0, UART1, UART2).
- When a FreeRTOS task wants to transmit over UART, it will:
  1. Format its data into a flat string of bytes (uint8_t).
  1. Load them into a 128-byte UART TX FIFO buffer.
- The UART hardware controller will then handle the following: 
  1. The UART hardware controller will format each byte into a message frame.
  1. The default message frame configuration is `8N1`, meaning 8 data bits, no parity bits, 1 stop bit (so 10 bits total, including the compulsory start bit). 
  1. The UART hardware controller has a baud rate generator, which sets the bit/second transmission rate.
  1. Each message frame is transferred from the FIFO buffer into a shift register by the UART hardware controller.
  1. The shift register then sends out each bit in the message at the baud rate. 
- The UART hardware controller can be configured to raise an interrupt once the FIFO buffer has been cleared to a certain count (so the MCU can top it up), as well as when the FIFO buffer has been cleared.
- Since the FIFO buffer is 128 bytes long, configuring UART to use DMA does not really provide much benefit unless more than 128 bytes are being sent at a time.


- The ESP32-S3 will communicate to the Raspberry Pi Zero 2 W using the UART2 controller via pins IO38 (for the ESP32 to receive) and IO39 (for the ESP32 to transmit). 