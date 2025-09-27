# Modifications Register
Note: make sure to set the ESP-IDF target as ESP32-S3 (via USB-Built in JTAG)

## Menuconfig
- ESPDrone Config > ESP_Drone hardware version: esp-drone with ESP32-S2/ESP32-S3 onboard

Disabling because these are unused (free some memory)
- ESPDrone Config > buzzer > enable buzzer to play sound: OFF

### Pinout
ESPDrone Config > sensors config:
- (11) I2C0_PIN_SDA GPIO number
- (13) I2C0_PIN_SCL GPIO number
- (40) I2C1_PIN_SDA GPIO number
- (41) I2C1_PIN_SCL GPIO number
- (37) SPI_PIN_MISO GPIO number
- (35) SPI_PIN_MOSI GPIO number
- (36) SPI_PIN_CLK GPIO number
- (10) SPI_PIN_CS0 GPIO number
- (12) MPU_PIN_INT GPIO number
- (2) EXT01_PIN GPIO number -- this actually becomes ADC1_PIN (i.e. BAT_ADC)
- (34) EXT01_PIN GPIO number -- what does this do?

ESPDrone Config > led config:
- (7) LED_PIN_BLUE GPIO number
- (9) LED_PIN_GREEN GPIO number
- (8) LED_PIN_RED GPIO number

ESPDrone Config > motors config:
- (5) MOTOR01_PIN GPIO number
- (6) MOTOR01_PIN GPIO number
- (3) MOTOR01_PIN GPIO number
- (4) MOTOR01_PIN GPIO number

## Client

## Wifi

## MultiRanger

## Optical Flow Deck + Multiranger 
The Optical Flow Deck is supported by ESP-Drone, but the multiranger is not. The multiranger file was ported over from the Bitcraze's crazyflie repository. There was an issue with how the VL53L1 was being addressed differently in the multiranger driver, the VL53L1 API provided by STM, and the I2C driver. The device address was reset everytime the device was being polled, so the address change (needed so that the sensors were not on the same address) would not work. The solution ended up being to update the API to STM's Ultralite Driver (ULD), and then make corresponding changes to the rest of the code to work with that API. The working testing folder is `multiranger_ULD-API`. I think rather than merging all onto the main file, just copy this over to main and merge all other changes into it.

The PMW3901 flow sensor is yet to be tested (as of 27/09/2025) due to a power issue from the onboard 1.8V LDO. Three VL53L1X sensors (down, left, right) are not functioning - either heat damaged or not seated correctly on the pads.


### Relevant files:
- `components/drivers/i2c_devices/vl531/include/vl531.h`
- `components/drivers/i2c_devices/vl531/vl531x.c` - 
- `components/drivers/i2c_devices/vl531/zranger2.c` - the zranger is the vl531 on the Flow Deck
- `components/drivers/i2c_bus/i2cdev_esp32.c` - I2C driver 
- `components/drivers/ported/multiranger/multiranger.c` - (added component). Multiranger driver
- `components/core/crazyflie/hal/src/pca95x4.c` - PCA9534 (GPIO expander) driver

### Modifications Log
1. Add the multiranger file into the ESP-Drone codebase
    - Download `multiranger.c` from (https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/multiranger.c) or use the one from our git repository in firmware/downloads/Multiranger
    - Create a folder call 'ported' in components/core/crazyflie/drivers, with the following structure (use your downloaded `multiranger.c`, and create new files `CMakeLists.txt` and `multiranger.h`):
        ```
        ported
        |--- multiranger
            |--- include
            |   |--- multiranger.h
            |--- CMakeLists.txt
            |--- multiranger.c
        ```
    - In `multiranger.h`, add:
        ```c
        // @@ New file - interfacing is done a bit differently in CF, so need to define stuff here

        #ifndef _MR_H_
        #define _MR_H_

        void mrInit();
        bool mrTest();

        #endif /* _MR_H_ */
        ```
    - In CMakeLists.txt, add:
        ```CMake
        idf_component_register(SRCS "multiranger.c"
                            INCLUDE_DIRS "." "include"
                            PRIV_REQUIRES crazyflie config i2c_bus deck)
        ```


2. ULD API addition: In our repo, go to `firmware/downloads\VL53L1_ULD\STSW-IMG009\STSW-IMG009_v3.5.5\API`.In the ESP-Drone code, go to `components/core/crazyflie/drivers/vl53l1`. Replace the ESP-Drone `core` folder in `vl53l1` with the `core` folder from the ULD API. In the `vl53l1/include` folder, delete all files EXCEPT for `zranger2.h` and `vl53l1x.h`. Find `platform/vl53l1_types.h` from the ULD API folder, and copy that into that include folder. In `vl53l1x/CMakeLists.txt`, replace whats there with:
    ```CMake
    idf_component_register(SRCS "vl53l1x.c" 
                        "zranger2.c"
                        "core/src/VL53L1X_calibration.c"
                        "core/src/VL53L1X_api.c"
                        INCLUDE_DIRS "." "include" "core/inc"
                        REQUIRES i2c_bus crazyflie platform config)

    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-overflow")
    ```
3. In `vl531x.h`: Modify default VL53L1 address
4. In `pca95x4.c`: Change PCA95X4_DEFAULT_ADDRESS to 8-bit address:
    ```c
    #define PCA95X4_DEFAULT_ADDRESS 0b0100000 << 1
    ```
5. In `vl531x.h`: Comment out the line `#define USE_I2C_2V8`
6. In `vl531x.h`: Declare struct `VL531_Dev_t` as follows:
    ```c
    // @@ Struct and type used for each VL53L1 device
    typedef struct {
        uint8_t   I2cDevAddr;
        I2C_Dev *I2Cx;
    } VL53L1_Dev_t;

    typedef VL53L1_Dev_t *VL53L1_DEV;
    ```
7. In `vl53l1x.c`: Modify to work with new API. Just copy from testing file, too many changes
    - Remove include of `#include 'vl53l1_register_strings'`
    - Change declaration of`nextI2CAddress` to be `0x60`
    - Declare a spinlock used to update the address: 
        ```c
        static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
        ```
    - Replace vl53l1xInit() with:
        ```c
        // @@ Initialisation follows the UM https://www.st.com/resource/en/user_manual/um2510-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf
        // @@ and some of the 2D_Lidar example (https://www.st.com/en/embedded-software/stsw-img017.html),
        bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle)
        {
        uint8_t status = 0;
        uint8_t Bootstate = 0;
        int newAddress;
        uint8_t count = 0;

        taskENTER_CRITICAL(&my_spinlock); 
        newAddress = nextI2CAddress + 2; // @@ Must increment address by 2 since ULD uses an 8-bit address and right shifts (not the 0x29). So bit 0 used for R/W and bit 1 is being incremented on
        nextI2CAddress = nextI2CAddress + 2; // @@ I'm just trying to be explicit here
        taskEXIT_CRITICAL(&my_spinlock);

        pdev->I2cDevAddr = VL53L1X_DEFAULT_ADDRESS;
        pdev->I2Cx = I2cHandle;

        i2cdevInit(pdev->I2Cx); //@@
        //i2c_scan(pdev->I2Cx); //@@ scan the i2c addresses

        // @@ Poll the device 4 times, skip if it does not respond
        while ((Bootstate == 0) && (count <= 3))
        {
            status = VL53L1X_BootState(pdev->I2cDevAddr, &Bootstate);
            count++;
        }
        DEBUG_PRINTI("Boot Status: %d", status); // @@DEBUG
        
        if (status == 0) {
            status = VL53L1X_SensorInit(pdev->I2cDevAddr); // @@ Load default config into sensor
            DEBUG_PRINTI("Sensor Init status: %d", status); // @@DEBUG
            // @@ modifications to sensor parameters can occur now

            if (status == 0) {
                status = vl53l1xSetI2CAddress(pdev,newAddress); //@@ set new I2C address
                DEBUG_PRINTI("VL53L1 Address to %x [Status: %d], and pdev->i2cDevAddr = %x\n", newAddress, status, pdev->I2cDevAddr); // @@DEBUG

                // @@ Just checking ID registers (stock espdrone) (DEBUG)
                uint8_t byteData;
                uint16_t wordData;
                VL53L1_RdByte(pdev->I2cDevAddr, 0x010F, &byteData);
                DEBUG_PRINTI( "VL53L1X Model_ID: %02X\n\r", byteData);
                VL53L1_RdByte(pdev->I2cDevAddr, 0x0110, &byteData);
                DEBUG_PRINTI( "VL53L1X Module_Type: %02X\n\r", byteData);
                VL53L1_RdWord(pdev->I2cDevAddr, 0x010F, &wordData);
                DEBUG_PRINTI( "VL53L1X: %02X\n\r", wordData);
            }
            
        }
        

        #ifdef SET_VL53LX_ROI
        VL53L1_UserRoi_t Roi0;
        Roi0.TopLeftX = 0; //set ROI according to requirement
        Roi0.TopLeftY = 15;
        Roi0.BotRightX = 15;
        Roi0.BotRightY = 0;
        status = VL53L1_SetUserROI(pdev, &Roi0); //SET region of interest
        #endif
            //Restart sensor remove to zranger task
            //VL53L1_StopMeasurement(pdev);
            //status = VL53L1_SetDistanceMode(pdev,VL53L1_DISTANCEMODE_LONG);
            //status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pdev, 160000);
            //status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pdev, 200);  
            //status = VL53L1_StartMeasurement(pdev);

        return status == 0;
        }
                ```
            - Replace vl53l1xSetI2CAddress with
                ```c
                uint8_t vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address)
                {
                uint8_t status = 0;

                status = VL53L1X_SetI2CAddress(pdev->I2cDevAddr, address); // @@ modified for ULD API
                pdev->I2cDevAddr = address;
                return  status;
                }
        ```
    - Debugging functions.
    - Replace all VL53L1X_ERROR's with uint8_t (make sure you don't replace the VL53L1X_ERROR_CONTROL_INTERFACE though!). Define `#define VL53L1X_ERROR_CONTROL_INTERFACE = 14` at the top of the file. The ULD does not have the same support for error codes, so I am bringing across this one from the original driver (but its unsigned int so making it positive). Would be cleaner to just rewrite the all of the functions in `vl53lx.c` properly, but these are also referenced outside of the API so it does get a bit messy if done wrong.
7. In `multiranger.c`: Modify to work with new API. Just copy the two files from the testing folder 'multiranger_ULD-API' into the main folder, as there are many changes here.
    - Remove the include to `deck.h`
    - Modify include of `debug.h` to `debug_cf.h`
    - Modify the `filterMask` definition to be `0x01` (not shifted by `VL53L1_RANGESTATUS_RANGE_VALID`)
    - Modify `mrGetMeasurementAndRestart` function to be the following (note the name is misleading, it does not restart the sensor, the sensor is continuously ranging):
        ```c
        static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *pdev)
        {
            // @@ Add DEBUG_PRINTI lines to check the status after each stage if errors are being encountered.
            
            VL53L1X_ERROR status = VL53L1X_ERROR_NONE;
            uint16_t distance;
            uint8_t dataReady = 0;
            uint8_t rangeStatus = 0;

            while (dataReady == 0)
            {
                status = VL53L1X_CheckForDataReady(pdev->I2cDevAddr, &dataReady);
                vTaskDelay(M2T(1));
            }
            status = VL53L1X_GetDistance(pdev->I2cDevAddr, &distance);
            status = VL53L1X_GetRangeStatus(pdev->I2cDevAddr, &rangeStatus);
            status = VL53L1X_ClearInterrupt(pdev->I2cDevAddr); // @@this might not be necessary since we aren't using the interrupt? UM recommends it though
            status = status; // @@ Status update, status is status. The status. Status is status. Status Update. 

            if (filterMask & (1 << rangeStatus))
            {
                distance = distance; // @@ SHUT UP SHUT UP SHUT UP
            }
            else
            {
                distance = 32767;
            }

            return distance;
        }
        ```
    - Replace the startup code in mrTask with this:
        ```c
            VL53L1X_ERROR status = VL53L1X_ERROR_NONE;

            systemWaitStart();
            
            // Restart all sensors
            // @@ ULD version -- may need some delay between these?
            status = VL53L1X_StartRanging(devFront.I2cDevAddr);
            status = VL53L1X_StartRanging(devBack.I2cDevAddr);
            status = VL53L1X_StartRanging(devUp.I2cDevAddr);
            status = VL53L1X_StartRanging(devLeft.I2cDevAddr);
            status = VL53L1X_StartRanging(devRight.I2cDevAddr);

            //i2c_scan(I2C1_DEV); // @@DEBUG

            status = status; // @@ GO AWAY GO AWAY GO AWAY

            TickType_t lastWakeTime = xTaskGetTickCount();

        ```
    - Remove the `static` keyword for `mrInit()` and `mrTest()` (so they are just `void mrInit()` and `void mrTest()`).
    - Data spoofing: At the top of the file, add `#define SPOOF`. In function `mrInit()`, add the following (for spoofing data if needed):
        ```c
            #ifdef SPOOF
                rangeSet(rangeFront, 1000 / 1000.0f);
                rangeSet(rangeBack, 1000 / 1000.0f);
                rangeSet(rangeUp, 1000 / 1000.0f);
                rangeSet(rangeLeft, 1000 / 1000.0f); 
                rangeSet(rangeRight, 1000 / 1000.0f); 
                rangeSet(rangeDown, 1000 / 1000.0f); 
            #endif
        ```
    - Add a test case in `mrTest()` before the other tests that calls `pca95x4Test`, since otherwise there pca9534 is never tested:
        ```c
            isPassed &= pca95x4Test(PCA95X4_DEFAULT_ADDRESS);
        ```
8. In `zranger2.c`: Modify to work with new API. Just copy the two files from the testing folder 'multiranger_ULD-API' into the main folder, as there are many changes here.
    - Modify `zRanger2GetMeasurementAndRestart` to be the following (note this is essentially `mrGetMeasurementAndRestart` without the filtermask)
        ```c
        static uint16_t zRanger2GetMeasurementAndRestart(VL53L1_Dev_t *dev)
        {
            VL53L1X_ERROR status = VL53L1X_ERROR_NONE;
            uint16_t distance; 
            uint8_t dataReady = 0;
            uint8_t rangeStatus = 0;

            while (dataReady == 0)
            {
                status = VL53L1X_CheckForDataReady(dev->I2cDevAddr, &dataReady);
                vTaskDelay(M2T(1));
            }

            status = VL53L1X_GetDistance(dev->I2cDevAddr, &distance);
            status = VL53L1X_GetRangeStatus(dev->I2cDevAddr, &rangeStatus);
            status = VL53L1X_ClearInterrupt(dev->I2cDevAddr); // @@ this might not be necessary since we aren't using the interrupt? UM recommends it though
            status = status;

            return distance;
        }
        ```
    - Modify `zRanger2Task()` to be the following. Removed some of the setup of the VL53L1 (originally set up as short mode and with 25ms Timing Budget), so that it would use the default long mode and 100ms Timing budget, which just better corresponds to the MR. Note that the UM for the ULD suggests a timing budget of 200ms for a full 4m range.
        ```c
        void zRanger2Task(void* arg)
        {
        TickType_t lastWakeTime;

        VL53L1X_ERROR status = VL53L1X_ERROR_NONE;

        systemWaitStart();

        // Restart sensor @@ I want it to init as default settings for now
        //VL53L1X_SetDistanceMode(dev, 1); // 1 = Short (1.3m), 2 = Long (4m), but Long says thats only with a 200ms timing budget.
        //VL53L1X_SetTimingBudgetInMs(&dev, 25);

        status = VL53L1X_StartRanging(dev.I2cDevAddr);
        DEBUG_PRINTI("Down Ranging Starting, Start Status %d", status);

        lastWakeTime = xTaskGetTickCount();

        while (1) {
            vTaskDelayUntil(&lastWakeTime, M2T(100)); // @@ changed from 25ms to match default values 

            range_last = zRanger2GetMeasurementAndRestart(&dev);
            rangeSet(rangeDown, range_last / 1000.0f);

            // DEBUG_PRINTI("Range Down: %.3f", rangeGet(rangeDown)); //@@ DEBUGGING

            // check if range is feasible and push into the estimator
            // the sensor should not be able to measure >5 [m], and outliers typically
            // occur as >8 [m] measurements
            if (range_last < RANGE_OUTLIER_LIMIT) {
            float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
            float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
            rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
            }
        }
        }
        ```
9. In `i2cdev_esp32.c`: In functions i2cdevWriteReg16 and i2cdevReadReg16, increase the ticks to weight until the bus is able to handle the speeds. May also need to adjust this in i2cdevWriteReg8 and i2cdevReadReg8, but was not necessary during testing. The testing script increased the value from 5 to 100, but this may need adjustment.
10. In `i2cdev_esp32.c`: In functions `i2cdevWriteReg16`, `i2cdevReadReg16`, `i2cdevWriteReg8`, `i2cdevReadReg8`, modifications must be made to account for the different cases of a 7-bit I2C address and 8-bit I2C address being passed into the functions*. The quick-fix is slightly cursed - the MPU6050 (I2C0) uses a 7-bit address, the PCA9534 (I2C1) uses a 7-bit address and the VL53L1 (I2C1) must be changed to an 8-bit address to work with the API. The PCA9534 code is very simple, so this address can easily be changed between 7-bit and 8-bit. The current modification hence assumes all I2C1 addresses are 8-bit and all I2C0 addresses are 7-bit. For the 7-bit I2C0 addresses, the address is left shifted and OR'd with a Read/Write bit to form the command. For the 8-bit I2C1 addresses, the address is not shifted, and OR'd with a Read/write bit to form the command. A conditional statement would be clearer here, but I am just shifting it by the NOT of the port identifier. An example for i2cdevReadReg16 is shown below:
    ``` c
    bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                        uint16_t len, uint8_t *data)
    {
        uint8_t shift = !(dev->def->i2cPort); // @@ shift=1 for I2C0, shift=0 for I2C1
        
        if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
            return false;
        }

        uint8_t memAddress8[2];
        memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
        memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (devAddress << shift) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN); // @@ Shift and OR
            i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
        }
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, (devAddress << shift) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);  // @@ Shift and OR
        
        .... // the rest of the function is identical.
    ```
11. (Optional) In `i2cdev_esp32.c`:  define `i2c_scan()` shown below. If not used, make sure there are no calls to it from other files Just used for debugging:
    ```c
    void i2c_scan(I2C_Dev *dev)
    {
        DEBUG_PRINTI("Scanning I2C bus...\n");
        for (int addr = 0x08; addr < 0x78; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            // @@ This Returns the 7-bit address, so it will not match the 0x62, 0x64, 0x66 addresses being assigned to the VL53L1s. Could do the shift method from the other files.
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(dev->def->i2cPort, cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                DEBUG_PRINTI("Found device at 0x%02X\n", addr);
            }
        }
        DEBUG_PRINTI("Scan done.\n");
    }
    ```

*Ideally, we would modify the VL53L1 ULD API so that all addresses can be 7-bit addresses, (or change all other code to handle 8-bit addresses). Changing the API would probably be the cleanest, and this is feasible with the ULD API since it is so small. The current modification works though, and there are bigger issues to deal with at the moment.

## Motor Driver
The Timer frequency is set to 500Hz. The functions motorsConv16ToBits(.) and motorsConvBitsTo16(.) were already defned in CrazyFlie for some of their own mapping. These have been modified to map down to a range allowing for 1ms-2ms (assuming f=500Hz).

The BLHeli ESC doesn't like it (as in it somehow causes the device to reboot) when the signal inputs are in their high-impedance (or undefined?) state before the signal GPIO are configured as PWM signals. So, the first thing done in the code is to set the motor signal pins as GPIO and just set them low during initialisation. The arming process works normally.

### Relevant files:
- `components/core/crazyflie/modules/src/power_distribution_stock.c`
- `components/drivers/general/motors/motors.c`
- `components/core/crazyflie/modules/src/system.c`

### Modifications Log
1. In motorsConv16ToBits(uint16_t bits), replace the return line with `return (bits >> (16 - MOTORS_PWM_BITS + 1)) + (1 << (MOTORS_PWM_BITS-1))`. This maps a thrust value into the PWM duty cycle range (1ms-2ms).
2. In motorsConvBitsTo16(uint16_t bits), replace the return line with return `(bits - (1 << (MOTORS_PWM_BITS-1))) << (16 - MOTORS_PWM_BITS + 1);`. This maps the PWM duty ratio bits back into the thrust bits (16 bit).
3. In pwm_timmer_init(), in the `ledc_timer` struct, set `freq_hz` to be 500.
4. (shouldn't be required) in motorsBeep(..), set `freq_hz` to 500.
5. In system.c, add this public function (and declare it in system.h):
    ```c
    void systemSetMotorsLow(void)
    {
        // @@ Set motor pins to be low during initialisation.

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<CONFIG_MOTOR01_PIN) | (1ULL<<CONFIG_MOTOR02_PIN) | (1ULL<<CONFIG_MOTOR03_PIN) | (1ULL<<CONFIG_MOTOR04_PIN), // @@ change 6 to whatever motor pin is
        .mode = GPIO_MODE_OUTPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
    gpio_set_level(CONFIG_MOTOR01_PIN, 0);
    gpio_set_level(CONFIG_MOTOR02_PIN, 0);
    gpio_set_level(CONFIG_MOTOR03_PIN, 0);
    gpio_set_level(CONFIG_MOTOR04_PIN, 0);
    }
    ```
6. In main.c, call `systemSetMotorsLow()` in app_main(). It MUST be the first function called.
7. TODO: Update this with the ThrustMap functions!!


## Power Management
Configuring the power management to read a 2S battery. NOTE that in testing, these changes have been integrated with the BLHeli_Driver folder rather than a separate power management folder.

### Relevant files
- `components/core/crazyflie/hal/interface/pm_esplane.c`
- `components/core/crazyflie/hal/interface/pm_esplane.c`

### Modifications Log
1. TODO: In pm_esplane.c, bat671723HS25C holds an LUT for mapping battery voltage->percentage. This must be updated with the battery discharge curve.
2. In pm_esplane.c, function pmInit() calls pmEnableExtBatteryVoltMeasuring(CONFIG_ADC1_PIN, xx). Change xx to 4. xx is the multiplier that corrects for our 4:1 voltage divider on the ADC input.
3. In pm_esplane.c, function pmInit() sets pmSyslinkInfo.vBat = 3.7f. Change this to 7.4f -- not actually sure what this does, may just be for logging
4. In pm_esplane.h, set PM_BAT_LOW_VOLTAGE  to 6.4f (original 3.2f)
5. In pm_esplane.h, set PM_BAT_CRITICAL_LOW_VOLTAGE to 6.0f (original 3.0f)


