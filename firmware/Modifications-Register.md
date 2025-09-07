# Modifications Register
Note: make sure to set the ESP-IDF target as ESP32-S3 (via USB-Built in JTAG)

## Menuconfig
- ESPDrone Config > ESP_Drone hardware version: esp-drone with ESP32-S2/ESP32-S3 onboard

Disabling because these are unused (free some memory)
- ESPDrone Config > buzzer > enable buzzer to play sound: OFF

### Pinout
ESPDrone Config > sensors config:
(11) I2C0_PIN_SDA GPIO number
(13) I2C0_PIN_SCL GPIO number
(40) I2C1_PIN_SDA GPIO number
(41) I2C1_PIN_SCL GPIO number
(37) SPI_PIN_MISO GPIO number
(35) SPI_PIN_MOSI GPIO number
(36) SPI_PIN_CLK GPIO number
(10) SPI_PIN_CS0 GPIO number
(12) MPU_PIN_INT GPIO number
(2) EXT01_PIN GPIO number -- this actually becomes ADC1_PIN (i.e. BAT_ADC)
(34) EXT01_PIN GPIO number -- what does this do?

ESPDrone Config > led config:
(7) LED_PIN_BLUE GPIO number
(9) LED_PIN_GREEN GPIO number
(8) LED_PIN_RED GPIO number

ESPDrone Config > motors config:
(5) MOTOR01_PIN GPIO number
(6) MOTOR01_PIN GPIO number
(3) MOTOR01_PIN GPIO number
(4) MOTOR01_PIN GPIO number

## Client

## Wifi

## MultiRanger

## Optical Flow Deck
While supported by ESP-Drone, there are hardware differences that must be addressed. Currently, VL531LX is working. Will require manufactured optical flow deck to test PMW3901.

### Relevant files:
- `components/drivers/i2c_devices/vl531/include/vl531.h`
- `components/drivers/i2c_devices/vl531/include/vl531.c`
- `components/drivers/i2c_bus/i2cdev_esp32.c`

### Modifications Log
1. In `vl531.h`: Comment out the line `#define USE_I2C_2V8`
2. In `i2cdev_esp32.c`: In functions i2cdevWriteReg16 and i2cdevReadReg16, increase the ticks to weight until the bus is able to handle the speeds. May also need to adjust this in i2cdevWriteReg8 and i2cdevReadReg8, but was not necessary during testing. The testing script increased the value from 5 to 100, but this may need adjustment. 

## Motor Driver
motors.c is the only modified file in this. The Timer frequency is set to 500Hz. The functions motorsConv16ToBits(.) and motorsConvBitsTo16(.) were already defned in CrazyFlie for some of their own mapping. These have been modified to map down to a range allowing for 1ms-2ms (assuming f=500Hz).

The BLHeli ESC seems to arm as normal on power-on, although I have not written an arming process. ESP-Drone does do a motor beep test on power-on, so this may actually be doing the arming for us. If, when all motors are connected, we hear them arm sequentially (as opposed to at the exact same time), this would mean it is that beeping function that is doing the arming. Otherwise, could just be noise on the PWM signal?

### Relevant files:
- `components/core/crazyflie/modules/src/power_distribution_stock.c`
- `components/drivers/general/motors/motors.c`

### Modifications Log
1. In motorsConv16ToBits(uint16_t bits), replace the return line with `return (bits >> (16 - MOTORS_PWM_BITS + 1)) + (1 << (MOTORS_PWM_BITS-1))`. This maps a thrust value into the PWM duty cycle range (1ms-2ms).
2. In motorsConvBitsTo16(uint16_t bits), replace the return line with return `(bits - (1 << (MOTORS_PWM_BITS-1))) << (16 - MOTORS_PWM_BITS + 1);`. This maps the PWM duty ratio bits back into the thrust bits (16 bit).
3. In pwm_timmer_init(), in the `ledc_timer` struct, set `freq_hz` to be 500.
4. (shouldn't be required) in motorsBeep(..), set `freq_hz` to 500.


## Magnetometer

