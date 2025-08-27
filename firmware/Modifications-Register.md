# Modifications Register

## Pinout

## Client

## Wifi

## MultiRanger

## Optical Flow Deck
While supported by ESP-Drone, there are hardware differences that must be addressed.
### Notes
- To Test: In `vl531.h`, the default address is defined as 0b0101001 (0x29, and note that only 7 bits were used), however the datasheet states that the default is 0b01010010 (0x52, note that the difference is just the extra 0 at the end - VL531 receives in big endian though (MSB arrives first)). Despite this, the inita; connection was confirmed as OKAY on I2C1 but then it got stuck trying to read data. Test with 0x52 to see effect.

### Relevant files:
- `components/drivers/i2c_devices/vl531/include/vl531.h`
- `components/drivers/i2c_devices/vl531/include/vl531.c`

### Modifications Log
1. In `vl531.h`: Comment out the line `#define USE_I2C_2V8`

## Motor Driver

### Modifications Log
1. PWM Frequency
2. PWM Duty Cycle

## Magnetometer

