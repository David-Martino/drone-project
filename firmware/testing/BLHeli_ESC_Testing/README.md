# BLHeli ESC Testing
This just a PWM Driver with a duty cycle controlled by a potentiometer. The PWM frequency must be set so that the high time of a PWM pulse can last from 1ms-2ms (ie f < 500Hz) with 1ms being 0% throttle and 2ms being 100% throttle. The current script uses f=500Hz with a 12 bit duty cycle resolution (so that 2048=1ms=0% and 4096=2ms=100%). The potentiometer is read with a 12 bit ADC, which is mapped into the range of 2048-4096 to be passed into the PWM Driver.


The BLHeli 14.2 manual is on teams under "Design>Flight Controller>Datasheets and Manuals". Very much worth reading if doing ESC stuff.

## Highlight of the main notes from the Manual
There are three versions of BLHeli 14.2
1. Main: For a helicopter main motor
2. Tail: For a helicopter tail motor
3. Multi: For a multirotor aircraft.
The ESC we bought (https://www.aliexpress.com/item/1005006412699316.html) does not specify exactly which version it uses, but the product is listed as "For 160-250 Multirotor FPV Racing RC Drone Quadcopter" so it may have the Multi one on there. The motor control inputs are the same regardless, the versions are just tailored more specifically to their respective applications.

The control input is detected on startup during the "arming" sequence. The different styles are:
- PWM: 1, 2, 4, 8, 12kHz (But this does NOT seem to work)
- PPM: 1150-1830us with a frequency of 50Hz up to 'several hundred hertz' (This DOES seem to work)
- Oneshot125: Same as PPM but all "timing is divided by 8" (the frequency? the pulse delay? AHHH)

# POSSIBLE PROBLEMS
There is an arming sequence as follows (read the one on the document if this is important for you as there are some caveats):
1. On power on, the motors are 'beeped' 3 times
2. MCU must apply throttle input. Once a nonzero throttle is detected, the ESC will send a low tone beep to the motors.
3. MCU must return throttle to zero. Once a zero throttle detected, the ESC will send a higher tone beep.
4. After the high beep, the ESC is armed.
This likely requires a modification to ESP-Drone firmware - ESP-Drone has its own propeller test as well.