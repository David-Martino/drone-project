# Understanding of `esp-drone`

- The `esp-drone` project (developed by Espressif) is a drone hardware design and codebase for a small drone using an ESP32, ESP32-S2, or ESP32-S3 microcontroller. 
- With the use of extension boards (specifically the multi-ranger deck and the flow deck), it claims to support a wide range of autonomous piloting features, inlcuding:
    - Stabilise mode,
    - Height-hold mode,
    - Position-hold mode.
- The drone firmware is primarily based on the `crazyflie-firmware` project (developed by Bitcraze), but that was written for a family of drones using STM32 microcontrollers. Espressif has ported the project to work on ESP32 microcontrollers with the ESP-IDF. 
- The default behaviour of the project is to boot into manual control via either the iOS or Android ESP-Drone mobile app, or via a controller connected to a computer running the Crazflie Client. 
- In this regard, Espressif's `esp-drone` project differs from the system developed by Bitcraze: instead of using a custom radio transceiver (the Crazyradio), the `esp-drone` project communicates over wifi using UDP (User Datagram Protocol) while still utilising the CRTP (Crazyflie Real-Time Protocol).
- This necessitated modification of both the drone firmware and the Crazyflie Client by Espressif.

# Kimberly McGuire 
- On Bitcraze's community projects blog, Kimberly McGuire posted about her demonstration of using a Crazyflie drone with the mutli-ranger deck and flow deck to perform room mapping with ROS 2. 
- This was possible with both manual control and ROS-based autonomous wall-following. 
- However, her project also used am Bitcraze STM32-based drone and a Crazyradio for communication.

# The Design Idea
- Have the drone boot into autonomous mode, being ultimately controlled by ROS from the groundstation via automatic connection.
- Use Kimberly McGuire's groundstation ROS setup to control the drone.
- Have the drone be able to switch to manual control and position-hold on command.   

# The Design Plan
- Understand how FreeRTOS works with the firmware:
    - How do FreeRTOS tasks work with the scheduler?
    - How do FreeRTOS objects like queues/mutexes work?
    - How does FreeRTOS launch the application code?
    - How does FreeRTOS create and manage the application tasks?

- Understand the `esp-drone` firmware boot and system launch process:
    - How does it establish and run communication via UDP?
    - How does it initialise its hardware?
    - How it enters manual flight control mode?
    - How much support has been ported for the multi-ranger deck?
    - How much support has been ported for the range deck?

- Understand the `crazyflie-firmware` boot and system launch process:
    - How does it launch and integrate the mutli-ranger deck?
    - How does it launch and integrate the flow deck?
    - What is its default flight control mode and how does it enter it?

- Understand the `crazyflie-firmware` flight modes:
    - How does it implement position-hold using the multi-ranger and flow decks?
    - How does it switch between modes?

- Understand Kimberly McGuire's autonomous wall-following control and mapping.
    - How does the drone communicate its sensor data to ROS?
    - How does ROS get navigation instructions to the drone?

- Incorporate extension boards into `esp-drone`:
    - Port multi-ranger deck support and implement.
    - Port flow deck support and implement.

- Incorporate flight modes into `esp-drone`:
    - Port position-hold mode support for the hover test.
    - Port mode switching.

- Implement Kimberly McGuire's groundstation setup.
    - Make any necessary modifications for UDP instead of Crazyradio.
    - Make any necessary modifications for gamepad-based manual control.
    - Make any necessary modifications for position-hold control mode (hover test).

# The Progress
