# README
There are two folders in the firmware folder
- Testing: For each different function, pull a new repo of ESP-Drone into this folder and make modifications in there so that they may be unit tested. Be aware of changes that will affect other functions (eg the same Vl531 drivers are shared by the Multiranger and optical flow decks). Take note of ALL modifications made to the base firmware in the `Modifications-Register.md`.
- Main: This is the main build to be released on the drone. When a unit function has been tested and is working, integrate it into the esp-drone repo underneath this folder, and make sure to test it in conjunction with the other functions.

The `Codebase-Notes.md` documents the reverse engineering and current understanding of the ESP-Drone codebase.
The `Modifications-Register.md` documents any changes made to the base esp-drone or cfclient firmware.

# Required Installations
This document details the various tools needed to work on the ESP-Drone codebase. The `main` folder already has the esp-drone and cfclient repositories.

## Python
Install python 3.10.11 (this will install pip as well, needed for CFClient)
https://www.python.org/downloads/release/python-31011/

## VS Code and ESP-IDF
VS Code is a lighweight environment for writing code. ESP-IDF is the most supported IDE for developing with ESP firmware. It is available in VS Code as an extension. See `Configuring VS Code` in `Codebase_Notes.md` for the setup guide.
Note:
- ESP-Drone should work with v4.4 or v5.0 of ESP-IDF
- We (Nathan and Erwin) are both on v5.0
- ESP-Drone recommends v4.4 in their official documentation, although users have recommended v5.0 in various forums. Testing has resulted in the mostly the same issues on v4.4 as v5.0.

## ESP-Drone
The ESP-Drone repository is found here:
https://github.com/espressif/esp-drone.git
The firmware team is using commit `527ee2e` of the project. See `Testing Build` in `Codebase_Notes.md` to do a test build of the project.

## CFClient
Note that CF GCS is actually split into two parts
- CFLib: This provides the API to interact with the drone
- CFClient: This provides the GUI for the CFLib API
Installing CFClient will also install CFLib at the same time

#### Working version of CFClient (JobitJoseph)
From https://github.com/espressif/esp-drone/issues/84 related to the 'WIFI_UDP: udp packet cksum unmatched' error, JobitJoseph recommends using this version of CFClient:
- CFClient: https://github.com/jobitjoseph/crazyflie-clients-python
- CFLib (this should be automatically installed when installing CFClient): https://github.com/jobitjoseph/crazyflie-lib-python
Installation:
In the CFClient folder, run:
`pip install -e .`
to install CFClient (python and pip should be installed)

Note: 
This is the CFClient version that has been tested to sustain ~3mins of connection. If you have already installed the broken CFClient, it will have also have installed the wrong CFLib. Uninstall both using 
`pip uninstall cflib cfclient`
and then reinstall using JobitJoseph's versions

#### ESP-Drone's Fork of CFClient
ESP-Drone forked their own version of CFClient, however there seems to be an issue with their UDP driver, causing the connection to the current version of ESP-Drone to only maintain connection for ~5 seconds, with a 'WIFI_UDP: udp packet cksum unmatched' error reported by the ESP-Drone over serial (Issue recreated both on ESP-IDFv4.4 and ESP-IDFv5.0). The link to this repository is here:
