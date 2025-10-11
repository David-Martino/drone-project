import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='udp://192.168.43.42')
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.3
BOX_LIMIT = 0.5

position_estimate = [0,0]

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.start_forward()

        while (1):
            if position_estimate[0] > BOX_LIMIT:
                mc.start_back()
            elif position_estimate[0] < -BOX_LIMIT:
                mc.start_forward()
            time.sleep(0.1)

def take_off_simple(scf):
    print("Take off!")
    #count  = 0
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(4)
        print("Going Forward")
        mc.forward(0.5)
        time.sleep(3)
        print("Going Right")
        mc.right(0.5)
        time.sleep(3)
        print("Going Back")
        mc.back(0.5)
        time.sleep(3)
        print("Going Left")
        mc.left(0.5)
        time.sleep(3)
        # print("Rolling right")
        # mc.right(0.5)
        # time.sleep(3)
        # print("Going Forward")
        # mc.forward(0.5)
        # time.sleep(3)
        # print("Turning Right")
        # mc.turn_right(90)
        # time.sleep(2)
        # print("Turning Left")
        # mc.turn_left(90)
        # time.sleep(2)
        # print("Rolling Left")
        # mc.left(1)
        # time.sleep(3)
        print("Landing!")
        mc.land()
        #mc.stop()

        # tute says it instantly closes with a land function, - ie won't just hover

    

def log_pos_callback(timestamp, data, logconf):
    print("Height: %d \t Voltage: %d", data['stateEstimate.z'], data['pm.vbat'])
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[0] = data['stateEstimate.y']


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        # Logging Setup
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('pm.vbat','float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Logging Init
        #logconf.start()

        # Motion Commands
        take_off_simple(scf)

        # Logging Stop
        #logconf.stop()

        