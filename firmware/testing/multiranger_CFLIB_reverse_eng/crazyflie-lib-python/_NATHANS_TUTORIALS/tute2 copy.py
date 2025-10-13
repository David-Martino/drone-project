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
from cflib.positioning.position_hl_commander import PositionHlCommander

uri = uri_helper.uri_from_env(default='udp://192.168.43.42')
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.5
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

def hover(scf):
    print("Take off!")
    #count  = 0
    with PositionHlCommander(scf, default_height=DEFAULT_HEIGHT, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            #time.sleep(2)
            print("Forwards")
            pc.forward(1, velocity=0.5)
            time.sleep(2)
            print("Right")
            pc.right(1, velocity=0.5)
            time.sleep(2)
            print("Back")
            pc.back(1, velocity=0.5)
            time.sleep(2)
            print("Left")
            pc.left(1, velocity=0.5)
            time.sleep(5)
            #pc.land()

    


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.set_value('commander.enHighLevel', '1')
        time.sleep(0.1)
        
        hover(scf)
        
            