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

#def hover(scf):
    #print("Take off!")
    #cf = scf.cf

    # for i in range(10):
    #     cf.commander.send_hover_setpoint(0, 0, 0, i/20) # slowly increase height

    # for i in range(10):
    #     cf.commander.send_hover_setpoint(0, 0, 0, i/20) # slowly increase height






    


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #scf.cf.commander.set_client_xmode(True)
        scf.cf.param.set_value('commander.enHighLevel', '1')
        time.sleep(0.1)

        scf.cf.platform.send_arming_request(True)
        
        print("start")
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        print("setpoint")
        scf.cf.commander.send_setpoint(0, 0, 0, 12000)
        print("sleep")
        time.sleep(2)
        print("stop")
        scf.cf.commander.send_stop_setpoint()
        
            