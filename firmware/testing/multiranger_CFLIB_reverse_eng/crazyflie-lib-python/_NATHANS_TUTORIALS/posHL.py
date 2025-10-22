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

import csv

uri = uri_helper.uri_from_env(default='udp://192.168.43.42')
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.3
BOX_LIMIT = 0.5

position_estimate = [0,0]
vActual = []
vCommand = []

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
            time.sleep(2)
            #pc.go_to(0,0)
            # time.sleep(40)
            print("Forwards")
            pc.forward(4, velocity=0.5)
            # time.sleep(2)
            # print("Right")
            # pc.right(1, velocity=0.5)
            # time.sleep(2)
            # print("Back")
            # pc.back(1, velocity=0.5)
            # time.sleep(2)
            # print("Left")
            # pc.left(1, velocity=0.5)
            # time.sleep(5)
            print("Landing!")


def log_pos_callback(timestamp, data, logconf):
    #print("Height: %d \t Voltage: %d", data['stateEstimate.z'], data['pm.vbat'])
    #position_estimate[0] = data['stateEstimate.x']
    #position_estimate[0] = data['stateEstimate.y']
    vActual.append(data['stateEstimate.vx'])
    vCommand.append(data['ctrltarget.vx'])


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #scf.cf.param.set_value('stabilizer.controller', 'indi')


        # Logging Setup
        logconf = LogConfig(name='velocity', period_in_ms=10)
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('ctrltarget.vx', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        scf.cf.param.set_value('commander.enHighLevel', '1')
        time.sleep(0.1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Logging Init
        logconf.start()
        
        hover(scf)

        OAdata = [[vActual],[vCommand]]
        with open('vel.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['actual', 'set'])   # Header
            for xi, yi in zip(vActual, vCommand):
                writer.writerow([xi, yi])


        logconf.stop()
        
            