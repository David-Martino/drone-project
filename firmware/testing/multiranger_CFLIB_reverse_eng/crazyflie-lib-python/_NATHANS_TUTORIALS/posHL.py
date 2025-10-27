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

DEFAULT_HEIGHT = 0.5
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
            time.sleep(5)
            # for i in range(10):
            #     print(f"{i}s")
            #     pc.go_to(0,0)
            #     time.sleep(1)
            # print("Done")
            # time.sleep(40)
            #pc.forward(2, velocity=0.25)
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
    # vActual.append(data['stateEstimate.vx'])
    # vCommand.append(data['ctrltarget.vx'])

    vx = data["stateEstimate.vx"]
    x = data["stateEstimate.x"]
    z = data["stateEstimate.z"]
    print(f"vx: {vx} \t x: {x} \t z: {z}")

def log_mot_callback(timestamp, data, logconf):


    m1 = data["motor.m1"]
    m2 = data["motor.m2"]
    m3 = data["motor.m3"]
    m4 = data["motor.m4"]

    print(f"M1: {m1} \t M2: {m2} \t M3: {m3} \t M4: {m4}")



if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #scf.cf.param.set_value('stabilizer.controller', 'indi')


        # Logging Setup
        logconf = LogConfig(name='velocity', period_in_ms=10)
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.z', 'float')

        lg_motor = LogConfig(name='thrust', period_in_ms=10)
        lg_motor.add_variable('motor.m1', 'uint32_t')
        lg_motor.add_variable('motor.m2', 'uint32_t')
        lg_motor.add_variable('motor.m3', 'uint32_t')
        lg_motor.add_variable('motor.m4', 'uint32_t')


        scf.cf.log.add_config(lg_motor)
        lg_motor.data_received_cb.add_callback(log_mot_callback)

        scf.cf.param.set_value('commander.enHighLevel', '1')
        time.sleep(0.1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Logging Init
        lg_motor.start()
        
        hover(scf)

        # OAdata = [[vActual],[vCommand]]
        # with open('vel.csv', 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['actual', 'set'])   # Header
        #     for xi, yi in zip(vActual, vCommand):
        #         writer.writerow([xi, yi])


        lg_motor.stop()
        
            