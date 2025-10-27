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

import csv
import numpy as np

uri = uri_helper.uri_from_env(default='udp://192.168.43.42')
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

position_estimate = [0,0]

vzList = []
vzctrlList = []
zList = []
vxList = []
vxctrlList = []
xList = []
vyList = []
vyctrlList = []
yList = []
pitchList = []
cmdList = []


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
        print("Forward")
        mc.forward(2, velocity = 0.5)

        #time.sleep(5)
        #time.sleep(2)at  
        #time.sleep(3)
        #mc.forward(4, velocity=0.5)
        #print("Finished")
        #time.sleep(10)
        #mc.forward(2,velocity=0.3)
        #mc.stop()
        # mc.left(1)
        # time.sleep(3)
        print("Landing!")
        #mc.land()
        #mc.stop()

        # tute says it instantly closes with a land function, - ie won't just hover

    

def log_pos_callback(timestamp, data, logconf):
    #print("Height: %d \t Voltage: %d", data['stateEstimate.z'], data['pm.vbat'])
    #position_estimate[0] = data['stateEstimate.x']
    #position_estimate[0] = data['stateEstimate.y']
    #xpos.append(data['stateEstimate.x'])
    #dist.append(data['range.front'])
    # vy = data["stateEstimate.vy"]
    # vx = data["stateEstimate.vx"]
    # print(f"vx: {vx} \t vy: {vy}")

    # vzctrl = data["posCtl.targetVZ"]
    # vz = data["stateEstimate.vz"]
    # z = data["stateEstimate.z"]
    # vzList.append(vz)
    # vzctrlList.append(vzctrl)
    # zList.append(z)

    vxctrl = data["posCtl.targetVX"]
    vx = data["stateEstimate.vx"]
    # x = data["stateEstimate.x"]
    vxList.append(vx)
    vxctrlList.append(vxctrl)
    # xList.append(x)

    # vyctrl = data["posCtl.targetVY"]
    # vy = data["stateEstimate.vy"]
    # y = data["stateEstimate.y"]
    # vyList.append(vy)
    # vyctrlList.append(vyctrl)
    # yList.append(y)

    cmd = data["controller.pitch"]
    pitch = data["stabilizer.pitch"]
    pitchList.append(pitch)
    cmdList.append(cmd)

def log_mot_callback(timestamp, data, logconf):

    m1 = data["motor.m1"]
    m2 = data["motor.m2"]
    m3 = data["motor.m3"]
    m4 = data["motor.m4"]

    print(f"M1: {m1} \t M2: {m2} \t M3: {m3} \t M4: {m4}")

if __name__ == '__main__':

    cflib.crtp.init_drivers()

    

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        # Logging Setup
        logconf = LogConfig(name='Position', period_in_ms=10)
        #logconf.add_variable('stateEstimate.vx', 'float')
        # logconf.add_variable('posCtl.targetVZ', 'float')
        # logconf.add_variable('stateEstimate.vz', 'float')
        # logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('posCtl.targetVX', 'float')
        logconf.add_variable('stateEstimate.vx', 'float')
        # logconf.add_variable('stateEstimate.x', 'float')
        # logconf.add_variable('posCtl.targetVY', 'float')
        # logconf.add_variable('stateEstimate.vy', 'float')
        # logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('controller.pitch', 'float')
        logconf.add_variable('stabilizer.pitch', 'float')
        
        #logconf.add_variable('range.front', 'int16_t')

        lg_motor = LogConfig(name='thrust', period_in_ms=10)
        lg_motor.add_variable('motor.m1', 'uint32_t')
        lg_motor.add_variable('motor.m2', 'uint32_t')
        lg_motor.add_variable('motor.m3', 'uint32_t')
        lg_motor.add_variable('motor.m4', 'uint32_t')



        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Logging Init
        logconf.start()

        # Motion Commands
        take_off_simple(scf)
        #time.sleep(10)


        # with open('v_pid.csv', 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['vz', 'vzctrl', 'z','vx', 'vxctrl', 'x','vy', 'vyctrl', 'y'])   # Header
        #     for xi, yi, zi, ai, bi, ci, di, ei, fi in zip(vzList, vzctrlList, zList,vxList, vxctrlList, xList,vyList, vyctrlList, yList):
        #         writer.writerow([xi, yi, zi, ai, bi, ci, di, ei, fi])

        # with open('pitch_pid.csv', 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['vx', 'vxctrl', 'x','vy', 'vyctrl', 'y'])   # Header
        #     for xi, yi, zi, ai, bi, ci in zip(vxList, vxctrlList, xList,vyList, vyctrlList, yList):
        #         writer.writerow([xi, yi, zi, ai, bi, ci])\

        with open('pitch_pid.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['pitch', 'cmd', 'vx', 'vxctrl'])   # Header
            for xi, yi, ai, bi in zip(pitchList, cmdList, vxList, vxctrlList):
                writer.writerow([xi, yi, ai, bi])

        # Logging Stop
        logconf.stop()

        