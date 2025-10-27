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

yawReal = []
yawSP = []

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


def const_thrust(scf):
    print("Take off!")
    cf = scf.cf

    for i in range(50):
        print(f"{i}")
        cf.commander.send_setpoint(0, 0, 0, 20000) # 
        time.sleep(0.2)

    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    print("stop")
    scf.cf.commander.send_stop_setpoint()

def hover(scf):
    print("Take off!")
    cf = scf.cf

    for i in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, i/20) # slowly increase height
        time.sleep(0.1)

    print("hover")
    for i in range(20):
        cf.commander.send_position_setpoint(0,0,0.5,0) # x y z yaw
        time.sleep(0.1)

    print("turn")
    for i in range(20):
        cf.commander.send_position_setpoint(0,0,0.5,90) # x y z yaw
        time.sleep(0.1)

    print("Descend")
    for i in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, (10-i)/20) # slowly decrease height
        time.sleep(0.1)

    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    print("stop")
    scf.cf.commander.send_stop_setpoint()

def log_pos_callback(timestamp, data, logconf):
    #print("Height: %d \t Voltage: %d", data['stateEstimate.z'], data['pm.vbat'])
    #position_estimate[0] = data['stateEstimate.x']
    #position_estimate[0] = data['stateEstimate.y']
    yawReal.append(data['stateEstimate.yaw'])
    yawSP.append(data['controller.yaw'])

def log_mot_callback(timestamp, data, logconf):

    m1 = data["motor.m1"]
    m2 = data["motor.m2"]
    m3 = data["motor.m3"]
    m4 = data["motor.m4"]

    print(f"M1: {m1} \t M2: {m2} \t M3: {m3} \t M4: {m4}")




if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #scf.cf.commander.set_client_xmode(True)
        #scf.cf.param.set_value('commander.enHighLevel', '1')

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('motor.m1', 'uint32_t')
        logconf.add_variable('motor.m2', 'uint32_t')
        logconf.add_variable('motor.m3', 'uint32_t')
        logconf.add_variable('motor.m4', 'uint32_t')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_mot_callback)

        scf.cf.platform.send_arming_request(True)

        #logconf.start()

        const_thrust(scf)
        
        # OAdata = [[yawReal],[yawSP]]
        # with open('yaw.csv', 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['actual', 'set'])   # Header
        #     for xi, yi in zip(yawReal, yawSP):
        #         writer.writerow([xi, yi])

        # Logging Stop
        #logconf.stop()
        
            