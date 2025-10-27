
''' From the CFLIB doc tutorial but trying to modify to use the UDP Driver:
https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/
'''


import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from collections import deque

import csv


# Literalyl just change the uri to a udp:// link and it will use the udp driver
uri = uri_helper.uri_from_env(default='udp://172.20.10.2')

accpxList = []
accpyList = []


def param_stab_est_callback(name, value):
    print('The crazyflie has parameter '+ name + ' set at number: ' + value)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr+"."+namestr

    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
    time.sleep(1)
    # cf.param.set_value(full_name,2)
    # time.sleep(1)
    # cf.param.set_value(full_name,1)
    # time.sleep(1)


def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def motion_log_callback(scf, data, logconf):
    accpy = -1*data["motion.deltaX"]
    accpx = -1*data["motion.deltaY"]
    print(f"dx: {accpx} \t dy: {accpy}")

    accpxList.append(accpx)
    accpyList.append(accpy)


def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)

    logconf.data_received_cb.add_callback(motion_log_callback)

    logconf.start()
    time.sleep(10)
    logconf.stop()

    with open('SideMotion.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['dx','dy'])   # Header
        for xi, yi in zip(accpxList, accpyList):
            writer.writerow([xi, yi])

def plain_log(scf, logconf):
    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:
            data = log_entry[1]

            vy = data["stateEstimate.vy"]
            vx = data["stateEstimate.vx"]
            print(f"vx: {vx:.3f} \t vy: {vy:.3f}")



def simple_log(scf, logconf):
    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:
            
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            #break # what we breaking for tho



def plot_velocity(scf, logconf):
        v = deque(maxlen=500)
        plt.ion()
        fig = plt.figure()
        plt.plot(0,0)
        plt.show()
        ax = fig.add_subplot(111)
        line, = ax.plot(0, 0, 'b-')
        plt.xlim(-2,2)
        plt.ylim(-2,2)

        with SyncLogger(scf, logconf) as logger:
            for log_entry in logger:
                data = log_entry[1]

                vNew = data["stateEstimate.vx"]
                # print(f"({xNew},{yNew})")
                #v.append(data["stateEstimate.x"])
                
                # print(f"({x},{y})")
                line.set_data([0],[vNew])
                fig.canvas.draw()
                fig.canvas.flush_events()

def plot_position(scf, logconf):
        x = deque(maxlen=500)
        y = deque(maxlen=500)
        plt.ion()
        fig = plt.figure()
        plt.plot(x,y)
        plt.show()
        ax = fig.add_subplot(111)
        line, = ax.plot(x, y, 'b-')
        plt.xlim(-2,2)
        plt.ylim(-2,2)

        with SyncLogger(scf, logconf) as logger:
            for log_entry in logger:
                data = log_entry[1]

                xNew = data["stateEstimate.x"]
                yNew = data["stateEstimate.y"]
                # print(f"({xNew},{yNew})")
                x.append(data["stateEstimate.x"])
                y.append(data["stateEstimate.y"])
                
                # print(f"({x},{y})")
                line.set_data(x,y)
                fig.canvas.draw()
                fig.canvas.flush_events()

            #break # what we breaking for tho
        




if __name__ == '__main__':
    # Initialize low-level dri
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    lg_range = LogConfig(name='Range', period_in_ms=10)
    lg_range.add_variable('range.front', 'int16_t')
    lg_range.add_variable('range.left', 'int16_t')
    lg_range.add_variable('range.right', 'int16_t')
    lg_range.add_variable('range.back', 'int16_t')
    lg_range.add_variable('range.up', 'int16_t')
    lg_range.add_variable('range.zrange', 'int16_t')


    lg_state = LogConfig(name='State', period_in_ms=10)
    lg_state.add_variable('stateEstimate.vx', 'float')
    lg_state.add_variable('stateEstimate.vy', 'float')
    lg_state.add_variable('stateEstimate.vz', 'float')

    lg_motion = LogConfig(name='Motion', period_in_ms=50)
    lg_motion.add_variable('motion.deltaX', 'float')
    lg_motion.add_variable('motion.deltaY', 'float')

    lg_pm = LogConfig(name='Power', period_in_ms=100)
    lg_pm.add_variable('pm.vbat','float')

    lg_mag = LogConfig(name='Motion', period_in_ms=100)
    lg_mag.add_variable('mag.x','float')
    lg_mag.add_variable('mag.y','float')
    lg_mag.add_variable('mag.z','float')

    lg_acc = LogConfig(name='Acceleration', period_in_ms=10)
    lg_acc.add_variable('acc.x','float')
    lg_acc.add_variable('acc.y','float')
    lg_acc.add_variable('acc.z','float')

    lg_sys = LogConfig(name='Sys', period_in_ms=10)
    lg_sys.add_variable('sys.armed','uint8_t')

    lg_motor = LogConfig(name='thrust', period_in_ms=10)
    lg_motor.add_variable('motor.m1', 'uint32_t')
    lg_motor.add_variable('motor.m2', 'uint32_t')
    lg_motor.add_variable('motor.m3', 'uint32_t')
    lg_motor.add_variable('motor.m4', 'uint32_t')

    group = "pm"
    name = "critflag"

    

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        #simple_log(scf, lg_motion)
        #scf.cf.platform.send_arming_request(True)
        #simple_log_async(scf, lg_motion)
        simple_log(scf,lg_acc)


        #simple_param_async(scf,group,name)
        #simple_param_async(scf,"pm","landed")
