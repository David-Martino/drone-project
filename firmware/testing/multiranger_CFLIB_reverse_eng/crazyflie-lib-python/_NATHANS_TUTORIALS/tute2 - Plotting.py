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
import matplotlib.pyplot as plt


uri = uri_helper.uri_from_env(default='udp://192.168.43.42')

DEFAULT_HEIGHT = 0.2
BOX_LIMIT = 0.5

position_estimate = [0,0] # Position data
#spoofx = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55]
#spoofy = [0,    0,    0,    0,    0,   0,   0,   0,   0,    0,   0]

# hl = plt.plot([], []) # Position plot

# def plot_init():
#     hl.set_ylim(-1,1)
#     hl.set_xlim(-1,1)
#     hl.show()

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

        print("Landing!")
        mc.land()
    
        # tute says it instantly closes with a land function, - ie won't just hover
    
def log_pos_callback(timestamp, data, logconf):
    #print(data['stateEstimate.z'])
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    position_estimate[0] = x
    position_estimate[1] = y

    print(f"x: {x} \t y: {y}")


    # hl.set_data(x,y)
    # plt.draw()

    #plt.plot(x,y)
    #plt.show()


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        # Logging Setup
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Logging Init
        logconf.start()

        # Motion Commands
        take_off_simple(scf)

        # Logging Stop
        logconf.stop()

        