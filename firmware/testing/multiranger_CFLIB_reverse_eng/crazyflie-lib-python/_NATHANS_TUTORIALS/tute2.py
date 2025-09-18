''' From the CFLIB doc tutorials, but trying to modify to use the UDP Driver'''

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Literalyl just change the uri to a udp:// link and it will use the udp driver
uri = uri_helper.uri_from_env(default='udp://192.168.43.42')

def simple_log(scf, logconf):
    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:
            
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            #break # what we breaking for tho

def simple_connect():
    print("Oh yeah, heck yeah, let's gooo, I'm connected")
    time.sleep(3)
    print("Imma head out")

if __name__ == '__main__':
    # Initialize low-level dri
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        simple_log(scf, lg_stab)