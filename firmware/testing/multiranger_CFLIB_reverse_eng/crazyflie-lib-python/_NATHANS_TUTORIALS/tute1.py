
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

# Literalyl just change the uri to a udp:// link and it will use the udp driver
uri = uri_helper.uri_from_env(default='udp://192.168.43.42')

def param_stab_est_callback(name, value):
    print('The crazyflie has parameter '+ name + ' set at number: ' + value)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr+"."+namestr

    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
    time.sleep(1)
    cf.param.set_value(full_name,2)
    time.sleep(1)
    cf.param.set_value(full_name,1)
    time.sleep(1)


def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)

    logconf.data_received_cb.add_callback(log_stab_callback)

    logconf.start()
    time.sleep(5)
    logconf.stop()


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

    group = "stabilizer"
    name = "estimator"

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        simple_param_async(scf, group, name)