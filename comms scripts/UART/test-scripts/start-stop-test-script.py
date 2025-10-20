#!/usr/bin/env python3
import logging
import time
from contextlib import contextmanager

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ========= CONFIG =========
URI               = 'udp://172.20.10.2:2390'
CYCLES            = 5
TAKEOFF_HEIGHT_M  = 0.25     # zDistance for hover setpoint (m)
ASCENT_TIME_S     = 2.0
HOVER_TIME_S      = 3.0
DESCENT_TIME_S    = 1.0      # short descent phase before idle
PAUSE_BETWEEN_S   = 1.5      # motors OFF during this window
SETPOINT_HZ       = 50
YAWRATE           = 0.0
VX = VY           = 0.0
# ==========================

logging.basicConfig(level=logging.ERROR)

def now():
    return f"{time.strftime('%H:%M:%S')}.{int((time.time()%1)*1000):03d}"

@contextmanager
def maybe_arm(cf: Crazyflie):
    try:
        print(f"[{now()}] ARM request: True")
        cf.platform.send_arming_request(True)
        time.sleep(0.4)
    except Exception as e:
        print(f"[{now()}] ARM not supported or failed: {e}")
    try:
        yield
    finally:
        try:
            print(f"[{now()}] ARM request: False")
            cf.platform.send_arming_request(False)
            time.sleep(0.4)
        except Exception as e:
            print(f"[{now()}] DISARM not supported or failed: {e}")

def steady_loop(duration_s, rate_hz, fn):
    period = 1.0 / rate_hz
    t_end = time.perf_counter() + duration_s
    nxt = time.perf_counter()
    while time.perf_counter() < t_end:
        fn()
        nxt += period
        slp = nxt - time.perf_counter()
        if slp > 0:
            time.sleep(slp)
        else:
            nxt = time.perf_counter()

def send_hover(cmd, z):
    cmd.send_hover_setpoint(VX, VY, YAWRATE, z)

def send_idle(cmd):
    # zero attitude, zero thrust -> motors off on real CF; for your LED rig: LEDs should go off
    cmd.send_setpoint(0.0, 0.0, 0.0, 0)

def do_cycle(scf: SyncCrazyflie):
    cmd = scf.cf.commander

    print(f"[{now()}] TAKEOFF (hover z={TAKEOFF_HEIGHT_M:.2f} m)")
    steady_loop(ASCENT_TIME_S, SETPOINT_HZ, lambda: send_hover(cmd, TAKEOFF_HEIGHT_M))

    print(f"[{now()}] HOVER start ({HOVER_TIME_S:.1f}s)")
    steady_loop(HOVER_TIME_S, SETPOINT_HZ, lambda: send_hover(cmd, TAKEOFF_HEIGHT_M))
    print(f"[{now()}] HOVER end")

    print(f"[{now()}] DESCEND (hover z≈0)")
    steady_loop(DESCENT_TIME_S, SETPOINT_HZ, lambda: send_hover(cmd, 0.0))

    # *** Motors off / LEDs off phase ***
    print(f"[{now()}] PAUSE/IDLE (motors OFF) {PAUSE_BETWEEN_S:.1f}s")
    # Some firmwares like also sending an explicit stop frame before idling
    try:
        scf.cf.commander.send_stop_setpoint()
    except Exception:
        pass
    steady_loop(PAUSE_BETWEEN_S, SETPOINT_HZ, lambda: send_idle(cmd))
    print(f"[{now()}] Cycle complete\n")

def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf, maybe_arm(scf.cf):
        try:
            for i in range(1, CYCLES + 1):
                print(f"[{now()}] ===== Cycle {i}/{CYCLES} =====")
                do_cycle(scf)
        except KeyboardInterrupt:
            print(f"[{now()}] Ctrl-C — sending idle setpoints…")
        finally:
            # Send a short burst of idle to ensure full stop
            cmd = scf.cf.commander
            try:
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass
            steady_loop(0.5, SETPOINT_HZ, lambda: send_idle(cmd))
            print(f"[{now()}] Done.")

if __name__ == '__main__':
    main()
