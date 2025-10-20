#!/usr/bin/env python3
import logging
import time
from contextlib import contextmanager

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ====== CONFIG ======
URI = 'udp://172.20.10.2:2390'  # your ESP32-S3 UDP endpoint
DEFAULT_Z = 0.3                 # meters (ignored if firmware lacks Z control; fine for LED rig)
SETPOINT_HZ = 50                # send rate; 50–100 Hz is typical
SQUARE_SIDE_S = 2.0             # seconds per side for velocity pattern
VEL = 0.2                       # m/s for velocity pattern
# ====================

logging.basicConfig(level=logging.ERROR)


@contextmanager
def arm_cf(cf: Crazyflie):
    """Arm on enter, disarm on exit (ignore if not supported)."""
    try:
        # Newer firmwares (and your ESP port) may expose platform arming:
        cf.platform.send_arming_request(True)
        time.sleep(0.5)
    except Exception:
        pass
    try:
        yield
    finally:
        try:
            cf.platform.send_arming_request(False)
            time.sleep(0.5)
        except Exception:
            pass


def steady_loop(duration_s, rate_hz, fn):
    """Call fn() at ~rate_hz for duration_s seconds."""
    period = 1.0 / rate_hz
    t_end = time.perf_counter() + duration_s
    next_t = time.perf_counter()
    while time.perf_counter() < t_end:
        fn()
        next_t += period
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            # fell behind; reset next_t to now to avoid drift explosion
            next_t = time.perf_counter()


def try_high_level_takeoff_land(scf: SyncCrazyflie) -> bool:
    """
    Try to use the high-level commander (takeoff/land).
    Returns True if it worked; False to fall back to velocity mode.
    """
    try:
        # Enable HLC if present
        scf.cf.param.set_value('commander/enHighLevel', '1')
        hlc = scf.cf.high_level_commander

        # Simple takeoff → hover → land
        hlc.takeoff(height=DEFAULT_Z, velocity=0.5)
        time.sleep(3.0)
        hlc.land(velocity=0.5)
        time.sleep(2.0)
        return True
    except Exception:
        return False


def velocity_square_pattern(scf: SyncCrazyflie):
    """
    Drive a simple square in world frame using velocity setpoints.
    If your firmware ignores Z, LEDs will still blink on commands—good enough for a comms test.
    """
    cmd = scf.cf.commander

    def send(vx, vy, yawrate=0.0, z=DEFAULT_Z):
        # Try world-frame velocity first; fall back to hover setpoint if needed
        try:
            cmd.send_velocity_world_setpoint(vx, vy, yawrate, z)
        except Exception:
            # Hover setpoint exists on older stacks; vx,vy are ‘forward/right’ in m/s, zDistance in m
            cmd.send_hover_setpoint(vx, vy, yawrate, z)

    # +X
    steady_loop(SQUARE_SIDE_S, SETPOINT_HZ, lambda: send(VEL, 0.0))
    # +Y
    steady_loop(SQUARE_SIDE_S, SETPOINT_HZ, lambda: send(0.0, VEL))
    # -X
    steady_loop(SQUARE_SIDE_S, SETPOINT_HZ, lambda: send(-VEL, 0.0))
    # -Y
    steady_loop(SQUARE_SIDE_S, SETPOINT_HZ, lambda: send(0.0, -VEL))

    # Stop (zero command) briefly
    steady_loop(0.5, SETPOINT_HZ, lambda: send(0.0, 0.0))


def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf, arm_cf(scf.cf):
        # First try high-level (nice if your firmware supports it)
        if try_high_level_takeoff_land(scf):
            return

        # Fallback: fire velocity commands in a square
        velocity_square_pattern(scf)

        # Best-effort land/stop (if HLC wasn’t available)
        try:
            scf.cf.high_level_commander.land(velocity=0.5)
            time.sleep(1.5)
        except Exception:
            # Send a few zero setpoints to “idle” the controller
            cmd = scf.cf.commander
            steady_loop(0.5, SETPOINT_HZ, lambda: cmd.send_hover_setpoint(0.0, 0.0, 0.0, DEFAULT_Z))

if __name__ == '__main__':
    main()
