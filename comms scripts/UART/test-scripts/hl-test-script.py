#!/usr/bin/env python3
import logging
import time
from contextlib import contextmanager

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort  # for a raw STOP poke, optional

# ========= CONFIG =========
URI               = 'udp://172.20.10.2:2390'
CYCLES            = 3
TAKEOFF_HEIGHT_M  = 0.25         # absolute height (m)
TAKEOFF_VEL_MPS   = 0.20         # only used to derive duration
HOVER_TIME_S      = 3.0
POST_LAND_WAIT_S  = 1.0
USE_STOP_INSTEAD_OF_LAND = False # True = send STOP instead of LAND
PRESTOP_POKE      = True         # send a raw STOP packet before first cycle
RESET_HL_PARAM_AT_END = True     # set commander/enHighLevel back to 0 on exit
# ==========================

logging.basicConfig(level=logging.ERROR)

def now():
    return f"{time.strftime('%H:%M:%S')}.{int((time.time()%1)*1000):03d}"

# --- Console tap so you see your DEBUG_PRINTD() messages ---
def _console_cb(text):
    print(f"[{now()}] [CF] {text}", end='', flush=True)

@contextmanager
def console_tap(cf: Crazyflie):
    try:
        cf.console.receivedChar.add_callback(_console_cb)
    except Exception as e:
        print(f"[{now()}] Console tap unavailable: {e}")
    try:
        yield
    finally:
        try:
            cf.console.receivedChar.remove_callback(_console_cb)
        except Exception:
            pass

# --- Optional arming wrapper (no-op if unsupported) ---
@contextmanager
def maybe_arm(cf: Crazyflie):
    try:
        print(f"[{now()}] ARM request: True")
        cf.platform.send_arming_request(True)
        time.sleep(0.3)
    except Exception as e:
        print(f"[{now()}] ARM not supported or failed: {e}")
    try:
        yield
    finally:
        try:
            print(f"[{now()}] ARM request: False")
            cf.platform.send_arming_request(False)
            time.sleep(0.3)
        except Exception as e:
            print(f"[{now()}] DISARM not supported or failed: {e}")

def seconds_for(height_m, vel_mps, base_min=1.0, extra=0.5):
    return max(base_min, height_m / max(vel_mps, 0.05)) + extra

def raw_hl_stop(cf: Crazyflie):
    pk = CRTPPacket()
    pk.port = CRTPPort.SETPOINT_HL
    pk.data = bytes([3, 0])  # COMMAND_STOP=3, groupMask=0 (ALL_GROUPS)
    print(f"[{now()}] RAW HL STOP poke")
    cf.send_packet(pk)

def enable_high_level(cf: Crazyflie, enable: bool = True):
    val = '1' if enable else '0'
    try:
        print(f"[{now()}] set commander/enHighLevel={val}")
        cf.param.set_value('commander.enHighLevel', val)
        time.sleep(0.1)
        return True
    except KeyError as e:
        print(f"[{now()}] HL param not in TOC, trying raw-by-name: {e}")
        try:
            # 0x08 = uint8_t
            cf.param.set_value_raw('commander.enHighLevel', 0x08, 1 if enable else 0)
            print(f"[{now()}] force (raw) commander/enHighLevel={val}")
            time.sleep(0.1)
            return True
        except Exception as e2:
            print(f"[{now()}] raw set_value failed: {e2}")
            return False
    except Exception as e:
        print(f"[{now()}] Failed to set HL param: {e}")
        return False


def do_hl_cycle(scf: SyncCrazyflie, use_stop=False):
    hlc = scf.cf.high_level_commander

    try:
        print(f"[{now()}] HL STOP (pre-cycle)")
        hlc.stop()
        time.sleep(0.2)
    except Exception as e:
        print(f"[{now()}] Pre-cycle STOP not available: {e}")

    t_rise = seconds_for(TAKEOFF_HEIGHT_M, TAKEOFF_VEL_MPS)
    print(f"[{now()}] TAKEOFF height={TAKEOFF_HEIGHT_M:.2f} m  duration={t_rise:.2f} s")
    hlc.takeoff(TAKEOFF_HEIGHT_M, t_rise)
    time.sleep(t_rise)

    print(f"[{now()}] HOVER start ({HOVER_TIME_S:.1f}s)")
    time.sleep(HOVER_TIME_S)
    print(f"[{now()}] HOVER end")

    if use_stop:
        print(f"[{now()}] STOP (planner stop)")
        hlc.stop()
    else:
        t_fall = seconds_for(TAKEOFF_HEIGHT_M, TAKEOFF_VEL_MPS)
        print(f"[{now()}] LAND to 0.00 m  duration={t_fall:.2f} s")
        hlc.land(0.0, t_fall)
        time.sleep(t_fall)

    time.sleep(POST_LAND_WAIT_S)
    print(f"[{now()}] Cycle complete\n")

def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf, console_tap(scf.cf), maybe_arm(scf.cf):
        # Enable HL mode via your new param
        hl_ok = enable_high_level(scf.cf, True)

        if PRESTOP_POKE:
            raw_hl_stop(scf.cf)
            time.sleep(0.3)

        try:
            for i in range(1, CYCLES + 1):
                print(f"[{now()}] ===== HL Cycle {i}/{CYCLES} =====")
                do_hl_cycle(scf, use_stop=USE_STOP_INSTEAD_OF_LAND)
        except KeyboardInterrupt:
            print(f"[{now()}] Ctrl-C — sending HL stop/land…")
            try:
                if USE_STOP_INSTEAD_OF_LAND:
                    scf.cf.high_level_commander.stop()
                    print(f"[{now()}] STOP sent")
                else:
                    scf.cf.high_level_commander.land(0.0, 1.0)
                    print(f"[{now()}] LAND sent")
                time.sleep(1.0)
            except Exception as e:
                print(f"[{now()}] Graceful shutdown failed: {e}")
        finally:
            try:
                scf.cf.high_level_commander.stop()
                print(f"[{now()}] Final STOP sent")
            except Exception:
                pass
            if RESET_HL_PARAM_AT_END and hl_ok:
                enable_high_level(scf.cf, False)
            print(f"[{now()}] Done.")

if __name__ == '__main__':
    main()
