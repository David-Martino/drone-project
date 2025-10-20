#!/usr/bin/env python3
import logging
import time
import csv
from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# ---- Config ----
URI = 'udp://172.20.10.2:2390'
LOG_PERIOD_MS = 10                 # requested period
PRINT_EVERY_S = 2.0                # print stats every N seconds
MAX_SAMPLES = 0                    # 0 = run forever; otherwise stop after N samples
CSV_PATH = None                    # e.g. "latency_samples.csv" to save raw deltas
RESERVOIR_SIZE = 5000              # how many deltas to keep for percentiles
# ----------------

logging.basicConfig(level=logging.ERROR)

class OnlineStats:
    """Welford’s online mean/stddev + min/max + loss estimate."""
    def __init__(self, period_ms):
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0
        self.minv = float('inf')
        self.maxv = float('-inf')
        self.period_ms = float(period_ms)
        self.lost = 0
        self.prev_recv_ms = None
        self.reservoir = deque(maxlen=RESERVOIR_SIZE)
        self.start_time = time.perf_counter()
        self.last_print = self.start_time

    def add(self, recv_ms):
        """recv_ms = receive timestamp in ms (monotonic)."""
        if self.prev_recv_ms is not None:
            delta = recv_ms - self.prev_recv_ms  # ms
            # Update streaming stats
            self.n += 1
            delta_v = delta - self.mean
            self.mean += delta_v / self.n
            self.M2 += delta_v * (delta - self.mean)
            self.minv = min(self.minv, delta)
            self.maxv = max(self.maxv, delta)
            self.reservoir.append(delta)

            # Estimate packet loss vs desired period
            expected = max(1, int(round(delta / self.period_ms)))  # #periods elapsed
            if expected > 1:
                self.lost += (expected - 1)
        # first sample just initializes prev
        self.prev_recv_ms = recv_ms

    def stddev(self):
        return (self.M2 / (self.n - 1)) ** 0.5 if self.n > 1 else 0.0

    def rate_hz(self):
        elapsed = max(1e-6, time.perf_counter() - self.start_time)
        # n is number of deltas; packets received ≈ n+1 after first
        pkts = self.n + 1 if self.n > 0 else 0
        return pkts / elapsed

    def percentiles(self, qs=(50, 90, 99)):
        if not self.reservoir:
            return {q: None for q in qs}
        arr = sorted(self.reservoir)
        out = {}
        for q in qs:
            k = (q / 100.0) * (len(arr) - 1)
            f = int(k)
            c = min(f + 1, len(arr) - 1)
            if f == c:
                out[q] = arr[f]
            else:
                out[q] = arr[f] + (k - f) * (arr[c] - arr[f])
        return out

    def maybe_print(self):
        now = time.perf_counter()
        if (now - self.last_print) >= PRINT_EVERY_S and self.n > 1:
            p = self.percentiles()
            print(
                f"[stats] n={self.n} lost≈{self.lost}  "
                f"rate={self.rate_hz():.1f} Hz  "
                f"Δt(ms): min={self.minv:.2f} avg={self.mean:.2f} "
                f"std={self.stddev():.2f} max={self.maxv:.2f}  "
                f"p50={p[50]:.2f} p90={p[90]:.2f} p99={p[99]:.2f}"
            )
            self.last_print = now


def run_with_stats(scf, logconf):
    stats = OnlineStats(LOG_PERIOD_MS)

    csv_writer = None
    csv_file = None
    try:
        if CSV_PATH:
            csv_file = open(CSV_PATH, "w", newline="")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["recv_time_ms", "delta_ms"])

        with SyncLogger(scf, logconf) as logger:
            prev_ms = None
            for i, log_entry in enumerate(logger):
                # log_entry[0] is a host-side timestamp from cflib (ms)
                # To reduce scheduler jitter, we’ll also stamp with monotonic:
                recv_ms = time.perf_counter() * 1000.0

                # Optional: read the actual data
                # timestamp_cf = log_entry[0]
                # data = log_entry[1]   # dict with variables
                # name = log_entry[2]

                # Update stats
                if prev_ms is not None:
                    delta_ms = recv_ms - prev_ms
                else:
                    delta_ms = None
                stats.add(recv_ms)
                prev_ms = recv_ms

                if csv_writer and delta_ms is not None:
                    csv_writer.writerow([f"{recv_ms:.3f}", f"{delta_ms:.3f}"])

                stats.maybe_print()

                if MAX_SAMPLES and i >= MAX_SAMPLES:
                    break

    finally:
        if csv_file:
            csv_file.close()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=LOG_PERIOD_MS)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        run_with_stats(scf, lg_stab)
