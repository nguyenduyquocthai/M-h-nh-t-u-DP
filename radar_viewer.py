"""
Ve bieu do radar RPLIDAR A1M8 tu ESP32 (firmware src/main.cpp voi PRINT_RAW_POINTS 1).

ESP32 gui moi dong:  angle_deg,distance_mm,quality
Chay:
    python radar_viewer.py                 # tu tim cong COM
    python radar_viewer.py --port COM5
    python radar_viewer.py --demo          # du lieu gia, khong can mach
    python radar_viewer.py --max 4000      # ban kinh hien thi (mm)
"""
import argparse
import math
import sys
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

BINS = 720            # do phan giai 0.5 do
MAX_AGE = 0.35        # giay: diem cu hon se bi xoa khoi man hinh


class PointStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.dist = np.zeros(BINS, dtype=np.float32)
        self.stamp = np.zeros(BINS, dtype=np.float64)
        self.count = 0
        self.rate_t = time.time()
        self.rate = 0.0
        self.revs = 0
        self.rev_t = time.time()
        self.rps = 0.0
        self._last_angle = 0.0

    def add(self, angle, dist):
        now = time.time()
        with self.lock:
            if angle < self._last_angle - 180:        # goc quay ve 0 -> vong moi
                self.revs += 1
                dt = now - self.rev_t
                if dt > 0:
                    self.rps = 0.7 * self.rps + 0.3 / dt if self.rps else 1.0 / dt
                self.rev_t = now
            self._last_angle = angle
            i = int(angle * 2) % BINS
            self.dist[i] = dist
            self.stamp[i] = now
            self.count += 1
            if now - self.rate_t >= 1.0:
                self.rate = self.count / (now - self.rate_t)
                self.count = 0
                self.rate_t = now

    def snapshot(self, max_mm):
        now = time.time()
        with self.lock:
            ok = (now - self.stamp < MAX_AGE) & (self.dist > 0) & (self.dist <= max_mm)
            ang = (np.nonzero(ok)[0] + 0.5) / 2.0
            dist = self.dist[ok].copy()
            return ang, dist, self.rate, self.rps


def serial_reader(store, port, baud, stop):
    import serial
    ser = serial.Serial(port, baud, timeout=1)
    ser.reset_input_buffer()
    while not stop.is_set():
        line = ser.readline().decode("ascii", errors="ignore").strip()
        parts = line.split(",")
        if len(parts) != 3:
            if line:
                print(line)               # in cac dong thong bao cua ESP32 (INFO, Health, ...)
            continue
        try:
            store.add(float(parts[0]), float(parts[1]))
        except ValueError:
            pass
    ser.close()


def demo_reader(store, stop):
    """Phong hinh chu nhat 4m x 3m, lidar o giua, quay 5.5 vong/s, 2000 diem/s."""
    t0 = time.time()
    n = 0
    while not stop.is_set():
        elapsed = time.time() - t0
        target = int(elapsed * 2000)
        while n < target:
            angle = (n * 360.0 / (2000 / 5.5)) % 360.0
            a = math.radians(angle)
            c, s = abs(math.cos(a)), abs(math.sin(a))
            d = min(2000 / c if c > 1e-6 else 1e9, 1500 / s if s > 1e-6 else 1e9)
            store.add(angle, d + np.random.normal(0, 8))
            n += 1
        time.sleep(0.005)


def find_port():
    from serial.tools import list_ports
    ports = [p.device for p in list_ports.comports()]
    if len(ports) == 1:
        return ports[0]
    if not ports:
        sys.exit("Khong thay cong COM nao. Cam ESP32 vao USB (hoac dung --demo).")
    sys.exit("Co nhieu cong COM: %s. Chon bang --port COMx" % ", ".join(ports))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--max", type=int, default=6000, help="ban kinh hien thi (mm)")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--save", help="(kiem thu) ve 1 khung roi luu ra file anh va thoat")
    args = ap.parse_args()

    store = PointStore()
    stop = threading.Event()
    if args.demo:
        target, targs = demo_reader, (store, stop)
    else:
        port = args.port or find_port()
        print("Mo %s @ %d baud" % (port, args.baud))
        target, targs = serial_reader, (store, port, args.baud, stop)
    th = threading.Thread(target=target, args=targs, daemon=True)
    th.start()

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")       # 0 do o phia tren
    ax.set_theta_direction(-1)            # chieu kim dong ho (giong chieu quay lidar)
    ax.set_rlim(0, args.max)
    ax.grid(color="#2a5", alpha=0.35)
    ax.set_facecolor("#021008")
    sc = ax.scatter([], [], s=6, c=[], cmap="viridis_r", vmin=0, vmax=args.max)
    info = fig.text(0.02, 0.02, "", color="#7f7", family="monospace")
    ax.set_title("RPLIDAR A1M8 (mm)", color="#7f7", pad=20)

    def update(_):
        ang, dist, rate, rps = store.snapshot(args.max)
        sc.set_offsets(np.column_stack([np.radians(ang), dist]) if len(ang) else np.empty((0, 2)))
        sc.set_array(dist)
        info.set_text("points/s: %4.0f   vong/s: %.1f   hien thi: %d diem" % (rate, rps, len(ang)))
        return sc, info

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    if args.save:
        time.sleep(1.5)
        update(0)
        fig.savefig(args.save, dpi=80)
        stop.set()
        return
    try:
        plt.show()
    finally:
        stop.set()


if __name__ == "__main__":
    main()
