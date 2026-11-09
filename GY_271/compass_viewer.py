"""
Man hinh test GY-271: kim la ban + do thi X/Y/Z, doc tu ESP32 qua cong COM.

Chay:   python compass_viewer.py            (tu tim cong COM neu chi co 1 cong)
        python compass_viewer.py --port COM12
Phim tren cua so:   c = hieu chuan 20 giay (xoay cam bien moi huong)
                    r = xoa hieu chuan
Dong cua so de nha cong COM.
"""
import argparse
import collections
import math
import re
import sys
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

LINE = re.compile(r"X=\s*(-?[\d.]+)\s+Y=\s*(-?[\d.]+)\s+Z=\s*(-?[\d.]+)\s*uT")
HEAD = re.compile(r"Huong=\s*(-?[\d.]+)")
HISTORY = 300          # so mau giu tren do thi (~60 giay)


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = self.y = self.z = self.heading = 0.0
        self.have = False
        self.calibrating = False
        self.calibrated = True
        self.message = "Dang cho du lieu..."
        self.hist = collections.deque(maxlen=HISTORY)   # (t, x, y, z)
        self.t0 = time.time()


def reader(state, ser, stop):
    while not stop.is_set():
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception as e:                      # rut cap
            state.message = "Loi cong COM: %s" % e
            return
        if not line:
            continue
        m = LINE.search(line)
        if m:
            h = HEAD.search(line)
            with state.lock:
                state.x, state.y, state.z = (float(g) for g in m.groups())
                state.calibrating = line.startswith("[HIEU CHUAN")
                state.calibrated = "chua hieu chuan" not in line
                if h:
                    state.heading = float(h.group(1))
                state.have = True
                state.hist.append((time.time() - state.t0, state.x, state.y, state.z))
        else:
            print(line)                             # thong bao cua ESP32
            if "Hieu chuan xong" in line or "Hieu chuan that bai" in line or "Da xoa" in line:
                state.message = line


def find_port():
    from serial.tools import list_ports
    ports = [p.device for p in list_ports.comports()]
    if len(ports) == 1:
        return ports[0]
    sys.exit("Chon cong bang --port (co: %s)" % (", ".join(ports) or "khong co cong nao"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    import serial
    port = args.port or find_port()
    ser = serial.Serial(port, args.baud, timeout=1)
    ser.setDTR(False)
    ser.setRTS(False)
    print("Mo %s @ %d baud" % (port, args.baud))

    state = State()
    stop = threading.Event()
    threading.Thread(target=reader, args=(state, ser, stop), daemon=True).start()

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(11, 5.5))
    ax = fig.add_subplot(1, 2, 1, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rticks([])
    ax.set_ylim(0, 1)
    ax.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
    ax.set_xticklabels(["N", "NE", "E", "SE", "S", "SW", "W", "NW"], color="#7f7")
    ax.set_facecolor("#021008")
    ax.grid(color="#2a5", alpha=0.35)
    needle, = ax.plot([0, 0], [0, 0.9], color="#ff5252", lw=4, solid_capstyle="round")
    tail, = ax.plot([0, 0], [0, -0.35], color="#cccccc", lw=4, solid_capstyle="round")
    title = ax.set_title("", color="#7f7", pad=18, fontsize=15)

    gx = fig.add_subplot(1, 2, 2)
    lines = {k: gx.plot([], [], label=k, lw=1.6)[0] for k in ("X", "Y", "Z")}
    gx.set_xlabel("giay")
    gx.set_ylabel("microTesla")
    gx.grid(alpha=0.25)
    gx.legend(loc="upper left")
    fig.subplots_adjust(left=0.05, right=0.97, wspace=0.4, bottom=0.1)
    info = fig.text(0.02, 0.02, "", color="#7f7", family="monospace")

    def update(_):
        with state.lock:
            have, hd = state.have, state.heading
            x, y, z = state.x, state.y, state.z
            hist = list(state.hist)
            cal, ing = state.calibrated, state.calibrating
        if not have:
            title.set_text(state.message)
            return ()
        th = math.radians(hd)
        needle.set_data([th, th], [0, 0.9])
        tail.set_data([th, th], [0, -0.35])
        title.set_text("%.1f deg" % hd + ("   HIEU CHUAN..." if ing else ""))
        title.set_color("#ffd54f" if ing else "#7f7")
        if hist:
            a = np.array(hist)
            for i, k in enumerate("XYZ"):
                lines[k].set_data(a[:, 0], a[:, i + 1])
            gx.set_xlim(max(0, a[-1, 0] - 60), a[-1, 0] + 1)
            lo, hi = a[:, 1:].min(), a[:, 1:].max()
            gx.set_ylim(lo - 5, hi + 5)
        mag = math.sqrt(x * x + y * y + z * z)
        info.set_text("X=%.1f Y=%.1f Z=%.1f uT   |B|=%.1f uT   %s   [c]=hieu chuan [r]=xoa   %s" % (
            x, y, z, mag, "da hieu chuan" if cal else "CHUA hieu chuan", state.message if state.message != "Dang cho du lieu..." else ""))
        return ()

    def on_key(ev):
        if ev.key in ("c", "C"):
            ser.write(b"c\n")
            state.message = "Dang hieu chuan: xoay cam bien moi huong trong 20 giay"
        elif ev.key in ("r", "R"):
            ser.write(b"r\n")

    fig.canvas.mpl_connect("key_press_event", on_key)
    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        ser.close()


if __name__ == "__main__":
    main()
