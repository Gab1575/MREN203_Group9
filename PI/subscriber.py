import socket
import threading
import queue
import time
import re
import math
from collections import deque
import argparse

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Listen on all available network interfaces by default
DEFAULT_IP = "0.0.0.0"
DEFAULT_PORT = 2390


def parse_acceleration(text):
    """Extract numeric values from incoming text and return an acceleration magnitude.
    Accepts formats like: "ax,ay,az", "ax ay az", JSON-ish, or single float.
    Returns None if no numeric value found.
    """
    nums = re.findall(r"-?\d+\.?\d*(?:[eE][+-]?\d+)?", text)
    if not nums:
        return None
    vals = [float(x) for x in nums]
    if len(vals) >= 3:
        ax, ay, az = vals[0], vals[1], vals[2]
        return math.sqrt(ax * ax + ay * ay + az * az)
    return vals[0]


def udp_receiver(ip, port, q, stop_event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    sock.setblocking(True)
    print(f"Listening for UDP packets on {ip}:{port}...")
    try:
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(1024)
            except OSError:
                break
            text = data.decode("utf-8", errors="ignore").strip()
            mag = parse_acceleration(text)
            if mag is not None:
                q.put((time.time(), mag))
    finally:
        sock.close()


def main(ip, port, window_seconds, sample_rate_hz):
    q = queue.Queue()
    stop_event = threading.Event()

    receiver = threading.Thread(target=udp_receiver, args=(ip, port, q, stop_event), daemon=True)
    receiver.start()

    max_samples = max(100, int(window_seconds * sample_rate_hz))
    times = deque(maxlen=max_samples)
    mags = deque(maxlen=max_samples)

    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)
    ax.set_title("Real-time Acceleration Magnitude")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (units)")
    ax.grid(True)

    start_time = time.time()

    def update(frame):
        # Drain queue
        updated = False
        while True:
            try:
                t, m = q.get_nowait()
            except queue.Empty:
                break
            times.append(t - start_time)
            mags.append(m)
            updated = True

        if not times:
            return line,

        # Use relative time for x axis
        x = list(times)
        y = list(mags)
        line.set_data(x, y)
        ax.set_xlim(max(0, x[-1] - window_seconds), x[-1])

        # auto-scale y with some padding
        ymin = min(y)
        ymax = max(y)
        if ymin == ymax:
            ax.set_ylim(ymin - 0.5, ymax + 0.5)
        else:
            padding = (ymax - ymin) * 0.1
            ax.set_ylim(ymin - padding, ymax + padding)

        return line,

    ani = animation.FuncAnimation(fig, update, interval=int(1000 / max(1, sample_rate_hz)), blit=True)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UDP subscriber that plots acceleration in real time")
    parser.add_argument("--ip", default=DEFAULT_IP, help="IP to bind to (default: all interfaces)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to listen on")
    parser.add_argument("--window", type=float, default=10.0, help="Window length in seconds to display")
    parser.add_argument("--rate", type=float, default=50.0, help="Expected display update/sample rate (Hz)")
    args = parser.parse_args()

    main(args.ip, args.port, args.window, args.rate)