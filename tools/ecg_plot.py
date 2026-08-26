"""
Live ECG plotter for the Wokwi ESP32 ECG simulator.

The ESP32 sends one ECG value in mV per line.

Example:
    0.52
    4.81
    25.33
    271.40

Run:
    py -m pip install matplotlib
    py ecg_plot.py
"""

import socket
import sys
import time
from collections import deque

import matplotlib.pyplot as plt


HOST = "localhost"
PORT = 4000

MAX_POINTS = 600
PLOT_INTERVAL = 0.033

# Fixed range prevents occasional noise spikes from crushing the
# ECG waveform.
Y_LIMIT = 350.0


class TCPLineReader:
    def __init__(self, host, port):
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )
        self.sock.settimeout(2.0)

        try:
            self.sock.connect((host, port))
        except OSError as exc:
            print(f"Could not connect to {host}:{port}: {exc}")
            print("Start Wokwi first and make sure the serial monitor is active.")
            sys.exit(1)

        self.sock.settimeout(0.05)
        self.buffer = b""

    def readline(self):
        while True:
            newline = self.buffer.find(b"\n")

            if newline >= 0:
                line = self.buffer[:newline + 1]
                self.buffer = self.buffer[newline + 1:]
                return line.decode("utf-8", errors="ignore").strip()

            try:
                chunk = self.sock.recv(4096)

                if not chunk:
                    return None

                self.buffer += chunk

            except socket.timeout:
                return None

            except OSError:
                return None

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def parse_ecg(text):
    text = text.strip()

    if not text:
        return None

    # New firmware: one bare float per line.
    try:
        value = float(text)
        if abs(value) <= 1000:
            return value
    except ValueError:
        pass

    # Backward-compatible CSV: ECG,BPM
    if "," in text:
        try:
            value = float(text.split(",")[0].strip())
            if abs(value) <= 1000:
                return value
        except (ValueError, IndexError):
            pass

    return None


plt.ion()

fig, ax = plt.subplots(
    figsize=(13, 5)
)

try:
    fig.canvas.manager.set_window_title(
        "Wokwi ECG Monitor"
    )
except Exception:
    pass

ax.set_title("Live ECG - Wokwi")
ax.set_xlabel("Samples at 100 Hz")
ax.set_ylabel("ECG (mV)")

ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(-Y_LIMIT, Y_LIMIT)

ax.grid(
    True,
    alpha=0.25,
    linestyle="--",
)

ax.axhline(
    0,
    linewidth=0.8,
    alpha=0.5,
)

(line,) = ax.plot(
    [],
    [],
    linewidth=1.6,
)

status = ax.text(
    0.01,
    0.97,
    "Connecting...",
    transform=ax.transAxes,
    fontsize=9,
    verticalalignment="top",
)

reader = TCPLineReader(
    HOST,
    PORT,
)

ecg = deque(maxlen=MAX_POINTS)

sample_count = 0
last_redraw = time.monotonic()

try:
    while plt.fignum_exists(fig.number):
        text = reader.readline()

        if text is None:
            plt.pause(0.001)
            continue

        value = parse_ecg(text)

        if value is None:
            continue

        ecg.append(value)
        sample_count += 1

        now = time.monotonic()

        if now - last_redraw < PLOT_INTERVAL:
            continue

        last_redraw = now

        y = list(ecg)
        x = list(range(len(y)))

        line.set_data(x, y)

        ax.set_xlim(
            0,
            MAX_POINTS,
        )
        ax.set_ylim(
            -Y_LIMIT,
            Y_LIMIT,
        )

        status.set_text(
            f"[LIVE] samples: {sample_count}"
        )

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("\nECG plotter stopped.")

finally:
    reader.close()
    plt.ioff()
    plt.close("all")