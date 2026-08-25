"""
ecg_plot.py  —  Live ECG plotter for Wokwi ESP32 ECG simulator
================================================================

NO EXTERNAL SERIAL LIBRARY NEEDED.
Connects directly via TCP socket (rfc2217 = plain TCP on port 4000).
Only requires:  pip install matplotlib

Usage:
  1. Start Wokwi simulation
  2. In Wokwi: click the serial monitor panel so it activates
  3. Run:  py ecg_plot.py

FILE MODE (no Wokwi needed — plot a saved serial log):
  Set READ_FROM_FILE = True and FILE_PATH = "your_log.txt"
"""

import sys
import os
import socket
import time
import re
from collections import deque


# ============================================================
# SETTINGS
# ============================================================

HOST           = "localhost"
PORT           = 4000            # Wokwi rfc2217 port (plain TCP)

READ_FROM_FILE = False           # True = replay a saved .txt log
FILE_PATH      = "serial_log.txt"

MAX_POINTS     = 600             # samples in scrolling window (6 s at 100 Hz)
PLOT_INTERVAL  = 0.033           # seconds between redraws (~30 fps)
FIXED_Y        = None            # e.g. (-150, 150) to lock Y; None = auto
DEBUG_RAW      = False


# ============================================================
# MATPLOTLIB CHECK
# ============================================================

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    print()
    print("ERROR: matplotlib not found.")
    print("  Fix:  py -m pip install matplotlib")
    sys.exit(1)


# ============================================================
# PARSER  — handles all three firmware serial formats
# ============================================================

_RE_LABELED = re.compile(r'(\w+):([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)')

def parse_line(text: str):
    """Returns (ecg_mV: float|None, bpm: float|None)."""
    text = text.strip()
    if not text:
        return None, None

    if DEBUG_RAW:
        print(f"  RAW: {text!r}")

    # Format 1 — labeled  "ECG_mV:12.34 Zero:0 MaxLimit:200 MinLimit:-200"
    if ':' in text:
        pairs = {k: float(v) for k, v in _RE_LABELED.findall(text)}
        ecg = pairs['ECG_mV'] if 'ECG_mV' in pairs else pairs.get('ECG')
        bpm = pairs['BPM']    if 'BPM'    in pairs else pairs.get('HR')
        return ecg, bpm

    # Format 2 — bare float  "12.34"
    try:
        return float(text), None
    except ValueError:
        pass

    # Format 3 — CSV  "12.34,72"
    parts = text.split(',')
    try:
        ecg = float(parts[0].strip())
        bpm = float(parts[1].strip()) if len(parts) > 1 else None
        return ecg, bpm
    except (ValueError, IndexError):
        pass

    return None, None


# ============================================================
# PLOTTER-SIDE DC REMOVAL
# Fast IIR tracker so display is centred from sample 1
# regardless of ADC startup state in the firmware.
# ============================================================

_dc_init  = False
_dc_level = 0.0
_DC_ALPHA = 0.05

def remove_dc(value: float) -> float:
    global _dc_init, _dc_level
    if not _dc_init:
        _dc_level = value
        _dc_init  = True
    else:
        _dc_level = (1.0 - _DC_ALPHA) * _dc_level + _DC_ALPHA * value
    return value - _dc_level


# ============================================================
# Y AUTO-SCALE — 2nd–98th percentile so spikes don't crush trace
# ============================================================

def autoscale_y(y_list):
    if len(y_list) < 20:
        return -50, 50
    s      = sorted(y_list)
    n      = len(s)
    lo     = s[max(0,     int(n * 0.02))]
    hi     = s[min(n - 1, int(n * 0.98))]
    span   = max(hi - lo, 5.0)
    margin = span * 0.30
    return lo - margin, hi + margin


# ============================================================
# TCP READER  — wraps a socket as a line iterator
# No pyserial needed. rfc2217 is plain TCP; the Wokwi endpoint
# sends raw ASCII bytes just like a serial port would.
# ============================================================

class TCPLineReader:
    """Reads newline-delimited text lines from a TCP socket."""

    RECV_SIZE = 4096

    def __init__(self, host, port, timeout=2.0):
        self._buf = b''
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(timeout)
        try:
            self._sock.connect((host, port))
        except ConnectionRefusedError:
            print()
            print(f"ERROR: Could not connect to {host}:{port}")
            print()
            print("  Make sure:")
            print("    1. Wokwi simulation is running (press Play)")
            print("    2. The serial monitor panel is open in Wokwi")
            print("    3. Port 4000 is correct (check Wokwi serial settings)")
            print()
            print("  Or set READ_FROM_FILE = True to plot a saved log.")
            sys.exit(1)
        except OSError as e:
            print(f"ERROR: Socket error: {e}")
            sys.exit(1)
        self._sock.settimeout(0.05)   # non-blocking reads after connect

    def readline(self):
        """Returns one decoded line, or None if no complete line yet."""
        while True:
            newline = self._buf.find(b'\n')
            if newline != -1:
                line = self._buf[:newline + 1].decode('utf-8', errors='ignore')
                self._buf = self._buf[newline + 1:]
                return line
            # Need more data
            try:
                chunk = self._sock.recv(self.RECV_SIZE)
                if not chunk:
                    return None    # server closed connection
                self._buf += chunk
            except socket.timeout:
                return None        # no data right now
            except OSError:
                return None

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


# ============================================================
# FILE READER  — replays a saved serial log
# ============================================================

class FileLineReader:
    def __init__(self, path):
        if not os.path.exists(path):
            print(f"ERROR: file not found: {path}")
            sys.exit(1)
        self._f = open(path, 'r', encoding='utf-8', errors='ignore')
        print(f"[ECG Plotter] File mode → {path}")

    def readline(self):
        line = self._f.readline()
        if not line:
            self._f.seek(0)     # loop
            return None
        time.sleep(0.01)        # throttle to ~100 Hz
        return line

    def close(self):
        self._f.close()


# ============================================================
# FIGURE
# ============================================================

plt.ion()
plt.style.use('dark_background')

fig = plt.figure(figsize=(13, 5))
fig.canvas.manager.set_window_title("Wokwi ECG Monitor")

ax = fig.add_subplot(111)
(ecg_line,) = ax.plot([], [], color='#00e676', linewidth=1.4, antialiased=True)
ax.axhline(0, color='#ffffff', linewidth=0.6, alpha=0.35, linestyle='--')

ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(-50, 50)
ax.set_xlabel("Samples  (100 Hz → 1 div = 10 ms)", color='#aaaaaa', fontsize=9)
ax.set_ylabel("ECG  (mV)",                          color='#aaaaaa', fontsize=9)
ax.tick_params(colors='#888888', labelsize=8)
ax.grid(True, alpha=0.18, linestyle='--')
for sp in ax.spines.values():
    sp.set_edgecolor('#333333')

mode_label  = "FILE" if READ_FROM_FILE else "LIVE"
status_text = ax.text(
    0.01, 0.97,
    f"[{mode_label}] Connecting...",
    transform=ax.transAxes, fontsize=8,
    color='#aaaaaa', verticalalignment='top',
    fontfamily='monospace'
)
fig.tight_layout(pad=1.4)


# ============================================================
# MAIN LOOP
# ============================================================

if READ_FROM_FILE:
    reader = FileLineReader(FILE_PATH)
else:
    print(f"[ECG Plotter] Connecting to {HOST}:{PORT} ...")
    reader = TCPLineReader(HOST, PORT)
    print("[ECG Plotter] Connected.")

ecg_buf        = deque(maxlen=MAX_POINTS)
bpm_latest     = None
sample_count   = 0
last_plot_time = time.monotonic()

try:
    while plt.fignum_exists(fig.number):

        text = reader.readline()

        if text is None:
            plt.pause(0.001)
            continue

        ecg_val, bpm_val = parse_line(text)
        if ecg_val is None:
            continue

        ecg_buf.append(remove_dc(ecg_val))
        sample_count += 1
        if bpm_val is not None:
            bpm_latest = bpm_val

        # Redraw at ~30 fps
        now = time.monotonic()
        if now - last_plot_time < PLOT_INTERVAL:
            continue
        last_plot_time = now

        y = list(ecg_buf)
        x = list(range(len(y)))
        ecg_line.set_data(x, y)

        if FIXED_Y is not None:
            ax.set_ylim(*FIXED_Y)
        elif len(y) > 20:
            ax.set_ylim(*autoscale_y(y))

        bpm_str = f"   BPM: {bpm_latest:.0f}" if bpm_latest else ""
        status_text.set_text(f"[{mode_label}] samples: {sample_count}{bpm_str}")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("\n[ECG Plotter] Stopped.")
except Exception as e:
    print(f"\n[ECG Plotter] Error: {e!r}")
finally:
    reader.close()
    plt.ioff()
    plt.close('all')
    print("[ECG Plotter] Closed.")