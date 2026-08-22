from collections import deque

import matplotlib.pyplot as plt
import serial


PORT = "rfc2217://localhost:4000"
BAUD = 115200
MAX_POINTS = 600

# The ESP32 firmware sends ECG values in millivolts (mV), not microvolts (uV).
# This is why the waveform looked flat: the plot was scaled for uV while the data
# was actually in mV.
ECG_MIN_MV = -500
ECG_MAX_MV = 700

DEBUG_RAW = True


def parse_sample(text):
    parts = [part.strip() for part in text.split(",")]
    if len(parts) < 2:
        return None

    try:
        ecg_mv = float(parts[0])
        bpm = float(parts[1]) if parts[1] else 0.0
    except ValueError:
        return None

    # Lead-off mode sends "0,0"
    if ecg_mv == 0.0 and bpm == 0.0:
        return 0.0, 0.0

    # Reject obvious garbage / boot noise.
    if abs(ecg_mv) > 2000.0:
        return None

    return ecg_mv, bpm


ser = serial.serial_for_url(PORT, baudrate=BAUD, timeout=1)
ecg_data = deque(maxlen=MAX_POINTS)

plt.ion()
fig, ax = plt.subplots(figsize=(12, 5))
line, = ax.plot([], [], linewidth=1.5)

ax.set_title("Live ECG - Wokwi")
ax.set_xlabel("Samples")
ax.set_ylabel("ECG (mV)")
ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(ECG_MIN_MV, ECG_MAX_MV)
ax.grid(True, alpha=0.35)
ax.axhline(0, color="black", linewidth=0.8, alpha=0.45)

try:
    while True:
        raw_line = ser.readline()
        if not raw_line:
            continue

        text = raw_line.decode("utf-8", errors="ignore").strip()
        if not text:
            continue

        if DEBUG_RAW:
            print(f"RAW: {text}")

        sample = parse_sample(text)
        if sample is None:
            continue

        ecg_mv, bpm = sample
        ecg_data.append(ecg_mv)

        if len(ecg_data) < 3:
            continue

        line.set_data(range(len(ecg_data)), ecg_data)
        if bpm is not None and bpm > 0:
            ax.set_title(f"Live ECG - Wokwi   HR: {bpm:.0f} BPM")
        else:
            ax.set_title("Live ECG - Wokwi   Lead off")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Plotter stopped.")

finally:
    ser.close()
    plt.close()
