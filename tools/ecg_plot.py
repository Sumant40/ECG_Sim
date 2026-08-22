from collections import deque

import matplotlib.pyplot as plt
import serial


PORT = "rfc2217://localhost:4000"
BAUD = 115200
MAX_POINTS = 600

ECG_MIN_UV = -1800
ECG_MAX_UV = 2200


def parse_sample(text):
    parts = [part.strip() for part in text.split(",")]
    if len(parts) < 1:
        return None

    try:
        ecg_uv = float(parts[0])
        bpm = float(parts[1]) if len(parts) > 1 else None
    except ValueError:
        return None

    # Reject boot logs and old binary ADC output such as 10110111111.
    if not (ECG_MIN_UV * 2 <= ecg_uv <= ECG_MAX_UV * 2):
        return None

    return ecg_uv, bpm


ser = serial.serial_for_url(PORT, baudrate=BAUD, timeout=1)
ecg_data = deque(maxlen=MAX_POINTS)

plt.ion()
fig, ax = plt.subplots(figsize=(12, 5))
line, = ax.plot([], [], linewidth=1.5)

ax.set_title("Live ECG - Wokwi")
ax.set_xlabel("Samples")
ax.set_ylabel("ECG (uV)")
ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(ECG_MIN_UV, ECG_MAX_UV)
ax.grid(True, alpha=0.35)
ax.axhline(0, color="black", linewidth=0.8, alpha=0.45)

try:
    while True:
        raw_line = ser.readline()
        if not raw_line:
            continue

        text = raw_line.decode("utf-8", errors="ignore").strip()
        sample = parse_sample(text)
        if sample is None:
            continue

        ecg_uv, bpm = sample
        ecg_data.append(ecg_uv)

        if len(ecg_data) < 3:
            continue

        line.set_data(range(len(ecg_data)), ecg_data)
        if bpm is not None:
            ax.set_title(f"Live ECG - Wokwi   HR: {bpm:.0f} BPM")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Plotter stopped.")

finally:
    ser.close()
    plt.close()
