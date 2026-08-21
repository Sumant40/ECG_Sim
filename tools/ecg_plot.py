import serial
import matplotlib.pyplot as plt
from collections import deque

PORT = "rfc2217://localhost:4000"
BAUD = 115200

MAX_POINTS = 500

ser = serial.serial_for_url(
    PORT,
    baudrate=BAUD,
    timeout=1
)

data = deque(maxlen=MAX_POINTS)

plt.ion()

fig, ax = plt.subplots(figsize=(12, 5))
line, = ax.plot([])

ax.set_title("Live ECG - Wokwi")
ax.set_xlabel("Samples")
ax.set_ylabel("ECG")
ax.grid(True)

try:

    while True:

        line_data = ser.readline()

        if not line_data:
            continue

        text = line_data.decode(
            "utf-8",
            errors="ignore"
        ).strip()

        if not text:
            continue

        try:

            values = [
                float(x.strip())
                for x in text.split(",")
            ]

            # First column = ECG
            data.append(values[0])

        except ValueError:
            # Ignore ESP32 boot messages
            continue

        if len(data) > 2:

            line.set_data(
                range(len(data)),
                data
            )

            ax.set_xlim(
                0,
                MAX_POINTS
            )

            ymin = min(data)
            ymax = max(data)

            if ymax == ymin:
                ymax += 1
                ymin -= 1

            margin = (
                ymax - ymin
            ) * 0.1

            ax.set_ylim(
                ymin - margin,
                ymax + margin
            )

            fig.canvas.draw_idle()
            fig.canvas.flush_events()

except KeyboardInterrupt:

    print("Plotter stopped.")

finally:

    ser.close()
    plt.close()