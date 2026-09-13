import serial
import matplotlib.pyplot as plt
from collections import deque
import time

# ==========================================
# CONFIGURATION
# ==========================================

PORT = "COM11"       # GANTI sesuai Arduino kamu
BAUDRATE = 115200

MAX_POINTS = 300

# ==========================================
# OPEN SERIAL
# ==========================================

try:
    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=1
    )

    print("Connected to", PORT)

except serial.SerialException as e:
    print("Tidak bisa membuka", PORT)
    print(e)
    input("Tekan ENTER untuk keluar...")
    exit()

# Beri waktu Arduino reset setelah COM dibuka
time.sleep(2)

# ==========================================
# DATA BUFFER
# ==========================================

time_data = deque(maxlen=MAX_POINTS)
raw_data = deque(maxlen=MAX_POINTS)
filtered_data = deque(maxlen=MAX_POINTS)

start_time = time.time()

# ==========================================
# GRAPH
# ==========================================

plt.ion()

fig, ax = plt.subplots(figsize=(11, 6))

raw_line, = ax.plot(
    [],
    [],
    label="Raw"
)

filtered_line, = ax.plot(
    [],
    [],
    label="Filtered",
    linewidth=2
)

ax.set_title("BMP280 Realtime Altitude")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Altitude (m)")

ax.grid(True)
ax.legend()

plt.show()

# ==========================================
# MAIN LOOP
# ==========================================

try:

    while True:

        if ser.in_waiting:

            line = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            print(line)

            # Abaikan START dan pesan lainnya
            if not line.startswith("Raw:"):
                continue

            try:

                # Contoh:
                # Raw:0.123 Filtered:0.100

                data = line.split()

                raw_value = float(
                    data[0].replace("Raw:", "")
                )

                filtered_value = float(
                    data[1].replace("Filtered:", "")
                )

                current_time = time.time() - start_time

                time_data.append(current_time)
                raw_data.append(raw_value)
                filtered_data.append(filtered_value)

            except (ValueError, IndexError):
                continue

        # ======================================
        # UPDATE GRAPH
        # ======================================

        if len(time_data) > 0:

            raw_line.set_data(
                time_data,
                raw_data
            )

            filtered_line.set_data(
                time_data,
                filtered_data
            )

            # ----------------------------------
            # X AXIS
            # ----------------------------------

            current_time = time_data[-1]

            if current_time < 30:
                ax.set_xlim(0, 30)
            else:
                ax.set_xlim(
                    current_time - 30,
                    current_time
                )

            # ----------------------------------
            # Y AXIS
            # ----------------------------------

            values = list(raw_data) + list(filtered_data)

            minimum = min(values)
            maximum = max(values)

            margin = max(
                0.1,
                (maximum - minimum) * 0.15
            )

            ymin = max(
                0,
                minimum - margin
            )

            ymax = maximum + margin

            if ymax <= 1:
                ymax = 1

            ax.set_ylim(ymin, ymax)

            # ----------------------------------
            # DRAW
            # ----------------------------------

            fig.canvas.draw_idle()
            fig.canvas.flush_events()

        time.sleep(0.01)

except KeyboardInterrupt:

    print("\nProgram dihentikan.")

finally:

    ser.close()
    plt.ioff()
    plt.show()