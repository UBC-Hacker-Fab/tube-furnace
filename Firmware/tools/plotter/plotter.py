import serial
import time
from collections import deque
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

# -----------------------------
# User settings
# -----------------------------
PORT = "/dev/tty.usbserial-0001"
BAUD = 115200
TIMEOUT = 1.0
MAX_POINTS = 300

# -----------------------------
# Serial setup
# -----------------------------
ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
time.sleep(2)

# -----------------------------
# Data buffers
# -----------------------------
t_data = deque(maxlen=MAX_POINTS)
temp_data = deque(maxlen=MAX_POINTS)
setpoint_data = deque(maxlen=MAX_POINTS)
duty_data = deque(maxlen=MAX_POINTS)

start_time = time.time()

# -----------------------------
# Plot setup
# -----------------------------
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

line_temp, = ax1.plot([], [], label="Temperature")
line_setpoint, = ax1.plot([], [], label="Setpoint")
line_duty, = ax2.plot([], [], label="Duty Cycle", color="red")

# Dummy legend entries for metrics
rmse_entry = Line2D([], [], color='none', label="RMSE: ---")
temp_slope_entry = Line2D([], [], color='none', label="Temp slope: --- °C/s")
sp_slope_entry = Line2D([], [], color='none', label="Setpoint slope: --- °C/s")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Temperature (°C)")
ax2.set_ylabel("Duty Cycle", color="red")
ax2.tick_params(axis='y', colors='red')

ax1.set_title("Live Temperature, Setpoint, Duty Cycle")
ax1.grid(True)

def init():
    line_temp.set_data([], [])
    line_setpoint.set_data([], [])
    line_duty.set_data([], [])
    return line_temp, line_setpoint, line_duty

def update(frame):
    while ser.in_waiting:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if not raw or raw == "Temp,Setpoint,Duty":
            continue

        parts = raw.split(",")
        if len(parts) != 3:
            continue

        try:
            temp = float(parts[0])
            setpoint = float(parts[1])
            duty = float(parts[2])
        except ValueError:
            continue

        t_now = time.time() - start_time

        t_data.append(t_now)
        temp_data.append(temp)
        setpoint_data.append(setpoint)
        duty_data.append(duty)

    if len(t_data) == 0:
        return line_temp, line_setpoint, line_duty

    line_temp.set_data(t_data, temp_data)
    line_setpoint.set_data(t_data, setpoint_data)
    line_duty.set_data(t_data, duty_data)

    # X axis autoscale
    ax1.set_xlim(t_data[0], max(t_data[-1], t_data[0] + 1))

    # Temperature axis autoscale
    y_min = min(min(temp_data), min(setpoint_data))
    y_max = max(max(temp_data), max(setpoint_data))
    pad = max(5, 0.05 * (y_max - y_min + 1))
    ax1.set_ylim(y_min - pad, y_max + pad)

    # Duty axis fixed
    ax2.set_ylim(0, 1)

    # -------- RMSE calculation --------
    temp_arr = np.array(temp_data)
    sp_arr = np.array(setpoint_data)
    rmse = np.sqrt(np.mean((temp_arr - sp_arr) ** 2))
    rmse_entry.set_label(f"RMSE: {rmse:.2f} °C")

    # -------- Mean slope over 3 samples --------
    if len(t_data) >= 3:

        dt = t_data[-1] - t_data[-3]

        if dt > 0:
            temp_slope = (temp_data[-1] - temp_data[-3]) / dt
            sp_slope = (setpoint_data[-1] - setpoint_data[-3]) / dt
        else:
            temp_slope = 0.0
            sp_slope = 0.0

        temp_slope_entry.set_label(f"Temp slope (3): {temp_slope:.3f} °C/s")
        sp_slope_entry.set_label(f"Setpoint slope (3): {sp_slope:.3f} °C/s")

    else:
        temp_slope_entry.set_label("Temp slope (3): --- °C/s")
        sp_slope_entry.set_label("Setpoint slope (3): --- °C/s")

    # Update legend dynamically
    lines = [
        line_temp,
        line_setpoint,
        line_duty,
        rmse_entry,
        temp_slope_entry,
        sp_slope_entry
    ]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc="upper left")

    return line_temp, line_setpoint, line_duty

ani = FuncAnimation(fig, update, init_func=init, interval=200, blit=False)

try:
    plt.show()
finally:
    ser.close()