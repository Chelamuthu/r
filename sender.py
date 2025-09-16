#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
from datetime import datetime
import sys
import os
import math
from collections import deque

# ===============================
# CONFIGURATION
# ===============================
UART_PORT = "/dev/ttyAMA0"  # UART GPIO14(TX), GPIO15(RX)
BAUDRATE = 9600
SEND_INTERVAL = 1.0          # Send interval in seconds
MIN_DIST_KM = 0.005          # Minimum distance threshold (5 meters)
SPEED_SMOOTH_WINDOW = 5      # Number of samples for moving average

# ===============================
# Initialize UART
# ===============================
def init_serial():
    if not os.path.exists(UART_PORT):
        print(f"[FATAL] {UART_PORT} not found! Enable UART in raspi-config.")
        sys.exit(1)
    try:
        ser = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
        print(f"[INFO] UART connected on GPIO14(TX)/GPIO15(RX)")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Unable to open UART: {e}")
        sys.exit(1)

# ===============================
# Haversine distance calculation
# ===============================
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0  # Earth radius in km
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# ===============================
# Moving average for smoothing speed
# ===============================
class SpeedSmoother:
    def __init__(self, window_size=5):
        self.window = deque(maxlen=window_size)

    def update(self, speed):
        self.window.append(speed)
        return sum(self.window) / len(self.window)

# ===============================
# Main Loop
# ===============================
def main():
    ser = init_serial()
    print("======================================")
    print("   GPS + Speed Calculation + LoRa Sender")
    print("======================================")

    last_lat = None
    last_lon = None
    last_time = None
    smoother = SpeedSmoother(SPEED_SMOOTH_WINDOW)

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)

                    # Check GPS fix
                    if line.startswith('$GPGGA') and msg.gps_qual == 0:
                        continue
                    if line.startswith('$GPRMC') and msg.status != 'A':
                        continue

                    lat = float(msg.latitude)
                    lon = float(msg.longitude)
                    current_time = time.time()
                    speed_kmh = 0.0

                    if last_lat is not None and last_lon is not None and last_time is not None:
                        dist_km = haversine(last_lat, last_lon, lat, lon)
                        if dist_km >= MIN_DIST_KM:
                            delta_time = current_time - last_time
                            if delta_time > 0:
                                speed_kmh = (dist_km / delta_time) * 3600.0  # km/h

                    # Apply moving average to smooth speed
                    speed_kmh = smoother.update(speed_kmh)

                    last_lat, last_lon, last_time = lat, lon, current_time

                    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                    message = f"{timestamp} LAT:{lat:.7f} LON:{lon:.7f} SPEED:{speed_kmh:.2f}km/h"

                    ser.write((message + "\n").encode('utf-8'))
                    print(f"[LORA] {message}")
                    time.sleep(SEND_INTERVAL)

                except pynmea2.ParseError:
                    continue

        except KeyboardInterrupt:
            print("\n[INFO] Exiting...")
            ser.close()
            sys.exit(0)
        except Exception as e:
            print(f"[ERROR] {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
