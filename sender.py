#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
from datetime import datetime
import sys
import os
import math

# ===============================
# CONFIGURATION
# ===============================
UART_PORT = "/dev/ttyAMA0"  # UART GPIO14(TX), GPIO15(RX)
BAUDRATE = 9600             # GPS & LoRa baud rate
SEND_INTERVAL = 0.2         # Faster update for racing (0.2s = 5Hz)

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
# Kalman Filter for position and speed
# ===============================
class KalmanFilter:
    def __init__(self, process_var=1e-4, measurement_var=1e-2):
        self.x = 0.0       # Position estimate
        self.v = 0.0       # Speed estimate
        self.P = 1.0       # Estimate uncertainty
        self.Q = process_var
        self.R = measurement_var

    def update(self, measured_pos, dt):
        # Prediction
        x_pred = self.x + self.v * dt
        P_pred = self.P + self.Q

        # Kalman gain
        K = P_pred / (P_pred + self.R)

        # Update
        self.x = x_pred + K * (measured_pos - x_pred)
        self.v = (self.x - self.x) / dt  # simple derivative
        self.P = (1 - K) * P_pred

        return self.x

# ===============================
# Main Loop
# ===============================
def main():
    ser = init_serial()
    print("======================================")
    print("   GPS + Kalman Speed Calculation + LoRa Sender")
    print("======================================")

    last_lat = None
    last_lon = None
    last_time = None
    kalman_lat = KalmanFilter()
    kalman_lon = KalmanFilter()

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)

                    # Only use valid fix
                    if line.startswith('$GPGGA') and msg.gps_qual == 0:
                        continue
                    if line.startswith('$GPRMC') and msg.status != 'A':
                        continue

                    lat = float(msg.latitude)
                    lon = float(msg.longitude)
                    current_time = time.time()
                    dt = current_time - last_time if last_time else SEND_INTERVAL

                    # Apply Kalman filter
                    kal_lat = kalman_lat.update(lat, dt)
                    kal_lon = kalman_lon.update(lon, dt)

                    # Calculate speed from filtered positions
                    speed_kmh = 0.0
                    if last_lat is not None and last_lon is not None:
                        dist_km = haversine(last_lat, last_lon, kal_lat, kal_lon)
                        if dt > 0:
                            speed_kmh = (dist_km / dt) * 3600.0

                    last_lat, last_lon, last_time = kal_lat, kal_lon, current_time

                    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                    message = f"{timestamp} LAT:{kal_lat:.7f} LON:{kal_lon:.7f} SPEED:{speed_kmh:.2f}km/h"

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
            time.sleep(0.1)

if __name__ == "__main__":
    main()
