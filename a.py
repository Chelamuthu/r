#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Live GPS Speed Tracking + LoRa Transmission
Optimized for continuous operation with accurate speed tracking (max 3 km/h).
"""

import os
import sys
import time
import csv
import math
import serial
import pynmea2
import RPi.GPIO as GPIO
from datetime import datetime, timezone
from collections import deque

# ==========================
# CONFIGURATION
# ==========================
GPS_PORT = "/dev/ttyAMA0"        # GPS connected here
LORA_PORT = "/dev/ttyUSB0"       # LoRa connected via USB-to-UART
GPS_BAUD = 9600
LORA_BAUD = 9600

SEND_INTERVAL = 0.2              # 200 ms updates
NO_FIX_TIMEOUT = 5.0             # seconds before declaring GNSS lost
CSV_DIR = "gps_logs"

# LoRa settings for LONG RANGE
DEFAULT_FREQ_MHZ = 433           # Frequency
DEFAULT_POWER = 20               # Max TX power for long range
DEFAULT_AIR_SPEED = 2400         # Lower airspeed -> longer range, less data

M0_PIN = 22                       # GPIO for M0
M1_PIN = 27                       # GPIO for M1

# ==========================
# Helper Functions
# ==========================
def ensure_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)

def current_timestamp():
    return datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")

def haversine_km(lat1, lon1, lat2, lon2):
    """Calculate distance between two lat/lon points in KM."""
    R = 6371.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

# ==========================
# LoRa Class
# ==========================
class LoRaModule:
    def __init__(self, port, baud, m0_pin, m1_pin):
        self.port = port
        self.baud = baud
        self.ser = None
        self.m0 = m0_pin
        self.m1 = m1_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.m0, GPIO.OUT)
        GPIO.setup(self.m1, GPIO.OUT)

        self._enter_normal_mode()
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        print(f"[INFO] LoRa connected on {self.port}")

    def _enter_normal_mode(self):
        GPIO.output(self.m0, GPIO.LOW)
        GPIO.output(self.m1, GPIO.LOW)
        time.sleep(0.1)

    def send(self, data):
        if self.ser:
            self.ser.write(data.encode('utf-8'))

    def close(self):
        if self.ser:
            self.ser.close()
        GPIO.cleanup([self.m0, self.m1])

# ==========================
# Main GPS + LoRa Sender
# ==========================
class GPSLoRaSender:
    def __init__(self):
        self.gps_ser = None
        self.lora = None
        self.last_lat = None
        self.last_lon = None
        self.last_time = None
        self.speed_buffer = deque(maxlen=5)  # smooth speed

        ensure_dir(CSV_DIR)
        self.csv_path = os.path.join(CSV_DIR, f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["UTC Time", "Latitude", "Longitude", "Speed (km/h)", "Status"])

    def open_ports(self):
        if not os.path.exists(GPS_PORT):
            raise FileNotFoundError(f"[ERROR] GPS port {GPS_PORT} not found")
        self.gps_ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.5)
        print(f"[INFO] GPS connected on {GPS_PORT}")

        if not os.path.exists(LORA_PORT):
            raise FileNotFoundError(f"[ERROR] LoRa port {LORA_PORT} not found")
        self.lora = LoRaModule(LORA_PORT, LORA_BAUD, M0_PIN, M1_PIN)

    def read_gps(self):
        try:
            line = self.gps_ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                msg = pynmea2.parse(line)
                if msg.status == 'A':  # valid fix
                    return float(msg.latitude), float(msg.longitude), msg.timestamp
            return None
        except:
            return None

    def calculate_speed(self, lat, lon, timestamp):
        """Calculate speed in km/h, capped at 3 km/h."""
        if self.last_lat is None or self.last_lon is None:
            self.last_lat, self.last_lon, self.last_time = lat, lon, timestamp
            return 0.0

        distance_km = haversine_km(self.last_lat, self.last_lon, lat, lon)
        delta_time = (timestamp - self.last_time).total_seconds() if timestamp else 1

        if delta_time <= 0:
            return 0.0

        raw_speed = (distance_km / delta_time) * 3600.0
        self.speed_buffer.append(raw_speed)

        smoothed_speed = sum(self.speed_buffer) / len(self.speed_buffer)
        self.last_lat, self.last_lon, self.last_time = lat, lon, timestamp

        return min(smoothed_speed, 3.0)  # cap at 3 km/h

    def log_and_send(self, lat, lon, speed):
        timestamp = current_timestamp()
        payload = f"{timestamp},{lat:.6f},{lon:.6f},{speed:.2f}"
        self.lora.send(payload)

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, lat, lon, speed, "OK"])

        print(f"[DATA] {payload}")

    def run(self):
        print("[INFO] Starting GPS → LoRa loop. Press Ctrl+C to stop.")
        next_send = time.time()
        last_fix_time = time.time()

        while True:
            gps_data = self.read_gps()
            now = time.time()

            if gps_data:
                lat, lon, gps_timestamp = gps_data
                last_fix_time = now
                speed = self.calculate_speed(lat, lon, datetime.combine(datetime.utcnow().date(), gps_timestamp))
                if now >= next_send:
                    self.log_and_send(lat, lon, speed)
                    next_send = now + SEND_INTERVAL
            else:
                if now - last_fix_time > NO_FIX_TIMEOUT:
                    print("[WARN] GNSS NOT FIX")
                    self.lora.send("GNSS NOT FIX")

            time.sleep(0.05)

    def close(self):
        print("[INFO] Closing connections...")
        if self.lora:
            self.lora.close()
        if self.gps_ser:
            self.gps_ser.close()
        GPIO.cleanup()
        print("[INFO] Shutdown complete.")

# ==========================
# Entry Point
# ==========================
if __name__ == "__main__":
    try:
        app = GPSLoRaSender()
        app.open_ports()
        app.run()
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received. Exiting...")
        app.close()
    except Exception as e:
        print(f"[FATAL] {e}")
        GPIO.cleanup()
        sys.exit(1)
