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
UART_PORT = "/dev/ttyAMA0"  # GPS UART port (example)
BAUDRATE = 9600             # GPS baud rate
SEND_INTERVAL = 1.0         # Send interval in seconds

# ===============================
# Initialize UART
# ===============================
def init_serial():
    if not os.path.exists(UART_PORT):
        print(f"[FATAL] {UART_PORT} not found! Check wiring and enable UART in raspi-config.")
        sys.exit(1)
    try:
        ser = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
        print(f"[INFO] UART connected on port {UART_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Unable to open UART: {e}")
        sys.exit(1)

# ===============================
# Haversine distance calculation
# ===============================
def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate distance in km between two points on Earth
    """
    R = 6371.0  # Earth radius in km
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# ===============================
# Main Loop
# ===============================
def main():
    ser = init_serial()
    print("======================================")
    print("  GPS + Speed Calculation + LoRa Sender")
    print("======================================")

    last_lat = None
    last_lon = None
    last_time = None

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            # Only process GGA or RMC sentences
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)

                    # Check if msg has valid coordinates
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude') and msg.latitude and msg.longitude:
                        lat = float(msg.latitude)
                        lon = float(msg.longitude)
                        current_time = time.time()
                        speed_kmh = 0.0

                        # Calculate speed if previous point exists
                        if last_lat is not None and last_lon is not None and last_time is not None:
                            dist_km = haversine(last_lat, last_lon, lat, lon)
                            delta_time = current_time - last_time
                            if delta_time > 0:
                                speed_kmh = (dist_km / delta_time) * 3600.0  # km/h

                        # Update last known location/time
                        last_lat, last_lon, last_time = lat, lon, current_time

                        # Format message with 7-digit precision
                        timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                        message = f"{timestamp} LAT:{lat:.7f} LON:{lon:.7f} SPEED:{speed_kmh:.2f}km/h"

                        # For LoRa sending, replace this line with the appropriate code for the LoRa module
                        # Currently this writes back to the same UART port (which may conflict with GPS input)
                        # For example, if LoRa is on a different serial port, open it similarly to ser_lora = serial.Serial(...)
                        ser.write((message + "\n").encode('utf-8'))

                        print(f"[LORA] {message}")

                        time.sleep(SEND_INTERVAL)

                except pynmea2.ParseError:
                    # Ignore invalid NMEA sentences
                    continue

        except KeyboardInterrupt:
            print("\n[INFO] Exiting gracefully...")
            ser.close()
            sys.exit(0)

        except Exception as e:
            print(f"[ERROR] {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
