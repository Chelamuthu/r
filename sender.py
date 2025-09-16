#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
from datetime import datetime, timezone
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
    Calculate distance in km between two points on Earth using haversine formula.
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
    last_fix_valid = False

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            # Process GGA or RMC sentences
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)

                    # Ensure GPS fix validity and lat/lon existence
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude') and msg.latitude and msg.longitude:
                        lat = float(msg.latitude)
                        lon = float(msg.longitude)

                        # Get GPS time from sentence if possible (useful for precise timing)
                        gps_time = None
                        if hasattr(msg, 'timestamp') and msg.timestamp:
                            # Combine GPS timestamp with current UTC date, careful around midnight rollover
                            now_utc = datetime.utcnow()
                            gps_time = datetime.combine(now_utc.date(), msg.timestamp, tzinfo=timezone.utc).timestamp()

                        current_time = gps_time if gps_time else time.time()

                        speed_kmh = 0.0

                        # Calculate speed only if last fix valid and position changed significantly
                        if last_fix_valid and last_lat is not None and last_lon is not None and last_time is not None:
                            dist_km = haversine(last_lat, last_lon, lat, lon)
                            delta_time = current_time - last_time
                            if delta_time > 0 and dist_km > 0.0001:  # >0.1 meters threshold to avoid jitter
                                speed_kmh = (dist_km / delta_time) * 3600.0  # km/h
                            else:
                                speed_kmh = 0.0
                        else:
                            # No valid last fix yet
                            speed_kmh = 0.0

                        last_lat, last_lon, last_time = lat, lon, current_time
                        last_fix_valid = True

                        # Format message with 7-digit precision lat/lon
                        timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                        message = f"{timestamp} LAT:{lat:.7f} LON:{lon:.7f} SPEED:{speed_kmh:.2f}km/h"

                        # Send message out via the UART
                        ser.write((message + "\n").encode('utf-8'))
                        print(f"[LORA] {message}")

                        time.sleep(SEND_INTERVAL)
                    else:
                        # No valid fix in this sentence
                        last_fix_valid = False

                except pynmea2.ParseError:
                    # Ignore faulty sentence
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
