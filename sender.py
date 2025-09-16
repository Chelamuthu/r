#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
from datetime import datetime
import sys
import os

# ===============================
# CONFIGURATION
# ===============================
UART_PORT = "/dev/ttyAMA0"    # Primary UART using GPIO14 (TX), GPIO15 (RX)
BAUDRATE = 9600               # GPS and LoRa must both use this baud rate
GPS_READ_TIME = 1.0           # Seconds to listen for GPS data
SEND_INTERVAL = 2.0           # Seconds between LoRa sends

# ===============================
# Initialize UART
# ===============================
def init_serial():
    if not os.path.exists(UART_PORT):
        print(f"[FATAL] {UART_PORT} not found! Check wiring and enable UART in raspi-config.")
        sys.exit(1)
    try:
        ser = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
        print(f"[INFO] UART connected on pins TX=GPIO14 (Pin 8), RX=GPIO15 (Pin 10)")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Unable to open UART: {e}")
        sys.exit(1)

# ===============================
# Main Logic
# ===============================
def main():
    ser = init_serial()
    print("======================================")
    print("  GPS + LoRa on Single UART (Direct GPIO)")
    print("======================================")

    latest_lat = None
    latest_lon = None

    while True:
        try:
            # --- Step 1: Listen for GPS data for a short time ---
            start_time = time.time()
            while time.time() - start_time < GPS_READ_TIME:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                    try:
                        msg = pynmea2.parse(line)
                        if msg.latitude and msg.longitude:
                            latest_lat = msg.latitude
                            latest_lon = msg.longitude
                            print(f"[GPS] LAT:{latest_lat:.6f} LON:{latest_lon:.6f}")
                    except pynmea2.ParseError:
                        continue

            # --- Step 2: Send latest GPS coordinates over LoRa ---
            if latest_lat and latest_lon:
                timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                message = f"{timestamp} LAT:{latest_lat:.6f} LON:{latest_lon:.6f}"
                ser.write((message + "\n").encode('utf-8'))
                print(f"[LORA] Sent: {message}")
                time.sleep(SEND_INTERVAL)

        except KeyboardInterrupt:
            print("\n[INFO] Exiting gracefully...")
            ser.close()
            sys.exit(0)
        except Exception as e:
            print(f"[ERROR] {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
