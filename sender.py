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
GPS_PORT = "/dev/serial0"    # GPS & LoRa UART
BAUDRATE = 9600              # Default baud rate for GPS and LoRa
SEND_INTERVAL = 1.0          # Time between sends (seconds)

# ===============================
# Initialize UART
# ===============================
def init_serial():
    if not os.path.exists(GPS_PORT):
        print(f"[FATAL] {GPS_PORT} does not exist! Check wiring and serial settings.")
        sys.exit(1)
    try:
        ser = serial.Serial(GPS_PORT, BAUDRATE, timeout=1)
        print(f"[INFO] Connected to {GPS_PORT} at {BAUDRATE} baud")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Unable to open serial port: {e}")
        sys.exit(1)

# ===============================
# Send data via LoRa
# ===============================
def send_lora(ser, message):
    try:
        ser.write((message + "\n").encode('utf-8'))
        print(f"[LORA] Sent: {message}")
    except Exception as e:
        print(f"[ERROR] Failed to send via LoRa: {e}")

# ===============================
# Main Loop
# ===============================
def main():
    ser = init_serial()
    print("======================================")
    print("   GPS → LoRa Sender Running")
    print("======================================")

    while True:
        try:
            # Read line from GPS
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            # Debug raw GPS data
            if line.startswith('$'):
                print("[GPS RAW]", line)

            # Parse only GPGGA or GPRMC sentences
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)

                latitude = msg.latitude
                longitude = msg.longitude

                if latitude and longitude:
                    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                    gps_data = f"{timestamp} LAT:{latitude:.6f} LON:{longitude:.6f}"

                    # Print and send
                    print(f"[GPS] {gps_data}")
                    send_lora(ser, gps_data)
                    time.sleep(SEND_INTERVAL)

        except pynmea2.ParseError:
            continue
        except UnicodeDecodeError:
            continue
        except KeyboardInterrupt:
            print("\n[INFO] Exiting gracefully...")
            ser.close()
            sys.exit(0)

if __name__ == "__main__":
    main()
