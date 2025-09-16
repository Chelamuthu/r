#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
import sys
import os

# =========================
# CONFIGURATION
# =========================
# Port connected to LoRa Module (SX1262/SX1268)
# When LoRa is in "Transmission Mode" and connected to Raspberry Pi GPIO
LORA_PORT = "/dev/serial0"  # UART on Raspberry Pi
BAUDRATE = 9600             # Default baud rate for both GPS and LoRa

# =========================
# FUNCTIONS
# =========================

def convert_to_decimal(degree, direction):
    """
    Convert NMEA GPS coordinates to decimal degrees
    """
    if not degree or degree == 0.0:
        return None
    decimal = float(degree)
    if direction in ['S', 'W']:
        decimal *= -1
    return round(decimal, 6)


def read_gps_data():
    """
    Read GPS data from LoRa UART
    """
    try:
        with serial.Serial(LORA_PORT, BAUDRATE, timeout=1) as ser:
            print(f"[INFO] Listening on {LORA_PORT} at {BAUDRATE} baud...")
            time.sleep(2)  # Allow LoRa/GPS to stabilize

            while True:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        # Debug raw data
                        print("Raw:", line)

                        # Process GPS sentences
                        if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                            msg = pynmea2.parse(line)

                            latitude = convert_to_decimal(msg.latitude, msg.lat_dir)
                            longitude = convert_to_decimal(msg.longitude, msg.lon_dir)
                            altitude = getattr(msg, 'altitude', 'N/A')

                            print(f"Time (UTC): {msg.timestamp}")
                            print(f"Latitude   : {latitude}")
                            print(f"Longitude  : {longitude}")
                            print(f"Altitude   : {altitude} m")
                            print("-" * 40)

                except pynmea2.ParseError:
                    # Ignore incomplete/invalid GPS sentences
                    continue
                except UnicodeDecodeError:
                    continue

    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port {LORA_PORT}: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
        sys.exit(0)

# =========================
# MAIN
# =========================
if __name__ == "__main__":
    try:
        read_gps_data()
    except KeyboardInterrupt:
        print("\n[INFO] Program terminated by user.")
