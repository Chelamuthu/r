#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import time
import pynmea2
import sys

# ===============================
# CONFIGURATION
# ===============================
GPS_PORT = "/dev/serial0"    # Neo-6M connected here
GPS_BAUDRATE = 9600

# LoRa UART configuration
LORA_PORT = "/dev/ttyUSB0"   # LoRa via USB-to-UART
LORA_BAUDRATE = 9600

# ===============================
# INIT SERIAL CONNECTIONS
# ===============================
try:
    gps = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1)
    print("[INFO] GPS module connected on", GPS_PORT)
except Exception as e:
    print("[ERROR] Could not open GPS port:", e)
    sys.exit(1)

try:
    lora = serial.Serial(LORA_PORT, LORA_BAUDRATE, timeout=1)
    print("[INFO] LoRa module connected on", LORA_PORT)
except Exception as e:
    print("[ERROR] Could not open LoRa port:", e)
    sys.exit(1)

time.sleep(2)  # Allow modules to initialize

# ===============================
# FUNCTIONS
# ===============================
def send_lora_message(message):
    """Send data over LoRa."""
    try:
        lora.write((message + "\n").encode('utf-8'))
        print(f"[LORA] Sent: {message}")
    except Exception as e:
        print("[ERROR] Failed to send via LoRa:", e)

# ===============================
# MAIN LOOP
# ===============================
print("[SYSTEM] Starting GPS → LoRa transmission...")

while True:
    try:
        line = gps.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue

        # Debugging raw NMEA data
        if line.startswith('$'):
            print("[GPS RAW]", line)

        # Process only GPGGA or GPRMC sentences
        if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
            msg = pynmea2.parse(line)

            latitude = msg.latitude
            longitude = msg.longitude

            if latitude and longitude:
                gps_data = f"LAT:{latitude:.6f},LON:{longitude:.6f}"
                print(f"[GPS] {gps_data}")

                # Send via LoRa
                send_lora_message(gps_data)
                time.sleep(1)

    except pynmea2.ParseError:
        continue
    except UnicodeDecodeError:
        continue
    except KeyboardInterrupt:
        print("\n[SYSTEM] Stopping...")
        gps.close()
        lora.close()
        sys.exit(0)
