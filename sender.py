#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import time
import pynmea2
import pigpio
import sys

# ===============================
# CONFIGURATION
# ===============================
GPS_PORT = "/dev/serial0"   # GPS uses main UART
GPS_BAUDRATE = 9600

# LoRa GPIO pins (Software UART)
LORA_TX_GPIO = 27  # Pi → LoRa RX
LORA_RX_GPIO = 17  # Pi ← LoRa TX
LORA_BAUDRATE = 9600

# ===============================
# INIT GPS SERIAL
# ===============================
try:
    gps = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1)
    print("[INFO] GPS connected on", GPS_PORT)
except Exception as e:
    print("[ERROR] Could not open GPS port:", e)
    sys.exit(1)

# ===============================
# INIT LORA SOFTWARE UART
# ===============================
pi = pigpio.pi()
if not pi.connected:
    print("[ERROR] pigpio daemon not running. Start with: sudo pigpiod")
    sys.exit(1)

lora = pi.serial_open(LORA_RX_GPIO, LORA_TX_GPIO, LORA_BAUDRATE)
print(f"[INFO] LoRa initialized on GPIO{LORA_RX_GPIO} (RX) / GPIO{LORA_TX_GPIO} (TX)")

# ===============================
# SEND TO LORA FUNCTION
# ===============================
def send_lora(message):
    try:
        pi.serial_write(lora, (message + "\n").encode('utf-8'))
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

        # Debug raw GPS data
        if line.startswith('$'):
            print("[GPS RAW]", line)

        if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
            msg = pynmea2.parse(line)

            latitude = msg.latitude
            longitude = msg.longitude

            if latitude and longitude:
                gps_data = f"LAT:{latitude:.6f},LON:{longitude:.6f}"
                print(f"[GPS] {gps_data}")

                # Send via LoRa
                send_lora(gps_data)
                time.sleep(1)

    except pynmea2.ParseError:
        continue
    except UnicodeDecodeError:
        continue
    except KeyboardInterrupt:
        print("\n[SYSTEM] Stopping...")
        gps.close()
        pi.serial_close(lora)
        pi.stop()
        sys.exit(0)
