#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import sys
import sx126x
import time
import termios
import tty
import serial
import pynmea2
import threading
from datetime import datetime

# =========================
# Terminal setup
# =========================
old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

# =========================
# LoRa Modules
# =========================
lora_gnss = sx126x.sx126x(serial_num="/dev/ttyAMA0", freq=868, addr=1, power=22, rssi=True, air_speed=2400, relay=False)
lora_general = sx126x.sx126x(serial_num="/dev/ttyAMA1", freq=868, addr=2, power=22, rssi=True, air_speed=2400, relay=False)

# =========================
# GPS Serial
# =========================
GPS_PORT = "/dev/ttyUSB0"
GPS_BAUD = 9600
gps_serial = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)

# =========================
# Continuous GNSS + CPU Sender
# =========================
SEND_INTERVAL = 0.02  # 20 ms

def continuous_send():
    try:
        while True:
            # Read GPS data
            if gps_serial.in_waiting > 0:
                line = gps_serial.readline().decode('ascii', errors='replace').strip()
                if line.startswith("$GPRMC"):  # Recommended minimum GPS data
                    try:
                        msg = pynmea2.parse(line)
                        if msg.status == 'A':  # Data valid
                            # Extract GPS info
                            latitude = msg.latitude
                            longitude = msg.longitude
                            speed_kmh = float(msg.spd_over_grnd) * 1.852  # knots -> km/h
                            course = msg.true_course
                            timestamp = msg.datestamp.strftime("%Y-%m-%d") + " " + msg.timestamp.strftime("%H:%M:%S")

                            # Limit max speed if needed
                            speed_kmh = min(speed_kmh, 300)

                            # Create payload
                            gnss_payload = f"LAT:{latitude:.6f},LON:{longitude:.6f},SPD:{speed_kmh:.2f}km/h,CRS:{course:.2f},TIME:{timestamp}"
                            data = bytes([0,1,18,0,1,12]) + gnss_payload.encode()

                            # Send GNSS over LoRa
                            lora_gnss.send(data)
                            print(f"[GNSS] Sent: {gnss_payload}")
                    except Exception as e:
                        print(f"[GNSS PARSE ERROR] {e}")

            # Send CPU temperature
            cpu_msg = f"CPU Temperature:{get_cpu_temp():.2f} C"
            cpu_data = bytes([255,255,18,255,255,12]) + cpu_msg.encode()
            lora_general.send(cpu_data)

            # Wait 20 ms
            time.sleep(SEND_INTERVAL)
    except Exception as e:
        print(f"[ERROR] {e}")

# =========================
# CPU Temperature Function
# =========================
def get_cpu_temp():
    with open("/sys/class/thermal/thermal_zone0/temp") as tempFile:
        cpu_temp = tempFile.read()
    return float(cpu_temp) / 1000

# =========================
# Start sending thread
# =========================
send_thread = threading.Thread(target=continuous_send)
send_thread.daemon = True
send_thread.start()

# =========================
# Main loop: exit on ESC
# =========================
try:
    print("Press ESC to exit continuous GNSS + CPU sending...")
    while True:
        if sys.stdin.read(1) == '\x1b':
            break
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    gps_serial.close()
    print("Program terminated.")
