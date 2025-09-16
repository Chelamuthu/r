#!/usr/bin/python3
# -- coding: UTF-8 --

import time
import serial
import threading
from datetime import datetime
import os
import sys
import RPi.GPIO as GPIO
import pynmea2  # Library for parsing NMEA sentences

# ===============================
# CONFIGURATION
# ===============================
BAUDRATE = 9600                     # LoRa and GPS baud rate
SERIAL_PORTS = ["/dev/serial0", "/dev/ttyAMA0"]  # Auto-detect UART
SEND_INTERVAL = 2.0                 # Seconds between transmissions

# GPIO pins for LEDs
LED_TX = 17  # GPIO17 (Pin 11)
LED_RX = 27  # GPIO27 (Pin 13)

# ===============================
# LoRa UART Class
# ===============================
class SX126x:
    def __init__(self, baudrate=9600):
        self.ser = None
        self.baudrate = baudrate
        self.packets_sent = 0
        self.packets_received = 0
        self.errors = 0

        # Detect available serial port
        for port in SERIAL_PORTS:
            if os.path.exists(port):
                try:
                    self.ser = serial.Serial(port, baudrate, timeout=1)
                    self.ser.flushInput()
                    print(f"[INFO] Connected to {port} at {baudrate} baud")
                    break
                except serial.SerialException as e:
                    print(f"[ERROR] Failed to open {port}: {e}")
        
        if self.ser is None:
            print("[FATAL] No serial port available! Exiting...")
            sys.exit(1)

    def read_raw(self):
        """Read raw data from serial (NMEA GPS + LoRa RX)"""
        try:
            if self.ser.in_waiting > 0:
                return self.ser.readline().decode('utf-8', errors='ignore').strip()
            return None
        except serial.SerialException as e:
            self.errors += 1
            print(f"[ERROR] UART read failed: {e}")
            return None

    def send(self, data):
        """Send data to LoRa module"""
        try:
            bytes_written = self.ser.write((data + "\n").encode())
            self.packets_sent += 1

            # Blink TX LED
            GPIO.output(LED_TX, GPIO.HIGH)
            time.sleep(0.05)
            GPIO.output(LED_TX, GPIO.LOW)

            return bytes_written
        except serial.SerialException as e:
            self.errors += 1
            print(f"[ERROR] UART write failed: {e}")
            return 0

    def close(self):
        if self.ser:
            self.ser.close()
            print("[INFO] Serial port closed.")

# ===============================
# GPS Parsing
# ===============================
def parse_gps_sentence(sentence):
    """Extract latitude and longitude from NMEA sentences."""
    try:
        if sentence.startswith('$GPGGA') or sentence.startswith('$GPRMC'):
            msg = pynmea2.parse(sentence)
            if msg.latitude and msg.longitude:
                return {
                    "latitude": msg.latitude,
                    "lat_dir": msg.lat_dir,
                    "longitude": msg.longitude,
                    "lon_dir": msg.lon_dir,
                    "timestamp": getattr(msg, "timestamp", None)
                }
    except pynmea2.ParseError:
        return None
    return None

# ===============================
# GPIO Setup
# ===============================
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_TX, GPIO.OUT)
    GPIO.setup(LED_RX, GPIO.OUT)
    GPIO.output(LED_TX, GPIO.LOW)
    GPIO.output(LED_RX, GPIO.LOW)

# ===============================
# Main Logic
# ===============================
def main():
    setup_gpio()
    lora = SX126x(baudrate=BAUDRATE)

    print("===============================================")
    print(" LoRa GPS Sender (Raspberry Pi UART) ")
    print("===============================================\n")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            raw_data = lora.read_raw()
            if raw_data:
                # Blink RX LED to indicate GPS data received
                GPIO.output(LED_RX, GPIO.HIGH)
                time.sleep(0.02)
                GPIO.output(LED_RX, GPIO.LOW)

                gps_data = parse_gps_sentence(raw_data)
                if gps_data:
                    # Display GPS data locally
                    print(f"[GPS] Time: {gps_data['timestamp']}, "
                          f"Lat: {gps_data['latitude']} {gps_data['lat_dir']}, "
                          f"Lon: {gps_data['longitude']} {gps_data['lon_dir']}")

                    # Prepare LoRa message
                    lora_message = (
                        f"LAT:{gps_data['latitude']:.6f}{gps_data['lat_dir']}, "
                        f"LON:{gps_data['longitude']:.6f}{gps_data['lon_dir']}"
                    )

                    # Send over LoRa
                    lora.send(lora_message)
                    print(f"[TX] Sent over LoRa -> {lora_message}")

                    time.sleep(SEND_INTERVAL)
                else:
                    print(f"[RAW] {raw_data}")  # Show raw NMEA sentence for debugging

    except KeyboardInterrupt:
        print("\n[INFO] Exiting gracefully...")
        lora.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
