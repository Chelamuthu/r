#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
lora_gnss_sender.py
Waveshare SX126x (register-AT-like) configuration + GNSS → LoRa sender.

Features:
 - Configure LoRa module via M0/M1 (config mode) and serial register packet (cfg_reg)
 - Read NMEA from GPS, compute (and smooth) speed, log to CSV
 - Send position every SEND_INTERVAL (default 200 ms)
 - If no GNSS fix for NO_FIX_TIMEOUT seconds, send "GNSS NOT FIX"
 - Simple interactive console: i to send manual payload, s to toggle CPU broadcast, Esc to exit
"""

import os
import sys
import time
import csv
import serial
import pynmea2
import threading
import termios
import tty
import select
import RPi.GPIO as GPIO
from datetime import datetime, timezone
from collections import deque
import math
import subprocess

# -------------------------
# CONFIGURATION - adapt these
# -------------------------
GPS_PORT = "/dev/ttyAMA0"       # GPS serial (NMEA)
LORA_PORT = "/dev/ttyAMA0"      # LoRa serial (same UART only if using an external multiplexer; otherwise use a different port)
GPS_BAUD = 9600
LORA_BAUD = 9600

# M0 / M1 GPIO pins for Waveshare HAT (BCM numbering)
M0_PIN = 22   # example: BCM22 -> physical pin 15
M1_PIN = 27   # example: BCM27 -> physical pin 13

SEND_INTERVAL = 0.2            # seconds (200 ms)
NO_FIX_TIMEOUT = 5.0           # seconds to declare "GNSS NOT FIX"
CSV_DIR = "gps_logs"           # directory for CSV logs

# LoRa parameter defaults (matches module docs)
DEFAULT_FREQ_MHZ = 433         # 433 or 868 etc.
DEFAULT_ADDR = 0x0014          # example node address (16-bit)
DEFAULT_POWER = 17             # module power index (refer to module's power table)
DEFAULT_RSSI_ENABLE = True     # whether to append RSSI byte in responses
DEFAULT_AIR_SPEED = 2400       # air speed code in module (2400, 4800, ... as supported)
DEFAULT_NET_ID = 0
DEFAULT_BUFFER_SIZE = 240
DEFAULT_CRYPT = 0              # encryption key (0 = off)
DEFAULT_RELAY = False
DEFAULT_LBT = False
DEFAULT_WOR = False

# -------------------------
# Helper / module register template (as seen in the source)
# cfg_reg length assumed 12 bytes (indexing consistent with provided code)
# -------------------------
# We'll construct cfg_reg similar to the sample's structure:
# [0] 0xC0 (set) or similar, depending on module; using sample pattern.
DEFAULT_CFG_REG = [
    0xC0,   # 0 command, set register
    0x00,   # placeholder
    0x09,   # length? (module specific) keep similar to examples
    0x00,   # high addr
    0x00,   # low addr
    0x00,   # net id
    0x00,   # uart baud + air speed index
    0x00,   # buffer size + power + extra bits
    0x00,   # frequency offset
    0x00,   # options + rssi bit
    0x00,   # crypt high
    0x00    # crypt low
]

# These dictionaries map human values to module register encodings.
# The real values depend on module datasheet. Adapt if necessary.
LORA_AIR_SPEED_MAP = {
    2400: 0x00,
    4800: 0x01,
    9600: 0x02,
    19200: 0x03,
    38400: 0x04,
    57600: 0x05,
    115200: 0x06
}
LORA_BUFFER_SIZE_MAP = {
    240: 0x00,
    128: 0x01,
    64: 0x02
}
# power map — module-specific indices (example mapping)
LORA_POWER_MAP = {
    17: 0x00,
    20: 0x04
}

# -------------------------
# Utility functions
# -------------------------
def ensure_dir(d):
    if not os.path.exists(d):
        os.makedirs(d, exist_ok=True)

def current_timestamp_str():
    return datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

def calc_haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1); dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# -------------------------
# LoRa class: configure and send
# -------------------------
class WaveshareLoRa:
    def __init__(self, serial_port, baudrate, m0_pin, m1_pin,
                 freq_mhz=DEFAULT_FREQ_MHZ,
                 addr=DEFAULT_ADDR,
                 power=DEFAULT_POWER,
                 rssi=DEFAULT_RSSI_ENABLE,
                 air_speed=DEFAULT_AIR_SPEED,
                 net_id=DEFAULT_NET_ID,
                 buffer_size=DEFAULT_BUFFER_SIZE,
                 crypt=DEFAULT_CRYPT,
                 relay=DEFAULT_RELAY,
                 lbt=DEFAULT_LBT,
                 wor=DEFAULT_WOR):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.m0_pin = m0_pin
        self.m1_pin = m1_pin

        self.freq = freq_mhz
        self.addr = addr
        self.power = power
        self.rssi = bool(rssi)
        self.air_speed = air_speed
        self.net_id = net_id
        self.buffer_size = buffer_size
        self.crypt = crypt
        self.relay = relay
        self.lbt = lbt
        self.wor = wor

        self.cfg_reg = DEFAULT_CFG_REG.copy()
        self.start_freq = 410
        self.offset_freq = 0

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.m0_pin, GPIO.OUT)
        GPIO.setup(self.m1_pin, GPIO.OUT)

        # Serial
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.5)
        self.ser.flushInput()
        time.sleep(0.05)

        # Create initial configuration on the module
        self.set(self.freq, self.addr, self.power, self.rssi,
                 air_speed=self.air_speed, net_id=self.net_id,
                 buffer_size=self.buffer_size, crypt=self.crypt,
                 relay=self.relay, lbt=self.lbt, wor=self.wor)

    def _enter_config_mode(self):
        # M0 low, M1 high — per provided example
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.HIGH)
        time.sleep(0.1)

    def _enter_transceiver_mode(self):
        # Both low
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.LOW)
        time.sleep(0.1)

    def _send_cfg_and_wait(self, cfg_bytes, tries=2):
        for attempt in range(tries):
            self.ser.flushInput()
            self.ser.write(bytes(cfg_bytes))
            time.sleep(0.2)
            if self.ser.in_waiting:
                time.sleep(0.1)
                r = self.ser.read(self.ser.in_waiting)
                if len(r) > 0 and r[0] == 0xC1:
                    # success (module echoed success code)
                    return True, r
                else:
                    return False, r
            else:
                # no response; retry
                time.sleep(0.2)
        return False, b''

    def set(self, freq, addr, power, rssi, air_speed=2400,
            net_id=0, buffer_size=240, crypt=0, relay=False, lbt=False, wor=False):
        """
        Build and write cfg_reg, switch to config mode, write to serial, then back to transceiver mode.
        """
        self._enter_config_mode()

        low_addr = addr & 0xff
        high_addr = (addr >> 8) & 0xff
        net_id_temp = net_id & 0xff

        if freq > 850:
            # like example code: store offset
            freq_temp = freq - 850
            self.start_freq = 850
            self.offset_freq = freq_temp
        elif freq > 410:
            freq_temp = freq - 410
            self.start_freq = 410
            self.offset_freq = freq_temp
        else:
            freq_temp = freq - self.start_freq

        air_speed_temp = LORA_AIR_SPEED_MAP.get(air_speed, 0x00)
        buffer_size_temp = LORA_BUFFER_SIZE_MAP.get(buffer_size, 0x00)
        power_temp = LORA_POWER_MAP.get(power, 0x00)

        rssi_temp = 0x80 if rssi else 0x00
        l_crypt = crypt & 0xff
        h_crypt = (crypt >> 8) & 0xff

        if not relay:
            self.cfg_reg[3] = high_addr
            self.cfg_reg[4] = low_addr
            self.cfg_reg[5] = net_id_temp
            self.cfg_reg[6] = 0x00 + air_speed_temp   # 0x00 placeholder for UART baud bits + air speed mapping
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp & 0xff
            self.cfg_reg[9] = 0x43 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        else:
            # simplified relay mode pattern from example
            self.cfg_reg[3] = 0x01
            self.cfg_reg[4] = 0x02
            self.cfg_reg[5] = 0x03
            self.cfg_reg[6] = 0x00 + air_speed_temp
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp & 0xff
            self.cfg_reg[9] = 0x03 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt

        # send config bytes and wait for confirmation
        ok, resp = self._send_cfg_and_wait(self.cfg_reg, tries=3)
        if not ok:
            print("[WARN] LoRa config: module didn't acknowledge, response:", resp)
        else:
            print("[INFO] LoRa configured, response:", resp)

        # return to transceiver mode
        self._enter_transceiver_mode()

    def send(self, data_bytes):
        """
        Put module in TX mode (transceiver mode), write bytes.
        Caller must ensure data_bytes is bytes.
        """
        # ensure transceiver mode
        self._enter_transceiver_mode()
        time.sleep(0.05)
        self.ser.write(data_bytes)
        # small pause to allow TX to initiate
        time.sleep(0.05)

    def receive_available(self):
        return self.ser.in_waiting > 0

    def read_all(self):
        if self.ser.in_waiting:
            time.sleep(0.1)
            return self.ser.read(self.ser.in_waiting)
        return b''

    def close(self):
        try:
            self.ser.close()
        except:
            pass
        GPIO.output(self.m0_pin, GPIO.LOW)
        GPIO.output(self.m1_pin, GPIO.LOW)

# -------------------------
# Main application: GNSS read, speed calc, CSV log, and periodic send
# -------------------------
class GNSSLoRaSender:
    def __init__(self,
                 gps_port=GPS_PORT, gps_baud=GPS_BAUD,
                 lora_port=LORA_PORT, lora_baud=LORA_BAUD,
                 send_interval=SEND_INTERVAL):
        # serial connections
        self.gps_port = gps_port
        self.gps_baud = gps_baud
        self.lora_port = lora_port
        self.lora_baud = lora_baud
        self.send_interval = send_interval

        # device instances
        self.gps_ser = None
        self.lora = None

        # last known position/time
        self.last_lat = None
        self.last_lon = None
        self.last_time = None
        self.last_fix_valid = False
        self.last_valid_fix_time = time.time()

        self.speed_history = deque(maxlen=5)

        # csv logging
        ensure_dir(CSV_DIR)
        self.csv_file = os.path.join(CSV_DIR, f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_utc", "lat", "lon", "speed_kmh", "note"])

        # console task flags
        self.cpu_broadcast = False
        self.cpu_broadcast_interval = 10.0
        self.cpu_broadcast_timer = None

        self.keep_running = True

    def open_ports(self):
        # open GPS serial
        if not os.path.exists(self.gps_port):
            raise FileNotFoundError(f"GPS port {self.gps_port} not found")
        self.gps_ser = serial.Serial(self.gps_port, self.gps_baud, timeout=0.5)
        print(f"[INFO] GPS UART opened on {self.gps_port} @ {self.gps_baud}")

        # open LoRa via Waveshare class (which opens serial internally)
        self.lora = WaveshareLoRa(self.lora_port, self.lora_baud, M0_PIN, M1_PIN,
                                  freq_mhz=DEFAULT_FREQ_MHZ, addr=DEFAULT_ADDR,
                                  power=DEFAULT_POWER, rssi=DEFAULT_RSSI_ENABLE,
                                  air_speed=DEFAULT_AIR_SPEED, net_id=DEFAULT_NET_ID,
                                  buffer_size=DEFAULT_BUFFER_SIZE, crypt=DEFAULT_CRYPT,
                                  relay=DEFAULT_RELAY)

        print(f"[INFO] LoRa UART opened on {self.lora_port} @ {self.lora_baud}")

    def log_row(self, lat, lon, speed_kmh, note=""):
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_timestamp_str(), f"{lat:.7f}", f"{lon:.7f}", f"{speed_kmh:.2f}", note])

    def compute_and_send(self, lat, lon, current_time):
        # compute speed
        speed_kmh = 0.0
        if self.last_fix_valid and self.last_lat is not None:
            dist_km = calc_haversine_km(self.last_lat, self.last_lon, lat, lon)
            delta = current_time - self.last_time if self.last_time else None
            if delta and delta > 0 and dist_km > 0.00005:
                raw = (dist_km / delta) * 3600.0
                self.speed_history.append(raw)
                speed_kmh = sum(self.speed_history) / len(self.speed_history)
            else:
                speed_kmh = 0.0
        else:
            speed_kmh = 0.0

        self.last_lat, self.last_lon, self.last_time = lat, lon, current_time
        self.last_fix_valid = True
        self.last_valid_fix_time = time.time()

        # prepare payload (compact ASCII CSV)
        # Format: "addr,freq,payload"
        payload = f"{DEFAULT_ADDR},{DEFAULT_FREQ_MHZ},POS:{lat:.6f},{lon:.6f},{speed_kmh:.2f}"
        # send over LoRa
        try:
            self.lora.send(payload.encode('utf-8'))
            print(f"[LORA] {payload}")
            self.log_row(lat, lon, speed_kmh, "OK")
        except Exception as e:
            print("[ERROR] LoRa send failed:", e)
            self.log_row(lat, lon, speed_kmh, f"send_err:{e}")

    def send_gnss_not_fix(self):
        note = "GNSS NOT FIX"
        payload = f"{DEFAULT_ADDR},{DEFAULT_FREQ_MHZ},{note}"
        try:
            self.lora.send(payload.encode('utf-8'))
            print(f"[LORA] {payload}")
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([current_timestamp_str(), "", "", "", note])
        except Exception as e:
            print("[ERROR] LoRa send failed:", e)

    def read_gps_once(self):
        try:
            line = self.gps_ser.readline().decode('ascii', errors='ignore').strip()
            return line
        except Exception as e:
            # serial read error
            return ""

    def start_loop(self):
        print("[INFO] Starting main loop. Press Esc to exit, 'i' to send manual, 's' to toggle CPU broadcast.")
        # set terminal to raw to capture Esc/i/s without Enter
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            next_send_time = time.monotonic()
            while self.keep_running:
                # read any GPS lines available until send time
                now = time.monotonic()
                # read all available NMEA lines until it's time to send
                while time.monotonic() < next_send_time:
                    # use non-blocking select to check stdin and serial
                    if self.gps_ser.in_waiting:
                        line = self.read_gps_once()
                        if line:
                            # basic NMEA filtering
                            if line.startswith('$GPGGA') or line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                                try:
                                    msg = pynmea2.parse(line)
                                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude') and msg.latitude and msg.longitude:
                                        lat = float(msg.latitude)
                                        lon = float(msg.longitude)
                                        # get timestamp from NMEA if available
                                        gps_time = None
                                        if hasattr(msg, 'timestamp') and msg.timestamp:
                                            now_utc = datetime.utcnow()
                                            gps_time = datetime.combine(now_utc.date(), msg.timestamp, tzinfo=timezone.utc).timestamp()
                                        t = gps_time if gps_time else time.time()
                                        # store last fix but don't send here (we send at interval)
                                        self.last_lat, self.last_lon, self.last_time = lat, lon, t
                                        self.last_fix_valid = True
                                        self.last_valid_fix_time = time.time()
                                except pynmea2.ParseError:
                                    pass
                            # log raw GPS to stdout for debug
                            if line:
                                print(f"[GPS] {line}")
                    # handle user keystroke without blocking
                    if select.select([sys.stdin], [], [], 0)[0]:
                        ch = sys.stdin.read(1)
                        if ch == '\x1b':  # Esc
                            print("[INFO] Esc pressed — exiting loop.")
                            self.keep_running = False
                            break
                        elif ch == 'i':
                            # manual send: read latest known position or send custom payload
                            if self.last_fix_valid and self.last_lat is not None:
                                payload = f"{DEFAULT_ADDR},{DEFAULT_FREQ_MHZ},MANUAL_POS:{self.last_lat:.6f},{self.last_lon:.6f}"
                                self.lora.send(payload.encode('utf-8'))
                                print(f"[LORA][MANUAL] {payload}")
                                self.log_row(self.last_lat, self.last_lon, 0.0, "manual")
                            else:
                                print("[WARN] No GPS fix to send manually.")
                        elif ch == 's':
                            # toggle CPU broadcast
                            if not self.cpu_broadcast:
                                self.cpu_broadcast = True
                                print("[INFO] Starting CPU broadcast every", self.cpu_broadcast_interval, "s. Press 'c' to cancel.")
                                # start a separate thread
                                threading.Thread(target=self._cpu_broadcast_task, daemon=True).start()
                            else:
                                self.cpu_broadcast = False
                                print("[INFO] CPU broadcast toggled off.")
                        elif ch == 'c':
                            if self.cpu_broadcast:
                                self.cpu_broadcast = False
                                print("[INFO] CPU broadcast cancelled.")
                    time.sleep(0.01)

                # Time to send an update
                now_time = time.time()
                if self.last_fix_valid and (now_time - self.last_valid_fix_time) <= NO_FIX_TIMEOUT:
                    # send last known position (using last_time stored)
                    if self.last_lat is not None:
                        self.compute_and_send(self.last_lat, self.last_lon, self.last_time)
                else:
                    # no valid fix in recent period -> send GNSS NOT FIX
                    self.send_gnss_not_fix()

                next_send_time += self.send_interval

        finally:
            # restore terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.shutdown()

    def _cpu_broadcast_task(self):
        while self.cpu_broadcast and self.keep_running:
            # get CPU temp (Linux)
            try:
                temp = None
                if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                    with open("/sys/class/thermal/thermal_zone0/temp") as f:
                        t = int(f.read().strip())
                        temp = t / 1000.0
                else:
                    # fallback using vcgencmd if available
                    p = subprocess.Popen(["/opt/vc/bin/vcgencmd", "measure_temp"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                    out = p.communicate()[0].decode() if p else ""
                    if out and "temp=" in out:
                        temp = float(out.split("temp=")[1].split("'")[0])
                payload = f"{DEFAULT_ADDR},{DEFAULT_FREQ_MHZ},CPU_TEMP:{temp:.2f}" if temp is not None else f"{DEFAULT_ADDR},{DEFAULT_FREQ_MHZ},CPU_TEMP:NA"
                self.lora.send(payload.encode('utf-8'))
                print(f"[LORA][CPU] {payload}")
            except Exception as e:
                print("[ERROR] CPU broadcast failed:", e)
            # sleep for interval or until cancelled
            for _ in range(int(self.cpu_broadcast_interval * 10)):
                if not self.cpu_broadcast or not self.keep_running:
                    break
                time.sleep(0.1)

    def shutdown(self):
        print("[INFO] shutting down, closing serials and GPIO...")
        try:
            if self.lora:
                self.lora.close()
            if self.gps_ser:
                self.gps_ser.close()
        except Exception:
            pass
        GPIO.cleanup()
        print("[INFO] closed.")

# -------------------------
# Entry point
# -------------------------
def main():
    app = GNSSLoRaSender(gps_port=GPS_PORT, gps_baud=GPS_BAUD,
                         lora_port=LORA_PORT, lora_baud=LORA_BAUD,
                         send_interval=SEND_INTERVAL)
    try:
        app.open_ports()
    except Exception as e:
        print("[FATAL] Failed to open ports:", e)
        GPIO.cleanup()
        sys.exit(1)

    # run main loop (blocks until Esc pressed)
    app.start_loop()

if __name__ == "__main__":
    main()
