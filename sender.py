import SX126X
import time

# Initialize LoRa module
LoRa = SX126X.LoRa()
LoRa.set_freq(433.0)   # Frequency in MHz
LoRa.set_tx_power(22)  # Max TX power (dBm)

print("LoRa Sender started...")

count = 0
try:
    while True:
        message = f"Hello LoRa {count}"
        LoRa.send(bytes(message, 'utf-8'))
        print("Sent:", message)
        count += 1
        time.sleep(2)  # Send every 2 seconds
except KeyboardInterrupt:
    print("Sender stopped.")
