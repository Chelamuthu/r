import SX126X
import time

# Initialize LoRa module
LoRa = SX126X.LoRa()
LoRa.set_freq(433.0)   # Frequency in MHz

print("LoRa Receiver started... Waiting for messages")

try:
    while True:
        data = LoRa.receive()
        if data:
            print("Received:", data.decode('utf-8'))
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Receiver stopped.")
