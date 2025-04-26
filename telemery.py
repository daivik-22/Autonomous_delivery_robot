# src/autopkg/telemetry.py
import time
import json
import random
import paho.mqtt.client as mqtt

BROKER = 'localhost'
TOPIC = 'bot/telemetry'

def simulate_telemetry():
    client = mqtt.Client()
    client.connect(BROKER)
    while True:
        msg = {
            'lat': 12.9 + random.uniform(-0.0005,0.0005),
            'lon': 80.0 + random.uniform(-0.0005,0.0005),
            'status': 'moving'
        }
        client.publish(TOPIC, json.dumps(msg))
        print("Published:", msg)
        time.sleep(1)

if __name__ == '__main__':
    simulate_telemetry()
