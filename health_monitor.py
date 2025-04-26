# src/autopkg/health_monitor.py
import time
import paho.mqtt.client as mqtt

BROKER = 'localhost'
ALERT_TOPIC = 'bot/alert'

def simulate_health():
    client = mqtt.Client()
    client.connect(BROKER)
    stuck_counter = 0
    while True:
        # suppose encoder_delta = 0 means stuck; random here for demo
        encoder_delta = 0 if time.time() % 10 < 2 else 1
        if encoder_delta == 0:
            stuck_counter += 1
        else:
            stuck_counter = 0
        if stuck_counter > 3:
            client.publish(ALERT_TOPIC, 'Robot stuck! Last location: (12.900,80.000)')
            print("Alert sent")
            stuck_counter = 0
        time.sleep(1)

if __name__ == '__main__':
    simulate_health()
