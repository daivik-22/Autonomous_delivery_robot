# src/autopkg/__init__.py

"""
autopkg: Core modules for the autonomous delivery bot.

Provides:
  - init_db, add_tag, check_tag, add_delivery, verify_code
  - dijkstra planner
  - Navigator class for path execution
  - simulate_telemetry over MQTT
  - simulate_health for failure alerts
  - delivery_app (Flask API)
"""

from .db import init_db, add_tag, check_tag, add_delivery, verify_code
from .planner import dijkstra
from .navigator import Navigator
from .telemetry import simulate_telemetry
from .health_monitor import simulate_health
from .delivery_api import app as delivery_app

__all__ = [
    'init_db', 'add_tag', 'check_tag',
    'add_delivery', 'verify_code',
    'dijkstra', 'Navigator',
    'simulate_telemetry', 'simulate_health',
    'delivery_app'
]
