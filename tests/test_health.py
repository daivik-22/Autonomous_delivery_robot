import pytest
from autobot_pkg.health_monitor import simulate_health

def test_health_monitor():
    # Stub always returns True
    assert simulate_health() is True
