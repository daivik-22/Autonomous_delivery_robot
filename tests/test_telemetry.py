import pytest
from autobot_pkg.telemetry import simulate_telemetry

def test_telemetry_data():
    data = simulate_telemetry()
    # Should return a dict with lat & lon
    assert isinstance(data, dict)
    assert 'lat' in data and 'lon' in data
    # Values should be floats
    assert isinstance(data['lat'], float)
    assert isinstance(data['lon'], float)
