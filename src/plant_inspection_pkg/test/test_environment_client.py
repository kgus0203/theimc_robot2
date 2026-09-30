"""Validate measured data mapping and missing-data serialization."""
import json
from dataclasses import asdict
from types import SimpleNamespace

from std_msgs.msg import Float32MultiArray

from plant_inspection_pkg.environment_client import EnvironmentClient
from plant_inspection_pkg.models import Environment, Inspection, Location


def test_environment_values_and_expiry(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('plant_inspection_pkg.environment_client.time.monotonic', lambda: now[0])
    subscriptions = []
    node = SimpleNamespace(create_subscription=lambda *args, **kwargs: subscriptions.append(args))
    client = EnvironmentClient(node)
    location = Location('sj', 1, 1, 1)
    empty = dict(temperature_c=None, humidity_pct=None, co2_ppm=None, solar_radiation_w_m2=None)
    assert asdict(client.read(location)) == empty
    assert subscriptions[0][1] == '/environment/data'
    callback = subscriptions[0][2]
    callback(Float32MultiArray(data=[550., 26.5, 70., 650.]))
    assert asdict(client.read(location)) == dict(
        temperature_c=26.5, humidity_pct=70., co2_ppm=650, solar_radiation_w_m2=550.)
    now[0] = 15.0
    assert asdict(client.read(location)) == empty
    assert asdict(Environment(**json.loads(json.dumps(asdict(client.read(location)))))) == empty
    callback(Float32MultiArray(data=[0., 0., 0., 0.]))
    assert client.read(location).co2_ppm == 0
    assert client.read(location).temperature_c == 0
    assert client.read(location).solar_radiation_w_m2 == 0


def test_invalid_data_does_not_reuse_previous_measurements():
    node = SimpleNamespace(create_subscription=lambda *args, **kwargs: None)
    client = EnvironmentClient(node)
    location = Location('sj', 1, 1, 1)
    client._on_data(Float32MultiArray(data=[100., float('nan'), 55., float('inf')]))
    result = client.read(location)
    assert result.temperature_c is None
    assert result.humidity_pct == 55.
    assert result.co2_ppm is None
    assert result.solar_radiation_w_m2 == 100.
    client._on_data(Float32MultiArray(data=[float('nan'), 20., 55., 650.]))
    assert client.read(location).solar_radiation_w_m2 is None
    client._on_data(Float32MultiArray(data=[1., 2.]))
    assert all(value is None for value in asdict(client.read(location)).values())


def test_legacy_queued_inspection_does_not_convert_lux_to_solar():
    data = dict(
        inspection_id='old', timestamp='2026-09-29T11:00:00+09:00',
        location=dict(region='sj', device_number=1, bed_id=1, hole_id=1),
        environment=dict(temperature_c=25., humidity_pct=60., co2_ppm=500,
                         illuminance_lux=15000), captures=[],
    )
    result = Inspection.from_dict(data).to_dict()
    assert result['environment']['solar_radiation_w_m2'] is None
    assert 'illuminance_lux' not in result['environment']
    assert data['environment']['illuminance_lux'] == 15000
    result['environment']['solar_radiation_w_m2'] = 123.5
    assert Inspection.from_dict(result).environment.solar_radiation_w_m2 == 123.5
