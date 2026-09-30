"""Read measured environmental data; missing values remain JSON null."""
import math
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray

from plant_inspection_pkg.models import Environment, Location


class EnvironmentClient:
    def __init__(self, node, stale_after_sec: float = 5.0):
        if not math.isfinite(stale_after_sec) or stale_after_sec <= 0:
            raise ValueError('stale_after_sec must be positive and finite')
        self._stale_after_sec = stale_after_sec
        self._lock = threading.Lock()
        self._received_at = None
        self._values = (None, None, None, None)
        # Inspection execution can block the default callback group.
        self._callback_group = ReentrantCallbackGroup()
        self._subscription = node.create_subscription(
            Float32MultiArray, '/environment/data', self._on_data, 10,
            callback_group=self._callback_group,
        )

    def _on_data(self, message: Float32MultiArray) -> None:
        # agv_node.cpp publishes [solar W/m^2, temperature C, humidity %, CO2 ppm].
        values = (None, None, None, None)
        if len(message.data) == 4:
            solar, temperature, humidity, co2 = message.data
            values = (
                float(solar) if math.isfinite(solar) else None,
                float(temperature) if math.isfinite(temperature) else None,
                float(humidity) if math.isfinite(humidity) else None,
                int(round(co2)) if math.isfinite(co2) else None,
            )
        with self._lock:
            self._values = values
            self._received_at = time.monotonic()

    def read(self, location: Location) -> Environment:
        # This sensor reports the robot's current surroundings, not per-bed data.
        with self._lock:
            if (self._received_at is None or
                    time.monotonic() - self._received_at >= self._stale_after_sec):
                solar, temperature, humidity, co2 = None, None, None, None
            else:
                solar, temperature, humidity, co2 = self._values
        return Environment(
            temperature_c=temperature,
            humidity_pct=humidity,
            co2_ppm=co2,
            solar_radiation_w_m2=solar,
        )
