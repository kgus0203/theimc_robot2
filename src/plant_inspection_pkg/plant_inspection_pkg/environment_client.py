import random

from plant_inspection_pkg.models import Environment, Location


class RandomEnvironmentClient:
    def read(self, location: Location) -> Environment:
        return Environment(
            temperature_c=round(random.uniform(24.0, 29.0), 1),
            humidity_pct=round(random.uniform(60.0, 80.0), 1),
            co2_ppm=random.randint(500, 800),
            illuminance_lux=random.randint(14000, 22000),
        )
