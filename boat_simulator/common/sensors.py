from dataclasses import dataclass

from numpy.typing import NDArray


from boat_simulator.common.types import (
    Scalar,
    ScalarOrArray,
    WindSensorGenerators,
    GPSGenerators,
)


@dataclass
class Sensor:
    def update(self, **kwargs):
        for attr_name, attr_val in kwargs.items():
            if attr_name in self.__annotations__:
                setattr(self, attr_name, attr_val)
            else:
                raise ValueError(
                    f"""{attr_name} not a property in {self.__class__.__name__}
                    expected {self.__annotations__}"""
                )

    def read(self, key):
        if key in self.__annotations__:
            return getattr(self, key)


@dataclass
class WindSensor(Sensor):
    wind: ScalarOrArray
    wind_noisemaker: WindSensorGenerators

    @property  # type: ignore
    def wind(self) -> ScalarOrArray:
        return (
            self._wind + self._wind_noisemaker.next()
            if self.wind_noisemaker
            else self._wind
        )

    @wind.setter
    def wind(self, wind: ScalarOrArray):
        self._wind = wind

    @property  # type: ignore
    def wind_noisemaker(self) -> WindSensorGenerators:
        return self._wind_noisemaker

    @wind_noisemaker.setter
    def wind_noisemaker(self, noisemaker: WindSensorGenerators):
        self._wind_noisemaker = noisemaker


@dataclass
class GPS(Sensor):
    lat_lon: NDArray
    speed: Scalar
    heading: Scalar

    lat_lon_noisemaker: GPSGenerators
    speed_noisemaker: GPSGenerators
    heading_noisemaker: GPSGenerators

    @property  # type: ignore
    def lat_lon(self) -> NDArray:
        return (
            self._lat_lon + self._lat_lon_noisemaker.next()
            if not isinstance(self._lat_lon_noisemaker, GPSGenerators)
            else self._lat_lon
        )

    @lat_lon.setter
    def lat_lon(self, lat_lon: NDArray):
        self._lat_lon = lat_lon

    @property  # type: ignore
    def speed(self) -> Scalar:
        return (
            self._speed + self._speed_noisemaker.next()
            if not isinstance(self._speed_noisemaker, GPSGenerators)
            else self._speed
        )

    @speed.setter
    def speed(self, speed: Scalar):
        self._speed = speed

    @property  # type: ignore
    def heading(self) -> Scalar:
        return (
            self._heading + self._heading_noisemaker.next()
            if not isinstance(self._heading_noisemaker, GPSGenerators)
            else self._heading
        )

    @heading.setter
    def heading(self, heading: Scalar):
        self._heading = heading

    @property  # type: ignore
    def lat_lon_noisemaker(self) -> GPSGenerators:
        return self._lat_lon_noisemaker

    @lat_lon_noisemaker.setter
    def lat_lon_noisemaker(self, noisemaker: GPSGenerators):
        self._lat_lon_noisemaker = noisemaker

    @property  # type: ignore
    def speed_noisemaker(self) -> GPSGenerators:
        return self._speed_noisemaker

    @speed_noisemaker.setter
    def speed_noisemaker(self, noisemaker: GPSGenerators):
        self._speed_noisemaker = noisemaker

    @property  # type: ignore
    def heading_noisemaker(self) -> GPSGenerators:
        return self._heading_noisemaker

    @heading_noisemaker.setter
    def heading_noisemaker(self, noisemaker: GPSGenerators):
        self._heading_noisemaker = noisemaker
