from dataclasses import dataclass

from typing import Optional, Any
from numpy.typing import NDArray


from boat_simulator.common.types import Scalar, ScalarOrArray

from boat_simulator.common.generators import (
    ConstantGenerator,
    MVGaussianGenerator,
    GaussianGenerator,
)

WindSensorGenerators = Optional[MVGaussianGenerator | ConstantGenerator]
GPSGenerators = Optional[GaussianGenerator | ConstantGenerator]


@dataclass
class Sensor:
    """Interface for sensors in the Boat Simulation."""

    def update(self, **kwargs):
        """
        Update attributes in Sensor using keyword arguments.

        Usage: Sensor.update(attr1=val1, attr2=val2, ...)

        Raises:
            ValueError: If kwarg is not a defined attribute in Sensor
        """
        for attr_name, attr_val in kwargs.items():
            if attr_name in self.__annotations__:
                setattr(self, attr_name, attr_val)
            else:
                raise ValueError(
                    f"{attr_name} not a property in {self.__class__.__name__} \
                    expected one of {self.__annotations__}"
                )

    def read(self, key: str) -> Any:
        """
        Read the value from an attribute in Sensor.

        Args:
            key (str): Attribute name to read from

        Raises:
            ValueError: If key is not an a defined attribute in Sensor

        Returns:
            Any: Value stored in attribute with name supplied in "key" argument
        """
        if key in self.__annotations__:
            return getattr(self, key)
        else:
            raise ValueError(
                f"{key} not a property in {self.__class__.__name__}. \
                Available keys: {self.__annotations__}"
            )


@dataclass
class WindSensor(Sensor):
    wind: ScalarOrArray
    wind_noisemaker: WindSensorGenerators = None

    @property  # type: ignore
    def wind(self) -> ScalarOrArray:
        return (
            self._wind + self.wind_noisemaker.next()  # type: ignore
            if self.wind_noisemaker is not None
            else self._wind
        )

    @wind.setter
    def wind(self, wind: ScalarOrArray):
        self._wind = wind


@dataclass
class GPS(Sensor):
    lat_lon: NDArray
    speed: Scalar
    heading: Scalar

    lat_lon_noisemaker: GPSGenerators = None  # type: ignore
    speed_noisemaker: GPSGenerators = None  # type: ignore
    heading_noisemaker: GPSGenerators = None  # type: ignore

    @property  # type: ignore
    def lat_lon(self) -> NDArray:
        return (
            self._lat_lon + self.lat_lon_noisemaker.next()
            if self.lat_lon_noisemaker is not None
            else self._lat_lon
        )

    @lat_lon.setter
    def lat_lon(self, lat_lon: NDArray):
        self._lat_lon = lat_lon

    @property  # type: ignore
    def speed(self) -> Scalar:
        return (
            self._speed + self.speed_noisemaker.next()  # type: ignore
            if self.speed_noisemaker is not None
            else self._speed
        )

    @speed.setter
    def speed(self, speed: Scalar):
        self._speed = speed

    @property  # type: ignore
    def heading(self) -> Scalar:
        return (
            self._heading + self.heading_noisemaker.next()  # type: ignore
            if self.heading_noisemaker is not None
            else self._heading
        )

    @heading.setter
    def heading(self, heading: Scalar):
        self._heading = heading
