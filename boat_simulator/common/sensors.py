from dataclasses import dataclass
from boat_simulator.common.types import ScalarOrArray, Scalar
from numpy.typing import NDArray
from boat_simulator.common.generators import (
    MVGaussianGenerator,
    ConstantGenerator,
    GaussianGenerator,
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
    wind_noisemaker: MVGaussianGenerator | ConstantGenerator | None

    @property
    def wind(self) -> ScalarOrArray:
        return self._wind + self._wind_noisemaker.next() if self.wind_noisemaker else self._wind

    @wind.setter
    def wind(self, wind: ScalarOrArray):
        self._wind = wind

    @property
    def wind_noisemaker(self) -> MVGaussianGenerator | ConstantGenerator | None:
        return self._wind_noisemaker

    @wind_noisemaker.setter
    def wind_noisemaker(self, noisemaker: MVGaussianGenerator | ConstantGenerator | None):
        self._wind_noisemaker = noisemaker


@dataclass
class GPS(Sensor):
    lat_lon: NDArray
    speed: Scalar
    heading: Scalar

    lat_lon_noisemaker: GaussianGenerator | ConstantGenerator | None
    speed_noisemaker: GaussianGenerator | ConstantGenerator | None
    heading_noisemaker: GaussianGenerator | ConstantGenerator | None

    @property
    def lat_lon(self) -> NDArray:
        return (
            self._lat_lon + self._lat_lon_noisemaker.next()
            if self._lat_lon_noisemaker
            else self._lat_lon
        )

    @lat_lon.setter
    def lat_lon(self, lat_lon: NDArray):
        self._lat_lon = lat_lon

    @property
    def speed(self) -> Scalar:
        return (
            self._speed + self._speed_noisemaker.next() if self._speed_noisemaker else self._speed
        )

    @speed.setter
    def speed(self, speed: Scalar):
        self._speed = speed

    @property
    def heading(self) -> Scalar:
        return (
            self._heading + self._heading_noisemaker.next()
            if self._heading_noisemaker
            else self._heading
        )

    @heading.setter
    def heading(self, heading: NDArray):
        self._heading = heading

    @property
    def lat_lon_noisemaker(self) -> GaussianGenerator | ConstantGenerator | None:
        return self._lat_lon_noisemaker

    @lat_lon_noisemaker.setter
    def lat_lon_noisemaker(self, noisemaker):
        self._lat_lon_noisemaker = noisemaker

    @property
    def speed_noisemaker(self) -> GaussianGenerator | ConstantGenerator | None:
        return self._speed_noisemaker

    @speed_noisemaker.setter
    def speed_noisemaker(self, noisemaker):
        self._speed_noisemaker = noisemaker

    @property
    def heading_noisemaker(self) -> GaussianGenerator | ConstantGenerator | None:
        return self._heading_noisemaker

    @heading_noisemaker.setter
    def heading_noisemaker(self, noisemaker):
        self._heading_noisemaker = noisemaker
