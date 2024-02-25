from typing import Any
from numpy.typing import NDArray
from collections import deque
import numpy as np


from boat_simulator.common.types import Scalar, ScalarOrArray

from boat_simulator.common.generators import (
    MVGaussianGenerator,
    GaussianGenerator,
)


class Sensor:
    """

    Interface for sensors in the Boat Simulation.


    Data delay and noise models are supported.

    Delay model will delay sensor value updates by one update cycle:
        - Sensor has initial data x0 at t = 0
        - Sensor provided new data x_1 at t = 1
        - Sensor provided new data x_2 at t = 2. At t = 2, x1 is registered into the sensor.
        - Sensor provided new data x_i at t = i. At t = i, x_{i-1} is registered into the sensor.

    Noise model will add noise to sensor values drawn from a
    Gaussian or Multi-variate Gaussian distribution.
    """

    def __init__(self, enable_delay: bool = False, enable_noise: bool = False) -> None:
        """

        Args:
            enable_delay (bool, optional): _description_. Defaults to False.
            enable_noise (bool, optional): _description_. Defaults to False.
        """

        self.enable_delay = enable_delay
        self.enable_noise = enable_noise

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


class WindSensor(Sensor):
    """
    Abstraction for wind sensor.

    Properties:
        wind (ScalarOrArray): Wind x, y components or single value
        enable_noise (bool): Enables noise for fields. False by default.
        enable_delay (bool): Enables delay for fields. False by default.
    """

    wind: ScalarOrArray

    def __init__(
        self,
        wind: ScalarOrArray,
        enable_noise: bool = False,
        enable_delay: bool = False,
    ) -> None:
        super().__init__(enable_noise=enable_noise, enable_delay=enable_delay)
        self._wind = wind

        self.wind_queue_next = False
        self.wind_queue: deque = deque()
        self.wind_noisemaker = MVGaussianGenerator(mean=np.array([0, 0]), cov=np.eye(2))

    @property  # type: ignore
    def wind(self) -> ScalarOrArray:
        # TODO: Ensure attribute value and noisemakers are using the same value shape.
        # - wind scalars should add with noise scalars.
        # - wind vectors should add with noise vectors.

        return (
            self._wind + self.wind_noisemaker.next()  # type: ignore
            if self.enable_noise
            else self._wind
        )

    @wind.setter
    def wind(self, wind: ScalarOrArray):

        self.wind_queue.append(wind)

        if self.wind_queue_next or not self.enable_delay:
            self._wind = self.wind_queue.popleft()
        else:
            self.wind_queue_next = True


class GPS(Sensor):
    """
    Abstraction for GPS.

    Properties:
        lat_lon (NDArray): Boat latitude and longitude (2x1 array)
        speed (Scalar): Boat speed
        heading (Scalar): Boat heading
        enable_noise (bool): Enables noise for fields. False by default.
        enable_delay (bool): Enables delay for fields. False by default.
    """

    lat_lon: NDArray
    speed: Scalar
    heading: Scalar

    def __init__(
        self,
        lat_lon: NDArray,
        speed: Scalar,
        heading: Scalar,
        enable_noise: bool = False,
        enable_delay: bool = False,
    ):
        super().__init__(enable_noise=enable_noise, enable_delay=enable_delay)
        self._lat_lon = lat_lon
        self._speed = speed
        self._heading = heading

        # Delay Controls
        self.lat_lon_queue_next = False
        self.lat_lon_queue: deque = deque()

        self.speed_queue_next = False
        self.speed_queue: deque = deque()

        self.heading_queue_next = False
        self.heading_queue: deque = deque()

        self.lat_lon_noisemaker: GaussianGenerator = GaussianGenerator(mean=0, stdev=1)
        self.speed_noisemaker: GaussianGenerator = GaussianGenerator(mean=0, stdev=1)
        self.heading_noisemaker: GaussianGenerator = GaussianGenerator(mean=0, stdev=1)

    @property  # type: ignore
    def lat_lon(self) -> NDArray:
        return (
            self._lat_lon + self.lat_lon_noisemaker.next()
            if self.enable_noise
            else self._lat_lon
        )

    @lat_lon.setter
    def lat_lon(self, lat_lon: NDArray):

        self.lat_lon_queue.append(lat_lon)

        if self.lat_lon_queue_next or not self.enable_delay:
            update_value = self.lat_lon_queue.popleft()
            self._lat_lon = update_value
        else:
            self.lat_lon_queue_next = True

    @property  # type: ignore
    def speed(self) -> Scalar:
        return (
            self._speed + self.speed_noisemaker.next()  # type: ignore
            if self.enable_noise
            else self._speed
        )

    @speed.setter
    def speed(self, speed: Scalar):

        self.speed_queue.append(speed)

        if self.speed_queue_next or not self.enable_delay:
            self._speed = self.speed_queue.popleft()
        else:
            self.speed_queue_next = True

    @property  # type: ignore
    def heading(self) -> Scalar:
        return (
            self._heading + self.heading_noisemaker.next()  # type: ignore
            if self.enable_noise
            else self._heading
        )

    @heading.setter
    def heading(self, heading: Scalar):

        self.heading_queue.append(heading)

        if self.heading_queue_next or not self.enable_delay:
            self._heading = self.heading_queue.popleft()
        else:
            self.heading_queue_next = True
