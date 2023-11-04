from boat_simulator.common.dataline import Dataline
from boat_simulator.common.generators import MVGaussianGenerator, GaussianGenerator
from abc import ABC
import numpy as np


class Sensor(ABC):
    # in_data should be a dictionary of values mapping labels to values
    def __init__(self, in_data) -> None:
        super().__init__()
        self.__in_data = in_data
        self.__out_data = in_data
        self.__datalines = {}

    @property
    def in_data(self):
        return self.__in_data

    @property
    def out_data(self):
        return self.__out_data

    @property
    def datalines(self):
        return self.__datalines

    @datalines.setter
    def datalines(self, new_datalines):
        self.__datalines = new_datalines

    def update(self, new_data):
        for label, data in new_data.items():
            if label in self.datalines.keys():
                self.datalines[label].update(data)
            else:
                raise ValueError(
                    f"""Sensor Dataline with label {label} does not exist and cannot be
                    updated. Available labels: {self.datalines.keys()}"""
                )

    def read(self, label):
        if label not in self.datalines.keys():
            raise ValueError(
                f"""Sensor Dataline with {label} does not exist and cannot be read.
                Available labels: {self.datalines.keys()}"""
            )
        else:
            return self.datalines[label].read()


class WindSensor(Sensor):
    def __init__(self, in_data) -> None:
        super().__init__(in_data)
        # might need to pull parameters from ROS for what type of noise generator...
        self.datalines = {
            "wind": Dataline(
                data=in_data["wind"],
                noise_generator=MVGaussianGenerator(mean=[1, 1], cov=np.ones((2, 2))),
            )
        }


class GPS(Sensor):
    def __init__(self, in_data) -> None:
        super().__init__(in_data)
        self.datalines = {
            "latlon": Dataline(
                data=in_data["latlon"],
                noise_generator=MVGaussianGenerator(mean=[1, 1], cov=np.ones((2, 2))),
            ),
            "speed": Dataline(
                data=in_data["speed"], noise_generator=GaussianGenerator(mean=1, stdev=1)
            ),
            "heading": Dataline(
                data=in_data["heading"],
                noise_generator=GaussianGenerator(mean=[1, 1], cov=np.ones((2, 2))),
            ),
        }
