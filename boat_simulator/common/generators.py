from abc import ABC, abstractmethod

import numpy as np


class NumberGenerator(ABC):
    def __init__(self, seed=0):
        self.seed = seed

    @abstractmethod
    def next(self):
        pass


class MVGGenerator(NumberGenerator):
    def __init__(self, mean, cov):
        self.mean = mean
        self.cov = cov
        self.next()

    def next(self):
        self.value = np.random.multivariate_normal(self.mean, self.cov)
        return self.value


MVGG = MVGGenerator(np.array([1, 1]), np.array([[1, 0], [0, 1]]))
print(MVGG.next())
