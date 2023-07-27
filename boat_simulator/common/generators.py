"""Random vector generator classes."""

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import ArrayLike

from boat_simulator.common.types import Scalar


class VectorGenerator(ABC):
    """This class's purpose is to generate random arrays. It acts as a base class for other random
    array generators.

    Attributes:
        seed (int): The seed used to seed the random number generator.
    """

    def __init__(self, seed: int = 0):
        """Initializes an instance of `VectorGenerator`. Note that this class cannot be
        instantiated directly.

        Args:
            seed (int, optional): The seed used to seed the random number generator. Defaults to 0.
        """
        self.__seed = seed

    def next(self) -> ArrayLike:
        """Generates the next random array in the sequence. This function acts as an alias to the
        function _next().

        Returns:
            ArrayLike: Random array.
        """
        return self._next()

    @abstractmethod
    def _next(self) -> ArrayLike:
        """Generates the next random array in the sequence.

        Returns:
            ArrayLike: Random array.
        """
        pass

    @property
    def seed(self) -> int:
        return self.__seed


class GaussianGenerator(VectorGenerator):
    """This class generates random scalars using a univariate gaussian distribution.

    Attributes:
        mean (Scalar): The mean of the gaussian distribution.
        stdev (Scalar): The standard deviation of the gaussian distribution. This value is
            strictly positive.
        value (Scalar): The latest generated scalar.

    Extends: VectorGenerator
    """

    def __init__(self, mean: Scalar, stdev: Scalar, seed: int = 0):
        """Initializes an instance of GaussianGenerator.

        Args:
            mean (Scalar): _description_
            stdev (Scalar): _description_
            seed (int, optional): _description_. Defaults to 0.
        """
        super().__init__(seed=seed)
        self.__mean = mean
        self.__stdev = stdev
        self.next()

    def _next(self) -> Scalar:
        raise NotImplementedError()

    @property
    def mean(self) -> Scalar:
        return self.__mean

    @property
    def stdev(self) -> Scalar:
        return self.__stdev

    @property
    def value(self) -> Scalar:
        # TODO Assign self.__value in _next and return it here
        raise NotImplementedError()


class MVGaussianGenerator(VectorGenerator):
    """This class generates random vectors using a multivariate gaussian distribution.

    Attributes:
        mean (ArrayLike): The mean of the gaussian distribution. Shape should be (N,).
        cov (ArrayLike): The covairance matrix of the gaussian distribution. Should be positive
            semi-definite and have a shape of (N,N).
        value (ArrayLike): The latest generated array.

    Extends: VectorGenerator
    """

    def __init__(self, mean: ArrayLike, cov: ArrayLike, seed: int = 0):
        """Initializes an instance of MVGaussianGenerator.

        Args:
            mean (ArrayLike): The mean of the gaussian distribution. Shape should be (N,).
            cov (ArrayLike): The covariance matrix of the gaussian distribution. Should be positive
                semo-definite and have a shape of (N,N).
            seed (int, optional): The seed that seeds the random number generator. Defaults to 0.
        """
        super().__init__(seed=seed)
        self.__mean = mean
        self.__cov = cov
        self.next()

    def _next(self) -> ArrayLike:
        self.__value = np.random.multivariate_normal(self.mean, self.cov)
        return self.__value

    @property
    def mean(self) -> ArrayLike:
        return self.__mean

    @property
    def cov(self) -> ArrayLike:
        return self.__cov

    @property
    def value(self) -> ArrayLike:
        return self.__value


class ConstantGenerator(VectorGenerator):
    """This class returns the same specified when asked to generate a new value.

    Attributes:
        constant (ArrayLike): The constant array to return upon array generation.
    """

    def __init__(self, constant: ArrayLike):
        """Initializes an instance of ConstantGenerator.

        Args:
            constant (ArrayLike): The constant array to return upon array generation.
        """
        super().__init__(seed=0)
        self.__constant = constant

    def _next(self) -> ArrayLike:
        raise NotImplementedError()

    @property
    def constant(self) -> ArrayLike:
        return self.__constant


MVGG = MVGaussianGenerator(np.array([1, 1]), np.array([[1, 0], [0, 1]]))
print(MVGG.next())
