"""Custom types used for type hinting in the boat simulator package."""

from enum import Enum
from typing import TypeVar, Union

# Any attribute of a class that extends Enum
EnumAttr = TypeVar("EnumAttr", bound=Enum)

# Scalar value that can be an integer or a float
Scalar = Union[int, float]
