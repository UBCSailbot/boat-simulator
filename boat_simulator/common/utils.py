from boat_simulator.common.types import Scalar


def bound_to_180(angle: Scalar) -> Scalar:
    raise NotImplementedError()


def bound_to_360(angle: Scalar) -> Scalar:
    raise NotImplementedError()


def symmetric_clockwise_angle_to_true_bearing(angle: Scalar) -> Scalar:
    raise NotImplementedError()


def true_bearing_to_symmetric_clockwise_angle(angle: Scalar) -> Scalar:
    raise NotImplementedError()
