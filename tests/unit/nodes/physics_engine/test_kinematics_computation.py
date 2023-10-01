"""Tests classes and functions in boat_simulator/nodes/physics_engine/kinematics_computation.py"""

from dataclasses import Field, dataclass, field
from typing import Tuple

import numpy as np
import pytest
from numpy.typing import ArrayLike

import boat_simulator.common.constants as constants
import boat_simulator.common.utils as utils
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.kinematics_formulas import KinematicsFormulas


@dataclass
class ExpectedData:
    position: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    velocity: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    acceleration: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))


class TestKinematicsComputation:
    @dataclass
    class ExpectedData:
        position: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
        velocity: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
        acceleration: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))

    def __test_ang_kinematics(
        self,
        timestep: Scalar,
        inertia_inverse: ArrayLike,
        net_torque: ArrayLike,
        prev_expected_data: ExpectedData,
        relative_data: KinematicsData,
        global_data: KinematicsData,
    ) -> Tuple[KinematicsData, Scalar]:
        expected_ang_acc = utils.bound_to_180(
            KinematicsFormulas.next_ang_acceleration(net_torque, inertia_inverse)
        )
        assert np.isclose(relative_data.angular_acceleration, expected_ang_acc).all()
        assert np.isclose(global_data.angular_acceleration, expected_ang_acc).all()

        expected_ang_vel = utils.bound_to_180(
            KinematicsFormulas.next_velocity(
                prev_expected_data.velocity,
                prev_expected_data.acceleration,
                timestep,
            )
        )
        assert np.isclose(relative_data.angular_velocity, expected_ang_vel).all()
        assert np.isclose(global_data.angular_velocity, expected_ang_vel).all()

        expected_ang_pos = utils.bound_to_180(
            KinematicsFormulas.next_position(
                prev_expected_data.position,
                prev_expected_data.velocity,
                prev_expected_data.acceleration,
                timestep,
            )
        )
        assert np.isclose(
            relative_data.angular_position, np.array([0, 0, 0], dtype=np.float32)
        ).all()  # relative angular position is unused
        assert np.isclose(global_data.angular_position, expected_ang_pos).all()

        # update previous expected data
        prev_expected_data.acceleration = expected_ang_acc
        prev_expected_data.velocity = expected_ang_vel
        prev_expected_data.position = expected_ang_pos

        yaw_radians = expected_ang_pos[constants.ORIENTATION_INDICES.YAW.value]
        return (prev_expected_data, yaw_radians)

    def __test_rel_lin_kinematics(
        self,
        timestep: Scalar,
        mass: Scalar,
        net_force: ArrayLike,
        prev_expected_data: ExpectedData,
        actual_data: KinematicsData,
    ) -> KinematicsData:
        expected_rel_lin_acc = KinematicsFormulas.next_lin_acceleration(mass, net_force)
        assert np.isclose(actual_data.linear_acceleration, expected_rel_lin_acc).all()

        expected_rel_lin_vel = KinematicsFormulas.next_velocity(
            prev_expected_data.velocity,
            prev_expected_data.acceleration,
            timestep,
        )
        assert np.isclose(actual_data.linear_velocity, expected_rel_lin_vel).all()

        expected_rel_lin_pos = KinematicsFormulas.next_position(
            prev_expected_data.position,
            prev_expected_data.velocity,
            prev_expected_data.acceleration,
            timestep,
        )
        assert np.isclose(
            actual_data.linear_position, np.array([0, 0, 0], dtype=np.float32)
        ).all()  # relative linear position is unused

        # update previous expected data
        prev_expected_data.acceleration = expected_rel_lin_acc
        prev_expected_data.velocity = expected_rel_lin_vel
        prev_expected_data.position = expected_rel_lin_pos

        return prev_expected_data

    def __test_glo_lin_kinematics(
        self,
        timestep: Scalar,
        mass: Scalar,
        net_force: ArrayLike,
        prev_expected_data: ExpectedData,
        actual_data: KinematicsData,
    ) -> KinematicsData:
        expected_glo_lin_acc = KinematicsFormulas.next_lin_acceleration(mass, net_force)
        assert np.isclose(actual_data.linear_acceleration, expected_glo_lin_acc).all()

        expected_glo_lin_vel = KinematicsFormulas.next_velocity(
            prev_expected_data.velocity,
            prev_expected_data.acceleration,
            timestep,
        )
        assert np.isclose(actual_data.linear_velocity, expected_glo_lin_vel).all()

        expected_glo_lin_pos = KinematicsFormulas.next_position(
            prev_expected_data.position,
            prev_expected_data.velocity,
            prev_expected_data.acceleration,
            timestep,
        )
        assert np.isclose(actual_data.linear_position, expected_glo_lin_pos).all()

        # update previous expected data
        prev_expected_data.acceleration = expected_glo_lin_acc
        prev_expected_data.velocity = expected_glo_lin_vel
        prev_expected_data.position = expected_glo_lin_pos

        return prev_expected_data

    @pytest.mark.parametrize(
        "timestep, mass, inertia, rel_net_force, net_torque",
        [
            (
                0.1,
                10.0,
                np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]], dtype=np.float32),
                np.array([1.0, 2.0, 3.0], dtype=np.float32),
                np.array([0.1, 0.2, 0.3], dtype=np.float32),
            ),
            (
                0.05,
                15.0,
                np.array([[0.5, 0.5, 0.5], [0.0, 0.5, 0.5], [0.0, 0.0, 0.5]], dtype=np.float32),
                np.array([2.0, 3.0, 1.0], dtype=np.float32),
                np.array([0.2, 0.3, 0.1], dtype=np.float32),
            ),
        ],
    )
    def test_boat_kinematics(self, timestep, mass, inertia, rel_net_force, net_torque):
        inertia_inverse = np.linalg.inv(inertia)
        boat_kinematics = BoatKinematics(timestep, mass, inertia)

        prev_expected_ang_data = ExpectedData()
        prev_expected_rel_lin_data = ExpectedData()
        prev_expected_glo_lin_data = ExpectedData()

        num_steps = 3

        for _ in range(num_steps):
            relative_data, global_data = boat_kinematics.step(rel_net_force, net_torque)

            prev_expected_ang_data, yaw_radians = self.__test_ang_kinematics(
                timestep,
                inertia_inverse,
                net_torque,
                prev_expected_ang_data,
                relative_data,
                global_data,
            )

            prev_expected_rel_lin_data = self.__test_rel_lin_kinematics(
                timestep,
                mass,
                rel_net_force,
                prev_expected_rel_lin_data,
                relative_data,
            )

            # z-directional acceleration and velocity are neglected
            glo_net_force = rel_net_force * np.array([np.cos(yaw_radians), np.sin(yaw_radians), 0])
            prev_expected_glo_lin_data = self.__test_glo_lin_kinematics(
                timestep,
                mass,
                glo_net_force,
                prev_expected_glo_lin_data,
                global_data,
            )
