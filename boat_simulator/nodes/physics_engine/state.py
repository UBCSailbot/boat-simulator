import math
import numpy as np


class BoatState():
    def __init__(self, origin=np.array([0, 0, 0]), axes=[], mass=1, moment_of_inertia=np.array([1, 1, 1]), update_frequency=1):
        self._origin = origin
        self._axes = axes
        self._mass = mass
        self._cartesian_position = np.array([0, 0, 0])
        self._cartesian_velocity = np.array([0, 0, 0])
        self._cartesian_acceleration = np.array([0, 0, 0])
        self._angular_position = np.array([0, 0, 0])
        self._angular_velocity = np.array([0, 0, 0])
        self._angular_acceleration = np.array([0, 0, 0])  # [heave, surge, sway]
        self._moment_of_inertia = moment_of_inertia
        self._update_frequency = update_frequency
        self._net_force = np.array([0, 0, 0])

    @property
    def cartesian_position(self):
        self._cartesian_position += (self.cartesian_velocity / self._update_frequency) + (self.cartesian_acceleration / (2 * self._update_frequency**2))
        return self._cartesian_position

    @property
    def cartesian_velocity(self):
        self._cartesian_velocity += (self._cartesian_acceleration / self._update_frequency)
        return self._cartesian_velocity

    @property
    def cartesian_acceleration(self):
        return self._cartesian_acceleration

    @property
    def angular_position(self):
        self._angular_position += (self.angular_velocity / self._update_frequency) + (self.angular_acceleration / (2 * self._update_frequency**2))
        return self._angular_position

    @property
    def angular_velocity(self):
        self._angular_velocity += (self.angular_acceleration / self._update_frequency)
        return self._angular_velocity

    @property
    def angular_acceleration(self):
        return self._angular_acceleration

    @property
    def model_state(self):
        return np.concatenate((self.cartesian_acceleration[:-1], self.angular_acceleration), axis=0)

    def update_kinematics(self, net_force, torque):
        self._cartesian_acceleration = (net_force / self._mass)
        self._angular_acceleration = np.divide(torque, self._moment_of_inertia)
        self._net_force = net_force

    def convert_frame(self, basis_vector):
        '''
        Returns the boat state in the global reference frame.
        '''
        new_x_acceleration = self.angular_acceleration[0] * math.sin(self.angular_acceleration[1])
        new_y_acceleration = self.angular_acceleration[0] * math.cos(self.angular_acceleration[1])
        new_surge_acceleration = (1 / self._mass) * (np.dot(basis_vector, self._net_force))
        return np.array([new_x_acceleration, new_y_acceleration, new_surge_acceleration, self.angular_acceleration[1], self.angular_acceleration[2]])
import math
import numpy as np


class BoatState():
    def __init__(self, origin=np.array([0, 0, 0]), axes=[], mass=1, moment_of_inertia=np.array([1, 1, 1]), update_frequency=1):
        self._origin = origin
        self._axes = axes
        self._mass = mass
        self._cartesian_position = np.array([0, 0, 0])
        self._cartesian_velocity = np.array([0, 0, 0])
        self._cartesian_acceleration = np.array([0, 0, 0])
        self._angular_position = np.array([0, 0, 0])  # rads
        self._angular_velocity = np.array([0, 0, 0])  # rads/s
        self._angular_acceleration = np.array([0, 0, 0])  # rads/s^2
        self._moment_of_inertia = moment_of_inertia
        self._update_frequency = update_frequency
        self._net_force = np.array([0, 0, 0])

    @property
    def cartesian_position(self):
        self._cartesian_position += (self.cartesian_velocity / self._update_frequency) + (self.cartesian_acceleration / (2 * self._update_frequency**2))
        return self._cartesian_position

    @property
    def cartesian_velocity(self):
        self._cartesian_velocity += (self._cartesian_acceleration / self._update_frequency)
        return self._cartesian_velocity

    @property
    def cartesian_acceleration(self):
        return self._cartesian_acceleration

    @property
    def angular_position(self):
        self._angular_position += (self.angular_velocity / self._update_frequency) + (self.angular_acceleration / (2 * self._update_frequency**2))
        return self._angular_position

    @property
    def angular_velocity(self):
        self._angular_velocity += (self.angular_acceleration / self._update_frequency)
        return self._angular_velocity

    @property
    def angular_acceleration(self):
        return self._angular_acceleration

    @property
    def model_state(self):
        return np.concatenate(self.cartesian_acceleration[:-1], self.angular_acceleration, axis=0)

    def update_kinematics(self, net_force, torque):
        self._cartesian_acceleration = (net_force / self._mass)
        self._angular_acceleration = np.divide(torque, self._moment_of_inertia)
        self._net_force = net_force

    def convert_frame(self, basis_vector):
        '''
        Returns the boat state in the global reference frame.
        '''
        new_x_acceleration = self.angular_acceleration[0] * math.sin(self.angular_acceleration[1])
        new_y_acceleration = self.angular_acceleration[0] * math.cos(self.angular_acceleration[1])
        new_surge_acceleration = (1 / self._mass) * (np.dot(basis_vector, self._net_force))
        return np.array([new_x_acceleration, new_y_acceleration, new_surge_acceleration, self.angular_acceleration[1], self.angular_acceleration[2]])
