import numpy as np

class Policy:
    """A policy to control the robot's hand to draw a circle"""
    def __init__(self, center, speed) -> None:
        assert isinstance(center, np.ndarray) and center.shape == (3,)
        self.center = center
        self.speed = speed

    def __call__(self, robot_state: np.ndarray) -> np.ndarray:
        """
        A policy to control the robot's hand to draw a circle.
        Input:
            - robot_state: O_T_EE_c of the robot (a 16-D numpy ndarray) [m]
        Output:
            - velocity: the velocity of the robot's hand (v_x, v_y, v_z, 0, 0, 0) [m/s]
        """
        assert isinstance(robot_state, np.ndarray) and robot_state.shape == (16,)

        xyz = robot_state[12: 15] # the cartesian of robot's hand
        vel = np.cross(np.array([0, 0, 1]), xyz - self.center)
        action = np.zeros(6)
        action[:3] = vel / np.linalg.norm(vel) * self.speed

        return action



