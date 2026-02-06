import math


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity in the abscence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

    # Angle
    SETPOINT = 0.2
    KP = 5.0
    KD = 5.0
    DIAGONAL_CONSTANT = 1.35

    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._simulation: bool = simulation
        self.last_error = 0.0
        self._turn: bool = False
        self._following_wall: int = 0
        self._cambio_estado: bool = True

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            The linear and angular velocity commands for the robot. They must
                v: Linear velocity [m/s].
                w: Angular velocity [rad/s].

        """
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
        v = 0.15
        w = 0.0

        dist_front = 0.1 if math.isnan(z_scan[0]) else z_scan[0]
        dist_right = (
            0.1 if math.isnan(z_scan[3 * len(z_scan) // 4]) else z_scan[3 * len(z_scan) // 4]
        )
        dist_left = 0.1 if math.isnan(z_scan[len(z_scan) // 4]) else z_scan[len(z_scan) // 4]
        dist_wall = self.SETPOINT

        if not self._turn:
            if self._cambio_estado:
                self._cambio_estado = False

                if dist_left < dist_right:
                    self._following_wall = -1  # left
                else:
                    self._following_wall = 1  # right

            dist_wall = dist_left if self._following_wall == -1 else dist_right

            error = self.SETPOINT - dist_wall
            der_error = (error - self.last_error) / self._dt
            w = (self.KP * error + self.KD * der_error) * self._following_wall
            self.last_error = error

            if dist_front < 0.21:
                v = 0.0
                self._turn = True
                self._cambio_estado = True

        elif self._turn:
            v = 0.0

            if self._cambio_estado:
                self._cambio_estado = False
                if dist_left < dist_right:
                    self._following_wall = -1  # left
                else:
                    self._following_wall = 1  # right
            
            if self._following_wall == -1:
                dist_diagonal = 0.1 if math.isnan(z_scan[len(z_scan) // 8]) else z_scan[len(z_scan) // 8]
            elif self._following_wall == 1:
                dist_diagonal = 0.1 if math.isnan(z_scan[7 * len(z_scan) // 8]) else z_scan[7 * len(z_scan) // 8]

            if dist_diagonal > self.DIAGONAL_CONSTANT * self.SETPOINT and dist_front > 0.6:
                w = 0.0
                self._turn = False
                self._cambio_estado = True
            else:
                w = 0.5 * self._following_wall

        return v, w
