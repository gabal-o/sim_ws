import math
import numpy as np

class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.5,
        logger=None,
        simulation: bool = False,
    ):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._simulation: bool = simulation

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.11. Complete the function body with your code (i.e., compute v and w).
        v = 0.0
        w = 0.0
        if not self._path:
            return v, w
        current_point, current_index = self._find_closest_point(float(x), float(y))
        xl, yl = self._find_target_point(current_point, current_index)
        beta = math.atan2((yl - y), (xl - x))
        theta %= math.tau
        alpha = beta - theta
        alpha_norm = (alpha + math.pi) % (2 * math.pi) - math.pi

        v = 0.10  # Hemos asumido que tenemos un grado de libertad
        w = (2 * v * math.sin(alpha)) / self._lookahead_distance
        goal_x, goal_y = self._path[-1]
        is_goal_target = (xl, yl) == (goal_x, goal_y)
        # self._logger.warn(f"Beta: {beta} Theta: {theta} Alpha: {alpha} Alpha_norm: {alpha_norm}")
        if abs(alpha_norm) > math.radians(30) and not is_goal_target:          
            # self._logger.warn(f"Beta: {beta} Theta: {theta} Alpha: {alpha} Alpha_norm: {alpha_norm}")
            w = 0.6 * np.sign(alpha_norm)
            v = 0.0
        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
        closest_xy = min(
            self._path,
            key=lambda p: math.dist(p, (x, y)),
        )
        closest_idx = self._path.index(closest_xy)

        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.10. Complete the function body with your code (i.e., determine target_xy).

        for i in range(origin_idx, len(self._path)):
            if math.dist(origin_xy, self.path[i]) >= self._lookahead_distance:
                return self.path[i]
                
        return self.path[-1]
