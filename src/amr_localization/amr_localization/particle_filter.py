import datetime
import math
import numpy as np
import os
import pytz
import random

from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        sensor_range_max: float = 8.0,
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        initial_pose_sigma: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        logger=None,
        simulation: bool = False,
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._global_localization: bool = global_localization
        self._initial_particle_count: int = particle_count
        self._initial_pose: tuple[float, float, float] = initial_pose
        self._initial_pose_sigma: tuple[float, float, float] = initial_pose_sigma
        self._logger = logger
        self._mean_likelihood: float = float("inf")
        self._particle_count: int = particle_count
        self._sensor_range_min: float = sensor_range_min
        self._sensor_range_max: float = sensor_range_max
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._simulation: bool = simulation
        self._iteration: int = 0

        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        # TODO: 3.10. Complete the missing function body with your code.
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))
        particles_sincos = np.array(
            [[particle[0], particle[1], math.sin(particle[-1]), math.cos(particle[-1])]
            for particle in self._particles]
        )
        clustering = DBSCAN(eps=0.2, min_samples=20).fit(particles_sincos)
        n_clusters = len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)
        min_number_particles = 300
        max_number_particles = self._initial_particle_count
        wanted_particles = (
            min_number_particles * n_clusters
            if min_number_particles * n_clusters < max_number_particles
            else max_number_particles
        )
        self._particle_count = wanted_particles
        
        if n_clusters == 1 and self._mean_likelihood > 1e-6:
            localized = True
            self._particle_count = 50
            pose_sincos = (
                np.mean(particles_sincos[:, 0]),
                np.mean(particles_sincos[:, 1]),
                np.mean(particles_sincos[:, 2]),
                np.mean(particles_sincos[:, 3]),
            )
            pose = pose_sincos[0], pose_sincos[1], math.atan2(pose_sincos[-2], pose_sincos[-1])
        return localized, pose

    @property
    def mean_likelihood(self) -> float:
        """Mean particle likelihood before normalization."""
        return self._mean_likelihood

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # TODO: 3.5. Complete the function body with your code.
        for i in range(len(self._particles)):
            v_new = np.random.normal(v, self._sigma_v)
            w_new = np.random.normal(w, self._sigma_w)
            x_last = self._particles[i][0]
            y_last = self._particles[i][1]
            x = self._particles[i][0] + v_new * math.cos(self._particles[i][2]) * self._dt
            y = self._particles[i][1] + v_new * math.sin(self._particles[i][2]) * self._dt
            theta = (self._particles[i][2] + w_new * self._dt) % (2 * math.pi)
            intersection, _ = self._map.check_collision([(x_last, y_last), (x, y)], False)
            if intersection:
                x, y = intersection
            self._particles[i][0] = x
            self._particles[i][1] = y
            self._particles[i][2] = theta

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: 3.9. Complete the function body with your code (i.e., replace the pass statement).

        weights_unnormalized = np.array(
            [self._measurement_probability(measurements, p) for p in self._particles]
        )
        w_sum = weights_unnormalized.sum()
        if w_sum == 0.0:
            self.reset()
            return
        #  normalizar y construir CDF
        weights = weights_unnormalized / w_sum
        cdf = np.cumsum(weights)

        #  Stochastic Universal Sampling (systematic resampling)
        step = 1.0 / self._particle_count
        u0 = np.random.uniform(0.0, step)
        u = u0 + step * np.arange(self._particle_count)

        #  mapear punteros u a índices vía CDF
        idx = np.digitize(u, cdf, right=False)

        #  construir nuevo conjunto de partículas
        self._particles = np.array([self._particles[i] for i in idx])
        weights_unnormalized = np.array([weights_unnormalized[i] for i in idx])
        self._mean_likelihood = float(np.median(weights_unnormalized))

    def reset(self) -> None:
        """Reinitialize the particle set with the original configuration."""
        self._particles = self._init_particles(
            self._initial_particle_count,
            self._global_localization,
            self._initial_pose,
            self._initial_pose_sigma,
        )
        self._particle_count = self._initial_particle_count
        self._mean_likelihood = float("inf")

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(
        self,
        particle_count: int,
        global_localization: bool,
        initial_pose: tuple[float, float, float],
        initial_pose_sigma: tuple[float, float, float],
    ) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        particles = np.empty((particle_count, 3), dtype=object)

        # TODO: 3.4. Complete the missing function body with your code.
        xmin, ymin, xmax, ymax = self._map.bounds()
        valid_thetas = [0, math.pi / 2, math.pi, 3 * math.pi / 2]
        if global_localization:
            for i in range(particle_count):
                x = np.random.uniform(xmin, xmax)
                y = np.random.uniform(ymin, ymax)

                while not self._map.contains((x, y)):
                    x = np.random.uniform(xmin, xmax)
                    y = np.random.uniform(ymin, ymax)

                theta = valid_thetas[np.random.randint(0, 4)]
                particles[i] = (x, y, theta)
        else:
            for i in range(particle_count):
                x = np.random.normal(initial_pose[0], initial_pose_sigma[0])
                y = np.random.normal(initial_pose[1], initial_pose_sigma[1])

                while not self._map.contains((x, y)):
                    x = np.random.normal(initial_pose[0], initial_pose_sigma[0])
                    y = np.random.normal(initial_pose[1], initial_pose_sigma[1])

                theta = np.random.normal(initial_pose[2], initial_pose_sigma[2])
                particles[i] = (x, y, theta % (2 * math.pi))

        return particles

    def _sense(self, pose: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            pose: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """
        z_hat: list[float] = []

        # TODO: 3.6. Complete the missing function body with your code.
        rays = self._lidar_rays(pose, indices=range(8), degree_increment=45) # cambiar?

        for segment in rays:
            _, distancia = self._map.check_collision(segment, True)
            z_hat.append(distancia)
        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # TODO: 3.7. Complete the function body (i.e., replace the code below).
        return 1 / (sigma * math.sqrt(2 * math.pi)) * math.exp(-((x - mu) ** 2 / (2 * sigma**2)))

    def _lidar_rays(
        self, pose: tuple[float, float, float], indices: tuple[float], degree_increment: float = 1.5
    ) -> list[list[tuple[float, float]]]:
        """Determines the simulated LiDAR ray segments for a given robot pose.

        Args:
            pose: Robot pose (x, y, theta) in [m] and [rad].
            indices: Rays of interest in counterclockwise order (0 for to the forward-facing ray).
            degree_increment: Angle difference of the sensor between contiguous rays [degrees].

        Returns: Ray segments. Format:
                 [[(x0_start, y0_start), (x0_end, y0_end)],
                  [(x1_start, y1_start), (x1_end, y1_end)],
                  ...]

        """
        x, y, theta = pose

        # Convert the sensor origin to world coordinates
        x_start = x - 0.035 * math.cos(theta)
        y_start = y - 0.035 * math.sin(theta)

        rays = []

        for index in indices:
            ray_angle = math.radians(degree_increment * index)
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        the minimum sensor range to perform the computation because the environment is smaller
        than the maximum range.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0

        # TODO: 3.8. Complete the missing function body with your code
        # REVISAR.
        measurements = [
            measure if not math.isnan(measure) else self._sensor_range_min
            for measure in measurements
        ]
        z_hat = self._sense(pose=particle)
        z_hat = [z if not math.isnan(z) else self._sensor_range_min for z in z_hat]
        for z, measurement in zip(z_hat, measurements[::len(measurements)//8]):
            probability *= self._gaussian(mu=z, sigma=self._sigma_z, x=measurement)
        return probability
