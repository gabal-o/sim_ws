import datetime
import numpy as np
import math
import os
import pytz
import random
import time

# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt


class PRM:
    """Class to plan a path to a given destination using probabilistic roadmaps (PRM)."""

    def __init__(
        self,
        map_path: str,
        obstacle_safety_distance=0.08,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
        sensor_range_max: float = 8.0,
        logger=None,
        simulation: bool = False,
    ):
        """Probabilistic roadmap (PRM) class initializer.

        Args:
            map_path: Path to the map of the environment.
            obstacle_safety_distance: Distance to grow the obstacles by [m].
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].
            sensor_range_max: Sensor measurement range [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._map: Map = Map(
            map_path,
            sensor_range=sensor_range_max,
            safety_distance=obstacle_safety_distance,
            compiled_intersect=False,
            use_regions=False,
        )

        self._graph: dict[tuple[float, float], list[tuple[float, float]]] = self._create_graph(
            use_grid,
            node_count,
            grid_size,
            connection_distance,
        )

        self._logger = logger
        self._simulation: bool = simulation

        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def find_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        """Computes the shortest path from a start to a goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Destination in (x, y) [m] format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.

        """
        # Check if the goal is valid
        if not self._map.contains(goal):
            raise ValueError("Goal location is outside the environment.")

        ancestors: dict[tuple[float, float], tuple[float, float]] = {}  # {(x, y: (x_prev, y_prev)}
        # TODO: 4.3. Complete the function body (i.e., replace the code below).
        # path: list[tuple[float, float]] = []
        start_in_graph = False
        if start in self._graph:
            start_in_graph = True
            start_node = start

        goal_in_graph = False
        if goal in self._graph:
            goal_in_graph = True
            goal_node = goal

        if not start_in_graph or not goal_in_graph:    
            start_x, start_y = start
            closest_dist_goal = np.inf
            closest_dist_start = np.inf
            for x1, y1 in self._graph.keys():
                dist = np.sqrt((x1-start_x)**2 + (y1-start_y)**2)
                if dist < closest_dist_start and not start_in_graph:
                    closest_dist_start = dist
                    start_node = (x1, y1)
                if dist < closest_dist_goal and not goal_in_graph:
                    closest_dist_goal = dist
                    goal_node = (x1, y1)


        # h = heuristica (distancia euclídea)
        #(f,g) f = h + g, g numero de pasos acumulados
        open_list = {start_node: (math.dist(start_node, goal_node), 0)} 
        closed_list = set() 
        while True: # open_list
            if not open_list:
                print(current_node)
                raise Exception("Open list is empty.")

            current_node = min(open_list, key=lambda k: open_list.get(k)[0])
            _, g = open_list[current_node]
            open_list.pop(current_node)
            closed_list.add(current_node)

            if current_node == goal_node:
                if not start_in_graph:
                    ancestors[start_node] = start
                if not goal_in_graph:
                    ancestors[goal] = goal_node # current_node
                return self._reconstruct_path(start, goal)

            for neighbour in self._graph[current_node]:
                if neighbour not in open_list and neighbour not in closed_list:
                    g_new = g + math.dist(neighbour, current_node)  
                    f_new = g_new + math.dist(goal_node, neighbour)  
                    open_list[neighbour] = (f_new, g_new)
                    ancestors[neighbour] = current_node
        



    @staticmethod
    def smooth_path(
        path: list[tuple[float, float]],
        data_weight: float = 0.1,
        smooth_weight: float = 0.3,
        additional_smoothing_points: int = 0,
        tolerance: float = 1e-6,
    ) -> list[tuple[float, float]]:
        """Computes a smooth path from a piecewise linear path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            additional_smoothing_points: Number of equally spaced intermediate points to add
                between two nodes of the original path.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes
                less than this value.

        Returns: Smoothed path (initial location first) in (x, y) [m] format.

        """
        # TODO: 4.5. Complete the function body (i.e., load smoothed_path).
        smoothed_path: list[tuple[float, float]] = []
        # smoothed_path.append(path[0])
        # while iter_error > tolerance:
        #     iter_error = 0.0
        #     for si in path[1:-1]:
        #         si = si + data_weight*(pi - si) + smooth_weight*(si_next + si_last - 2*si)
            
        # smoothed_path.append(path[-1])
        return smoothed_path

    def plot(
        self,
        axes,
        path: list[tuple[float, float]] = (),
        smoothed_path: list[tuple[float, float]] = (),
    ):
        """Draws particles.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        # Plot the nodes
        x, y = zip(*self._graph.keys())
        axes.plot(list(x), list(y), "co", markersize=1)

        # Plot the edges
        for node, neighbors in self._graph.items():
            x_start, y_start = node

            if neighbors:
                for x_end, y_end in neighbors:
                    axes.plot([x_start, x_end], [y_start, y_end], "c-", linewidth=0.25)

        # Plot the path
        if path:
            x_val = [x[0] for x in path]
            y_val = [x[1] for x in path]

            axes.plot(x_val, y_val)  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "bo", markersize=4)  # Draw nodes as blue circles

        # Plot the smoothed path
        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "yo", markersize=2)  # Draw nodes as yellow circles

        if path or smoothed_path:
            axes.plot(
                x_val[0], y_val[0], "rs", markersize=7
            )  # Draw a red square at the start location
            axes.plot(
                x_val[-1], y_val[-1], "g*", markersize=12
            )  # Draw a green star at the goal location

        return axes

    def show(
        self,
        title: str = "",
        path=(),
        smoothed_path=(),
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
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
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait for 1 ms or the figure won't be displayed

        if display:
            plt.show(block=block)

        if save_figure:
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _connect_nodes(
        self,
        graph: dict[tuple[float, float], list[tuple[float, float]]],
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Connects every generated node with all the nodes that are closer than a given threshold.

        Args:
            graph: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A modified graph with lists of connected nodes as values.

        """
        # TODO: 4.2. Complete the missing function body with your code.
        for x1,y1 in graph.keys():
            for x2, y2 in graph.keys():
                if np.sqrt((x1-x2)**2 + (y1-y2)**2) < connection_distance:
                    if self._map.crosses([(x1,y1), (x2,y2)]):
                        graph[(x1,y1)].append((x2,y2))
        return graph

    def _create_graph(
        self,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a roadmap as a graph with edges connecting the closest nodes.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A dictionary with (x, y) [m] tuples as keys and lists of connected nodes as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph = self._generate_nodes(use_grid, node_count, grid_size)
        graph = self._connect_nodes(graph, connection_distance)

        return graph

    def _generate_nodes(
        self, use_grid: bool = False, node_count: int = 50, grid_size=0.1
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a set of valid nodes to build a roadmap with.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.

        Returns: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph: dict[tuple[float, float], list[tuple[float, float]]] = {}

        # TODO: 4.1. Complete the missing function body with your code.
        xmin, ymin, xmax, ymax = self._map.bounds()
        if use_grid:
            for x in np.arange(xmin, xmax, grid_size):
                for y in np.arange(ymin, ymax, grid_size):
                    if self._map.contains((x,y)):
                        x = np.round(float(x), 2)
                        y = np.round(float(y), 2)
                        graph[(x,y)] = [] # (None, None)
        else:
            nodes_added = 0
            while nodes_added < node_count:
                x_candidate = np.random.uniform(xmin, xmax)
                y_candidate = np.random.uniform(ymin, ymax)
                if self._map.contains((x_candidate,y_candidate)):
                    x = np.round(float(x), 2)
                    y = np.round(float(y), 2)
                    graph[(x_candidate,y_candidate)] = [] # (None, None)
                    nodes_added += 1
        return graph

    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """Computes the path from the start to the goal given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Goal location in (x, y) [m] format.
            ancestors: Dictionary with (x, y) [m] tuples as keys and the node (x_prev, y_prev) [m]
                from which it was added to the open list as values.

        Returns: Path to the goal (start location first) in (x, y) [m] format.

        """
        path: list[tuple[float, float]] = []

        # TODO: 4.4. Complete the missing function body with your code.
        current_node = goal
        while current_node != start:
            path.append(current_node)
            current_node = self.ancestors[current_node]
        path.append(start)
        path.reverse()
        return path


if __name__ == "__main__":
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap
    start_time = time.perf_counter()
    prm = PRM(map_path, use_grid=True, node_count=250, grid_size=0.1, connection_distance=0.15)
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find the path
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(-0.6, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the path
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path, data_weight=0.1, smooth_weight=0.3, additional_smoothing_points=3
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
