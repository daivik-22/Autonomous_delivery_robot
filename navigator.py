# src/autopkg/navigator.py

import logging
import time
from typing import List, Tuple, Callable

from autopkg.planner import dijkstra

Point = Tuple[int, int]
Grid = List[List[int]]

# Configure module-level logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('[%(asctime)s][%(levelname)s] %(message)s'))
logger.addHandler(handler)


class Navigator:
    """
    Navigator computes and follows a path on a grid.
    """

    def __init__(self, grid: Grid):
        """
        :param grid: 2D map where 0=free cell, 1=obstacle.
        """
        self.grid = grid

    def compute_path(self, start: Point, goal: Point) -> List[Point]:
        """
        Uses Dijkstra to find the shortest path.
        
        :raises ValueError: if no valid path exists.
        """
        logger.info("Computing path from %s to %s", start, goal)
        path = dijkstra(self.grid, start, goal)
        if not path or path[0] != start or path[-1] != goal:
            raise ValueError(f"No path found from {start} to {goal}")
        logger.info("Path found: %s", path)
        return path

    def execute_path(
        self,
        path: List[Point],
        step_callback: Callable[[Point], None] = None,
        delay: float = 0.5
    ) -> None:
        """
        Simulates moving along `path`.
        
        :param step_callback: optional fn called with each waypoint.
        :param delay: pause (s) between steps.
        """
        for idx, pt in enumerate(path, start=1):
            logger.info("Step %d/%d → %s", idx, len(path), pt)
            if step_callback:
                try:
                    step_callback(pt)
                except Exception as e:
                    logger.exception("Callback error at %s: %s", pt, e)
            time.sleep(delay)
        logger.info("Destination reached.")


if __name__ == "__main__":
    sample_grid = [
        [0, 0, 0, 0],
        [1, 1, 0, 1],
        [0, 0, 0, 0],
        [0, 1, 1, 0],
    ]
    nav = Navigator(sample_grid)
    try:
        p = nav.compute_path((0, 0), (2, 3))
        nav.execute_path(p)
    except ValueError as e:
        logger.error(e)
