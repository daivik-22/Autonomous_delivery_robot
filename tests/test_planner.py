import pytest
from autobot_pkg.planner import dijkstra

def test_simple_grid_path():
    grid = [
        [0, 0, 0],
        [1, 1, 0],
        [0, 0, 0]
    ]
    path = dijkstra(grid, (0, 0), (2, 2))
    # Path should start and end at the correct points
    assert path[0] == (0, 0)
    assert path[-1] == (2, 2)

def test_no_path_raises():
    grid = [
        [0, 1],
        [1, 0]
    ]
    with pytest.raises(ValueError):
        dijkstra(grid, (0, 0), (1, 1))
