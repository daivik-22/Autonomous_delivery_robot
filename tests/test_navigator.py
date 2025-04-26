import pytest
from autobot_pkg.navigator import Navigator

def test_navigator_success():
    grid = [
        [0, 0],
        [0, 0]
    ]
    nav = Navigator(grid)
    path = nav.compute_path((0, 0), (1, 1))
    assert path[0] == (0, 0)
    assert path[-1] == (1, 1)

def test_navigator_fail():
    # All obstacles, no valid start/goal
    grid = [[1]]
    nav = Navigator(grid)
    with pytest.raises(Exception):
        nav.compute_path((0, 0), (0, 0))
