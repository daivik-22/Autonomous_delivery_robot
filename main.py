#!/usr/bin/env python3
"""
End-to-end integration: simulate load -> plan -> navigate -> return home.
"""

from autopkg.db import init_db, add_delivery
from autopkg.navigator import Navigator
from autopkg.planner import dijkstra  # for clarity
import time

def main():
    # 1. Initialize DB and create a delivery record
    init_db()
    delivery_id = add_delivery("demo123")
    print(f"[Main] Delivery #{delivery_id} created.")

    # 2. Define a demo grid (0=free, 1=obstacle)
    grid = [
        [0, 0, 0, 0],
        [1, 1, 0, 1],
        [0, 0, 0, 0],
        [0, 1, 1, 0]
    ]
    start = (0, 0)
    dropoff = (2, 3)

    # 3. Compute and execute path to dropoff
    nav = Navigator(grid)
    print("[Main] Computing route to dropoff...")
    path = nav.compute_path(start, dropoff)
    nav.execute_path(path, delay=0.2)

    # 4. Simulate confirmation & unloading
    print(f"[Main] Arrived at {dropoff}. Awaiting customer code...")
    # (In real life, here you’d call the Flask API)

    # 5. Compute and execute return path
    print("[Main] Returning home...")
    return_path = nav.compute_path(dropoff, start)
    nav.execute_path(return_path, delay=0.2)

    print("[Main] Mission complete.")

if __name__ == "__main__":
    main()
