# Autonomous Delivery Robot (ROS 2)

A small-scale indoor delivery robot built on **ROS 2 (Humble)** and **Python 3.10+**, designed to autonomously ferry packages within confined spaces (e.g., apartment complexes, office buildings, campuses). This repository contains all source code, launch configurations, and tests needed to simulate and run the complete “Autobot” stack—from item loading via RFID to navigation, telemetry, health monitoring, and delivery confirmation.

---

## 📖 Overview

This project demonstrates a modular, publish/subscribe architecture powered by ROS 2:

1. **RFID Reader**  
   Detects when an item has been placed in the robot’s carriage. In hardware mode (Raspberry Pi), it uses `mfrc522` and `RPi.GPIO`; on desktop/WSL it falls back to a timed simulation.

2. **Loader Node**  
   Subscribes to `/item_loaded` and, upon detection, publishes a JSON-encoded navigation goal (`/nav_goal`), specifying start and drop-off coordinates.

3. **Navigator Node**  
   Wraps a Dijkstra path planner tailored for grid maps. Receives `/nav_goal`, computes the shortest free-cell path, and simulates step-by-step movement, publishing a `/nav_done` status on completion or failure.

4. **Telemetry Node**  
   Publishes simulated GPS coordinates to `/bot/telemetry` at 1 Hz, mimicking live location updates. Easily replaceable with real sensor feeds (e.g., ROS 2 NavSat or Odometry topics).

5. **Health Monitor Node**  
   Subscribes to `/odom` and checks for zero movement over five consecutive updates. If “stuck,” emits a warning on `/bot/alert`, enabling remote intervention or failsafe behaviors.

6. **Flask Delivery API**  
   A lightweight REST service (endpoints `/issue` and `/confirm`) that lets external clients issue new deliveries (generates confirmation codes) and confirm receipt at the drop-off location.

---

## 🚀 Features & Highlights

- **Fully Modular Architecture**: Each component lives in its own ROS 2 node. Swap or extend functionality without touching other modules.
- **Simulated & Real Hardware**: RFID, navigation, telemetry, and health-monitoring all run in simulation on your desktop; they seamlessly switch to real sensors/motors on a Raspberry Pi.
- **Automated Testing**: Comprehensive `pytest` suite covers database layer, Flask API, Dijkstra planner, telemetry stub, health monitor stub, and navigator integration.
- **CI-Ready**: Zero-touch build/test via `colcon` and `pytest`, ideal for GitHub Actions pipelines or any other CI/CD toolchain.
- **Extensible Map Format**: Presently uses a fixed 2D grid, but you can migrate to occupancy grids (`nav_msgs/OccupancyGrid`) or dynamic maps with minimal changes.

---
