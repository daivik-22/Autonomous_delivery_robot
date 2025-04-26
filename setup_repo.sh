#!/usr/bin/env bash
set -e

# Project root assumed current directory
PKG="autobot_pkg"

# Top‐level metadata
cat > .gitignore << 'EOF'
__pycache__/
build/
install/
log/
*.pyc
*.db
.venv/
EOF

cat > LICENSE << 'EOF'
MIT License

Copyright (c) 2025 Your Name

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), ...
EOF

cat > requirements.txt << 'EOF'
flask>=2.0.0
paho-mqtt>=1.6.0
opencv-python>=4.5.0
numpy>=1.23.0
imutils>=0.5.4
networkx>=2.6.0
python-dotenv>=0.19.0
pytest>=7.0.0
pytest-cov>=4.0.0
EOF

# ROS manifest & setup
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>autobot_pkg</name>
  <version>0.0.1</version>
  <description>ROS2 package for the autonomous delivery robot</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
</package>
EOF

cat > setup.py << 'EOF'
from setuptools import setup, find_packages

package_name = 'autobot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 package for the autonomous delivery robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = autobot_pkg.hello_node:main',
            'rfid_reader_node = autobot_pkg.rfid_reader_node:main',
            'loader_node = autobot_pkg.loader_node:main',
            'navigator_node = autobot_pkg.navigator_node:main',
            'telemetry_node = autobot_pkg.telemetry_node:main',
            'health_monitor_node = autobot_pkg.health_monitor_node:main',
        ],
    },
)
EOF

# Launch folder
mkdir -p launch
cat > launch/autobot_launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='autobot_pkg', executable='rfid_reader_node'),
        Node(package='autobot_pkg', executable='loader_node'),
        Node(package='autobot_pkg', executable='navigator_node'),
        Node(package='autobot_pkg', executable='telemetry_node'),
        Node(package='autobot_pkg', executable='health_monitor_node'),
    ])
EOF

# ROS2 package folder
mkdir -p $PKG/$PKG

# __init__.py
cat > $PKG/$PKG/__init__.py << 'EOF'
\"\"\"Autobot ROS2 package.\"\"\"
from .db import init_db, add_tag, check_tag, add_delivery, verify_code
from .planner import dijkstra
from .navigator import Navigator
from .telemetry import simulate_telemetry
from .health_monitor import simulate_health
EOF

# (Continue for db.py, delivery_api.py, planner.py, navigator.py, telemetry.py, health_monitor.py...)

# Node scripts
for node in hello rfid_reader loader navigator telemetry health_monitor; do
  cat > $PKG/$PKG/${node}_node.py << 'EOF'
#!/usr/bin/env python3
# (node-specific code goes here)
EOF
  chmod +x $PKG/$PKG/${node}_node.py
done

# Tests folder
mkdir -p tests
for t in db api planner telemetry health navigator; do
  cat > tests/test_${t}.py << 'EOF'
# (test_${t}.py content here)
EOF
done


