# Entity Spawner GUI

PySide6 desktop frontend for the `entity_spawner` ROS 2 services. The application lets you select STL models, queue them for spawning, and place them in a planning scene with pose and scale controls.

## Requirements

- ROS 2 Jazzy (`rclpy`)
- PySide6 (`python3-pyside6` Debian package or `pip install PySide6`)
- `entity_spawner` package available in the same workspace

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select gui
source install/setup.bash
```

## Run

```bash
ros2 run gui entity_spawner_gui
```

## Features

- Browse for STL files and queue them via the `loadEntity` service.
- Enter XYZ position, RPY orientation, frame ID, and scale for spawning.
- Displays status feedback from both services directly in the GUI.

Ensure the `entity_spawner` node is running before launching the GUI so the services are available.
