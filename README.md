# ROS2 Robot Manipulator Toolkit

Package for an industrial robot manipulator in ROS2 Humble.

---

## Overview

This package provides tools and interfaces to control an industrial robot manipulator using ROS2 Humble. It includes terminal-based control, launch files for teleoperation, and integration with various devices such as SpaceMouse, joystick, and Xbox controllers. The toolkit is designed to simplify robot manipulator operation while offering flexibility for customization.

---

## Features

- **Terminal Control**:
  - Direct control of robot joints, tools, and movements through a terminal-based interface.
  - Teach and move to waypoints with simple commands.

- **Teleoperation**:
  - Integration with SpaceMouse and joystick devices.
  - Xbox controller support for intuitive robot control.

- **ROS2 Launch Files**:
  - Pre-configured launch files for teleoperation and direct control.
  - Dynamic frame selection and servo configuration.

- **Service-Based Architecture**:
  - Provides ROS2 services for robot alignment, tool selection, waypoint teaching, and movement.

---

## Installation

### Prerequisites

1. ROS2 Humble installed on your system.
2. Required packages:
   - `moveit_configs_utils`
   - `robot_teleoperation_interface`
   - `joy`
   - `spacenav`

### Build

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_robot_manipulator_toolkit
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## Usage

### Terminal Control

Run the terminal-based control node:
```bash
ros2 run ros2_robot_manipulator_toolkit direct_movement_node
```

#### Commands:
- Align the TCP:
  ```bash
  align
  ```
- Select a Tool:
  ```bash
  select <tool_id>
  ```
- Teach a Waypoint:
  ```bash
  teach <waypoint_name>
  ```
- Move the Robot:
  - In joint space:
    ```bash
    move joint J1 90 J2 45
    ```
  - In world coordinates:
    ```bash
    move world X 100 Y 200 Z 300 RX 0 RY 0 RZ 90
    ```
  - In tool coordinates:
    ```bash
    move tool X 50 Y 100 Z 150
    ```
- Navigate to a Waypoint:
  ```bash
  point <waypoint_name>
  ```
- Exit:
  ```bash
  exit
  ```

### Launch Files

#### Teleoperation
Launch teleoperation with SpaceMouse and joystick support:
```bash
ros2 launch ros2_robot_manipulator_toolkit teleoperation.launch.py
```

#### Direct Control
Launch direct control with the terminal interface:
```bash
ros2 launch ros2_robot_manipulator_toolkit direct_control.launch.py
```

---

## Dependencies

This package depends on the following:
- ROS2 Humble
- MoveIt
- `robot_teleoperation_interface`
- `joy`
- `spacenav`

---

## License

This package is licensed under the MIT License. See the `LICENSE` file for details.

---

## Author

**Felix Pfeifer**  
Email: [fpfeifer@stud.hs-heilbronn.de](mailto:fpfeifer@stud.hs-heilbronn.de)

