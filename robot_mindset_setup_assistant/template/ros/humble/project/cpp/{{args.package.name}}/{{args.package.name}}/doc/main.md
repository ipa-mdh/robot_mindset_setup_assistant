@mainpage robot_fanuc
# robot_fanuc

This repository provides setup and launch instructions for working with **robot_fanuc** in a development container environment with ROS 2 and MoveIt.

## Setup Instructions

### 1. Build the Development Images

Open a terminal in the project root and run:
```bash
bash .dev-setup/docker/build-image.sh
```

### 2. Open in VS Code Dev Container

- Open the `robot_fanuc` folder in Visual Studio Code.
- Reopen the project in the Dev Container when prompted.

### 3. Build the Workspace

Inside the dev container terminal:
```bash
colcon build --symlink-install
```
And source with:
```bash
source install/setup.bash
```

## Running the Robot Fanuc with MoveIt

### 1. Standard MoveIt Launch

```bash
ros2 launch robot_fanuc multi_serve.py.launch
```

### 2. MoveIt Servo Modes

- For a servo example:
  ```bash
  ros2 launch robot_fanuc servo_example.py.launch
  ```
- For keyboard-based servoing:
  ```bash
  ros2 run robot_fanuc servo_keyboard_input
  ```

**Note:** Ensure all commands are run in the dev container terminal for the appropriate environment setup.

## Example Kinematic Chain

An example kinematic chain is provided to illustrate usage and structure. This serves as a reference for how to define and visualize models within this project.

- **File location:** `example/urdf/example.xacro`
- **Description:** This file contains an example kinematic chain in URDF (Unified Robot Description Format), written as a Xacro macro.

\image html urdf/robot.png "Example Kinematic Cahin: example/urdf/example.xacro"
