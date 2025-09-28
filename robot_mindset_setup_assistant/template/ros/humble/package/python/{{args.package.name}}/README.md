# ROS 2 Python Package – `{{ args.package.name |regex_replace('[^A-Za-z0-9]', '_')}}`

This template bootstraps a ROS 2 (Humble) Python package wired up identically to the C++ variant. It exposes the same launch structure, instance configuration folder and resource files, but implements the runtime node with `rclpy`.

## Build & Run

```bash
colcon build --packages-select {{ args.package.name |regex_replace('[^A-Za-z0-9]', '_')}}
source install/setup.bash
ros2 run {{ args.package.name |regex_replace('[^A-Za-z0-9]', '_')}} {{ args.package.name |regex_replace('[^A-Za-z0-9]', '_')}}_node
```

## Launch Example

```bash
ros2 launch {{ args.package.name |regex_replace('[^A-Za-z0-9]', '_')}} instance.launch.py
```

The example launch files mirror the C++ template and mount the contents of `example/instance_config/{{ args.package.name |regex_replace('[^A-Za-z0-9]', '_')}}`.
