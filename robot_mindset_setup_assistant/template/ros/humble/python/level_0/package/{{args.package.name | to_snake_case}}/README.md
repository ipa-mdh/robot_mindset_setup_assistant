# ROS 2 Python Package – `{{ args.package.name |to_snake_case}}`

This template bootstraps a ROS 2 (Humble) Python package wired up identically to the C++ variant. It exposes the same launch structure, instance configuration folder and resource files, but implements the runtime node with `rclpy`.

## Build & Run

```bash
colcon build --packages-select {{ args.package.name |to_snake_case}}
source install/setup.bash
ros2 run {{ args.package.name |to_snake_case}} {{ args.package.name |to_snake_case}}_node
```

## Launch Example

```bash
ros2 launch {{ args.package.name |to_snake_case}} instance.launch.py
```

The example launch files mirror the C++ template and mount the contents of `example/instance_config/{{ args.package.name |to_snake_case}}`.
