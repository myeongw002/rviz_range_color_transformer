# RViz Range Color Transformer

RViz2 `PointCloud2` color transformer for ROS 2 Humble that colors points by their 3D Euclidean range

\[
r = \sqrt{x^2 + y^2 + z^2}
\]

without adding a `range` field or republishing the point cloud.

The plugin appears in the existing RViz2 `PointCloud2` display under **Color Transformer → Range**.

## Features

- Uses the existing `x`, `y`, `z` fields only.
- Does not modify or republish the input `PointCloud2`.
- Works with any compatible `sensor_msgs/msg/PointCloud2` topic containing numeric `x`, `y`, and `z` fields.
- Automatic per-frame range bounds or manual min/max bounds.
- Selectable color maps:
  - Turbo
  - Viridis
  - Plasma
  - Inferno
  - Magma
  - Rainbow
  - Grayscale
- Turbo is the default color map.
- Optional color inversion.
- Intended for ROS 2 Humble / Ubuntu 22.04.

## Build

Clone the package into the `src` directory of your ROS 2 workspace. Replace the workspace path below with your own if necessary.

```bash
WS=~/ros2_ws

mkdir -p "$WS/src"
cd "$WS/src"
git clone https://github.com/myeongw002/rviz_range_color_transformer.git

cd "$WS"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rviz_range_color_transformer
source install/setup.bash
```

If the repository is already cloned:

```bash
WS=~/ros2_ws

cd "$WS/src/rviz_range_color_transformer"
git pull

cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rviz_range_color_transformer
source install/setup.bash
```

Then restart RViz2 from a terminal where the workspace has been sourced.

## Use

1. Add or select a `PointCloud2` display in RViz2.
2. Set **Topic** to the `sensor_msgs/msg/PointCloud2` topic you want to visualize.
3. Set **Color Transformer** to **Range**.
4. Select the desired **Color Map**.
5. Leave **Auto Compute Bounds** enabled, or disable it and set **Min Range** / **Max Range** manually.
6. Enable **Invert Colors** to reverse the selected map.

No specific LiDAR driver or point-cloud topic name is required. The input cloud only needs numeric `x`, `y`, and `z` fields.

The range is computed in the incoming point cloud coordinate frame, before the RViz fixed-frame transform. For a cloud expressed in a sensor-centered frame, this corresponds to sensor-relative 3D range.

The named scientific color maps are implemented with compact RGB anchor tables and linear interpolation, keeping the plugin dependency-free beyond RViz/ROS 2.

## Notes

This package is an RViz visualization plugin only. It does not alter recorded data, add fields to the point cloud, or modify the perception pipeline.

## License

Apache-2.0
