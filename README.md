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
- Automatic per-frame range bounds or manual min/max bounds.
- Rainbow color map, near → blue and far → red by default.
- Optional color inversion.
- Intended for ROS 2 Humble / Ubuntu 22.04.

## Build

```bash
cd ~/ROS2/compa_ws/src
git clone https://github.com/myeongw002/rviz_range_color_transformer.git

cd ~/ROS2/compa_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rviz_range_color_transformer
source install/setup.bash
```

Then restart RViz2 from a terminal where the workspace has been sourced.

## Use

1. Add or select a `PointCloud2` display in RViz2.
2. Set the cloud topic, for example `/velodyne_points` or `/pc_interpoled`.
3. Set **Color Transformer** to **Range**.
4. Leave **Auto Compute Bounds** enabled, or disable it and set **Min Range** / **Max Range** manually.
5. Enable **Invert Colors** if you want near → red and far → blue.

The range is computed in the incoming point cloud coordinate frame, before the RViz fixed-frame transform. This corresponds to sensor-relative range for clouds expressed in the sensor frame.

## Notes

This package is an RViz visualization plugin only. It does not alter recorded data or the perception pipeline.

## License

Apache-2.0
