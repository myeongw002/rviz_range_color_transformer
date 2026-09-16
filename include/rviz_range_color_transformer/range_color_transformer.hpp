#pragma once

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"

namespace rviz_range_color_transformer
{

class RangeColorTransformer : public rviz_default_plugins::PointCloudTransformer
{
public:
  uint8_t supports(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  bool transform(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    uint32_t mask,
    const Ogre::Matrix4 & transform,
    rviz_default_plugins::V_PointCloudPoint & points_out) override;

  uint8_t score(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  void createProperties(
    rviz_common::properties::Property * parent_property,
    uint32_t mask,
    QList<rviz_common::properties::Property *> & out_props) override;

private:
  rviz_common::properties::EnumProperty * color_map_property_ = nullptr;
  rviz_common::properties::BoolProperty * auto_compute_bounds_property_ = nullptr;
  rviz_common::properties::FloatProperty * min_range_property_ = nullptr;
  rviz_common::properties::FloatProperty * max_range_property_ = nullptr;
  rviz_common::properties::BoolProperty * invert_colors_property_ = nullptr;
};

}  // namespace rviz_range_color_transformer
