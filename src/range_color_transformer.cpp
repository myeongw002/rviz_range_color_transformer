#include "rviz_range_color_transformer/range_color_transformer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace rviz_range_color_transformer
{
namespace
{

bool is_numeric_datatype(uint8_t datatype)
{
  using PointField = sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
    case PointField::INT16:
    case PointField::UINT16:
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
    case PointField::FLOAT64:
      return true;
    default:
      return false;
  }
}

}  // namespace

uint8_t RangeColorTransformer::supports(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  const int32_t x_index = rviz_default_plugins::findChannelIndex(cloud, "x");
  const int32_t y_index = rviz_default_plugins::findChannelIndex(cloud, "y");
  const int32_t z_index = rviz_default_plugins::findChannelIndex(cloud, "z");

  if (x_index < 0 || y_index < 0 || z_index < 0) {
    return Support_None;
  }

  if (!is_numeric_datatype(cloud->fields[static_cast<std::size_t>(x_index)].datatype) ||
      !is_numeric_datatype(cloud->fields[static_cast<std::size_t>(y_index)].datatype) ||
      !is_numeric_datatype(cloud->fields[static_cast<std::size_t>(z_index)].datatype))
  {
    return Support_None;
  }

  return Support_Color;
}

uint8_t RangeColorTransformer::score(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  (void) cloud;
  // Keep this transformer selectable without overriding RGB/intensity as RViz's
  // automatic first choice for ordinary point clouds.
  return 0;
}

bool RangeColorTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  rviz_default_plugins::V_PointCloudPoint & points_out)
{
  (void) transform;

  if (!(mask & Support_Color)) {
    return false;
  }

  const int32_t x_index = rviz_default_plugins::findChannelIndex(cloud, "x");
  const int32_t y_index = rviz_default_plugins::findChannelIndex(cloud, "y");
  const int32_t z_index = rviz_default_plugins::findChannelIndex(cloud, "z");
  if (x_index < 0 || y_index < 0 || z_index < 0) {
    return false;
  }

  const auto & x_field = cloud->fields[static_cast<std::size_t>(x_index)];
  const auto & y_field = cloud->fields[static_cast<std::size_t>(y_index)];
  const auto & z_field = cloud->fields[static_cast<std::size_t>(z_index)];

  const std::size_t num_points =
    static_cast<std::size_t>(cloud->width) * static_cast<std::size_t>(cloud->height);
  if (points_out.size() < num_points) {
    return false;
  }

  std::vector<float> ranges(num_points, std::numeric_limits<float>::quiet_NaN());

  float min_range = std::numeric_limits<float>::infinity();
  float max_range = -std::numeric_limits<float>::infinity();

  for (std::size_t i = 0; i < num_points; ++i) {
    const float x = rviz_default_plugins::valueFromCloud<float>(
      cloud, x_field.offset, x_field.datatype, cloud->point_step, i);
    const float y = rviz_default_plugins::valueFromCloud<float>(
      cloud, y_field.offset, y_field.datatype, cloud->point_step, i);
    const float z = rviz_default_plugins::valueFromCloud<float>(
      cloud, z_field.offset, z_field.datatype, cloud->point_step, i);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const float range = std::sqrt(x * x + y * y + z * z);
    if (!std::isfinite(range)) {
      continue;
    }

    ranges[i] = range;
    min_range = std::min(min_range, range);
    max_range = std::max(max_range, range);
  }

  if (!auto_compute_bounds_property_->getBool()) {
    min_range = min_range_property_->getFloat();
    max_range = max_range_property_->getFloat();
  }

  if (!std::isfinite(min_range) || !std::isfinite(max_range)) {
    min_range = 0.0f;
    max_range = 1.0f;
  }

  if (max_range < min_range) {
    std::swap(min_range, max_range);
  }

  float span = max_range - min_range;
  if (span < 1e-6f) {
    span = 1.0f;
  }

  const bool invert = invert_colors_property_->getBool();

  for (std::size_t i = 0; i < num_points; ++i) {
    auto & color = points_out[i].color;
    const float range = ranges[i];

    if (!std::isfinite(range)) {
      color.r = 0.2f;
      color.g = 0.2f;
      color.b = 0.2f;
      color.a = 1.0f;
      continue;
    }

    float normalized = (range - min_range) / span;
    normalized = std::clamp(normalized, 0.0f, 1.0f);
    if (invert) {
      normalized = 1.0f - normalized;
    }

    // RViz's rainbow helper maps 0 -> blue and 1 -> red.
    rviz_default_plugins::getRainbowColor(normalized, color);
    color.a = 1.0f;
  }

  return true;
}

void RangeColorTransformer::createProperties(
  rviz_common::properties::Property * parent_property,
  uint32_t mask,
  QList<rviz_common::properties::Property *> & out_props)
{
  if (!(mask & Support_Color)) {
    return;
  }

  auto_compute_bounds_property_ = new rviz_common::properties::BoolProperty(
    "Auto Compute Bounds", true,
    "Compute the minimum and maximum 3D range from each incoming cloud.",
    parent_property, SIGNAL(needRetransform()), this);

  min_range_property_ = new rviz_common::properties::FloatProperty(
    "Min Range", 0.0,
    "Manual minimum range in metres. Used when Auto Compute Bounds is disabled.",
    parent_property, SIGNAL(needRetransform()), this);
  min_range_property_->setMin(0.0);

  max_range_property_ = new rviz_common::properties::FloatProperty(
    "Max Range", 50.0,
    "Manual maximum range in metres. Used when Auto Compute Bounds is disabled.",
    parent_property, SIGNAL(needRetransform()), this);
  max_range_property_->setMin(0.0);

  invert_colors_property_ = new rviz_common::properties::BoolProperty(
    "Invert Colors", false,
    "Default is near=blue, far=red. Enable for near=red, far=blue.",
    parent_property, SIGNAL(needRetransform()), this);

  out_props.push_back(auto_compute_bounds_property_);
  out_props.push_back(min_range_property_);
  out_props.push_back(max_range_property_);
  out_props.push_back(invert_colors_property_);
}

}  // namespace rviz_range_color_transformer

PLUGINLIB_EXPORT_CLASS(
  rviz_range_color_transformer::RangeColorTransformer,
  rviz_default_plugins::PointCloudTransformer)
