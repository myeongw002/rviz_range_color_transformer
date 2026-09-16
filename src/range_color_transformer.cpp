#include "rviz_range_color_transformer/range_color_transformer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace rviz_range_color_transformer
{
namespace
{

enum ColorMap
{
  Turbo = 0,
  Viridis = 1,
  Plasma = 2,
  Inferno = 3,
  Magma = 4,
  Rainbow = 5,
  Grayscale = 6,
};

struct ColorStop
{
  float position;
  float r;
  float g;
  float b;
};

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

template<std::size_t N>
void sample_stops(
  const std::array<ColorStop, N> & stops,
  float value,
  Ogre::ColourValue & color)
{
  value = std::clamp(value, 0.0f, 1.0f);

  if (value <= stops.front().position) {
    color.r = stops.front().r;
    color.g = stops.front().g;
    color.b = stops.front().b;
    return;
  }

  if (value >= stops.back().position) {
    color.r = stops.back().r;
    color.g = stops.back().g;
    color.b = stops.back().b;
    return;
  }

  for (std::size_t i = 1; i < N; ++i) {
    if (value <= stops[i].position) {
      const auto & a = stops[i - 1];
      const auto & b = stops[i];
      const float span = b.position - a.position;
      const float t = span > 0.0f ? (value - a.position) / span : 0.0f;
      color.r = a.r + t * (b.r - a.r);
      color.g = a.g + t * (b.g - a.g);
      color.b = a.b + t * (b.b - a.b);
      return;
    }
  }
}

void apply_color_map(int color_map, float value, Ogre::ColourValue & color)
{
  // Compact anchor tables are linearly interpolated. They provide the visual
  // character of the named maps without adding an external plotting dependency.
  static constexpr std::array<ColorStop, 6> turbo{{
    {0.0f, 0.188f, 0.071f, 0.231f},
    {0.2f, 0.231f, 0.318f, 0.545f},
    {0.4f, 0.275f, 0.667f, 0.741f},
    {0.6f, 0.631f, 0.992f, 0.286f},
    {0.8f, 0.961f, 0.510f, 0.125f},
    {1.0f, 0.478f, 0.016f, 0.012f},
  }};

  static constexpr std::array<ColorStop, 6> viridis{{
    {0.0f, 0.267f, 0.005f, 0.329f},
    {0.2f, 0.255f, 0.267f, 0.530f},
    {0.4f, 0.165f, 0.471f, 0.558f},
    {0.6f, 0.133f, 0.658f, 0.518f},
    {0.8f, 0.478f, 0.821f, 0.318f},
    {1.0f, 0.992f, 0.906f, 0.145f},
  }};

  static constexpr std::array<ColorStop, 6> plasma{{
    {0.0f, 0.050f, 0.030f, 0.528f},
    {0.2f, 0.417f, 0.001f, 0.658f},
    {0.4f, 0.693f, 0.165f, 0.565f},
    {0.6f, 0.882f, 0.392f, 0.383f},
    {0.8f, 0.988f, 0.652f, 0.212f},
    {1.0f, 0.940f, 0.975f, 0.131f},
  }};

  static constexpr std::array<ColorStop, 6> inferno{{
    {0.0f, 0.001f, 0.000f, 0.014f},
    {0.2f, 0.258f, 0.039f, 0.406f},
    {0.4f, 0.578f, 0.149f, 0.404f},
    {0.6f, 0.865f, 0.316f, 0.226f},
    {0.8f, 0.988f, 0.645f, 0.039f},
    {1.0f, 0.988f, 1.000f, 0.644f},
  }};

  static constexpr std::array<ColorStop, 6> magma{{
    {0.0f, 0.001f, 0.000f, 0.014f},
    {0.2f, 0.232f, 0.059f, 0.439f},
    {0.4f, 0.550f, 0.161f, 0.506f},
    {0.6f, 0.868f, 0.288f, 0.409f},
    {0.8f, 0.995f, 0.624f, 0.427f},
    {1.0f, 0.987f, 0.991f, 0.749f},
  }};

  switch (color_map) {
    case Turbo:
      sample_stops(turbo, value, color);
      break;
    case Viridis:
      sample_stops(viridis, value, color);
      break;
    case Plasma:
      sample_stops(plasma, value, color);
      break;
    case Inferno:
      sample_stops(inferno, value, color);
      break;
    case Magma:
      sample_stops(magma, value, color);
      break;
    case Rainbow:
      rviz_default_plugins::getRainbowColor(value, color);
      break;
    case Grayscale:
      color.r = value;
      color.g = value;
      color.b = value;
      break;
    default:
      sample_stops(turbo, value, color);
      break;
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
  const int color_map = color_map_property_->getOptionInt();

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

    apply_color_map(color_map, normalized, color);
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

  color_map_property_ = new rviz_common::properties::EnumProperty(
    "Color Map", "Turbo",
    "Color map used to visualize normalized 3D range.",
    parent_property, SIGNAL(needRetransform()), this);
  color_map_property_->addOption("Turbo", Turbo);
  color_map_property_->addOption("Viridis", Viridis);
  color_map_property_->addOption("Plasma", Plasma);
  color_map_property_->addOption("Inferno", Inferno);
  color_map_property_->addOption("Magma", Magma);
  color_map_property_->addOption("Rainbow", Rainbow);
  color_map_property_->addOption("Grayscale", Grayscale);

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
    "Reverse the selected color map so near and far colors are exchanged.",
    parent_property, SIGNAL(needRetransform()), this);

  out_props.push_back(color_map_property_);
  out_props.push_back(auto_compute_bounds_property_);
  out_props.push_back(min_range_property_);
  out_props.push_back(max_range_property_);
  out_props.push_back(invert_colors_property_);
}

}  // namespace rviz_range_color_transformer

PLUGINLIB_EXPORT_CLASS(
  rviz_range_color_transformer::RangeColorTransformer,
  rviz_default_plugins::PointCloudTransformer)
