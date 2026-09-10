#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
struct CloudLayout
{
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t z_offset;
  std::optional<uint32_t> color_offset;
};

std::optional<uint32_t> find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return field.offset;
    }
  }
  return std::nullopt;
}

bool read_float(const std::vector<uint8_t> & data, size_t offset, float & value)
{
  if (offset + sizeof(float) > data.size()) {
    return false;
  }
  std::memcpy(&value, data.data() + offset, sizeof(float));
  return true;
}
}  // namespace

class FusedMapExporter : public rclcpp::Node
{
public:
  FusedMapExporter()
  : Node("fused_map_exporter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/zed_front/zed_node/mapping/fused_cloud");
    output_path_ = declare_parameter<std::string>(
      "output_path", "maps/spatial_mapping/front_room.ply");
    const bool auto_save = declare_parameter<bool>("auto_save", true);
    const double snapshot_period = declare_parameter<double>("snapshot_period_sec", 5.0);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = std::move(message);
        ++cloud_generation_;
      });

    save_service_ = create_service<std_srvs::srv::Trigger>(
      "save_fused_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = save_latest_cloud();
        response->message = response->success ?
          "Fused map saved to '" + output_path_ + "'" :
          "No valid fused map is available to save yet";
      });

    if (auto_save) {
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(snapshot_period, 0.5)));
      save_timer_ = create_wall_timer(period, [this]() {
        save_latest_cloud_if_new();
      });
    }

    RCLCPP_INFO(
      get_logger(), "Saving fused maps from '%s' to '%s'", input_topic_.c_str(),
      output_path_.c_str());
  }

private:
  bool save_latest_cloud_if_new()
  {
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      generation = cloud_generation_;
    }
    if (generation == 0 || generation == saved_generation_) {
      return false;
    }
    return save_latest_cloud();
  }

  bool save_latest_cloud()
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
      generation = cloud_generation_;
    }
    if (!cloud) {
      return false;
    }

    if (!write_ply(*cloud)) {
      return false;
    }
    saved_generation_ = generation;
    return true;
  }

  bool write_ply(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto x_offset = find_field(cloud, "x");
    const auto y_offset = find_field(cloud, "y");
    const auto z_offset = find_field(cloud, "z");
    const auto rgb_offset = find_field(cloud, "rgb");
    const auto rgba_offset = find_field(cloud, "rgba");
    if (!x_offset || !y_offset || !z_offset || cloud.point_step == 0) {
      RCLCPP_ERROR(get_logger(), "Fused cloud does not contain usable x/y/z fields");
      return false;
    }

    const CloudLayout layout{
      *x_offset, *y_offset, *z_offset,
      rgb_offset ? rgb_offset : rgba_offset};
    const size_t point_count = static_cast<size_t>(cloud.width) * cloud.height;
    size_t valid_points = 0;

    for (size_t point = 0; point < point_count; ++point) {
      float x, y, z;
      const size_t row = point / cloud.width;
      const size_t column = point % cloud.width;
      const size_t offset = row * cloud.row_step + column * cloud.point_step;
      if (read_float(cloud.data, offset + layout.x_offset, x) &&
          read_float(cloud.data, offset + layout.y_offset, y) &&
          read_float(cloud.data, offset + layout.z_offset, z) &&
          std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      {
        ++valid_points;
      }
    }
    if (valid_points == 0) {
      RCLCPP_WARN(get_logger(), "Fused cloud contains no finite points yet");
      return false;
    }

    const std::filesystem::path output(output_path_);
    std::error_code error;
    if (output.has_parent_path()) {
      std::filesystem::create_directories(output.parent_path(), error);
      if (error) {
        RCLCPP_ERROR(get_logger(), "Cannot create map directory: %s", error.message().c_str());
        return false;
      }
    }
    const std::filesystem::path temporary = output.string() + ".tmp";
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Cannot write fused map to '%s'", temporary.string().c_str());
      return false;
    }

    file << "ply\nformat binary_little_endian 1.0\n";
    file << "comment ZED fused spatial map exported by vision_bringup\n";
    file << "element vertex " << valid_points << "\n";
    file << "property float x\nproperty float y\nproperty float z\n";
    file << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

    for (size_t point = 0; point < point_count; ++point) {
      float x, y, z;
      const size_t row = point / cloud.width;
      const size_t column = point % cloud.width;
      const size_t offset = row * cloud.row_step + column * cloud.point_step;
      if (!read_float(cloud.data, offset + layout.x_offset, x) ||
          !read_float(cloud.data, offset + layout.y_offset, y) ||
          !read_float(cloud.data, offset + layout.z_offset, z) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        continue;
      }

      uint8_t red = 255;
      uint8_t green = 255;
      uint8_t blue = 255;
      if (layout.color_offset && *layout.color_offset + sizeof(uint32_t) <= cloud.point_step) {
        uint32_t packed_color = 0;
        if (offset + *layout.color_offset + sizeof(uint32_t) <= cloud.data.size()) {
          std::memcpy(&packed_color, cloud.data.data() + offset + *layout.color_offset,
            sizeof(uint32_t));
          red = static_cast<uint8_t>((packed_color >> 16) & 0xFF);
          green = static_cast<uint8_t>((packed_color >> 8) & 0xFF);
          blue = static_cast<uint8_t>(packed_color & 0xFF);
        }
      }
      file.write(reinterpret_cast<const char *>(&x), sizeof(x));
      file.write(reinterpret_cast<const char *>(&y), sizeof(y));
      file.write(reinterpret_cast<const char *>(&z), sizeof(z));
      file.put(static_cast<char>(red));
      file.put(static_cast<char>(green));
      file.put(static_cast<char>(blue));
    }
    file.close();
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Failed while writing fused map");
      return false;
    }

    std::filesystem::rename(temporary, output, error);
    if (error) {
      std::filesystem::remove(output, error);
      error.clear();
      std::filesystem::rename(temporary, output, error);
    }
    if (error) {
      RCLCPP_ERROR(get_logger(), "Cannot finalize fused map: %s", error.message().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Saved %zu points to '%s'", valid_points, output.string().c_str());
    return true;
  }

  std::string input_topic_;
  std::string output_path_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  uint64_t cloud_generation_{0};
  uint64_t saved_generation_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusedMapExporter>());
  rclcpp::shutdown();
  return 0;
}
