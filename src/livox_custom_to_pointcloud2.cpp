// Convert Livox CustomMsg (/livox/lidar) -> sensor_msgs/PointCloud2 (/livox/lidar_pc2)
// Purpose: RViz/raw inspection without changing Fast-LIO input (CustomMsg).

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "livox_ros_driver2/msg/custom_msg.hpp"

class LivoxCustomToPointCloud2 final : public rclcpp::Node
{
public:
  LivoxCustomToPointCloud2()
  : Node("livox_custom_to_pointcloud2")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar_pc2");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "");

    // Subscribe with sensor-data QoS (best effort) to match typical LiDAR driver settings.
    const auto sub_qos = rclcpp::SensorDataQoS();
    // Publish with reliable QoS so RViz (often reliable by default) can subscribe without QoS tweaks.
    const auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);
    sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, sub_qos,
      std::bind(&LivoxCustomToPointCloud2::cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Converting %s (livox_ros_driver2/msg/CustomMsg) -> %s (sensor_msgs/msg/PointCloud2)",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void cb(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    if (!output_frame_id_.empty()) {
      out.header.frame_id = output_frame_id_;
    }

    // Create XYZ + intensity cloud (intensity from reflectivity [0..255]).
    out.height = 1;
    out.width = msg->point_num;
    out.is_bigendian = false;
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(out.width);

    sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(out, "intensity");

    const auto n = std::min<std::size_t>(msg->points.size(), out.width);
    for (std::size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z, ++it_i) {
      const auto & p = msg->points[i];
      *it_x = p.x;
      *it_y = p.y;
      *it_z = p.z;
      *it_i = static_cast<float>(p.reflectivity);
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomToPointCloud2>());
  rclcpp::shutdown();
  return 0;
}