#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>   

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("pointcloud_filter")
  {
    // --- Define ROS Parameters ---
    // Topics
    this->declare_parameter<std::string>("pointcloud_topic", "/ouster/points");
    this->declare_parameter<std::string>("filtered_pc_topic", "/ouster/points_filtered");
    this->declare_parameter<std::string>("laser_scan_topic", "/scan");
    // Filtering Parameters
    this->declare_parameter<float>("min_x", -1.4);
    this->declare_parameter<float>("max_x", 0.25);
    this->declare_parameter<float>("min_y", -0.25);
    this->declare_parameter<float>("max_y", 1.2);
    this->declare_parameter<float>("min_z", -1.0);
    this->declare_parameter<float>("max_z", 1.0);
    this->declare_parameter<bool>("filter_rings", false);
    this->declare_parameter<bool>("filter_box", true);
    this->declare_parameter<bool>("obtain_outside_box", true);
    this->declare_parameter<bool>("publish_filtered_pointcloud", true);
    this->declare_parameter<bool>("publish_laserscan", true);
    // LaserScan Parameters
    this->declare_parameter<float>("scan_angle_min", -M_PI);
    this->declare_parameter<float>("scan_angle_max", M_PI);
    this->declare_parameter<float>("scan_angle_increment", 2 * M_PI / 1023.0);
    this->declare_parameter<float>("scan_range_min", 0.3);
    this->declare_parameter<float>("scan_range_max", 75.0);
    this->declare_parameter<std::string>("laserscan_frame", ""); // PENDING WORK: need to transform the laser scan frame

    // --- Initialize parameters ---
    update_parameters();

    // --- Get Topic Names ---
    std::string pointcloud_topic, filtered_pc_topic, laser_scan_topic;
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    this->get_parameter("filtered_pc_topic", filtered_pc_topic);
    this->get_parameter("laser_scan_topic", laser_scan_topic);

    // --- Publishers ---
    filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_pc_topic, 10);
    laser_scan_pub_  = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic, 10);

    // --- Subscribers ---
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&PointCloudFilter::pointCloudCallback, this, std::placeholders::_1));

    // --- Parameter callback for dynamic reconfigure ---
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PointCloudFilter::onParameterChange, this, std::placeholders::_1)
    );
  }

private:
  // Parameter callback
  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params)
  {
    for (const auto &param : params)
    {
      if (param.get_name() == "min_z") min_z_ = param.as_double();
      else if (param.get_name() == "max_z") max_z_ = param.as_double();
      else if (param.get_name() == "min_x") min_x_ = param.as_double();
      else if (param.get_name() == "max_x") max_x_ = param.as_double();
      else if (param.get_name() == "min_y") min_y_ = param.as_double();
      else if (param.get_name() == "max_y") max_y_ = param.as_double();
      else if (param.get_name() == "filter_rings") filter_rings_ = param.as_bool();
      else if (param.get_name() == "filter_box") filter_box_ = param.as_bool();
      else if (param.get_name() == "obtain_outside_box") obtain_outside_box_ = param.as_bool();
      else if (param.get_name() == "publish_filtered_pointcloud") pub_filtered_pointcloud_ = param.as_bool();
      else if (param.get_name() == "publish_laserscan") pub_laserscan_ = param.as_bool();
      else if (param.get_name() == "scan_angle_min") scan_angle_min_ = param.as_double();
      else if (param.get_name() == "scan_angle_max") scan_angle_max_ = param.as_double();
      else if (param.get_name() == "scan_angle_increment") scan_angle_increment_ = param.as_double();
      else if (param.get_name() == "scan_range_min") scan_range_min_ = param.as_double();
      else if (param.get_name() == "scan_range_max") scan_range_max_ = param.as_double();
      else if (param.get_name() == "laserscan_frame") laserscan_frame_ = param.as_string();
    }
    // Recalculate num_bins_ if relevant parameters changed
    num_bins_ = static_cast<int>((scan_angle_max_ - scan_angle_min_) / scan_angle_increment_) + 1;
    RCLCPP_INFO(this->get_logger(), "Parameters updated dynamically.");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  // Helper to initialize parameters at startup
  void update_parameters()
  {
    this->get_parameter("min_x", min_x_);
    this->get_parameter("max_x", max_x_);
    this->get_parameter("min_y", min_y_);
    this->get_parameter("max_y", max_y_);
    this->get_parameter("min_z", min_z_);
    this->get_parameter("max_z", max_z_);
    this->get_parameter("filter_rings", filter_rings_);
    this->get_parameter("filter_box", filter_box_);
    this->get_parameter("obtain_outside_box", obtain_outside_box_);
    this->get_parameter("publish_filtered_pointcloud", pub_filtered_pointcloud_);
    this->get_parameter("publish_laserscan", pub_laserscan_);
    this->get_parameter("scan_angle_min", scan_angle_min_);
    this->get_parameter("scan_angle_max", scan_angle_max_);
    this->get_parameter("scan_angle_increment", scan_angle_increment_);
    this->get_parameter("scan_range_min", scan_range_min_);
    this->get_parameter("scan_range_max", scan_range_max_);
    this->get_parameter("laserscan_frame", laserscan_frame_);
    num_bins_ = static_cast<int>((scan_angle_max_ - scan_angle_min_) / scan_angle_increment_) + 1;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // --- Step 0: Detect if the incoming pointcloud has intensity data ---
    bool has_intensity = false;
    for (const auto & field : msg->fields)
    {
      if (field.name == "intensity")
      {
        has_intensity = true;
        break;
      }
    }

    // --- Step 1: Convert the incoming message to a unified point cloud type ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (has_intensity)
    {
      // Convert directly if intensity exists
      pcl::fromROSMsg(*msg, *input_cloud);
    }
    else
    {
      // Convert to pcl::PointXYZ first
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *temp_cloud);
      // Then copy to a pcl::PointXYZI cloud, setting intensity to 0
      for (const auto & pt : temp_cloud->points)
      {
        pcl::PointXYZI pt_i;
        pt_i.x = pt.x;
        pt_i.y = pt.y;
        pt_i.z = pt.z;
        pt_i.intensity = 0.0f;
        input_cloud->points.push_back(pt_i);
      }
      input_cloud->header = temp_cloud->header;
    }

    // --- Step 2: Remove LiDAR Rings (if enabled) ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr intermediate_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (filter_rings_)
    {
      const int block_size = 16;
      size_t total_points = input_cloud->points.size();
      size_t num_blocks = total_points / block_size;
      intermediate_cloud->header = input_cloud->header;

      for (size_t i = 0; i < num_blocks; ++i)
      {
        if ((i % 2) == 0)  // Keep even-indexed blocks
        {
          for (int j = 0; j < block_size; ++j)
          {
            size_t idx = i * block_size + j;
            if (idx < input_cloud->points.size())
            {
              const auto & pt = input_cloud->points[idx];
              if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
              {
                intermediate_cloud->points.push_back(pt);
              }
            }
          }
        }
      }
    }
    else
    {
      intermediate_cloud = input_cloud;
    }

    // --- Step 3: Apply BOX Filtering (get outside) ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr box_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (filter_box_) {
      if (obtain_outside_box_)
      {
        for (const auto & pt : intermediate_cloud->points)
        {
          if (pt.z >= min_z_ && pt.z <= max_z_ &&
              (pt.x < min_x_ || pt.x > max_x_ || pt.y < min_y_ || pt.y > max_y_))
          {
            box_filtered_cloud->points.push_back(pt);
          }
        }
      }
      else
      {
        for (const auto & pt : intermediate_cloud->points)
        {
          if (pt.z >= min_z_ && pt.z <= max_z_ &&
              pt.x >= min_x_ && pt.x <= max_x_ &&
              pt.y >= min_y_ && pt.y <= max_y_)
          {
            box_filtered_cloud->points.push_back(pt);
          }
        }
      }
      box_filtered_cloud->header = intermediate_cloud->header;
    }
    else {
      // No box filtering, just copy the input
      box_filtered_cloud = intermediate_cloud;
    }

    // --- Step 4: Publish the Filtered 3D Point Cloud ---
    if(pub_filtered_pointcloud_)
    {
      // If the original cloud had intensity, publish as-is.
      // Otherwise, convert to a point cloud without intensity.
      if (has_intensity)
      {
        publishFilteredPointCloud(msg->header, box_filtered_cloud);
      }
      else
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_intensity(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_no_intensity->header = box_filtered_cloud->header;
        for (const auto & pt : box_filtered_cloud->points)
        {
          pcl::PointXYZ pt_xyz;
          pt_xyz.x = pt.x;
          pt_xyz.y = pt.y;
          pt_xyz.z = pt.z;
          cloud_no_intensity->points.push_back(pt_xyz);
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_no_intensity, cloud_msg);
        cloud_msg.header = msg->header;
        filtered_pc_pub_->publish(cloud_msg);
      }
    }

    // --- Step 5: Create and Publish a 2D LaserScan Message ---
    // LaserScan generation uses only spatial coordinates, so intensity is irrelevant.
    if (pub_laserscan_) publishLaserScan(msg->header, box_filtered_cloud);
  }


  void publishFilteredPointCloud(const std_msgs::msg::Header &header, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    filtered_pc_pub_->publish(cloud_msg);
  }

  void publishLaserScan(const std_msgs::msg::Header &header, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    if (cloud->points.empty())
      return;

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = header;
    // Set frame_id if laserscan_frame_ is not empty
    if (!laserscan_frame_.empty()) {
      scan_msg.header.frame_id = laserscan_frame_;
    }    
    scan_msg.angle_min = scan_angle_min_;
    scan_msg.angle_max = scan_angle_max_;
    scan_msg.angle_increment = scan_angle_increment_;
    scan_msg.range_min = scan_range_min_;
    scan_msg.range_max = scan_range_max_;
    scan_msg.ranges.resize(num_bins_, std::numeric_limits<float>::infinity());

    for (const auto & pt : cloud->points)
    {
      float angle = std::atan2(pt.y, pt.x);
      float range = std::hypot(pt.x, pt.y);

      if (range < scan_range_min_ || range > scan_range_max_)
        continue;

      int index = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
      if (index >= 0 && index < num_bins_)
      {
        scan_msg.ranges[index] = std::min(scan_msg.ranges[index], range);
      }
    }

    laser_scan_pub_->publish(scan_msg);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  
  // Filtering parameters
  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  float min_z_;
  float max_z_;
  bool filter_rings_;
  bool filter_box_;
  bool obtain_outside_box_;
  bool pub_filtered_pointcloud_;
  bool pub_laserscan_;

  // LaserScan parameters
  float scan_angle_min_;
  float scan_angle_max_;
  float scan_angle_increment_;
  float scan_range_min_;
  float scan_range_max_;
  int num_bins_;
  std::string laserscan_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
