#ifndef NDT_SLAM_H_
#define NDT_SLAM_H_

// Standard libraries
#include <memory>
#include <thread>
#include <future>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <slam_msgs/msg/map_array.hpp>
#include "std_srvs/srv/trigger.hpp"

// TF2 libraries for transformations
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL libraries for point cloud processing
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

class Ndt_slam : public rclcpp::Node
{
public:
  explicit Ndt_slam(const rclcpp::NodeOptions &options);

private:
  // ROS Types
  rclcpp::Clock clock_;
  rclcpp::Time last_map_time_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<slam_msgs::msg::MapArray>::SharedPtr map_array_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;

  // msgs 
  geometry_msgs::msg::PoseStamped current_pose_stamped_;
  slam_msgs::msg::MapArray map_array_msg_;
  nav_msgs::msg::Path path_;

  // Frames and TF
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;

  // Point Cloud Processing
  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_method_;
  pcl::PointCloud<pcl::PointXYZI> targeted_cloud_;

  bool mapping_flag_{false};
  bool is_map_updated_{false};

  std::thread mapping_thread_;
  std::packaged_task<void()> mapping_task_;
  std::future<void> mapping_future_;

  bool initial_pose_received_{false};
  bool is_map_init_flag_{false};

  // Mapping Parameters
  double dist_threshold_update_;
  double reading_voxel_grid_size_;
  double map_voxel_grid_size_;
  bool use_min_max_filter_{false};
  double scan_min_range_{0.1};
  double scan_max_range_{100.0};
  double map_publish_period_;
  int num_targeted_cloud_;

  // Flags
  bool is_intial_pose_set{false};

  // map
  Eigen::Vector3d previous_position_;

  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;

  // odom
  double latest_distance_{0};
  double movement_distance_{0};

  // NDT settings
  double ndt_resolution_;
  int ndt_num_threads_;

  // Private Methods
  void initializePubSub();
  void map_init(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr, const std_msgs::msg::Header &header);
  void receiveCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_ptr, const rclcpp::Time &stamp);
  void handlePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr, const std_msgs::msg::Header &header);
  void publishTransformedPose(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_ptr, const Eigen::Matrix4f &final_transformation, const rclcpp::Time &stamp);
  Eigen::Matrix4f poseToEigenMatrix(const geometry_msgs::msg::Pose pose);
  void map_pub(const slam_msgs::msg::MapArray &map_array_msg, const std::string &map_frame_id);
  void updateMap(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr, const Eigen::Matrix4f final_transformation, const geometry_msgs::msg::PoseStamped current_pose_stamped);
  void prepareMapUpdateTask(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_ptr, const Eigen::Matrix4f &final_transformation);
  void initial_pose_callback(const typename geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cloud_callback(const typename sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void setInitialPose();
  void savePointCloud(const std::string& filename);
};

#endif // NDT_SLAM_H_
