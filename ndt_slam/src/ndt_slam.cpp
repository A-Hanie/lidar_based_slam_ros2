#include "ndt_slam/ndt_slam.hpp"
#include <chrono>

using namespace std::chrono_literals;

Ndt_slam::Ndt_slam(const rclcpp::NodeOptions &options)
    : Node("scan_matcher"),
      clock_(RCL_ROS_TIME),
      tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
      listener_(tfbuffer_),
      broadcaster_(this),
      map_frame_("map"),
      base_frame_("base_link"),
      odom_frame_("odom"),
      ndt_resolution_(1.0),
      ndt_num_threads_(2),
      dist_threshold_update_(0.1),
      reading_voxel_grid_size_(0.2),
      map_voxel_grid_size_(0.1),
      scan_min_range_(0.1),
      scan_max_range_(20.0),
      map_publish_period_(0.1),
      num_targeted_cloud_(250),
      initial_pose_x_(0.0),
      initial_pose_y_(0.0),
      initial_pose_z_(0.0),
      initial_pose_qx_(0.0),
      initial_pose_qy_(0.0),
      initial_pose_qz_(0.0),
      initial_pose_qw_(0.0),
      is_intial_pose_set(true)
{

  auto ndt = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();

  ndt->setResolution(ndt_resolution_);
  ndt->setTransformationEpsilon(0.01);

  ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt->setNumThreads(ndt_num_threads_);

  auto deleter = [ndt](pcl::Registration<pcl::PointXYZI, pcl::PointXYZI> *) mutable
  { ndt.reset(); };
  registration_method_ = boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>>(ndt.get(), deleter);

  // Set message headers
  map_array_msg_.header.frame_id = map_frame_;
  map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;
  path_.header.frame_id = map_frame_;

  initializePubSub();

  save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        this->savePointCloud("final_map.pcd");
        response->success = true;
        response->message = "Map saved successfully.";
      });

  if (is_intial_pose_set)
  {
    setInitialPose();
  }

  RCLCPP_INFO(get_logger(), "initialization end");

  
}

void Ndt_slam::setInitialPose()
{
  RCLCPP_INFO(get_logger(), "Setting initial pose");
  auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_msg->header.stamp = now();
  pose_msg->header.frame_id = map_frame_;
  pose_msg->pose.position.x = initial_pose_x_;
  pose_msg->pose.position.y = initial_pose_y_;
  pose_msg->pose.position.z = initial_pose_z_;
  pose_msg->pose.orientation.x = initial_pose_qx_;
  pose_msg->pose.orientation.y = initial_pose_qy_;
  pose_msg->pose.orientation.z = initial_pose_qz_;
  pose_msg->pose.orientation.w = initial_pose_qw_;
  current_pose_stamped_ = *pose_msg;
  pose_pub_->publish(current_pose_stamped_);
  initial_pose_received_ = true;
  path_.poses.push_back(*pose_msg);
}

// callback functions
void Ndt_slam::cloud_callback(const typename sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!initial_pose_received_)
  {
    RCLCPP_WARN(get_logger(), "initial_pose is not received");
    return;
  }

  sensor_msgs::msg::PointCloud2 transformed_msg;
  try
  {
    tf2::TimePoint time_point = tf2::TimePoint(
        std::chrono::seconds(msg->header.stamp.sec) +
        std::chrono::nanoseconds(msg->header.stamp.nanosec));
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
        base_frame_, msg->header.frame_id, time_point);
    tf2::doTransform(*msg, transformed_msg, transform);
  }
  catch (tf2::TransformException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(transformed_msg, *pc_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr2(new pcl::PointCloud<pcl::PointXYZI>());
  tmp_ptr2->points.reserve(pc_ptr->points.size()); // Preallocate

  double squared_scan_min_range = scan_min_range_ * scan_min_range_;
  double squared_scan_max_range = scan_max_range_ * scan_max_range_;

  // Filter point cloud based on distace
  for (const auto &p : pc_ptr->points)
  {
    double squared_distance = p.x * p.x + p.y * p.y;
    if (squared_distance > squared_scan_min_range && squared_distance < squared_scan_max_range)
    {
      tmp_ptr2->points.push_back(p); // Add point within the valid range
    }
  }

  pc_ptr.swap(tmp_ptr2); // update pc_ptr

  handlePointCloud(pc_ptr, msg->header);
}

void Ndt_slam::initial_pose_callback(const typename geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id != map_frame_)
  {
    RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
    return;
  }
  RCLCPP_INFO(get_logger(), "initial_pose is received");

  current_pose_stamped_ = *msg;
  previous_position_.x() = current_pose_stamped_.pose.position.x;
  previous_position_.y() = current_pose_stamped_.pose.position.y;
  previous_position_.z() = current_pose_stamped_.pose.position.z;
  initial_pose_received_ = true;

  pose_pub_->publish(current_pose_stamped_);
}

/**
 * @brief Sets up publishers and subscribers for the node.
 **/
void Ndt_slam::initializePubSub()
{
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
  map_array_pub_ = this->create_publisher<slam_msgs::msg::MapArray>("map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("initial_pose", 10, std::bind(&Ndt_slam::initial_pose_callback, this, std::placeholders::_1));
  input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input_cloud", rclcpp::SensorDataQoS(), std::bind(&Ndt_slam::cloud_callback, this, std::placeholders::_1));
}

/**
 * @brief Processes and updates the map with incoming point cloud data.
 * 
 * Initializes the map on the first call and continuously updates it with new point clouds.
 * 
 * @param pc_ptr Shared pointer to the point cloud data.
 * @param header Header of the point cloud message.
 */
void Ndt_slam::handlePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr, const std_msgs::msg::Header &header)
{
    try
    {
        if (!is_map_init_flag_)
        {
            RCLCPP_INFO(get_logger(), "Initializing map with the first point cloud.");
            map_init(pc_ptr, header);
            is_map_init_flag_ = true;
            last_map_time_ = clock_.now();
            RCLCPP_INFO(get_logger(), "Map initialization completed.");
        }

        RCLCPP_DEBUG(get_logger(), "Processing received point cloud.");
        receiveCloud(pc_ptr, header.stamp);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to process point cloud: %s, Stamp: %u",
                     e.what(), header.stamp.sec);
        throw;
    }
}

void Ndt_slam::map_init(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr, const std_msgs::msg::Header &header)
{
    auto cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(map_voxel_grid_size_, map_voxel_grid_size_, map_voxel_grid_size_);
    voxel_grid.setInputCloud(pc_ptr);
    voxel_grid.filter(*cloud_ptr);

    if (cloud_ptr->empty())
    {
        RCLCPP_WARN(get_logger(), "Filtered cloud is empty");
        return;
    }

    // Transform the cloud to the current pose.
    auto sim_trans = poseToEigenMatrix(current_pose_stamped_.pose);
    auto transformed_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);

    // Set the transformed cloud as the new target for registration.
    registration_method_->setInputTarget(transformed_cloud_ptr);

    // publish transformed cloud  .
    auto map_msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

    // Prepare a map segment for the map array.
    auto cloud_msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
    slam_msgs::msg::MapSeg mapseg;
    mapseg.header = header;
    mapseg.pose = current_pose_stamped_.pose;
    mapseg.cloud = *cloud_msg_ptr;
    map_array_msg_.header = header;
    map_array_msg_.mapsegs.push_back(mapseg);

    // Publish the map
    map_pub_->publish(mapseg.cloud);

    RCLCPP_INFO(get_logger(), "Map initialized with a new segment, total points: %lu", transformed_cloud_ptr->size());
}


void Ndt_slam::receiveCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_ptr, const rclcpp::Time &stamp)
{
    // Check if a map update is ready to process new input.
    if (mapping_flag_ && mapping_future_.valid())
    {
        if (mapping_future_.wait_for(0s) == std::future_status::ready)
        {
            if (is_map_updated_)
            {
                auto targeted_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(targeted_cloud_);
                registration_method_->setInputTarget(targeted_cloud_ptr);
                is_map_updated_ = false;
            }
            mapping_flag_ = false;
            mapping_thread_.detach(); // Safely detach completed thread
        }
    }

    // Preprocess the input cloud using voxel grid filtering to reduce data size and improve processing time.
    auto filtered_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(reading_voxel_grid_size_, reading_voxel_grid_size_, reading_voxel_grid_size_);
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.filter(*filtered_cloud_ptr);

    // Exit if no points remain after filtering.
    if (filtered_cloud_ptr->empty())
    {
        RCLCPP_WARN(get_logger(), "Filtered cloud is empty, skipping registration.");
        return;
    }

    // Register the filtered cloud using NDT.
    registration_method_->setInputSource(filtered_cloud_ptr);
    auto sim_trans = poseToEigenMatrix(current_pose_stamped_.pose);

    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto system_clock = rclcpp::Clock(RCL_SYSTEM_TIME);
    auto time_align_start = system_clock.now();
    registration_method_->align(*output_cloud, sim_trans);
    auto time_align_end = system_clock.now();

    // Evaluate and log the registration performance.
    auto duration = time_align_end - time_align_start;
    RCLCPP_INFO(get_logger(), "Alignment took %f ms.", duration.seconds() * 1000.0);

    // Handle the case where alignment fails.
    if (!registration_method_->hasConverged())
    {
        RCLCPP_ERROR(get_logger(), "NDT alignment did not converge.");
        return;
    }

    auto final_transformation = registration_method_->getFinalTransformation();

    // Publish the updated pose based on the registration result.
    publishTransformedPose(cloud_ptr, final_transformation, stamp);
}


void Ndt_slam::publishTransformedPose(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_ptr,
    const Eigen::Matrix4f &final_transformation,
    const rclcpp::Time &stamp)
{
  // Extract translation and rotation from the transformation matrix
  Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond orientation(final_transformation.block<3, 3>(0, 0).cast<double>());

  // Convert the Eigen quaternion to a ROS message
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(orientation);

  // Set up the transform to be published
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = base_frame_;
  transform_stamped.transform.translation.x = position.x();
  transform_stamped.transform.translation.y = position.y();
  transform_stamped.transform.translation.z = position.z();
  transform_stamped.transform.rotation = quat_msg;

  // Broadcast the transformation
  broadcaster_.sendTransform(transform_stamped);

  // Update the current pose stamped message
  current_pose_stamped_.header.stamp = stamp;
  current_pose_stamped_.pose.position = tf2::toMsg(position);
  current_pose_stamped_.pose.orientation = quat_msg;

  // Publish the current pose
  pose_pub_->publish(current_pose_stamped_);

  // Update the path and publish
  path_.poses.push_back(current_pose_stamped_);
  path_pub_->publish(path_);

  // Check if a map update is needed based on the movement threshold
  movement_distance_ = (position - previous_position_).norm();
  if (movement_distance_ >= dist_threshold_update_ && !mapping_flag_)
  {
    prepareMapUpdateTask(cloud_ptr, final_transformation);
  }
}

void Ndt_slam::prepareMapUpdateTask(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_ptr,
    const Eigen::Matrix4f &final_transformation)
{
  previous_position_ = Eigen::Vector3d(final_transformation.block<3, 1>(0, 3).cast<double>());

  // Set up and dispatch the map update task
  mapping_task_ = std::packaged_task<void()>(
      std::bind(&Ndt_slam::updateMap, this, cloud_ptr, final_transformation, current_pose_stamped_));
  mapping_future_ = mapping_task_.get_future();
  mapping_thread_ = std::thread(std::move(mapping_task_));
  mapping_flag_ = true;
}

void Ndt_slam::updateMap(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr,
    const Eigen::Matrix4f final_transformation,
    const geometry_msgs::msg::PoseStamped current_pose_stamped)
{
    // Reduce the number of points using voxel grid
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(map_voxel_grid_size_, map_voxel_grid_size_, map_voxel_grid_size_);
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.filter(*filtered_cloud_ptr);

    if (filtered_cloud_ptr->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty after voxel grid filtering.");
        return;
    }

    // Transform the filtered cloud according to the given transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, final_transformation);

    // Update the targeted cloud
    targeted_cloud_.clear();
    targeted_cloud_ += *transformed_cloud_ptr;

    size_t num_mapsegs = map_array_msg_.mapsegs.size();
    for (int i = 0; i < std::min(num_targeted_cloud_ - 1, static_cast<int>(num_mapsegs)); ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(map_array_msg_.mapsegs[num_mapsegs - 1 - i].cloud, *pc_ptr);
        
        Eigen::Affine3d mapseg_affine;
        tf2::fromMsg(map_array_msg_.mapsegs[num_mapsegs - 1 - i].pose, mapseg_affine);
        pcl::transformPointCloud(*pc_ptr, *pc_ptr, mapseg_affine.matrix());

        targeted_cloud_ += *pc_ptr;
    }

    // Prepare the map segment for publication
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*filtered_cloud_ptr, cloud_msg);
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.header.stamp = this->get_clock()->now();

    slam_msgs::msg::MapSeg mapseg;
    mapseg.header.frame_id = map_frame_;
    mapseg.header.stamp = current_pose_stamped.header.stamp;
    latest_distance_ += movement_distance_;   
    mapseg.distance = latest_distance_;
    mapseg.pose = current_pose_stamped.pose;
    mapseg.cloud = cloud_msg;
    map_array_msg_.mapsegs.push_back(mapseg);

    map_array_pub_->publish(map_array_msg_);

    is_map_updated_ = true;


    // Publish the complete map
    rclcpp::Time map_time = this->get_clock()->now();
    double dt = (map_time - last_map_time_).seconds();
    if (dt > map_publish_period_) {
        map_pub(map_array_msg_, map_frame_);
        last_map_time_ = map_time;
    }

    RCLCPP_INFO(this->get_logger(), "Map updated successfully.");
}

Eigen::Matrix4f Ndt_slam::poseToEigenMatrix(const geometry_msgs::msg::Pose pose)
{
  // Convert ROS Pose to Eigen Affine transformation
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);

  // Cast the transformation matrix to single precision float
  return affine.matrix().cast<float>();
}

void Ndt_slam::map_pub(const slam_msgs::msg::MapArray &map_array_msg, const std::string &map_frame_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_map(new pcl::PointCloud<pcl::PointXYZI>);

  // Process each mapseg and transform to the map frame
  for (const auto &mapseg : map_array_msg.mapsegs)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(mapseg.cloud, *cloud_ptr);

    Eigen::Affine3d transformation;
    tf2::fromMsg(mapseg.pose, transformation);
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, transformation.matrix().cast<float>());

    *aggregated_map += *cloud_ptr; // Aggregated map
  }

  // Convert the aggregated point cloud to a ROS message
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*aggregated_map, map_msg);
  map_msg.header.frame_id = map_frame_id;
  map_msg.header.stamp = this->get_clock()->now();

  // Publish the complete map
  map_pub_->publish(map_msg);

}

void Ndt_slam::savePointCloud(const std::string& filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_map(new pcl::PointCloud<pcl::PointXYZI>());

    // Aggregate all map segments
    for (const auto& mapseg : map_array_msg_.mapsegs)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(mapseg.cloud, *cloud_ptr);

        Eigen::Affine3d transformation;
        tf2::fromMsg(mapseg.pose, transformation);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, transformation.matrix().cast<float>());

        *aggregated_map += *cloud_ptr; // Aggregated map
    }

    // Save the aggregated map to a PCD file
    if (pcl::io::savePCDFileBinary(filename, *aggregated_map) == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Saved point cloud map to %s", filename.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud map to %s", filename.c_str());
    }
}
