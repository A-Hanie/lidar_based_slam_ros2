#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PCBMapPublisher : public rclcpp::Node {
public:
    PCBMapPublisher() : Node("pcb_map_publisher") {
        std::string package_path = ament_index_cpp::get_package_share_directory("ndt_slam");
        std::string full_path = package_path + "/data/map.pcd";

        map_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(full_path, *map_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map file.");
            return;
        }
        
        auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization(
            RMW_QOS_POLICY_HISTORY_KEEP_ALL,    
            2                                   
        ));
        custom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", custom_qos);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&PCBMapPublisher::publishMap, this)
        );
    }

private:
    void publishMap() {
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud_, map_msg);
        map_msg.header.frame_id = "global_map";
        map_msg.header.stamp = this->now();
        map_publisher_->publish(map_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCBMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
