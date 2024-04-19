#include "ndt_slam/ndt_slam.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true); // Enable intra-process communication

        auto node = std::make_shared<Ndt_slam>(options);
        RCLCPP_INFO(node->get_logger(), "Ndt_slam node has started.");

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("Ndt_slam"), "Failed to start Ndt_slam node: %s", e.what());
        rclcpp::shutdown();
        return 1; // Error
    }

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("Ndt_slam"), "Ndt_slam node has shut down.");
    return 0;
}
