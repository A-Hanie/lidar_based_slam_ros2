#include "localization/localization.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true); // Enable intra-process communication

        auto node = std::make_shared<localization>(options);
        RCLCPP_INFO(node->get_logger(), "localization node has started.");

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("localization"), "Failed to start localization node: %s", e.what());
        rclcpp::shutdown();
        return 1; // Error
    }

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("localization"), "localization node has shut down.");
    return 0;
}
