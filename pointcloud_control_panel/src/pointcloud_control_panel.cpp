#include "pointcloud_control_panel/pointcloud_control_panel.hpp"

namespace pointcloud_control_panel {
    PointCloudControlPanel::PointCloudControlPanel(QWidget* parent) : rviz_common::Panel(parent) {
        setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    }

    void PointCloudControlPanel::save(rviz_common::Config config) const {
        rviz_common::Panel::save(config);
    }

    void PointCloudControlPanel::load(const rviz_common::Config& config) {
        rviz_common::Panel::load(config);
    }

    void PointCloudControlPanel::onInitialize() {
        context_ = this->getDisplayContext();

        auto rviz_ros_node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        this->cmdVelSubscription_ = rviz_ros_node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            1,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                double groundTruthX = 0 /* Obtain ground truth X value */;
                double groundTruthY = 0 /* Obtain ground truth Y value */;
                double estimatedX = 0 /* Obtain estimated X value */;
                double estimatedY = 0 /* Obtain estimated Y value */;
            });

        QVBoxLayout* layout = new QVBoxLayout;

        localizationSection_ = new LocalizationSection(this, context_->getRootDisplayGroup());
        layout->addWidget(localizationSection_);


        setLayout(layout);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pointcloud_control_panel::PointCloudControlPanel, rviz_common::Panel)