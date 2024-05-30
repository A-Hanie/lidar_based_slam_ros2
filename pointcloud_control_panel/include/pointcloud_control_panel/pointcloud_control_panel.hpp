#ifndef TURTLEBOT_CONTROL_PANEL_H
#define TURTLEBOT_CONTROL_PANEL_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QVBoxLayout>
#include <QString>
#include <QSizePolicy>

#include "pointcloud_control_panel/visibility_control.hpp"
#include "pointcloud_control_panel/localization_section.hpp"
#include "pointcloud_control_panel/errorInjection_section.hpp"

namespace pointcloud_control_panel {
    class PointCloudControlPanel : public rviz_common::Panel {
        Q_OBJECT
        public:
          PointCloudControlPanel(QWidget* parent = 0);

          virtual void save( rviz_common::Config config ) const;
          virtual void load( const rviz_common::Config& config );
          void onInitialize() override;
        
        private:
          rviz_common::DisplayContext* context_;

          LocalizationSection* localizationSection_;
          ErrorInjectionSection* errorInjectionSection_;

          rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription_;
    };
}

#endif