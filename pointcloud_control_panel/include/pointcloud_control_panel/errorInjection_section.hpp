#ifndef TELEOP_SECTION_H
#define TELEOP_SECTION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "pointcloud_control_panel/dummy_node.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QString>
#include <QPushButton>
#include <QDoubleSpinBox>

using std::placeholders::_1;

namespace pointcloud_control_panel {
  class ErrorInjectionSection : public QGroupBox {
    Q_OBJECT
    public:
      ErrorInjectionSection(QWidget *parent = 0);

    protected Q_SLOTS:
      void sendVel(char dir);
      void updateMeanValue(double value);
      void updateStddevValue(double value);
      void updateTopic();
      void toggleInjection();  

    private:
      std::shared_ptr<DummyNode> dummy_node_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

      QLineEdit* outputTopicEditor_;
      QString outputTopic_;

      QDoubleSpinBox* meanValueEditor_;
      QDoubleSpinBox* stddevValueEditor_;
      QPushButton* toggleButton_;  

      float meanValue_;
      float stddevValue_;

      void startInjection();
      void stopInjection(); 


  };
}

#endif
