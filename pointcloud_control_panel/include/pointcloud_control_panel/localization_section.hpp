#ifndef POINTCLOUD_CONTROL_PANEL_LOCALIZATION_HPP_
#define POINTCLOUD_CONTROL_PANEL_LOCALIZATION_HPP_

#include <QWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <rclcpp/rclcpp.hpp>

namespace rviz_common {
    class DisplayGroup;
}

namespace pointcloud_control_panel {
    class LocalizationSection : public QGroupBox {
        Q_OBJECT
    public:
        explicit LocalizationSection(QWidget *parent = nullptr, rviz_common::DisplayGroup *displayGroup = nullptr);

    private:
        QPushButton *localizationButton_;
        QPushButton *toggleInjectionButton_;
        
        QGroupBox *errorInjectionGroup_;
        QDoubleSpinBox *meanValueEditor_;
        QDoubleSpinBox *stddevValueEditor_;

        bool localizationRunning_ = false;
        bool errorInjectionActive_ = false; 

        void toggleLocalization_();
        void restartLocalization_();
        void resumeLocalization_();
        void pauseLocalization_();
        void toggleErrorInjection_(); 
    };
}

#endif // POINTCLOUD_CONTROL_PANEL_LOCALIZATION_HPP_
