#include "pointcloud_control_panel/localization_section.hpp"

namespace pointcloud_control_panel
{
    LocalizationSection::LocalizationSection(QWidget *parent, rviz_common::DisplayGroup *displayGroup)
        : QGroupBox("📍 | Localization", parent)
    {

        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        // Localization controls
        localizationButton_ = new QPushButton("▶ Start");
        QPushButton *restartButton = new QPushButton("⟳ Restart");
        QHBoxLayout *buttonLayout = new QHBoxLayout;
        buttonLayout->addWidget(localizationButton_);
        buttonLayout->addWidget(restartButton);
        mainLayout->addLayout(buttonLayout);

        connect(localizationButton_, &QPushButton::clicked, [this](void)
                { toggleLocalization_(); });
        connect(restartButton, &QPushButton::clicked, [this](void)
                { restartLocalization_(); });

        // Error Injection Subsection
        errorInjectionGroup_ = new QGroupBox("👾 | Error Injection", this);
        QVBoxLayout *errorLayout = new QVBoxLayout(errorInjectionGroup_);

        QHBoxLayout *errorValueLayout = new QHBoxLayout;
        QLabel *meanLabel = new QLabel("Mean value (μ): ");
        meanValueEditor_ = new QDoubleSpinBox;
        meanValueEditor_->setRange(-99.0, 99.0);
        meanValueEditor_->setValue(0.0);
        meanValueEditor_->setSingleStep(0.1);
        QLabel *varianceLabel = new QLabel("std deviation (σ) : ");
        stddevValueEditor_ = new QDoubleSpinBox;
        stddevValueEditor_->setRange(0, 99.0);
        stddevValueEditor_->setValue(0.0);
        stddevValueEditor_->setSingleStep(0.1);

        errorValueLayout->addWidget(meanLabel);
        errorValueLayout->addWidget(meanValueEditor_);
        errorValueLayout->addWidget(varianceLabel);
        errorValueLayout->addWidget(stddevValueEditor_);
        errorLayout->addLayout(errorValueLayout);

        // Toggle Error Injection Button
        toggleInjectionButton_ = new QPushButton("▶ Start Injection");
        errorLayout->addWidget(toggleInjectionButton_);

        mainLayout->addWidget(errorInjectionGroup_);

        connect(toggleInjectionButton_, &QPushButton::clicked, [this](void)
                { toggleErrorInjection_(); });
    }

    void LocalizationSection::toggleLocalization_()
    {
        if (localizationRunning_)
        {
            pauseLocalization_();
            localizationButton_->setText("▶  Resume");

            // meanValueEditor_->setEnabled(true);
            // stddevValueEditor_->setEnabled(true);
        }
        else
        {
            resumeLocalization_();
            localizationButton_->setText("▐▐  Pause");

            // meanValueEditor_->setEnabled(false);
            // stddevValueEditor_->setEnabled(false);
        }
    }

    void LocalizationSection::restartLocalization_()
    {
        // resumeLocalization_();

        // system("ros2 service call /restart_localization std_srvs/srv/Trigger \"{}\"&");
        // system("ros2 service call /reset_pointcloud_index std_srvs/srv/Trigger \"{}\"&");

        // system("ros2 service call /resume_slam std_srvs/srv/Trigger \"{}\"&");
        // localizationButton_->setText("▐▐  Pause");
        // meanValueEditor_->setEnabled(false);
        // stddevValueEditor_->setEnabled(false);

        
        pauseLocalization_();

        system("ros2 service call /reset_pointcloud_index std_srvs/srv/Trigger \"{}\"&");

        system("ros2 service call /restart_localization std_srvs/srv/Trigger \"{}\"&");

        localizationButton_->setText("▶  Start");

        // meanValueEditor_->setEnabled(true);
        // stddevValueEditor_->setEnabled(true);
    }

    void LocalizationSection::resumeLocalization_()
    {
        if (localizationRunning_)
            return;


        system("ros2 service call /resume_slam std_srvs/srv/Trigger \"{}\"&");

        localizationRunning_ = true;
    }

    void LocalizationSection::pauseLocalization_()
    {
        if (!localizationRunning_)
            return;

        // service Call
        system("ros2 service call /pause_slam std_srvs/srv/Trigger \"{}\"&");
        localizationRunning_ = false;
    }

    void LocalizationSection::toggleErrorInjection_()
    {
        errorInjectionActive_ = !errorInjectionActive_;

        if (errorInjectionActive_)
        {
            // Start error injection

            meanValueEditor_->setEnabled(false);
            stddevValueEditor_->setEnabled(false);

            // set mean and std
            std::ostringstream command;

            command << "ros2 service call /set_noise_values pointcloud_interfaces/srv/SetNoiseValues \"{mean: ";
            command << meanValueEditor_->value();
            command << ", stddev: ";
            command << stddevValueEditor_->value();
            command << "}\"&";
            // service Call
            system(command.str().c_str());

            toggleInjectionButton_->setText("❌ Stop Injection");
        }
        else
        {
            // Stop error injection

            // set mean and std
            std::ostringstream command;

            command << "ros2 service call /set_noise_values pointcloud_interfaces/srv/SetNoiseValues \"{mean: ";
            command << 0.0f;
            command << ", stddev: ";
            command << 0.0f;
            command << "}\"&";
            // service Call
            system(command.str().c_str());

            // Relocalize with ground truth value
            system("ros2 service call /relocalize std_srvs/srv/Trigger \"{}\"&");

            meanValueEditor_->setEnabled(true);
            stddevValueEditor_->setEnabled(true);
            toggleInjectionButton_->setText("▶ Start Injection");
        }
    }
}
