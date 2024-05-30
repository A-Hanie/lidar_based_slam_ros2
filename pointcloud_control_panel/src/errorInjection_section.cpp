#include "pointcloud_control_panel/errorInjection_section.hpp"
#include <QPushButton>

namespace pointcloud_control_panel {
    ErrorInjectionSection::ErrorInjectionSection(QWidget *parent) : QGroupBox("👾 | Error Injection", parent), meanValue_(0.5), stddevValue_(0.5) {
        dummy_node_ = std::make_shared<DummyNode>("errorInjection_dummy_node");

        QVBoxLayout* layout = new QVBoxLayout;
        QHBoxLayout* rowLayout = new QHBoxLayout;

        // Mean and Variance editors
        rowLayout->addWidget(new QLabel("Mean value: "));
        meanValueEditor_ = new QDoubleSpinBox;
        meanValueEditor_->setRange(0, 1.0);
        meanValueEditor_->setValue(meanValue_);
        meanValueEditor_->setSingleStep(0.1);
        rowLayout->addWidget(meanValueEditor_);
        rowLayout->addWidget(new QLabel("Variance value: "));
        stddevValueEditor_ = new QDoubleSpinBox;
        stddevValueEditor_->setRange(0, 1.0);
        stddevValueEditor_->setValue(stddevValue_);
        stddevValueEditor_->setSingleStep(0.1);
        rowLayout->addWidget(stddevValueEditor_);
        layout->addLayout(rowLayout);

        // Toggle button for start/stop injection
        QPushButton* toggleButton = new QPushButton("Start Injecting", this);
        connect(toggleButton, &QPushButton::clicked, this, &ErrorInjectionSection::toggleInjection);
        layout->addWidget(toggleButton);

        setLayout(layout);
    }

    void ErrorInjectionSection::updateMeanValue(double value) {
        meanValue_ = value;
    }

    void ErrorInjectionSection::updateStddevValue(double value) {
        stddevValue_ = value;
    }

    void ErrorInjectionSection::toggleInjection() {
        QPushButton* button = qobject_cast<QPushButton*>(sender());
        if (button->text() == "Start Injecting") {
            button->setText("Stop Injecting");
            startInjection(); // Start the injection process
        } else {
            button->setText("Start Injecting");
            stopInjection(); // Stop the injection process
        }
    }

    void ErrorInjectionSection::startInjection() {
        // Function to start injecting errors
    }

    void ErrorInjectionSection::stopInjection() {
        // Function to stop injecting errors
    }
}
