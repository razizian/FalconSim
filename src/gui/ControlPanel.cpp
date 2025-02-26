#include "ControlPanel.hpp"
#include "ui_ControlPanel.h"

ControlPanel::ControlPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ControlPanel)
{
    ui->setupUi(this);
    
    // Connect slider signals
    connect(ui->sliderThrottle, &QSlider::valueChanged, this, &ControlPanel::onThrottleSliderChanged);
    connect(ui->sliderAileron, &QSlider::valueChanged, this, &ControlPanel::onAileronSliderChanged);
    connect(ui->sliderElevator, &QSlider::valueChanged, this, &ControlPanel::onElevatorSliderChanged);
    connect(ui->sliderRudder, &QSlider::valueChanged, this, &ControlPanel::onRudderSliderChanged);
    
    // Connect reset button
    connect(ui->btnResetControls, &QPushButton::clicked, this, &ControlPanel::onResetControlsClicked);
}

ControlPanel::~ControlPanel()
{
}

void ControlPanel::updateControlDisplay(double throttle, double aileron, 
                                       double elevator, double rudder)
{
    // Set flag to prevent feedback loops
    m_updatingFromTelemetry = true;
    
    // Update sliders
    ui->sliderThrottle->setValue(static_cast<int>(throttle * 100.0));
    ui->sliderAileron->setValue(static_cast<int>(aileron * 100.0));
    ui->sliderElevator->setValue(static_cast<int>(elevator * 100.0));
    ui->sliderRudder->setValue(static_cast<int>(rudder * 100.0));
    
    // Update value labels
    ui->lblThrottleValue->setText(QString::number(throttle, 'f', 2));
    ui->lblAileronValue->setText(QString::number(aileron, 'f', 2));
    ui->lblElevatorValue->setText(QString::number(elevator, 'f', 2));
    ui->lblRudderValue->setText(QString::number(rudder, 'f', 2));
    
    m_updatingFromTelemetry = false;
}

void ControlPanel::onThrottleSliderChanged(int value)
{
    if (m_updatingFromTelemetry) {
        return;
    }
    
    double throttleValue = value / 100.0;
    ui->lblThrottleValue->setText(QString::number(throttleValue, 'f', 2));
    
    emit throttleChanged(throttleValue);
}

void ControlPanel::onAileronSliderChanged(int value)
{
    if (m_updatingFromTelemetry) {
        return;
    }
    
    double aileronValue = value / 100.0;
    ui->lblAileronValue->setText(QString::number(aileronValue, 'f', 2));
    
    emit aileronChanged(aileronValue);
}

void ControlPanel::onElevatorSliderChanged(int value)
{
    if (m_updatingFromTelemetry) {
        return;
    }
    
    double elevatorValue = value / 100.0;
    ui->lblElevatorValue->setText(QString::number(elevatorValue, 'f', 2));
    
    emit elevatorChanged(elevatorValue);
}

void ControlPanel::onRudderSliderChanged(int value)
{
    if (m_updatingFromTelemetry) {
        return;
    }
    
    double rudderValue = value / 100.0;
    ui->lblRudderValue->setText(QString::number(rudderValue, 'f', 2));
    
    emit rudderChanged(rudderValue);
}

void ControlPanel::onResetControlsClicked()
{
    // Reset all sliders to default positions
    ui->sliderThrottle->setValue(0);  // Throttle to zero
    ui->sliderAileron->setValue(0);   // Aileron centered
    ui->sliderElevator->setValue(0);  // Elevator centered
    ui->sliderRudder->setValue(0);    // Rudder centered
} 