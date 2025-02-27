#include "TelemetryWidget.hpp"
#include "ui_TelemetryWidget.h"
#include "TelemetryData.hpp"

#include <QLabel>
#include <QProgressBar>
#include <cmath>

TelemetryWidget::TelemetryWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::TelemetryWidget)
{
    ui->setupUi(this);
    
    // Initialize displays with zeros
    double zeros[3] = {0.0, 0.0, 0.0};
    updatePositionDisplay(zeros);
    updateVelocityDisplay(zeros);
    updateOrientationDisplay(zeros);
}

TelemetryWidget::~TelemetryWidget()
{
}

void TelemetryWidget::updateTelemetry(const TelemetryData& data)
{
    // Call the existing method with the data from the struct
    updateTelemetry(data.position, data.velocity, data.orientation);
}

void TelemetryWidget::updateTelemetry(const double position[3],
                                     const double velocity[3],
                                     const double orientation[3])
{
    updatePositionDisplay(position);
    updateVelocityDisplay(velocity);
    updateOrientationDisplay(orientation);
}

void TelemetryWidget::updatePositionDisplay(const double position[3])
{
    // Update position labels
    ui->lblNorth->setText(formatPosition(position[0]));
    ui->lblEast->setText(formatPosition(position[1]));
    ui->lblDown->setText(formatPosition(position[2]));
    
    // Update altitude (negative of down)
    ui->lblAltitude->setText(formatPosition(-position[2]));
    
    // Update position gauges/indicators if needed
    ui->gaugeAltitude->setValue(static_cast<int>(-position[2]));
}

void TelemetryWidget::updateVelocityDisplay(const double velocity[3])
{
    // Update velocity labels
    ui->lblVelocityX->setText(formatSpeed(velocity[0]));
    ui->lblVelocityY->setText(formatSpeed(velocity[1]));
    ui->lblVelocityZ->setText(formatSpeed(velocity[2]));
    
    // Calculate and update airspeed (simplified)
    double airspeed = std::sqrt(velocity[0]*velocity[0] + 
                               velocity[1]*velocity[1] + 
                               velocity[2]*velocity[2]);
    ui->lblAirspeed->setText(formatSpeed(airspeed));
    
    // Update speed gauges
    ui->gaugeAirspeed->setValue(static_cast<int>(airspeed));
}

void TelemetryWidget::updateOrientationDisplay(const double orientation[3])
{
    // Update orientation labels
    ui->lblRoll->setText(formatAngle(orientation[0]));
    ui->lblPitch->setText(formatAngle(orientation[1]));
    ui->lblYaw->setText(formatAngle(orientation[2]));
    
    // Update heading indicator (compass)
    double headingDeg = orientation[2] * 180.0 / M_PI;
    if (headingDeg < 0) {
        headingDeg += 360.0;
    }
    ui->gaugeHeading->setValue(static_cast<int>(headingDeg));
    
    // Update attitude indicator (artificial horizon)
    // This would be implemented with custom drawing in a real application
    
    // Update roll and pitch gauges
    ui->gaugeRoll->setValue(static_cast<int>(orientation[0] * 180.0 / M_PI));
    ui->gaugePitch->setValue(static_cast<int>(orientation[1] * 180.0 / M_PI));
}

QString TelemetryWidget::formatAngle(double radians) const
{
    // Convert radians to degrees and format
    double degrees = radians * 180.0 / M_PI;
    return QString::number(degrees, 'f', 1) + "°";
}

QString TelemetryWidget::formatSpeed(double speed) const
{
    // Format speed in m/s
    return QString::number(speed, 'f', 1) + " m/s";
}

QString TelemetryWidget::formatPosition(double pos) const
{
    // Format position in meters
    return QString::number(pos, 'f', 1) + " m";
} 