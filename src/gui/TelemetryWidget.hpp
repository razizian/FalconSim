#pragma once

#include <QWidget>
#include <memory>
#include "TelemetryData.hpp"

// Forward declarations
class MainWindow;
namespace Ui {
class TelemetryWidget;
}

/**
 * @brief Widget for displaying telemetry data from the UAV
 * 
 * This widget shows real-time information about the UAV's position,
 * velocity, orientation, and other telemetry data.
 */
class TelemetryWidget : public QWidget {
    Q_OBJECT

public:
    explicit TelemetryWidget(QWidget *parent = nullptr);
    ~TelemetryWidget();
    
    /**
     * Update the telemetry display with new position, velocity and orientation data
     * 
     * @param position Array of 3 doubles [north, east, down]
     * @param velocity Array of 3 doubles [x, y, z] in body frame
     * @param orientation Array of 3 doubles [roll, pitch, yaw] in radians
     */
    void updateTelemetry(const double position[3],
                         const double velocity[3],
                         const double orientation[3]);
                         
    /**
     * Update the telemetry display with data from a TelemetryData struct
     * 
     * @param data The telemetry data struct containing all telemetry information
     */
    void updateTelemetry(const TelemetryData& data);

private:
    std::unique_ptr<Ui::TelemetryWidget> ui;
    
    // Formatting helpers
    QString formatAngle(double radians) const;
    QString formatSpeed(double speed) const;
    QString formatPosition(double pos) const;
    
    // Update specific elements
    void updatePositionDisplay(const double position[3]);
    void updateVelocityDisplay(const double velocity[3]);
    void updateOrientationDisplay(const double orientation[3]);
}; 