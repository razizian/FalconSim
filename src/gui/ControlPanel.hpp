#pragma once

#include <QWidget>
#include <memory>

namespace Ui {
class ControlPanel;
}

/**
 * @brief Control panel for UAV inputs
 * 
 * This widget provides sliders and buttons for controlling
 * the UAV, including throttle, aileron, elevator, and rudder inputs.
 */
class ControlPanel : public QWidget {
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = nullptr);
    ~ControlPanel();
    
    /**
     * Update display based on telemetry
     * @param throttle Throttle value (0.0-1.0)
     * @param aileron Aileron value (-1.0 to 1.0)
     * @param elevator Elevator value (-1.0 to 1.0)
     * @param rudder Rudder value (-1.0 to 1.0)
     */
    void updateControlDisplays(double throttle, double aileron, 
                              double elevator, double rudder);

signals:
    // Signals emitted when user changes control inputs
    void throttleChanged(double value);
    void aileronChanged(double value);
    void elevatorChanged(double value);
    void rudderChanged(double value);
    
    // Simulation control signals
    void startSimulation();
    void pauseSimulation();
    void stopSimulation();

private slots:
    // UI interactions
    void onThrottleChanged(int value);
    void onAileronChanged(int value);
    void onElevatorChanged(int value);
    void onRudderChanged(int value);
    void onResetControlsClicked();

private:
    std::unique_ptr<Ui::ControlPanel> ui;
    
    // Flag to prevent feedback loops when updating sliders from telemetry
    bool m_updatingFromTelemetry{false};
}; 