#pragma once

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include <vector>

// Forward declarations
namespace Ui {
class MainWindow;
}

class TelemetryWidget;
class ControlPanel;
class Flight3DView;
class QUdpSocket;

/**
 * @brief Main window for the FalconSim GUI application.
 * 
 * This class manages the primary user interface, including telemetry displays,
 * control panels, and 3D visualization of the UAV.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    /**
     * Connect to a telemetry server
     * @param host Hostname or IP address
     * @param port UDP port number
     */
    void connectToServer(const QString &host, quint16 port);

private slots:
    // Network slots
    void onConnectButtonClicked();
    void onDisconnectButtonClicked();
    void onDataReceived();
    
    // UI interaction slots
    void onStartSimulation();
    void onPauseSimulation();
    void onStopSimulation();
    void onUpdateSimulationRate(int value);
    
    // Timer for updating UI
    void onUpdateTimer();

private:
    // UI components
    std::unique_ptr<Ui::MainWindow> ui;
    TelemetryWidget* m_telemetryWidget{nullptr};
    ControlPanel* m_controlPanel{nullptr};
    Flight3DView* m_flight3DView{nullptr};
    
    // Network components
    QUdpSocket* m_socket{nullptr};
    QString m_serverHost;
    quint16 m_serverPort{12345};
    bool m_connected{false};
    
    // Telemetry data
    struct TelemetryData {
        double timestamp{0.0};
        double position[3]{0.0, 0.0, 0.0};  // North, East, Down
        double velocity[3]{0.0, 0.0, 0.0};  // X, Y, Z (body frame)
        double orientation[3]{0.0, 0.0, 0.0};  // Roll, Pitch, Yaw
        double controls[4]{0.0, 0.0, 0.0, 0.0};  // Throttle, Aileron, Elevator, Rudder
    };
    TelemetryData m_telemetryData;
    
    // Simulation parameters
    bool m_simRunning{false};
    int m_updateRateHz{10};
    QTimer m_updateTimer;
    
    // Methods
    void setupUi();
    void setupConnections();
    void parseTelemetryData(const QString &data);
    void updateDisplays();
}; 