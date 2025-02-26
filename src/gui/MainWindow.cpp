#include "MainWindow.hpp"
#include "ui_MainWindow.h"
#include "TelemetryWidget.hpp"
#include "ControlPanel.hpp"
#include "Flight3DView.hpp"

#include <QUdpSocket>
#include <QMessageBox>
#include <QDebug>
#include <QDockWidget>
#include <QStatusBar>
#include <QSettings>
#include <QCloseEvent>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_socket(new QUdpSocket(this))
{
    ui->setupUi(this);
    setupUi();
    setupConnections();
    
    // Set up update timer
    connect(&m_updateTimer, &QTimer::timeout, this, &MainWindow::onUpdateTimer);
    m_updateTimer.start(1000 / m_updateRateHz); // 10 Hz default
    
    // Restore window state
    QSettings settings("FalconSim", "GUI");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    
    statusBar()->showMessage("Ready");
}

MainWindow::~MainWindow()
{
    // Save window state
    QSettings settings("FalconSim", "GUI");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::connectToServer(const QString &host, quint16 port)
{
    m_serverHost = host;
    m_serverPort = port;
    onConnectButtonClicked();
}

void MainWindow::setupUi()
{
    // Create central widget for 3D view
    m_flight3DView = new Flight3DView(this);
    setCentralWidget(m_flight3DView);
    
    // Create telemetry widget as dock widget
    QDockWidget *telemetryDock = new QDockWidget("Telemetry", this);
    telemetryDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_telemetryWidget = new TelemetryWidget(telemetryDock);
    telemetryDock->setWidget(m_telemetryWidget);
    addDockWidget(Qt::RightDockWidgetArea, telemetryDock);
    
    // Create control panel as dock widget
    QDockWidget *controlDock = new QDockWidget("Controls", this);
    controlDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    m_controlPanel = new ControlPanel(controlDock);
    controlDock->setWidget(m_controlPanel);
    addDockWidget(Qt::LeftDockWidgetArea, controlDock);
    
    // Set window properties
    setWindowTitle("FalconSim - UAV Simulation Framework");
    setMinimumSize(800, 600);
    resize(1024, 768);
}

void MainWindow::setupConnections()
{
    // Connect button signals
    connect(ui->actionConnect, &QAction::triggered, this, &MainWindow::onConnectButtonClicked);
    connect(ui->actionDisconnect, &QAction::triggered, this, &MainWindow::onDisconnectButtonClicked);
    connect(ui->actionExit, &QAction::triggered, this, &QWidget::close);
    
    // Simulation control
    connect(ui->actionStart, &QAction::triggered, this, &MainWindow::onStartSimulation);
    connect(ui->actionPause, &QAction::triggered, this, &MainWindow::onPauseSimulation);
    connect(ui->actionStop, &QAction::triggered, this, &MainWindow::onStopSimulation);
    
    // Network connections
    connect(m_socket, &QUdpSocket::readyRead, this, &MainWindow::onDataReceived);
    
    // Connect control panel signals to the telemetry server (if local)
    connect(m_controlPanel, &ControlPanel::throttleChanged, [this](double value) {
        // Send control commands to server if needed
        qDebug() << "Throttle:" << value;
    });
}

void MainWindow::onConnectButtonClicked()
{
    if (m_connected) {
        QMessageBox::information(this, "Already Connected", 
            "Already connected to telemetry server.");
        return;
    }
    
    // Bind to any port for receiving
    if (!m_socket->bind(QHostAddress::Any, 0)) {
        QMessageBox::critical(this, "Connection Error", 
            "Could not bind socket: " + m_socket->errorString());
        return;
    }
    
    m_connected = true;
    ui->actionConnect->setEnabled(false);
    ui->actionDisconnect->setEnabled(true);
    
    statusBar()->showMessage(QString("Connected to %1:%2").arg(m_serverHost).arg(m_serverPort));
}

void MainWindow::onDisconnectButtonClicked()
{
    if (!m_connected) {
        return;
    }
    
    m_socket->close();
    m_connected = false;
    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    
    statusBar()->showMessage("Disconnected from telemetry server");
}

void MainWindow::onDataReceived()
{
    while (m_socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
        
        m_socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        
        // Process the datagram
        QString data = QString::fromUtf8(datagram);
        parseTelemetryData(data);
    }
}

void MainWindow::parseTelemetryData(const QString &data)
{
    // Parse CSV format from TelemetryServer
    QStringList fields = data.split(',');
    if (fields.size() >= 14) {  // Ensure we have enough fields
        m_telemetryData.timestamp = fields[0].toDouble();
        
        // Position (NED)
        m_telemetryData.position[0] = fields[1].toDouble();
        m_telemetryData.position[1] = fields[2].toDouble();
        m_telemetryData.position[2] = fields[3].toDouble();
        
        // Velocity (body)
        m_telemetryData.velocity[0] = fields[4].toDouble();
        m_telemetryData.velocity[1] = fields[5].toDouble();
        m_telemetryData.velocity[2] = fields[6].toDouble();
        
        // Orientation (euler)
        m_telemetryData.orientation[0] = fields[7].toDouble();
        m_telemetryData.orientation[1] = fields[8].toDouble();
        m_telemetryData.orientation[2] = fields[9].toDouble();
        
        // Controls
        m_telemetryData.controls[0] = fields[10].toDouble();
        m_telemetryData.controls[1] = fields[11].toDouble();
        m_telemetryData.controls[2] = fields[12].toDouble();
        m_telemetryData.controls[3] = fields[13].toDouble();
        
        // Update displays immediately if we receive data
        updateDisplays();
    }
}

void MainWindow::updateDisplays()
{
    if (m_telemetryWidget) {
        m_telemetryWidget->updateTelemetry(
            m_telemetryData.position,
            m_telemetryData.velocity,
            m_telemetryData.orientation
        );
    }
    
    if (m_flight3DView) {
        m_flight3DView->updateAircraftState(
            m_telemetryData.position,
            m_telemetryData.orientation
        );
    }
    
    if (m_controlPanel) {
        m_controlPanel->updateControlDisplay(
            m_telemetryData.controls[0],
            m_telemetryData.controls[1],
            m_telemetryData.controls[2],
            m_telemetryData.controls[3]
        );
    }
}

void MainWindow::onUpdateTimer()
{
    // This is called at regular intervals for smooth UI updates
    // even when telemetry data is not coming in fast enough
    updateDisplays();
}

void MainWindow::onStartSimulation()
{
    m_simRunning = true;
    statusBar()->showMessage("Simulation running");
    
    // Could send start command to server if needed
}

void MainWindow::onPauseSimulation()
{
    m_simRunning = false;
    statusBar()->showMessage("Simulation paused");
    
    // Could send pause command to server if needed
}

void MainWindow::onStopSimulation()
{
    m_simRunning = false;
    
    // Reset displays
    TelemetryData resetData;
    m_telemetryData = resetData;
    updateDisplays();
    
    statusBar()->showMessage("Simulation stopped");
    
    // Could send stop command to server if needed
}

void MainWindow::onUpdateSimulationRate(int value)
{
    m_updateRateHz = value;
    m_updateTimer.setInterval(1000 / m_updateRateHz);
    
    statusBar()->showMessage(QString("Update rate: %1 Hz").arg(m_updateRateHz));
} 