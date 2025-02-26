#include <QApplication>
#include <QCommandLineParser>
#include <QDebug>
#include "MainWindow.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("FalconSim");
    app.setApplicationDisplayName("FalconSim - UAV Simulation Framework");
    app.setApplicationVersion("0.1.0");

    // Parse command line arguments
    QCommandLineParser parser;
    parser.setApplicationDescription("High-performance UAV simulation and visualization");
    parser.addHelpOption();
    parser.addVersionOption();
    
    QCommandLineOption serverOption(QStringList() << "s" << "server",
        "Connect to telemetry server at <address>:<port>",
        "address:port", "127.0.0.1:12345");
    parser.addOption(serverOption);
    
    parser.process(app);
    
    // Create and show the main window
    MainWindow mainWindow;
    
    // Connect to telemetry server if specified
    if (parser.isSet(serverOption)) {
        QString serverAddress = parser.value(serverOption);
        QStringList parts = serverAddress.split(":");
        if (parts.size() == 2) {
            QString host = parts[0];
            quint16 port = parts[1].toUShort();
            mainWindow.connectToServer(host, port);
        } else {
            qCritical() << "Invalid server address format. Use address:port";
            return 1;
        }
    }
    
    mainWindow.show();
    return app.exec();
} 