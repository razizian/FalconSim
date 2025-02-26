#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <functional>
#include <queue>
#include <mutex>
#include "../physics/FlightDynamics.hpp"

namespace falconsim {

/**
 * @brief Telemetry data structure for network transmission
 */
struct TelemetryData {
    // Timestamp
    double timestamp{0.0};
    
    // Position (NED frame)
    double position_north{0.0};
    double position_east{0.0};
    double position_down{0.0};
    
    // Velocity (body frame)
    double velocity_x{0.0};
    double velocity_y{0.0};
    double velocity_z{0.0};
    
    // Orientation (euler angles in radians)
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    
    // Control inputs
    double throttle{0.0};
    double aileron{0.0};
    double elevator{0.0};
    double rudder{0.0};
};

/**
 * @brief Telemetry server for UAV data streaming over UDP
 */
class TelemetryServer {
public:
    explicit TelemetryServer(uint16_t port = 12345);
    ~TelemetryServer();
    
    // Deleted copy and move operations
    TelemetryServer(const TelemetryServer&) = delete;
    TelemetryServer& operator=(const TelemetryServer&) = delete;
    TelemetryServer(TelemetryServer&&) = delete;
    TelemetryServer& operator=(TelemetryServer&&) = delete;
    
    // Server control
    void start();
    void stop();
    
    // Telemetry data handling
    void sendTelemetry(const TelemetryData& data);
    void updateFromState(const AircraftState& state, const ControlInputs& controls);
    
    // Client handling
    void addClient(const std::string& address, uint16_t port);
    void removeClient(const std::string& address, uint16_t port);
    
    // Configuration
    void setUpdateRate(double rate); // Hz
    
private:
    void serverLoop();
    void processTelemetryQueue();
    std::string serializeTelemetry(const TelemetryData& data);
    
    // Network components
    boost::asio::io_context m_ioContext{};
    std::unique_ptr<boost::asio::ip::udp::socket> m_socket{};
    
    // Client list
    std::vector<boost::asio::ip::udp::endpoint> m_clients{};
    std::mutex m_clientsMutex{};
    
    // Telemetry queue
    std::queue<TelemetryData> m_telemetryQueue{};
    std::mutex m_queueMutex{};
    
    // Threading
    std::atomic<bool> m_running{false};
    std::thread m_serverThread{};
    
    // Configuration
    uint16_t m_port{12345};
    double m_updateRate{10.0}; // 10 Hz default
};

} // namespace falconsim 