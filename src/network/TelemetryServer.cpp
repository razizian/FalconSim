#include "TelemetryServer.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace falconsim {

TelemetryServer::TelemetryServer(uint16_t port)
    : m_port{port} {
}

TelemetryServer::~TelemetryServer() {
    stop();
}

void TelemetryServer::start() {
    if (m_running) {
        return;
    }
    
    m_running = true;
    
    try {
        // Initialize networking components
        m_socket = std::make_unique<boost::asio::ip::udp::socket>(
            m_ioContext, 
            boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), m_port)
        );
        
        // Start server thread
        m_serverThread = std::thread{&TelemetryServer::serverLoop, this};
        
        std::cout << "Telemetry server started on port " << m_port << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to start telemetry server: " << e.what() << std::endl;
        m_running = false;
    }
}

void TelemetryServer::stop() {
    if (!m_running) {
        return;
    }
    
    m_running = false;
    
    // Close socket
    if (m_socket && m_socket->is_open()) {
        boost::system::error_code ec;
        m_socket->close(ec);
    }
    
    // Stop IO context
    m_ioContext.stop();
    
    // Join server thread
    if (m_serverThread.joinable()) {
        m_serverThread.join();
    }
    
    std::cout << "Telemetry server stopped" << std::endl;
}

void TelemetryServer::serverLoop() {
    auto lastUpdateTime = std::chrono::high_resolution_clock::now();
    double updateIntervalSec = 1.0 / m_updateRate;
    
    while (m_running) {
        // Process any pending telemetry data
        processTelemetryQueue();
        
        // Sleep to maintain desired update rate
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastUpdateTime;
        
        if (elapsed.count() < updateIntervalSec) {
            std::this_thread::sleep_for(std::chrono::duration<double>(
                updateIntervalSec - elapsed.count()
            ));
        }
        
        lastUpdateTime = std::chrono::high_resolution_clock::now();
    }
}

void TelemetryServer::processTelemetryQueue() {
    std::lock_guard<std::mutex> lock{m_queueMutex};
    
    if (m_telemetryQueue.empty()) {
        return;
    }
    
    // Get next telemetry data to send
    TelemetryData data = m_telemetryQueue.front();
    m_telemetryQueue.pop();
    
    // Serialize data
    std::string message = serializeTelemetry(data);
    
    // Send to all clients
    {
        std::lock_guard<std::mutex> clientsLock{m_clientsMutex};
        
        for (const auto& endpoint : m_clients) {
            try {
                m_socket->send_to(
                    boost::asio::buffer(message), 
                    endpoint
                );
            } catch (const std::exception& e) {
                std::cerr << "Error sending telemetry: " << e.what() << std::endl;
            }
        }
    }
}

void TelemetryServer::sendTelemetry(const TelemetryData& data) {
    std::lock_guard<std::mutex> lock{m_queueMutex};
    
    // Add to queue (limit queue size to prevent memory issues)
    if (m_telemetryQueue.size() < 100) {
        m_telemetryQueue.push(data);
    }
}

void TelemetryServer::updateFromState(const AircraftState& state, const ControlInputs& controls) {
    TelemetryData data;
    
    // Set timestamp
    data.timestamp = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();
    
    // Position
    data.position_north = state.position.x();
    data.position_east = state.position.y();
    data.position_down = state.position.z();
    
    // Velocity
    data.velocity_x = state.velocity.x();
    data.velocity_y = state.velocity.y();
    data.velocity_z = state.velocity.z();
    
    // Orientation
    data.roll = state.euler_angles.x();
    data.pitch = state.euler_angles.y();
    data.yaw = state.euler_angles.z();
    
    // Control inputs
    data.throttle = controls.throttle;
    data.aileron = controls.aileron;
    data.elevator = controls.elevator;
    data.rudder = controls.rudder;
    
    // Send telemetry
    sendTelemetry(data);
}

void TelemetryServer::addClient(const std::string& address, uint16_t port) {
    std::lock_guard<std::mutex> lock{m_clientsMutex};
    
    boost::asio::ip::udp::endpoint endpoint{
        boost::asio::ip::make_address(address),
        port
    };
    
    // Check if client already exists
    auto it = std::find(m_clients.begin(), m_clients.end(), endpoint);
    if (it == m_clients.end()) {
        m_clients.push_back(endpoint);
        std::cout << "Added telemetry client: " << address << ":" << port << std::endl;
    }
}

void TelemetryServer::removeClient(const std::string& address, uint16_t port) {
    std::lock_guard<std::mutex> lock{m_clientsMutex};
    
    boost::asio::ip::udp::endpoint endpoint{
        boost::asio::ip::make_address(address),
        port
    };
    
    auto it = std::find(m_clients.begin(), m_clients.end(), endpoint);
    if (it != m_clients.end()) {
        m_clients.erase(it);
        std::cout << "Removed telemetry client: " << address << ":" << port << std::endl;
    }
}

void TelemetryServer::setUpdateRate(double rate) {
    m_updateRate = std::max(1.0, std::min(rate, 100.0)); // Clamp between 1 and 100 Hz
}

std::string TelemetryServer::serializeTelemetry(const TelemetryData& data) {
    // Simple CSV format for now
    // In a real application, consider using Protocol Buffers or a more efficient format
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    
    ss << data.timestamp << ","
       << data.position_north << "," << data.position_east << "," << data.position_down << ","
       << data.velocity_x << "," << data.velocity_y << "," << data.velocity_z << ","
       << data.roll << "," << data.pitch << "," << data.yaw << ","
       << data.throttle << "," << data.aileron << "," << data.elevator << "," << data.rudder;
    
    return ss.str();
}

} // namespace falconsim 