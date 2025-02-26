#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "../src/core/Simulation.hpp"
#include "../src/physics/FlightDynamics.hpp"
#include "../src/network/TelemetryServer.hpp"

using namespace falconsim;
using namespace std::chrono_literals;

// Signal handling for clean shutdown
std::atomic<bool> running{true};

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    running = false;
}

int main() {
    // Register signal handler
    std::signal(SIGINT, signalHandler);
    
    std::cout << "FalconSim - Telemetry Server Example" << std::endl;
    std::cout << "====================================" << std::endl;
    
    // Create simulation with 10ms timestep (100Hz)
    Simulation sim{0.01};
    
    // Set initial state: aircraft at 100m altitude
    AircraftState initialState;
    initialState.position = Eigen::Vector3d{0, 0, -100}; // -Z is up in NED frame
    sim.setState(initialState);
    
    // Create and configure telemetry server
    TelemetryConfig config;
    config.port = 12345;          // UDP port to listen on
    config.update_rate = 20;      // 20 Hz telemetry updates
    
    TelemetryServer telemetry{sim.getPhysics(), config};
    
    // Start the simulation and telemetry server
    sim.start();
    telemetry.start();
    
    std::cout << "Simulation started." << std::endl;
    std::cout << "Telemetry server listening on UDP port " << config.port << std::endl;
    std::cout << "Connect with a telemetry client or send 'REGISTER' via UDP to receive updates." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    
    // Simple flight pattern
    // First, let's accelerate with 80% throttle
    sim.setThrust(0.8);
    
    // Main loop - keep the program running and apply different control inputs over time
    auto startTime = std::chrono::steady_clock::now();
    
    while (running) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        
        // Every 10 seconds, change the flight pattern
        switch (elapsedSec / 10 % 4) {
            case 0: // Straight flight
                sim.setControlSurfaces(Eigen::Vector3d{0.0, 0.0, 0.0});
                break;
                
            case 1: // Roll right
                sim.setControlSurfaces(Eigen::Vector3d{0.2, 0.0, 0.0});
                break;
                
            case 2: // Level flight with climb
                sim.setControlSurfaces(Eigen::Vector3d{0.0, 0.2, 0.0});
                break;
                
            case 3: // Roll left
                sim.setControlSurfaces(Eigen::Vector3d{-0.2, 0.0, 0.0});
                break;
        }
        
        // Print connection status occasionally
        if (elapsedSec % 5 == 0) {
            std::cout << "Connected clients: " << telemetry.getClientCount() << std::endl;
        }
        
        std::this_thread::sleep_for(200ms);
    }
    
    // Cleanup
    telemetry.stop();
    sim.stop();
    
    std::cout << "Telemetry server stopped." << std::endl;
    std::cout << "Simulation stopped." << std::endl;
    
    return 0;
} 