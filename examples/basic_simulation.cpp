#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cmath>

#include "../src/core/Simulation.hpp"
#include "../src/physics/FlightDynamics.hpp"

using namespace falconsim;
using namespace std::chrono_literals;

void printState(const AircraftState& state) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Position: (" 
              << state.position.x() << ", " 
              << state.position.y() << ", " 
              << state.position.z() << ") m ";
    
    std::cout << "Velocity: (" 
              << state.velocity.x() << ", " 
              << state.velocity.y() << ", " 
              << state.velocity.z() << ") m/s ";
    
    std::cout << "Euler: (" 
              << state.euler_angles.x() * 180.0 / M_PI << ", " 
              << state.euler_angles.y() * 180.0 / M_PI << ", " 
              << state.euler_angles.z() * 180.0 / M_PI << ") deg";
              
    std::cout << std::endl;
}

int main() {
    std::cout << "FalconSim - Basic Simulation Example" << std::endl;
    std::cout << "====================================" << std::endl;
    
    // Create simulation with 10ms timestep (100Hz)
    Simulation sim{0.01};
    
    // Set initial state: aircraft at 100m altitude
    AircraftState initialState;
    initialState.position = Eigen::Vector3d{0, 0, -100}; // -Z is up in NED frame
    sim.setState(initialState);
    
    // Start the simulation
    sim.start();
    std::cout << "Simulation started..." << std::endl;
    
    // Initial state
    std::cout << "Initial state:" << std::endl;
    printState(sim.getState());
    
    // Let's apply throttle and track the state for 5 seconds
    std::cout << "\nApplying 80% throttle..." << std::endl;
    sim.setThrust(0.8);
    
    for (int i = 0; i < 50; ++i) {
        std::this_thread::sleep_for(100ms);
        printState(sim.getState());
    }
    
    // Now let's do a turn by applying aileron
    std::cout << "\nNow applying right aileron (roll right)..." << std::endl;
    sim.setControlSurfaces(Eigen::Vector3d{0.3, 0.0, 0.0});
    
    for (int i = 0; i < 30; ++i) {
        std::this_thread::sleep_for(100ms);
        printState(sim.getState());
    }
    
    // Now level out and apply some elevator for climb
    std::cout << "\nLeveling out and climbing..." << std::endl;
    sim.setControlSurfaces(Eigen::Vector3d{0.0, 0.3, 0.0});
    
    for (int i = 0; i < 30; ++i) {
        std::this_thread::sleep_for(100ms);
        printState(sim.getState());
    }
    
    // Bring back to level flight
    std::cout << "\nBringing back to level flight..." << std::endl;
    sim.setControlSurfaces(Eigen::Vector3d{0.0, 0.0, 0.0});
    
    for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_for(100ms);
        printState(sim.getState());
    }
    
    // Stop the simulation
    sim.stop();
    std::cout << "Simulation stopped." << std::endl;
    
    return 0;
} 