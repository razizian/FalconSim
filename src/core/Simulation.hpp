#pragma once

#include <Eigen/Dense>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "../physics/FlightDynamics.hpp"

namespace falconsim {

/**
 * @brief UAV state representing position, velocity, and orientation
 */
struct UAVState {
    Eigen::Vector3d position{0, 0, 0};    // Position in NED frame (m)
    Eigen::Vector3d velocity{0, 0, 0};    // Velocity in body frame (m/s)
    Eigen::Vector3d orientation{0, 0, 0}; // Euler angles (rad): roll, pitch, yaw
    Eigen::Vector3d angularVel{0, 0, 0}; // Angular velocity (rad/s)
};

/**
 * @brief Core simulation class managing UAV dynamics and real-time updates
 */
class Simulation {
public:
    // Constructor with default timestep
    explicit Simulation(double timestep = 0.01);
    
    // Destructor
    ~Simulation();
    
    // Copy operations deleted due to thread and unique_ptr
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    
    // Move operations deleted due to thread safety concerns
    Simulation(Simulation&&) = delete;
    Simulation& operator=(Simulation&&) = delete;

    // Simulation control
    void start();
    void stop();
    void pause();
    void resume();

    // State access and modification
    [[nodiscard]] AircraftState getState() const;
    void setState(const AircraftState& state);

    // Control inputs
    void setThrust(double throttle);
    void setControlSurfaces(const Eigen::Vector3d& controls); // aileron, elevator, rudder

    // Get physics model
    [[nodiscard]] FlightDynamics& getPhysics();
    [[nodiscard]] const FlightDynamics& getPhysics() const;

private:
    void simulationLoop();
    
    std::unique_ptr<FlightDynamics> m_physics{};
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_paused{false};
    double m_timestep{0.01};
    std::thread m_simThread{};

    // Control inputs cache
    ControlInputs m_controls{};
};

} // namespace falconsim 