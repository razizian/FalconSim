#include "Simulation.hpp"
#include <stdexcept>

namespace falconsim {

Simulation::Simulation(double timestep)
    : m_physics{std::make_unique<FlightDynamics>()}
    , m_running{false}
    , m_paused{false}
    , m_timestep{timestep}
    , m_controls{} {
}

Simulation::~Simulation() {
    stop();
}

void Simulation::start() {
    if (m_running) {
        throw std::runtime_error{"Simulation already running"};
    }
    m_running = true;
    m_paused = false;
    m_simThread = std::thread{&Simulation::simulationLoop, this};
}

void Simulation::stop() {
    m_running = false;
    if (m_simThread.joinable()) {
        m_simThread.join();
    }
}

void Simulation::pause() {
    m_paused = true;
}

void Simulation::resume() {
    m_paused = false;
}

AircraftState Simulation::getState() const {
    return m_physics->getState();
}

void Simulation::setState(const AircraftState& state) {
    m_physics->setState(state);
}

void Simulation::setThrust(double throttle) {
    m_controls.throttle = std::max(0.0, std::min(throttle, 1.0)); // Normalize to [0,1]
    m_physics->setControls(m_controls);
}

void Simulation::setControlSurfaces(const Eigen::Vector3d& controls) {
    // Extract controls for aileron, elevator, rudder
    m_controls.aileron = std::max(-1.0, std::min(controls.x(), 1.0));
    m_controls.elevator = std::max(-1.0, std::min(controls.y(), 1.0));
    m_controls.rudder = std::max(-1.0, std::min(controls.z(), 1.0));
    
    m_physics->setControls(m_controls);
}

FlightDynamics& Simulation::getPhysics() {
    return *m_physics;
}

const FlightDynamics& Simulation::getPhysics() const {
    return *m_physics;
}

void Simulation::simulationLoop() {
    using clock = std::chrono::high_resolution_clock;
    auto lastTime{clock::now()};

    while (m_running) {
        if (!m_paused) {
            auto currentTime{clock::now()};
            double dt{std::chrono::duration<double>{currentTime - lastTime}.count()};
            lastTime = currentTime;

            // Update physics
            m_physics->update(dt);
        }
        
        // Sleep to maintain desired timestep
        std::this_thread::sleep_for(std::chrono::duration<double>{m_timestep});
    }
}

} // namespace falconsim 