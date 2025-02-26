#include <gtest/gtest.h>
#include "core/Simulation.hpp"
#include "physics/FlightDynamics.hpp"
#include <thread>
#include <chrono>

using namespace falconsim;

TEST(SimulationTest, Initialization) {
    Simulation sim;
    auto state = sim.getState();
    
    // Check initial state is zero
    EXPECT_EQ(state.position, Eigen::Vector3d::Zero());
    EXPECT_EQ(state.velocity, Eigen::Vector3d::Zero());
    EXPECT_EQ(state.euler_angles, Eigen::Vector3d::Zero());
    EXPECT_EQ(state.angular_velocity, Eigen::Vector3d::Zero());
}

TEST(SimulationTest, ThrustControl) {
    Simulation sim;
    
    // Test thrust limits
    sim.setThrust(-1.0); // Should clamp to 0
    sim.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto state = sim.getState();
    EXPECT_GE(state.velocity.x(), 0.0); // Should not have negative thrust
    
    sim.stop();
}

TEST(SimulationTest, ControlSurfaces) {
    Simulation sim;
    
    // Test control surface limits
    Eigen::Vector3d controls(2.0, -2.0, 1.5); // Should clamp to [-1,1]
    sim.setControlSurfaces(controls);
    sim.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto state = sim.getState();
    EXPECT_LE(std::abs(state.angular_velocity.x()), 2.0); // Check angular velocity is bounded
    EXPECT_LE(std::abs(state.angular_velocity.y()), 2.0);
    EXPECT_LE(std::abs(state.angular_velocity.z()), 2.0);
    
    sim.stop();
}

TEST(SimulationTest, GravityEffect) {
    Simulation sim;
    sim.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto state = sim.getState();
    // Given our coordinate system, positive Z velocity means downward (toward ground) in NED frame
    EXPECT_GT(state.velocity.z(), 0.0); // Should fall due to gravity
    
    sim.stop();
}

TEST(SimulationTest, PauseResume) {
    Simulation sim;
    sim.start();
    
    // Let it run for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto state1 = sim.getState();
    
    // Pause and check state doesn't change
    sim.pause();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto state2 = sim.getState();
    
    EXPECT_EQ(state1.position, state2.position);
    
    sim.stop();
}

TEST(FlightDynamicsTest, LiftGeneration) {
    FlightDynamics physics;
    
    // Set up state with forward velocity
    AircraftState state;
    state.velocity = Eigen::Vector3d(10.0, 0.0, 0.0); // 10 m/s forward
    physics.setState(state);
    
    // Run a single update
    physics.update(0.1);
    
    // In NED coordinates, negative Z is up
    state = physics.getState();
    EXPECT_LT(state.position.z(), 0.0); // Negative Z indicates upward movement
}

TEST(FlightDynamicsTest, ControlInputEffects) {
    FlightDynamics physics;
    
    // Set up control inputs
    ControlInputs controls;
    controls.throttle = 1.0;    // Full throttle
    controls.aileron = 1.0;     // Full right roll
    physics.setControls(controls);
    
    // Run simulation
    physics.update(0.1);
    
    // Check effects
    auto state = physics.getState();
    EXPECT_GT(state.velocity.x(), 0.0);         // Forward acceleration from throttle
    EXPECT_GT(state.angular_velocity.x(), 0.0); // Roll right from aileron
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 