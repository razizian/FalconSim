#pragma once

/**
 * @brief Struct containing UAV telemetry data
 * 
 * This struct holds all telemetry information from the UAV,
 * including position, velocity, orientation, and control inputs.
 */
struct TelemetryData {
    double timestamp{0.0};
    double position[3]{0.0, 0.0, 0.0};      // North, East, Down
    double velocity[3]{0.0, 0.0, 0.0};      // X, Y, Z (body frame)
    double orientation[3]{0.0, 0.0, 0.0};   // Roll, Pitch, Yaw
    double controls[4]{0.0, 0.0, 0.0, 0.0}; // Throttle, Aileron, Elevator, Rudder
}; 