#include "FlightDynamics.hpp"
#include <cmath>
#include <iostream>

namespace falconsim {

// Gravitational constant
constexpr double g = 9.81; // m/s^2

FlightDynamics::FlightDynamics() {
    // Initialize inertia tensor for a small UAV (approximate values)
    m_inertiaTensor << 0.5, 0, 0,
                       0, 0.8, 0,
                       0, 0, 1.0;
}

AircraftState FlightDynamics::getState() const {
    return m_state;
}

void FlightDynamics::setState(const AircraftState& state) {
    m_state = state;
}

void FlightDynamics::setControls(const ControlInputs& controls) {
    // Clamp control inputs to valid ranges
    m_controls.throttle = std::max(0.0, std::min(controls.throttle, 1.0));
    m_controls.aileron = std::max(-1.0, std::min(controls.aileron, 1.0));
    m_controls.elevator = std::max(-1.0, std::min(controls.elevator, 1.0));
    m_controls.rudder = std::max(-1.0, std::min(controls.rudder, 1.0));
}

ControlInputs FlightDynamics::getControls() const {
    return m_controls;
}

void FlightDynamics::setProperties(const UAVPhysicalProperties& properties) {
    m_properties = properties;
}

const UAVPhysicalProperties& FlightDynamics::getProperties() const {
    return m_properties;
}

void FlightDynamics::setAirDensity(double density) {
    m_airDensity = std::max(0.01, density); // Ensure positive air density
}

void FlightDynamics::setWind(const Eigen::Vector3d& wind) {
    // Nothing to do yet, will implement in future when we add wind effects
}

void FlightDynamics::update(double dt) {
    // Calculate all forces and moments
    updateForces(dt);
    updateMoments(dt);
    
    // Integrate state forward in time
    integrateState(dt);
}

void FlightDynamics::setMass(double mass) {
    m_state.mass = std::max(0.1, mass); // Minimum mass of 0.1kg
}

void FlightDynamics::setWingspanArea(double area) {
    m_wingArea = std::max(0.01, area); // Minimum area of 0.01m²
}

void FlightDynamics::setLiftCoefficient(double cl) {
    m_liftCoefficient = cl;
}

void FlightDynamics::setDragCoefficient(double cd) {
    m_dragCoefficient = std::max(0.0, cd);
}

void FlightDynamics::updateRotationMatrices() {
    // Get Euler angles
    double phi = m_state.euler_angles.x();    // Roll
    double theta = m_state.euler_angles.y();  // Pitch
    double psi = m_state.euler_angles.z();    // Yaw
    
    // Calculate trig functions only once
    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);
    double cpsi = cos(psi);
    double spsi = sin(psi);
    
    // Create rotation matrix from body to NED
    m_rotationBodyToNED << cpsi*ctheta, cpsi*stheta*sphi-spsi*cphi, cpsi*stheta*cphi+spsi*sphi,
                           spsi*ctheta, spsi*stheta*sphi+cpsi*cphi, spsi*stheta*cphi-cpsi*sphi,
                           -stheta, ctheta*sphi, ctheta*cphi;
    
    // Inverse rotation (transpose for orthogonal matrices)
    m_rotationNEDToBody = m_rotationBodyToNED.transpose();
}

void FlightDynamics::updateForces(double dt) {
    // Calculate all forces in body frame
    Eigen::Vector3d lift{calculateLift()};
    Eigen::Vector3d drag{calculateDrag()};
    Eigen::Vector3d thrust{calculateThrust()};
    Eigen::Vector3d gravity{calculateGravity()};
    
    // Sum all forces (in body frame)
    Eigen::Vector3d totalForce{thrust + lift + drag + gravity};
    
    // F = ma -> a = F/m
    Eigen::Vector3d acceleration{totalForce / m_state.mass};
    
    // Update velocity (integrate acceleration)
    m_state.velocity += acceleration * dt;
}

void FlightDynamics::updateMoments(double dt) {
    // Calculate moments from control surfaces
    Eigen::Vector3d aileronMoment{calculateAileronMoment()};
    Eigen::Vector3d elevatorMoment{calculateElevatorMoment()};
    Eigen::Vector3d rudderMoment{calculateRudderMoment()};
    
    // Sum all moments
    Eigen::Vector3d totalMoment{aileronMoment + elevatorMoment + rudderMoment};
    
    // Calculate angular acceleration: α = I⁻¹ * M
    Eigen::Vector3d angularAccel{m_inertiaTensor.inverse() * totalMoment};
    
    // Update angular velocity
    m_state.angular_velocity += angularAccel * dt;
}

void FlightDynamics::integrateState(double dt) {
    // Update rotation matrices based on current orientation
    updateRotationMatrices();
    
    // Update position based on velocity
    // Convert velocity from body to NED frame
    Eigen::Vector3d velocityNED{m_rotationBodyToNED * m_state.velocity};
    
    // Update position in NED frame
    m_state.position += velocityNED * dt;
    
    // Update orientation (Euler angles) based on angular velocity
    // Note: this is a simple Euler integration - for accurate simulation,
    // consider using quaternions to avoid gimbal lock
    
    // Convert body rates to Euler rates
    Eigen::Matrix3d W;
    W << 1, sin(m_state.euler_angles.x()) * tan(m_state.euler_angles.y()), 
         cos(m_state.euler_angles.x()) * tan(m_state.euler_angles.y()),
         0, cos(m_state.euler_angles.x()), -sin(m_state.euler_angles.x()),
         0, sin(m_state.euler_angles.x()) / cos(m_state.euler_angles.y()), 
         cos(m_state.euler_angles.x()) / cos(m_state.euler_angles.y());
    
    // Apply to angular velocity
    Eigen::Vector3d eulerRates{W * m_state.angular_velocity};
    
    // Integrate to get new Euler angles
    m_state.euler_angles += eulerRates * dt;
}

Eigen::Vector3d FlightDynamics::calculateLift() const {
    // Simple lift model
    double airspeed{m_state.velocity.norm()};
    
    // No lift at zero airspeed
    if (airspeed < 0.1) {
        return Eigen::Vector3d::Zero();
    }
    
    // Basic lift equation: L = 0.5 * ρ * v² * CL * S
    double liftMagnitude{0.5 * m_airDensity * airspeed * airspeed * 
                       m_liftCoefficient * m_wingArea};
    
    // Lift is perpendicular to velocity, pointing upward in body frame
    return Eigen::Vector3d{0, 0, -liftMagnitude};
}

Eigen::Vector3d FlightDynamics::calculateDrag() const {
    // Simple drag model
    double airspeed{m_state.velocity.norm()};
    
    // No drag at zero airspeed
    if (airspeed < 0.1) {
        return Eigen::Vector3d::Zero();
    }
    
    // Basic drag equation: D = 0.5 * ρ * v² * CD * S
    double dragMagnitude{0.5 * m_airDensity * airspeed * airspeed * 
                       m_dragCoefficient * m_wingArea};
    
    // Drag is opposite to velocity
    Eigen::Vector3d dragDirection{-m_state.velocity.normalized()};
    return dragDirection * dragMagnitude;
}

Eigen::Vector3d FlightDynamics::calculateThrust() const {
    // Simple thrust model: Thrust acts in the body's x-direction
    double thrustMagnitude{m_controls.throttle * m_thrust_max};
    return Eigen::Vector3d{thrustMagnitude, 0, 0};
}

Eigen::Vector3d FlightDynamics::calculateGravity() const {
    // Gravity in NED frame is (0, 0, m*g)
    Eigen::Vector3d gravityNED{0, 0, m_state.mass * m_gravity};
    
    // Need to rotate to body frame
    return m_rotationNEDToBody * gravityNED;
}

Eigen::Vector3d FlightDynamics::calculateAileronMoment() const {
    // Aileron creates roll moment (around x-axis)
    double rollMoment{m_controls.aileron * 2.0 * m_wingspan};
    return Eigen::Vector3d{rollMoment, 0, 0};
}

Eigen::Vector3d FlightDynamics::calculateElevatorMoment() const {
    // Elevator creates pitch moment (around y-axis)
    double pitchMoment{m_controls.elevator * 1.5};
    return Eigen::Vector3d{0, pitchMoment, 0};
}

Eigen::Vector3d FlightDynamics::calculateRudderMoment() const {
    // Rudder creates yaw moment (around z-axis)
    double yawMoment{m_controls.rudder * 1.0};
    return Eigen::Vector3d{0, 0, yawMoment};
}

} // namespace falconsim 