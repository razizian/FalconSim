#pragma once

#include <Eigen/Dense>
#include <memory>

namespace falconsim {

/**
 * @brief UAV physical properties
 */
struct UAVPhysicalProperties {
    double mass{1.0};                      // Mass in kg
    Eigen::Vector3d inertia{1.0, 1.0, 1.0}; // Moment of inertia (Ixx, Iyy, Izz) in kg*m^2
    Eigen::Vector3d dimensions{1.0, 1.0, 0.2}; // Length, wingspan, height in meters
    double thrust_max{20.0};              // Maximum thrust in Newtons
};

/**
 * @brief Aircraft control inputs structure
 */
struct ControlInputs {
    double throttle{0.0};      // Normalized [0,1] throttle setting
    double aileron{0.0};       // Normalized [-1,1] aileron deflection (positive = right roll)
    double elevator{0.0};      // Normalized [-1,1] elevator deflection (positive = pitch up)
    double rudder{0.0};        // Normalized [-1,1] rudder deflection (positive = yaw right)
};

/**
 * @brief Aircraft state representing position, velocity, orientation, and other flight parameters
 */
struct AircraftState {
    Eigen::Vector3d position{0, 0, 0};          // Position in NED frame (m)
    Eigen::Vector3d velocity{0, 0, 0};          // Velocity in body frame (m/s)
    Eigen::Vector3d euler_angles{0, 0, 0};      // Euler angles (rad): roll, pitch, yaw
    Eigen::Vector3d angular_velocity{0, 0, 0};  // Angular velocity (rad/s)
    
    // Additional parameters
    double mass{1.0};                           // Aircraft mass (kg)
    double altitude() const { return -position.z(); } // Altitude (m) - NED frame, so negative z
};

/**
 * @brief 6-DOF flight dynamics model for UAV simulation
 */
class FlightDynamics {
public:
    FlightDynamics();
    ~FlightDynamics() = default;
    
    // Disable copy and move for now
    FlightDynamics(const FlightDynamics&) = delete;
    FlightDynamics& operator=(const FlightDynamics&) = delete;
    FlightDynamics(FlightDynamics&&) = delete;
    FlightDynamics& operator=(FlightDynamics&&) = delete;
    
    // State access and manipulation
    [[nodiscard]] AircraftState getState() const;
    void setState(const AircraftState& state);
    
    // Control input
    void setControls(const ControlInputs& controls);
    [[nodiscard]] ControlInputs getControls() const;
    
    // Physical properties
    void setProperties(const UAVPhysicalProperties& properties);
    [[nodiscard]] const UAVPhysicalProperties& getProperties() const;
    
    // Environment
    void setAirDensity(double density);
    void setWind(const Eigen::Vector3d& wind);
    
    // Physics update
    void update(double dt);
    
    // Aircraft parameters
    void setMass(double mass);
    void setWingspanArea(double area);
    void setLiftCoefficient(double cl);
    void setDragCoefficient(double cd);
    
private:
    // Forces and moments calculation
    void updateForces(double dt);
    void updateMoments(double dt);
    
    // Individual force calculations
    [[nodiscard]] Eigen::Vector3d calculateLift() const;
    [[nodiscard]] Eigen::Vector3d calculateDrag() const;
    [[nodiscard]] Eigen::Vector3d calculateThrust() const;
    [[nodiscard]] Eigen::Vector3d calculateGravity() const;
    
    // Individual moment calculations
    [[nodiscard]] Eigen::Vector3d calculateAileronMoment() const;
    [[nodiscard]] Eigen::Vector3d calculateElevatorMoment() const;
    [[nodiscard]] Eigen::Vector3d calculateRudderMoment() const;
    
    // Integrate state
    void integrateState(double dt);
    
    // Update rotation matrices
    void updateRotationMatrices();
    
    AircraftState m_state{};
    ControlInputs m_controls{};
    UAVPhysicalProperties m_properties{};
    
    // Cached rotation matrices
    Eigen::Matrix3d m_rotationBodyToNED{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d m_rotationNEDToBody{Eigen::Matrix3d::Identity()};
    
    // Aircraft parameters (defaults are for a small UAV)
    double m_wingArea{0.5};         // Wing area (m²)
    double m_wingspan{1.5};         // Wingspan (m)
    double m_liftCoefficient{1.2};  // Basic lift coefficient
    double m_dragCoefficient{0.1};  // Basic drag coefficient
    double m_thrust_max{20.0};      // Maximum thrust (N)
    
    // Environment
    double m_airDensity{1.225};     // Air density at sea level (kg/m³)
    double m_gravity{9.81};         // Gravity acceleration (m/s²)
    
    // Inertia tensor (3x3 matrix representing moments of inertia)
    Eigen::Matrix3d m_inertiaTensor{Eigen::Matrix3d::Identity()};
};

} // namespace falconsim 