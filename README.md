# FalconSim

A high-performance UAV simulation framework for defense, aerospace, and research applications.

## Features

- Real-time UAV flight dynamics simulation (3-DOF & 6-DOF)
- Multi-threaded processing for high-speed performance
- Networking & telemetry integration using Boost.Asio (UDP/TCP)
- Scalable modular architecture
- Sensor emulation support (IMU, GPS, LiDAR, Radar) - Future expansion
- Optional Qt-based GUI visualization

## Dependencies

- C++17 or later compiler
- CMake 3.15 or later
- Boost (system, thread components)
- Eigen 3.3 or later
- Protocol Buffers
- Google Test
- Qt6 (optional, for GUI)

## Building

```bash
# Clone the repository
git clone https://github.com/razizian/FalconSim.git
cd FalconSim

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build
cmake --build .

# Run tests
ctest --output-on-failure
```

For building with GUI support:
```bash
cmake -DFALCONSIM_BUILD_GUI=ON ..
```

## Project Structure

```
FalconSim/
├── src/
│   ├── core/       # Core simulation logic
│   ├── physics/    # Flight physics calculations
│   ├── network/    # Telemetry & networking
│   └── gui/        # Qt-based GUI (optional)
├── tests/          # Unit tests
├── CMakeLists.txt
└── README.md
```

## Usage

Basic example:

```cpp
#include <falconsim/core/Simulation.hpp>

int main() {
    falconsim::Simulation sim;
    
    // Set initial conditions
    falconsim::UAVState state;
    state.position = Eigen::Vector3d(0, 0, 100); // 100m altitude
    sim.setState(state);
    
    // Start simulation
    sim.start();
    
    // Control the UAV
    sim.setThrust(0.5);                          // 50% thrust
    sim.setControlSurfaces({0.1, 0.0, 0.0});    // Small aileron deflection
    
    // ... run simulation ...
    
    sim.stop();
    return 0;
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request 