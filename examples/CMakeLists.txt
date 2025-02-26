# Examples demonstrating how to use FalconSim

add_executable(basic_simulation basic_simulation.cpp)
target_link_libraries(basic_simulation PRIVATE falconsim)

add_executable(telemetry_server telemetry_server.cpp)
target_link_libraries(telemetry_server PRIVATE falconsim)

# Install examples
install(TARGETS basic_simulation telemetry_server
    RUNTIME DESTINATION bin/examples
) 