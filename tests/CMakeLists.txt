enable_testing()

add_executable(simulation_tests
    simulation_tests.cpp
)

target_link_libraries(simulation_tests
    PRIVATE
    falconsim
    GTest::GTest
    GTest::Main
)

include(GoogleTest)
gtest_discover_tests(simulation_tests) 