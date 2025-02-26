cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0167 NEW)
project(FalconSim VERSION 0.1.0 LANGUAGES CXX)

# Enable testing with CTest
include(CTest)
enable_testing()

# Set C++17 as minimum
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Protobuf REQUIRED)
find_package(GTest REQUIRED)

# Optional Qt GUI support
option(FALCONSIM_BUILD_GUI "Build FalconSim GUI" OFF)
if(FALCONSIM_BUILD_GUI)
    find_package(Qt6 COMPONENTS Core Widgets Quick REQUIRED)
endif()

# Main library target
add_library(falconsim "")

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)
if(FALCONSIM_BUILD_GUI)
    add_subdirectory(gui)
endif()

# Include directories
target_include_directories(falconsim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(falconsim PUBLIC
    Boost::system
    Boost::thread
    Eigen3::Eigen
    protobuf::libprotobuf
)

# Set compiler flags
if(MSVC)
    target_compile_options(falconsim PRIVATE /W4)
else()
    target_compile_options(falconsim PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Install rules
install(TARGETS falconsim
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
) 