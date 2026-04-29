cmake_minimum_required(VERSION 3.16)
project(ActiveGuidanceBBB VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Cross-compiling for BeagleBone Black: ${BBB_TARGET}")

# Compiler flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0 -Wall -Wextra)
else()
    add_compile_options(-O3 -DNDEBUG)
endif()

# Find required packages
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# Find ZeroMQ
pkg_check_modules(ZEROMQ REQUIRED libzmq)

# Eigen3 - for math operations
# find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${ZEROMQ_INCLUDE_DIRS}
)

# Source files for the active guidance module
set(ACTIVE_GUIDANCE_SOURCES
    src/activeGuidanceStandalone.cpp
    src/activeGuidanceMath.cpp
    src/activeGuidanceBBB.cpp
    src/zeromq_interface.cpp
    src/uart_interface.cpp
    src/spi_interface.cpp
)

# Create the active guidance library
add_library(active_guidance_bbb STATIC ${ACTIVE_GUIDANCE_SOURCES})

# Link libraries
target_link_libraries(active_guidance_bbb
    Threads::Threads
    ${ZEROMQ_LIBRARIES}
)

# Create the main BBB gateway executable
set(GATEWAY_SOURCES
    src/main.cpp
    src/bbb_gateway.cpp
    src/activeGuidanceBBB.cpp
    src/zeromq_interface.cpp
    src/uart_interface.cpp
    src/spi_interface.cpp
)

add_executable(bbb_gateway ${GATEWAY_SOURCES})

target_link_libraries(bbb_gateway
    active_guidance_bbb
    Threads::Threads
    ${ZEROMQ_LIBRARIES}
)

# Installation rules
install(TARGETS active_guidance_bbb bbb_gateway
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Print configuration summary
message(STATUS "===========================================")
message(STATUS "Active Guidance BBB Configuration Summary")
message(STATUS "===========================================")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "Install prefix:    ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Cross-compile:     ${BBB_TARGET}")
message(STATUS "===========================================")