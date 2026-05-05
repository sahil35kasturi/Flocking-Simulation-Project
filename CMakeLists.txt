cmake_minimum_required(VERSION 3.20)
project(FinalProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(GLFW_DIR "$ENV{GLFW_DIR}")
if(NOT GLFW_DIR)
    set(GLFW_DIR "/Users/sahilkasturi/Desktop/csce450/libs/glfw")
endif()

set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIR "/opt/homebrew/opt/eigen/include/eigen3")
endif()

set(GLFW_INCLUDE_DIR "${GLFW_DIR}/include")
set(GLFW_LIBRARY "${GLFW_DIR}/release/src/libglfw3.a")

if(NOT EXISTS "${GLFW_INCLUDE_DIR}/GLFW/glfw3.h")
    message(FATAL_ERROR "GLFW headers not found. Set GLFW_DIR to your local GLFW checkout.")
endif()

if(NOT EXISTS "${GLFW_LIBRARY}")
    message(FATAL_ERROR "GLFW static library not found at ${GLFW_LIBRARY}. Build GLFW first or update GLFW_DIR.")
endif()

find_package(OpenGL REQUIRED)

add_executable(FinalProject
    src/main.cpp
    src/Simulation.cpp
)

target_include_directories(FinalProject PRIVATE
    include
    "${GLFW_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
)

target_link_libraries(FinalProject PRIVATE
    "${GLFW_LIBRARY}"
    OpenGL::GL
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
    "-framework QuartzCore"
)
