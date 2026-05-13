# Minimum CMake version
cmake_minimum_required(VERSION 3.10)
# Project name and language
project(PBD CXX)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Add the source file
add_executable(pbd position_based_dynamic.cpp)

# Find Eigen library
find_package(Eigen3 REQUIRED)
#find_package(OpenMP)

find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

if(APPLE)
    # macOS: use Apple's native GLUT framework (no XQuartz required).
    # Homebrew freeglut requires an X11 display server and will fail with
    # "failed to open display ''" on a standard macOS setup.
    find_library(GLUT_LIBRARY GLUT REQUIRED)
    target_compile_definitions(pbd PRIVATE input_dir="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(pbd PRIVATE ${GLUT_LIBRARY} GLEW::GLEW OpenGL::GLU Eigen3::Eigen)
else()
    find_package(GLUT REQUIRED)
    target_compile_definitions(pbd PRIVATE input_dir="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(pbd PRIVATE GLUT::GLUT GLEW::GLEW OpenGL::GLU Eigen3::Eigen)
endif()