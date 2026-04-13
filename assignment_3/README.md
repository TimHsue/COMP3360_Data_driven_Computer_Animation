# Position-Based-Dynamics Assignment 3
## Objective
Complete the implementation of the `update_cloth_edges` function in `position_based_dynamics.cpp` and document experimental results.

## Implementation Requirements

### 1. Code Completion
- **File**: `position_based_dynamics.cpp`
- **Function**: `update_cloth_edges`
- **Purpose**: Implement edge constraints for cloth simulation using Position-Based Dynamics

## Deliverables
1. **Completed function** in `position_based_dynamics.cpp`
2. **Technical report** with experimental findings
3. **Visual documentation** (screenshots/videos if applicable)

## Report Structure
- Introduction and objectives
- Experimental setup
- Results and analysis
- Conclusion

Instructions
============
Support platforms: Windows, Linux 

## Dependencies

| Name                                   | Version | Usage                                               | Import         |
| -------------------------------------- | ------- | --------------------------------------------------- | -------------- |
| eigen3                                 | 3.4.0   | matrix calculation                                  | package        |
| freeglut                               | 3.4.0   | visualization                                       | package        |
| glew                                   | 2.2.0#3 | visualization                                       | package        |

### linux

We use CMake to build the project.
```bash
# Install dependencies
sudo apt install libglew-dev freeglut3-dev libeigen3-dev

# Create and enter build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build in Release mode (adjust -j16 to match your CPU cores)
make -j16

# Run the executable
./pbd
```


## Windows
This project uses **vcpkg** for dependency management and **CMake** for building.

## Prerequisites

- [CMake](https://cmake.org/) installed on your system
- [vcpkg](https://github.com/microsoft/vcpkg) installed from GitHub

## Configuration

### 1. vcpkg Setup

Set up environment variables for vcpkg:

- **CMAKE_TOOLCHAIN_FILE**
  - Variable: `CMAKE_TOOLCHAIN_FILE`
  - Value: `(YOUR_VCPKG_PARENT_FOLDER)/scripts/buildsystems/vcpkg.cmake`

- **Add to PATH**
  - Include `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg.exe` in your system's `PATH` variable

## Building the Project

Follow these steps to build and run:

```bash
# Install dependencies
vcpkg install eigen3 freeglut glew

# Create and enter build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build in Release mode (adjust -j16 to match your CPU cores)
cmake --build . --config Release -j16

# Run the executable
./Release/pbd.exe
```