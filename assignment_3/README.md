# Position-Based-Dynamics Assignment 3
## Objective
Complete the implementation of the `update_cloth_edges` function in `position_based_dynamic.cpp` and document experimental results.

## Implementation Requirements

### 1. Code Completion
- **File**: `position_based_dynamic.cpp`
- **Function**: `update_cloth_edges`
- **Purpose**: Implement edge constraints for cloth simulation using Position-Based Dynamics

## Deliverables
1. **Completed function** in `position_based_dynamic.cpp`
2. **Technical report** with experimental findings
3. **Visual documentation** (screenshots/videos if applicable)

## Report Structure
- Introduction and objectives
- Experimental setup
- Results and analysis
- Conclusion

Instructions
============
Support platforms: Windows, Linux, macOS

## Dependencies

| Name                                   | Version | Usage                                               | Import         |
| -------------------------------------- | ------- | --------------------------------------------------- | -------------- |
| eigen3                                 | 3.4.0   | matrix calculation                                  | package        |
| freeglut                               | 3.4.0   | visualization                                       | package        |
| glew                                   | 2.2.0#3 | visualization                                       | package        |

### macOS

This project uses **Homebrew** for dependency management and **CMake** for building.

## Prerequisites

- [CMake](https://cmake.org/) installed on your system
- [Homebrew](https://brew.sh/) installed on your system

## Building the Project

Follow these steps to build and run:

```bash
# Install dependencies
brew install eigen glew

# Create and enter build directory
mkdir build
cd build

# Configure with CMake (pass Homebrew prefix so CMake finds the packages)
cmake .. -DCMAKE_PREFIX_PATH=/opt/homebrew

# Build (adjust -j to match your CPU cores)
make -j$(sysctl -n hw.logicalcpu)

# Run the executable
./pbd
```

> **Note:** The project uses Apple's native GLUT framework on macOS instead of
> freeglut. Homebrew's freeglut requires XQuartz (an X11 display server) and will
> fail with `"failed to open display ''"` on a standard macOS setup. The
> `CMakeLists.txt` handles this automatically — do **not** install freeglut via
> Homebrew.

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