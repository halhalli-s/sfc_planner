# Safe Flight Corridor (SFC) Planner

A 3D path planning library for quadrotors that generates collision-free flight corridors using ADMM optimization and IRIS polytope inflation.

## Features

- **A* Path Planning**: Finds initial collision-free path through 3D obstacle field
- **IRIS Polytope Inflation**: Creates safe convex regions around the path
- **ADMM Optimization**: Jointly optimizes waypoints and ellipsoid shapes
- **3D Visualization**: Matplotlib visualization with collision detection analysis
- **ROS2 Compatible**: Integrated with ROS2 workspace structure

## Demo

[Add screenshot of your matplotlib visualization here]

## Algorithm Overview
```
1. A* Pathfinding → initial waypoints
2. Ellipsoid Initialization → corridor around path
3. Bounding Box Computation → local obstacle detection
4. ADMM Loop:
   - Inflate polytopes (IRIS)
   - Optimize waypoints (minimize path length)
   - Optimize ellipsoids (maximize volume)
   - Update Lagrangian multipliers
5. Validation → ensure obstacle-free corridors
```

## Dependencies

- **C++17** or later
- **Eigen3**: Linear algebra library
- **Python 3.x**: For visualization
  - matplotlib
  - numpy
  - scipy
- **libcdd**: Polytope H-rep to V-rep conversion
- **GMP**: GNU Multiple Precision library

### Install Dependencies (Ubuntu)
```bash
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    libcdd-dev \
    libgmp-dev
```

## Build Instructions
```bash
# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/sfc_planner.git

# Build
cd sfc_planner
mkdir -p build
cd build
cmake ..
make

# Run
./sfc_planner
```

## Usage
```cpp
#include "safe_flight_corridor.h"

int main() {
    // Create planner
    SafeFlightCorridor sfc;
    
    // Define start and goal
    Eigen::Vector3d start(1.0, 5.0, 3.0);
    Eigen::Vector3d goal(24.0, 5.0, 3.0);
    
    // Set parameters
    MapParams map_params;
    map_params.num_obstacles = 22;
    
    ADMMParams admm_params;
    admm_params.outer_max = 3;
    admm_params.epsilon = 0.3;
    
    // Run planner
    sfc.generate(start, goal, map_params, admm_params);
    
    // Get results
    auto waypoints = sfc.get_waypoints();
    
    // Visualize
    sfc.visualize();
    
    return 0;
}
```

## API Reference

### Main Functions

**`SafeFlightCorridor()`**
- Constructor - initializes planner with default parameters

**`void generate(start, goal, obstacles, ADMMParams)`**
- Generates SFC given user-provided obstacles

**`void generate(start, goal, MapParams, ADMMParams)`**
- Generates SFC with random obstacles for testing

**`void visualize()`**
- Opens matplotlib 3D visualization with collision analysis

**`std::vector<Vector3d> get_waypoints()`**
- Returns optimized waypoint positions

**`std::vector<Polytope> get_polytopes()`**
- Returns safe region polytopes (H-representation)

**`bool is_converged()`**
- Returns true if ADMM optimization converged

## Parameters

### MapParams
- `resolution`: Grid resolution for A* (default: 0.5m)
- `num_obstacles`: Number of random obstacles (default: 22)
- `map_x/y/z`: Map dimensions (default: 25×11×6m)

### ADMMParams
- `outer_max`: Outer ADMM iterations (default: 3)
- `inner_max`: Inner ADMM iterations (default: 5)
- `rho`: ADMM penalty parameter (default: 1.0)
- `epsilon`: Ellipsoid perpendicular radius (default: 0.3m)

## Project Structure
```
sfc_planner/
├── include/
│   └── safe_flight_corridor.h      # Main header
├── src/
│   ├── main.cpp                    # Test program
│   ├── safe_flight_corridor.cpp    # Core implementation
│   ├── sfc_path.cpp               # A* pathfinding
│   ├── sfc_ellipsoids.cpp         # Ellipsoid initialization
│   ├── sfc_bounding_box.cpp       # Bounding box computation
│   ├── sfc_iris.cpp               # IRIS polytope inflation
│   ├── sfc_admm.cpp               # ADMM optimization
│   └── sfc_visualize.cpp          # Visualization
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Validation

The planner automatically validates that all polytopes are obstacle-free:
```
=== POLYTOPE ANALYSIS ===
Polytope 0: 8 faces, 12 vertices
  ✓ Obstacle-free
Polytope 1: 6 faces, 8 vertices
  ✓ Obstacle-free
...
=== SUMMARY ===
✓ ALL POLYTOPES ARE OBSTACLE-FREE!
```

## Related Work

This implementation is based on:
- **IRIS**: Iterative Regional Inflation by Semidefinite programming
- **ADMM**: Alternating Direction Method of Multipliers
- References to key papers in your thesis

## Author

**Your Name**  
Master's Student, Northeastern University  
Advisors: Seth Hutchinson, Hanumant Singh

## License

[Choose: MIT, Apache 2.0, GPL, etc.]

## Citation

If you use this code in your research, please cite:
```bibtex
@mastersthesis{yourname2025sfc,
  title={Safe Flight Corridor Generation for Quadrotor Navigation},
  author={Your Name},
  year={2025},
  school={Northeastern University}
}
```

## Acknowledgments

- Advisors: Seth Hutchinson and Hanumant Singh
- IRIS algorithm developers
- ROS2 and Eigen communities