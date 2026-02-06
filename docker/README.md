# Docker Setup for SFC Planner

## Prerequisites
- **Mosek license file**: Place `mosek.lic` in this directory before building
  - Get free academic license: https://www.mosek.com/products/academic-licenses/
  - **Note:** Do not commit this file (already in `.gitignore`)

## What's in the Image
- Ubuntu 22.04
- ROS2 Humble
- IRIS library (with ARM64 fixes)
- Mosek 10.1 solver
- CasADi optimizer
- Eigen3, libcdd, and all dependencies

## Quick Start

### 1. Build Image (One Time - ~10 minutes)
```bash
./build_image.sh
```

### 2. Run Development Container
```bash
./run_container.sh
# You'll land at /home/prance/sfc_planner inside the container
```

### 3. Build SFC Planner
```bash
# Inside container
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -Dcasadi_DIR=/usr/local/lib/python3.10/dist-packages/casadi/cmake
make -j4
```

### 4. Run Tests
```bash
# Standalone test
./sfc_planner

# ROS2 node
./sfc_ros_node
```

### 5. Exit Container
```bash
exit
# Container is automatically deleted (--rm flag)
# Run ./run_container.sh again to start a fresh one
```

## Expected Performance (VOXL2)
- Total planning time: **0.8-1.4 seconds**
- Suitable for real-time flight at 3-5 m/s

## Troubleshooting

### "cannot find -lmosek"
Ensure `mosek.lic` is in this directory before building the image.

### "cannot find -lcasadi"
CasADi should be pre-installed in the image. Verify with:
```bash
ls /usr/local/lib/python3.10/dist-packages/casadi/
```

### Rebuild Image
If you need to rebuild after changes:
```bash
docker rmi prance:sfc-planner
./build_image.sh
```
