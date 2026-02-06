# Docker Setup for SFC Planner

## Prerequisites
- Mosek license file: Place `mosek.lic` in this directory before building

## What's in the Image
- Ubuntu 22.04
- ROS2 Humble
- IRIS library (with ARM64 fixes)
- Mosek 10.1 solver
- CasADi optimizer
- Eigen3, libcdd, and all dependencies

## Build Image
```bash
./build_image.sh
```

## Run Container
```bash
./run_container.sh
```

You'll land at `/home/prance/sfc_planner` with your code mounted.
```

