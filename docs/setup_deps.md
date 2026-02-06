# SFC Planner - VOXL2 ARM64 Setup Guide

## Overview

This document describes how to build and run the Safe Flight Corridor (SFC) planner on VOXL2 (ARM64/aarch64) hardware inside a Docker container with ROS2 Humble.

**Target Hardware:** ModalAI VOXL2 (M0054)  
**Architecture:** ARM64 (aarch64)  
**OS:** Ubuntu 22.04 (Jammy) in Docker  
**ROS Version:** ROS2 Humble  
**Date:** February 2026  

---

## Dependencies Required

| Library | Purpose | Installation Method |
|---------|---------|---------------------|
| Eigen3 | Linear algebra | apt (pre-installed) |
| libcdd | Polytope vertex enumeration | apt |
| CasADi | Nonlinear optimization | pip |
| Mosek 10.1 | Convex optimization solver (for IRIS) | Manual download |
| IRIS | Polytope inflation | Build from source (with fixes) |

---

## Step 1: Connect to VOXL2

### Via ADB (USB)
```bash
# Install ADB
sudo apt install android-tools-adb

# Fix permissions if needed
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"05c6\", MODE=\"0666\", GROUP=\"plugdev\"" > /etc/udev/rules.d/51-android.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
adb kill-server && adb start-server

# Connect
adb devices
adb shell
```

### Enter Docker Container
```bash
cd /home/root/prance-firmware
./run_docker.sh
# Prompt changes to: root@m0054:/home/prance#
```

---

## Step 2: Install Basic Dependencies

```bash
apt update
apt install -y build-essential cmake git wget pkg-config libcdd-dev libeigen3-dev libgmp-dev libmpfr-dev

# Install CasADi via pip
pip3 install casadi

# Create symlink for cdd headers (IRIS expects /usr/include/cdd/)
ln -s /usr/include/cddlib /usr/include/cdd
```

---

## Step 3: Install Mosek 10.1

### Download and Extract
```bash
cd /opt
wget https://download.mosek.com/stable/10.1.28/mosektoolslinuxaarch64.tar.bz2
tar -xjf mosektoolslinuxaarch64.tar.bz2
```

### Transfer License
From your laptop (where you have mosek.lic):
```bash
adb push ~/mosek/mosek.lic /home/root/mosek.lic
```

From VOXL2 base OS (outside Docker):
```bash
docker cp /home/root/mosek.lic prance_ros2_humble:/opt/mosek/mosek.lic
```

### Set Environment Variables
```bash
export MOSEK_DIR=/opt/mosek/10.1/tools/platform/linuxaarch64
export PATH=$PATH:$MOSEK_DIR/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MOSEK_DIR/bin
export MOSEKLM_LICENSE_FILE=/opt/mosek/mosek.lic
```

---

## Step 4: Build IRIS from Source (with ARM64 Fixes)

### Clone Repository
```bash
cd /opt
git clone https://github.com/rdeits/iris-distro.git
cd iris-distro
```

### Fix 1: Modify Main CMakeLists.txt (`/opt/iris-distro/src/CMakeLists.txt`)

We need to make several changes to this file:

**1a. Add Mosek paths after line 17 (replacing find_package):**

First, backup and create a modified version:
```bash
cd /opt/iris-distro/src
cp CMakeLists.txt CMakeLists.txt.bak

# Create Mosek path fix
cat > /tmp/mosek_fix.txt << 'EOF'
# find_package(Mosek CONFIG REQUIRED)
set(MOSEK_DIR "/opt/mosek/10.1/tools/platform/linuxaarch64")
set(MOSEK_INCLUDE_DIR "${MOSEK_DIR}/h")
set(MOSEK_LIBRARY "${MOSEK_DIR}/bin/libmosek64.so")
include_directories(${MOSEK_INCLUDE_DIR})
link_directories("${MOSEK_DIR}/bin")
set(mosek_LIBRARIES "${MOSEK_LIBRARY}")
EOF

# Insert after line 17 (replacing line 18 which has find_package(Mosek))
head -17 CMakeLists.txt.bak > CMakeLists.txt
cat /tmp/mosek_fix.txt >> CMakeLists.txt
tail -n +19 CMakeLists.txt.bak >> CMakeLists.txt
```

**1b. Comment out any remaining find_package(Mosek) calls:**
```bash
sed -i 's/find_package(Mosek CONFIG REQUIRED)/# find_package(Mosek CONFIG REQUIRED)/' /opt/iris-distro/src/CMakeLists.txt
```

**1c. Add GNUInstallDirs for install directory variables:**
```bash
sed -i '15i include(GNUInstallDirs)' /opt/iris-distro/src/CMakeLists.txt
```

**1d. Set CDD include path (before the find_path check around line 46):**
```bash
sed -i '46i set(CDD_INCLUDE_DIR "/usr/include/cdd")' /opt/iris-distro/src/CMakeLists.txt
```

**Verify the changes:**
```bash
grep -n "MOSEK_DIR\|find_package(Mosek\|GNUInstallDirs\|CDD_INCLUDE_DIR" /opt/iris-distro/src/CMakeLists.txt
```

### Fix 2: Modify cxx/CMakeLists.txt (`/opt/iris-distro/src/cxx/CMakeLists.txt`)

**2a. Add GNUInstallDirs at the very top:**
```bash
sed -i '1i include(GNUInstallDirs)' /opt/iris-distro/src/cxx/CMakeLists.txt
```

**2b. Add Mosek variables after line 2:**
```bash
sed -i '2i set(MOSEK_DIR "/opt/mosek/10.1/tools/platform/linuxaarch64")\nset(MOSEK_LIBRARY "${MOSEK_DIR}/bin/libmosek64.so")' /opt/iris-distro/src/cxx/CMakeLists.txt
```

**2c. Add IRIS install directory variables:**
```bash
sed -i '4i set(IRIS_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})\nset(IRIS_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})' /opt/iris-distro/src/cxx/CMakeLists.txt
```

**2d. Fix the Mosek library linking (change `mosek` to `${MOSEK_LIBRARY}`):**
```bash
sed -i 's/target_link_libraries(iris_mosek iris_geometry mosek)/target_link_libraries(iris_mosek iris_geometry ${MOSEK_LIBRARY})/' /opt/iris-distro/src/cxx/CMakeLists.txt
```

**Verify the top of the file:**
```bash
head -10 /opt/iris-distro/src/cxx/CMakeLists.txt
```

Should show:
```cmake
include(GNUInstallDirs)
set(MOSEK_DIR "/opt/mosek/10.1/tools/platform/linuxaarch64")
set(MOSEK_LIBRARY "${MOSEK_DIR}/bin/libmosek64.so")
set(IRIS_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(IRIS_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})
add_subdirectory(cvxgen)
...
```

**Verify the linking fix:**
```bash
grep "target_link_libraries(iris_mosek" /opt/iris-distro/src/cxx/CMakeLists.txt
```

Should show `${MOSEK_LIBRARY}` instead of `mosek`.

### Fix 3: Fix Deprecated Mosek API in iris_mosek.cpp

Mosek 10.x removed some API constants that existed in older versions. We need to fix these:

**3a. Replace `MSKCONST` with `const`:**
```bash
sed -i 's/MSKCONST/const/g' /opt/iris-distro/src/cxx/iris_mosek.cpp
```

**3b. Remove deprecated solution status constants:**
```bash
sed -i '/MSK_SOL_STA_NEAR_OPTIMAL/d' /opt/iris-distro/src/cxx/iris_mosek.cpp
sed -i '/MSK_SOL_STA_NEAR_DUAL_INFEAS_CER/d' /opt/iris-distro/src/cxx/iris_mosek.cpp
sed -i '/MSK_SOL_STA_NEAR_PRIM_INFEAS_CER/d' /opt/iris-distro/src/cxx/iris_mosek.cpp
```

**Why these fixes are needed:**
- `MSKCONST` was a macro in older Mosek that maps to `const` - removed in Mosek 10
- `MSK_SOL_STA_NEAR_OPTIMAL`, `MSK_SOL_STA_NEAR_DUAL_INFEAS_CER`, `MSK_SOL_STA_NEAR_PRIM_INFEAS_CER` were solution status enums removed in Mosek 10

### Build IRIS
```bash
cd /opt/iris-distro
mkdir build && cd build

cmake .. \
  -DIRIS_WITH_EIGEN=OFF \
  -DIRIS_WITH_CDD=OFF \
  -DIRIS_WITH_MOSEK=OFF \
  -DIRIS_WITH_PYBIND11=OFF \
  -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3

make -j4
```

### Install IRIS to System
```bash
cp /opt/iris-distro/build/install/lib/libiris* /usr/local/lib/
cp -r /opt/iris-distro/build/install/include/iris /usr/local/include/
ldconfig
```

### Verify Installation
```bash
ls /usr/local/lib/libiris*
# Should show: libiris.so, libiris_geometry.so, libiris_mosek.so, etc.

ls /usr/local/include/iris/
# Should show: iris.h, geometry.h, iris_mosek.h, cvxgen_ldp.h
```

---

## Step 5: Set Up CasADi for C++ Linking

CasADi was installed via pip, which puts libraries in a non-standard location. We need to:

### 5a. Create symlinks for the linker
```bash
ln -s /usr/local/lib/python3.10/dist-packages/casadi/libcasadi.so.3.7 /usr/local/lib/libcasadi.so
ln -s /usr/local/lib/python3.10/dist-packages/casadi/libcasadi.so.3.7 /usr/local/lib/libcasadi.so.3.7
ldconfig
```

### 5b. Verify symlinks
```bash
ls -la /usr/local/lib/libcasadi*
```

### Why this is needed
When compiling with `-lcasadi`, the linker searches standard paths like `/usr/local/lib/` and `/usr/lib/`. 
The pip-installed CasADi puts its library at `/usr/local/lib/python3.10/dist-packages/casadi/libcasadi.so`.
The symlink bridges this gap.

---

## Step 6: Transfer and Build SFC Planner

### Transfer Code
From laptop:
```bash
adb push ~/ros2_ws/src/sfc_planner /home/root/
```

From VOXL2 base OS:
```bash
docker cp /home/root/sfc_planner prance_ros2_humble:/home/prance/
```

### Fix CMakeLists.txt for ARM64

**6a. Change Mosek path from x86 to ARM64:**
```bash
cd /home/prance/sfc_planner
sed -i 's|$ENV{HOME}/mosek/9.3/tools/platform/linux64x86|/opt/mosek/10.1/tools/platform/linuxaarch64|' CMakeLists.txt
```

**6b. Add CasADi include path (after find_package(casadi)):**
```bash
sed -i '/find_package(casadi REQUIRED)/a include_directories(/usr/local/lib/python3.10/dist-packages/casadi/include)' CMakeLists.txt
```

**Verify the changes:**
```bash
grep -n "MOSEK_DIR\|casadi/include" CMakeLists.txt
```

### Build
```bash
cd /home/prance/sfc_planner
rm -rf build && mkdir build && cd build

source /opt/ros/humble/setup.bash

cmake .. -DCMAKE_BUILD_TYPE=Release \
  -Dcasadi_DIR=/usr/local/lib/python3.10/dist-packages/casadi/cmake

make -j4
```

---

## Step 7: Run SFC Planner

### Set Environment Variables
These must be set every time before running (or add to ~/.bashrc):
```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/mosek/10.1/tools/platform/linuxaarch64/bin:/usr/local/lib:/usr/local/lib/python3.10/dist-packages/casadi
export MOSEKLM_LICENSE_FILE=/opt/mosek/mosek.lic
```

**Why each path is needed:**
- `/opt/mosek/10.1/tools/platform/linuxaarch64/bin` - Mosek shared libraries (libmosek64.so)
- `/usr/local/lib` - IRIS libraries and CasADi symlink
- `/usr/local/lib/python3.10/dist-packages/casadi` - CasADi plugins (libcasadi_nlpsol_ipopt.so)

### Run Standalone Test
```bash
./sfc_planner
```

Expected output:
```
=== Safe Flight Corridor Generator ===
...
⏱️  ===== TOTAL TIME: 0.86 sec =====
```

### Run ROS2 Node
```bash
./sfc_ros_node
```

Expected output:
```
[INFO] [sfc_publisher]: ✓ SFC generated in 0.86 seconds
[INFO] [sfc_publisher]: ✓ Waypoints: 14
[INFO] [sfc_publisher]: ✓ Polytopes: 13
[INFO] [sfc_publisher]: Published to /sfc_path and /sfc_markers
```

---

## Troubleshooting

### "cannot find -lmosek"
Mosek library not in linker path. Ensure MOSEK_LIBRARY points to full path:
```bash
ls /opt/mosek/10.1/tools/platform/linuxaarch64/bin/libmosek64.so
```
And verify the CMakeLists.txt fix uses `${MOSEK_LIBRARY}` not just `mosek`.

### "cannot find -lcasadi"
CasADi symlinks missing:
```bash
ln -s /usr/local/lib/python3.10/dist-packages/casadi/libcasadi.so.3.7 /usr/local/lib/libcasadi.so
ldconfig
```

### "libcasadi_nlpsol_ipopt.so: cannot open shared object file"
IPOPT plugin not in LD_LIBRARY_PATH. Add CasADi directory:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/casadi
```

### "Could NOT find cdd.h"
Create symlink from cddlib to cdd:
```bash
ln -s /usr/include/cddlib /usr/include/cdd
ls /usr/include/cdd/cdd.h  # verify
```

### "Variable IRIS_INCLUDE_DIR does not exist"
Missing GNUInstallDirs include. Add to cxx/CMakeLists.txt:
```bash
sed -i '1i include(GNUInstallDirs)' /opt/iris-distro/src/cxx/CMakeLists.txt
sed -i '2i set(IRIS_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})\nset(IRIS_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})' /opt/iris-distro/src/cxx/CMakeLists.txt
```

### "install FILES given no DESTINATION"
Missing CMAKE_INSTALL_DOCDIR. Add GNUInstallDirs to main CMakeLists.txt:
```bash
sed -i '15i include(GNUInstallDirs)' /opt/iris-distro/src/CMakeLists.txt
```

### IRIS "MSKCONST has not been declared"
Mosek 10.x API changed. Run:
```bash
sed -i 's/MSKCONST/const/g' /opt/iris-distro/src/cxx/iris_mosek.cpp
```

### IRIS "MSK_SOL_STA_NEAR_OPTIMAL was not declared"
Deprecated Mosek constants. Remove them:
```bash
sed -i '/MSK_SOL_STA_NEAR_OPTIMAL/d' /opt/iris-distro/src/cxx/iris_mosek.cpp
sed -i '/MSK_SOL_STA_NEAR_DUAL_INFEAS_CER/d' /opt/iris-distro/src/cxx/iris_mosek.cpp
sed -i '/MSK_SOL_STA_NEAR_PRIM_INFEAS_CER/d' /opt/iris-distro/src/cxx/iris_mosek.cpp
```

### "casadi/casadi.hpp: No such file or directory"
CasADi include path not set. Add to your project's CMakeLists.txt:
```bash
sed -i '/find_package(casadi REQUIRED)/a include_directories(/usr/local/lib/python3.10/dist-packages/casadi/include)' CMakeLists.txt
```

### CMake "Could not find a package configuration file provided by casadi"
Tell CMake where CasADi's cmake files are:
```bash
cmake .. -Dcasadi_DIR=/usr/local/lib/python3.10/dist-packages/casadi/cmake
```

---

## Performance Results

| Metric | VOXL2 (ARM64) | Laptop (x86_64) |
|--------|---------------|-----------------|
| Total Time | 0.86 sec | 0.90 sec |
| A* Planning | 0.02 sec | 0.01 sec |
| IRIS Inflation | 0.40 sec | 0.35 sec |
| ADMM Optimization | 0.47 sec | 0.45 sec |
| Waypoints | 14 | ~14 |
| Polytopes | 13 | ~13 |

**Conclusion:** VOXL2 achieves comparable performance to laptop, suitable for real-time flight at 3-5 m/s.

---

## File Locations Summary

| Item | Path in Docker |
|------|----------------|
| Mosek | `/opt/mosek/10.1/tools/platform/linuxaarch64/` |
| Mosek License | `/opt/mosek/mosek.lic` |
| IRIS Source | `/opt/iris-distro/` |
| IRIS Headers | `/usr/local/include/iris/` |
| IRIS Libraries | `/usr/local/lib/libiris*.so` |
| CasADi | `/usr/local/lib/python3.10/dist-packages/casadi/` |
| CasADi Symlink | `/usr/local/lib/libcasadi.so` |
| libcdd Headers | `/usr/include/cdd/` (symlink to `/usr/include/cddlib/`) |
| SFC Planner | `/home/prance/sfc_planner/` |
| SFC Executables | `/home/prance/sfc_planner/build/` |

---

## Files Modified Summary

### IRIS Build Fixes

| File | Modification |
|------|--------------|
| `/opt/iris-distro/src/CMakeLists.txt` | Added GNUInstallDirs, Mosek paths, CDD path, commented find_package(Mosek) |
| `/opt/iris-distro/src/cxx/CMakeLists.txt` | Added GNUInstallDirs, Mosek paths, IRIS_INCLUDE/LIBRARY_DIR, fixed mosek linking |
| `/opt/iris-distro/src/cxx/iris_mosek.cpp` | Replaced MSKCONST→const, removed deprecated MSK_SOL_STA_NEAR_* |

### System Symlinks Created

| Symlink | Target | Purpose |
|---------|--------|---------|
| `/usr/include/cdd` | `/usr/include/cddlib` | IRIS expects cdd.h at /usr/include/cdd/ |
| `/usr/local/lib/libcasadi.so` | `.../casadi/libcasadi.so.3.7` | Linker can find CasADi |
| `/usr/local/lib/libcasadi.so.3.7` | `.../casadi/libcasadi.so.3.7` | Runtime loader can find CasADi |

### SFC Planner Fixes

| File | Modification |
|------|--------------|
| `/home/prance/sfc_planner/CMakeLists.txt` | Changed Mosek path to ARM64, added CasADi include path |

---

## Quick Reference: Complete Build Commands

```bash
# === IRIS Build (run once) ===
cd /opt/iris-distro
mkdir build && cd build
cmake .. \
  -DIRIS_WITH_EIGEN=OFF \
  -DIRIS_WITH_CDD=OFF \
  -DIRIS_WITH_MOSEK=OFF \
  -DIRIS_WITH_PYBIND11=OFF \
  -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
make -j4

# Install to system
cp /opt/iris-distro/build/install/lib/libiris* /usr/local/lib/
cp -r /opt/iris-distro/build/install/include/iris /usr/local/include/
ldconfig

# === SFC Planner Build ===
cd /home/prance/sfc_planner
rm -rf build && mkdir build && cd build
source /opt/ros/humble/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -Dcasadi_DIR=/usr/local/lib/python3.10/dist-packages/casadi/cmake
make -j4

# === Run SFC Planner ===
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/mosek/10.1/tools/platform/linuxaarch64/bin:/usr/local/lib:/usr/local/lib/python3.10/dist-packages/casadi
./sfc_planner      # standalone test
./sfc_ros_node     # ROS2 node
```