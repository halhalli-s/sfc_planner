cmake_minimum_required(VERSION 3.10)
project(sfc_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# Find CasADi
find_package(casadi REQUIRED)

# IRIS library (multiple components)
find_library(IRIS_LIBRARY iris HINTS /usr/local/lib)
find_library(IRIS_GEOMETRY_LIBRARY iris_geometry HINTS /usr/local/lib)
find_path(IRIS_INCLUDE_DIR iris/iris.h HINTS /usr/local/include)

# libcdd for H-rep to V-rep conversion
find_library(CDD_LIBRARY cdd HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)
find_path(CDD_INCLUDE_DIR cdd/cdd.h HINTS /usr/local/include /usr/include)

# Mosek (needed by IRIS)
set(MOSEK_DIR "$ENV{HOME}/mosek/9.3/tools/platform/linux64x86")
find_library(MOSEK_LIBRARY mosek64 HINTS ${MOSEK_DIR}/bin)
find_path(MOSEK_INCLUDE_DIR mosek.h HINTS ${MOSEK_DIR}/h)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${IRIS_INCLUDE_DIR}
    ${MOSEK_INCLUDE_DIR}
    ${CDD_INCLUDE_DIR}
)

# Source files
set(SFC_SOURCES
    src/safe_flight_corridor.cpp
    src/sfc_path.cpp
    src/sfc_ellipsoids.cpp
    src/sfc_bounding_box.cpp
    src/sfc_admm.cpp
    src/sfc_visualize.cpp
)

# Create library
add_library(sfc_lib ${SFC_SOURCES})

target_link_libraries(sfc_lib
    Eigen3::Eigen
    casadi
    ${IRIS_LIBRARY}
    ${IRIS_GEOMETRY_LIBRARY}
    ${MOSEK_LIBRARY}
    ${CDD_LIBRARY}
    Python3::Python
    Python3::NumPy
)

# Main executable
add_executable(sfc_planner src/main.cpp)

target_link_libraries(sfc_planner
    sfc_lib
)

# Install
install(TARGETS sfc_planner sfc_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)