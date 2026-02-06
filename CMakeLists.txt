cmake_minimum_required(VERSION 3.10)
project(sfc_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Check if building with ROS2
if(TRUE)
    set(ROS2_BUILD TRUE)
    message(STATUS "Building with ROS2 support")
else()
    set(ROS2_BUILD FALSE)
    message(STATUS "Building standalone (no ROS2)")
endif()

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(OpenMP REQUIRED)

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

# ROS2 packages (optional)
if(ROS2_BUILD)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
endif()

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
    OpenMP::OpenMP_CXX
)

# Main executable (standalone)
add_executable(sfc_planner src/main.cpp)
target_link_libraries(sfc_planner sfc_lib)

# Comparison tool
add_executable(compare_iterations src/compare_iterations.cpp)
target_link_libraries(compare_iterations sfc_lib)

# ROS2 node (only if ROS2 is available)
if(ROS2_BUILD)
    add_executable(sfc_ros_node src/sfc_ros_node.cpp)
    target_link_libraries(sfc_ros_node sfc_lib)
    ament_target_dependencies(sfc_ros_node
        rclcpp
        nav_msgs
        geometry_msgs
        visualization_msgs
        std_msgs
    )
    
    # Install ROS2 node
    install(TARGETS sfc_ros_node
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

# Install standalone executables
install(TARGETS sfc_planner sfc_lib compare_iterations
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# ROS2-specific install
if(ROS2_BUILD)
    ament_package()
endif()