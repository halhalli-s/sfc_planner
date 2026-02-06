// main.cpp
// Test script for Safe Flight Corridor generation

#include "safe_flight_corridor.h"
#include <iostream>

int main(int argc, char **argv)
{
    std::cout << "=== Safe Flight Corridor Generator ===" << std::endl;

    // Define start and goal
    Eigen::Vector3d start(1.0, 5.0, 3.0);
    Eigen::Vector3d goal(24.0, 5.0, 3.0);

    // Map parameters (for random obstacle generation)
    MapParams map_params;
    map_params.resolution = 1.0; // Increased from 0.05 to 0.5 for faster A*
    map_params.num_obstacles = 20;
    map_params.map_x = 25.0;
    map_params.map_y = 11.0;
    map_params.map_z = 6.0;
    map_params.obstacle_size = 1.0;
    map_params.clearance = 0.3;
    map_params.obstacle_range_x = {5.0, 20.0};
    map_params.obstacle_range_y = {0.0, 11.0};
    map_params.obstacle_range_z = {1.0, 5.0};

    // ADMM parameters
    ADMMParams admm_params;
    admm_params.outer_max = 1;
    admm_params.inner_max = 3;
    admm_params.rho = 1.0;
    admm_params.epsilon = 0.3;
    admm_params.bbox_size = 1.5;
    admm_params.wc = 1.0;
    admm_params.wv = 1.0;

    // Create SFC object
    SafeFlightCorridor sfc;

    // Generate SFC (with random obstacles)
    sfc.generate(start, goal, map_params, admm_params);

    // Print results
    // std::cout << "\n=== RESULTS ===" << std::endl;
    // std::cout << "Waypoints: " << sfc.get_waypoints().size() << std::endl;
    // std::cout << "Ellipsoids: " << sfc.get_ellipsoids().size() << std::endl;
    // std::cout << "Polytopes: " << sfc.get_polytopes().size() << std::endl;
    // std::cout << "Converged: " << (sfc.is_converged() ? "yes" : "no") << std::endl;

    // Print waypoints
    // std::cout << "\nOptimized waypoints:" << std::endl;
    // for (size_t i = 0; i < sfc.get_waypoints().size(); i++)
    // {
    //     auto wp = sfc.get_waypoints()[i];
    //     std::cout << "  " << i << ": [" << wp[0] << ", " << wp[1] << ", " << wp[2] << "]" << std::endl;
    // }

    // // Print ellipsoid info
    // std::cout << "\nEllipsoid centers:" << std::endl;
    // for (size_t i = 0; i < sfc.get_ellipsoids().size(); i++)
    // {
    //     auto ell = sfc.get_ellipsoids()[i];
    //     std::cout << "  " << i << ": [" << ell.center.x() << ", " << ell.center.y() << ", " << ell.center.z() << "]" << std::endl;
    // }

    // // Print polytope info
    // std::cout << "\nPolytope faces:" << std::endl;
    // for (size_t i = 0; i < sfc.get_polytopes().size(); i++)
    // {
    //     std::cout << "  " << i << ": " << sfc.get_polytopes()[i].A.rows() << " faces" << std::endl;
    // }

    // Visualize (toggle)
    bool visualize = false;
    if (visualize)
    {
        std::cout << "\nVisualizing..." << std::endl;
        sfc.visualize();
    }

    return 0;
}