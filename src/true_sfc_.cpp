// safe_flight_corridor.cpp
// Constructor, generate(), and main pipeline orchestration

#include "safe_flight_corridor.h"
#include <iostream>
#include <random>

SafeFlightCorridor::SafeFlightCorridor()
    : converged_(false), map_x_(25.0), map_y_(11.0), map_z_(6.0)
{
}

void SafeFlightCorridor::generate(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    const std::vector<Eigen::Vector3d> &obstacles,
    const ADMMParams &params)
{
    // Store inputs
    start_ = start;
    goal_ = goal;
    obstacles_ = obstacles;

    // Use default map params for path planning
    MapParams map_params;

    std::cout << "=== Starting SFC Generation ===" << std::endl;
    std::cout << "Start: [" << start.x() << ", " << start.y() << ", " << start.z() << "]" << std::endl;
    std::cout << "Goal: [" << goal.x() << ", " << goal.y() << ", " << goal.z() << "]" << std::endl;
    std::cout << "Obstacles: " << obstacles.size() << std::endl;

    // Step 1: Find path
    std::cout << "\n--- Step 1: Path Planning ---" << std::endl;
    get_path(start, goal, obstacles, map_params);
    std::cout << "Waypoints: " << waypoints_.size() << std::endl;

    // Step 2: Initialize ellipsoids
    std::cout << "\n--- Step 2: Initialize Ellipsoids ---" << std::endl;
    init_ellipsoids(params.epsilon);
    std::cout << "Ellipsoids: " << ellipsoids_.size() << std::endl;

    // Step 3: Compute bounding boxes
    std::cout << "\n--- Step 3: Compute Bounding Boxes ---" << std::endl;
    compute_bounding_boxes(params.bbox_size);
    std::cout << "Bounding boxes: " << bboxes_.size() << std::endl;

    // Step 4: Initialize Lagrangian multipliers
    int num_ellipsoids = ellipsoids_.size();
    y_ = Eigen::MatrixXd::Zero(2, num_ellipsoids);

    // Step 5: ADMM optimization loop
    std::cout << "\n--- Step 4: ADMM Optimization ---" << std::endl;

    for (int outer = 0; outer < params.outer_max; outer++)
    {
        std::cout << "\n========== OUTER ITERATION " << outer + 1 << "/" << params.outer_max << " ==========" << std::endl;

        // Inflate polytopes using IRIS
        inflate_polytopes();
        std::cout << "Inflated " << polytopes_.size() << " polytopes" << std::endl;

        // Inner loop
        for (int k = 0; k < params.inner_max; k++)
        {
            std::cout << "--- Inner iteration " << k + 1 << "/" << params.inner_max << " ---" << std::endl;

            optimize_waypoints(params.rho, params.wc);
            optimize_ellipsoids(params.rho, params.wv);
            update_lagrangian(params.rho);
        }
    }

    converged_ = true;
    std::cout << "\n=== SFC Generation Complete ===" << std::endl;
}

void SafeFlightCorridor::generate(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    const MapParams &map_params,
    const ADMMParams &admm_params)
{
    // Store map dimensions for visualization
    map_x_ = map_params.map_x;
    map_y_ = map_params.map_y;
    map_z_ = map_params.map_z;

    // Generate random obstacles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_x(map_params.obstacle_range_x[0], map_params.obstacle_range_x[1]);
    std::uniform_real_distribution<> dist_y(map_params.obstacle_range_y[0], map_params.obstacle_range_y[1]);
    std::uniform_real_distribution<> dist_z(map_params.obstacle_range_z[0], map_params.obstacle_range_z[1]);

    std::vector<Eigen::Vector3d> obstacles;
    for (int i = 0; i < map_params.num_obstacles; i++)
    {
        obstacles.emplace_back(dist_x(gen), dist_y(gen), dist_z(gen));
    }

    std::cout << "Generated " << obstacles.size() << " random obstacles" << std::endl;

    // Call main generate function
    generate(start, goal, obstacles, admm_params);
}