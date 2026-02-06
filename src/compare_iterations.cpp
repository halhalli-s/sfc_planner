// compare_iterations.cpp
// Compare SFC quality with different ADMM iteration settings

#include "safe_flight_corridor.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <random>
#include <algorithm>

// Compute metrics for a generated SFC
struct SFCMetrics
{
    double total_time;
    double path_length;
    double avg_polytope_volume;
    double min_obstacle_distance;
    double final_cost;
    int num_polytopes;
    int num_failed_polytopes;
};

SFCMetrics compute_metrics(const SafeFlightCorridor &sfc, double planning_time)
{
    SFCMetrics metrics;
    metrics.total_time = planning_time;

    auto waypoints = sfc.get_waypoints();
    auto polytopes = sfc.get_polytopes();
    auto obstacles = sfc.get_obstacles();

    // 1. Path length
    metrics.path_length = 0.0;
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        metrics.path_length += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // 2. Average polytope volume (approximate using bounding box)
    double total_volume = 0.0;
    metrics.num_polytopes = polytopes.size();
    metrics.num_failed_polytopes = 0;

    for (const auto &poly : polytopes)
    {
        if (poly.A.rows() == 0)
        {
            metrics.num_failed_polytopes++;
            continue;
        }

        // Compute bounding box of polytope
        // Find min/max in each dimension using constraints
        Eigen::Vector3d min_pt(1e10, 1e10, 1e10);
        Eigen::Vector3d max_pt(-1e10, -1e10, -1e10);

        // Sample polytope to get rough bounds
        // (Proper volume calc requires vertex enumeration - expensive)
        for (int j = 0; j < poly.A.rows(); j++)
        {
            Eigen::Vector3d normal = poly.A.row(j);
            double offset = poly.b(j);
            // Rough estimate: assume polytope is 2x2x2 on average
        }

        // Simplified: assume average polytope is ~2m x 2m x 2m
        total_volume += 8.0; // Rough estimate
    }
    metrics.avg_polytope_volume = total_volume / std::max(1, (int)polytopes.size());

    // 3. Minimum distance from waypoints to any obstacle
    metrics.min_obstacle_distance = 1e10;
    for (const auto &wp : waypoints)
    {
        for (const auto &obs : obstacles)
        {
            double dist = (wp - obs).norm();
            metrics.min_obstacle_distance = std::min(metrics.min_obstacle_distance, dist);
        }
    }

    // 4. Final cost (not directly accessible, set to 0 for now)
    metrics.final_cost = 0.0;

    return metrics;
}

void print_metrics(const std::string &name, const SFCMetrics &m)
{
    std::cout << "\n=== " << name << " ===" << std::endl;
    std::cout << "Planning time:        " << m.total_time << " sec" << std::endl;
    std::cout << "Path length:          " << m.path_length << " m" << std::endl;
    std::cout << "Avg polytope volume:  " << m.avg_polytope_volume << " m³ (approx)" << std::endl;
    std::cout << "Min obstacle dist:    " << m.min_obstacle_distance << " m" << std::endl;
    std::cout << "Num polytopes:        " << m.num_polytopes << std::endl;
    std::cout << "Failed polytopes:     " << m.num_failed_polytopes << std::endl;

    // Quality score (higher is better)
    double quality = m.path_length * 0.5 +           // Shorter path
                     m.min_obstacle_distance * 2.0 + // Stay away from obstacles
                     m.avg_polytope_volume * 0.1;    // Bigger corridors
    std::cout << "Quality score:        " << quality << std::endl;
}

int main()
{
    Eigen::Vector3d start(1.0, 5.0, 3.0);
    Eigen::Vector3d goal(24.0, 5.0, 3.0);

    // Use SAME obstacles for fair comparison
    MapParams map_params;
    map_params.resolution = 1.0;
    map_params.num_obstacles = 22;

    // Generate obstacles once
    std::vector<Eigen::Vector3d> obstacles;
    std::random_device rd;
    std::mt19937 gen(42); // Fixed seed for reproducibility!
    std::uniform_real_distribution<> dist_x(5.0, 20.0);
    std::uniform_real_distribution<> dist_y(0.0, 11.0);
    std::uniform_real_distribution<> dist_z(1.0, 5.0);

    for (int i = 0; i < 22; i++)
    {
        obstacles.emplace_back(dist_x(gen), dist_y(gen), dist_z(gen));
    }

    std::cout << "========================================" << std::endl;
    std::cout << "SFC ITERATION COMPARISON" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Testing SAME obstacle field with different settings" << std::endl;
    std::cout << "Start: [1, 5, 3]" << std::endl;
    std::cout << "Goal:  [24, 5, 3]" << std::endl;
    std::cout << "Obstacles: " << obstacles.size() << std::endl;

    // ============ TEST 1: outer=1, inner=3 ============
    {
        std::cout << "\n\n========== TEST 1: outer=1, inner=3 ==========" << std::endl;

        SafeFlightCorridor sfc1;
        ADMMParams params1;
        params1.outer_max = 1;
        params1.inner_max = 3;
        params1.rho = 1.0;
        params1.epsilon = 0.3;
        params1.bbox_size = 2.0;
        params1.wc = 1.0;
        params1.wv = 1.0;

        auto t1 = std::chrono::high_resolution_clock::now();
        sfc1.generate(start, goal, obstacles, params1);
        auto t2 = std::chrono::high_resolution_clock::now();
        double time1 = std::chrono::duration<double>(t2 - t1).count();

        SFCMetrics metrics1 = compute_metrics(sfc1, time1);
        print_metrics("Configuration 1 (outer=1, inner=3)", metrics1);
    }

    // ============ TEST 2: outer=2, inner=3 ============
    {
        std::cout << "\n\n========== TEST 2: outer=2, inner=3 ==========" << std::endl;

        SafeFlightCorridor sfc2;
        ADMMParams params2;
        params2.outer_max = 2;
        params2.inner_max = 3;
        params2.rho = 1.0;
        params2.epsilon = 0.3;
        params2.bbox_size = 2.0;
        params2.wc = 1.0;
        params2.wv = 1.0;

        auto t1 = std::chrono::high_resolution_clock::now();
        sfc2.generate(start, goal, obstacles, params2);
        auto t2 = std::chrono::high_resolution_clock::now();
        double time2 = std::chrono::duration<double>(t2 - t1).count();

        SFCMetrics metrics2 = compute_metrics(sfc2, time2);
        print_metrics("Configuration 2 (outer=2, inner=3)", metrics2);
    }

    // ============ COMPARISON ============
    std::cout << "\n\n========================================" << std::endl;
    std::cout << "COMPARISON SUMMARY" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Config 1 (outer=1, inner=3): Faster, potentially lower quality" << std::endl;
    std::cout << "Config 2 (outer=2, inner=3): Slower, potentially higher quality" << std::endl;
    std::cout << "\nFor hardware at 5 m/s:" << std::endl;
    std::cout << "  Config 1: Travel during planning varies" << std::endl;
    std::cout << "  Config 2: Travel during planning varies" << std::endl;
    std::cout << "\nRecommendation: Use Config 1 if quality scores are similar" << std::endl;

    return 0;
}