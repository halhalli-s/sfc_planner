// safe_flight_corridor.h
#ifndef SAFE_FLIGHT_CORRIDOR_H
#define SAFE_FLIGHT_CORRIDOR_H

#include <vector>
#include <Eigen/Dense>
#include <array>

// ============ DATA STRUCTURES ============

struct Ellipsoid
{
    Eigen::Vector3d center;
    Eigen::Matrix3d L; // Lower triangular matrix from Cholesky
};

struct Polytope
{
    Eigen::MatrixXd A; // Face normals (num_faces x 3)
    Eigen::VectorXd b; // Face offsets (num_faces x 1)
    // Represents: Ax <= b
};

struct BoundingBox
{
    Eigen::MatrixXd A;                      // 6x3 for axis-aligned box
    Eigen::VectorXd b;                      // 6x1
    Eigen::Vector3d lb;                     // Lower bound
    Eigen::Vector3d ub;                     // Upper bound
    std::vector<Eigen::MatrixXd> obstacles; // Each obstacle is 3x8 (vertices)
};

struct ADMMParams
{
    int outer_max = 3;
    int inner_max = 5;
    double rho = 1.0;
    double epsilon = 0.3;   // Ellipsoid perpendicular radius
    double bbox_size = 2.0; // Bounding box half-width
    double wc = 1.0;        // Waypoint cost weight
    double wv = 1.0;        // Volume weight for ellipsoid optimization
};

struct MapParams
{
    double resolution = 0.5;
    int num_obstacles = 22;
    double map_x = 25.0;
    double map_y = 11.0;
    double map_z = 6.0;
    double obstacle_size = 1.0;
    double clearance = 0.3;
    std::array<double, 2> obstacle_range_x = {5.0, 20.0};
    std::array<double, 2> obstacle_range_y = {0.0, 11.0};
    std::array<double, 2> obstacle_range_z = {1.0, 5.0};
};

// ============ MAIN CLASS ============

class SafeFlightCorridor
{
public:
    SafeFlightCorridor();

    // Main entry point: generates entire SFC pipeline
    void generate(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal,
        const std::vector<Eigen::Vector3d> &obstacles,
        const ADMMParams &params);

    // Alternative: generate with map parameters (creates random obstacles)
    void generate(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal,
        const MapParams &map_params,
        const ADMMParams &admm_params);

    // Visualization
    void visualize();

    // Export to MeshLab
    void export_to_meshlab(const std::string &output_dir = "./meshlab_output");

    // Getters for results
    std::vector<Eigen::Vector3d> get_waypoints() const { return waypoints_; }
    std::vector<Ellipsoid> get_ellipsoids() const { return ellipsoids_; }
    std::vector<Polytope> get_polytopes() const { return polytopes_; }
    std::vector<Eigen::Vector3d> get_obstacles() const { return obstacles_; }
    bool is_converged() const { return converged_; }

private:
    // ============ STORED DATA ============
    Eigen::Vector3d start_;
    Eigen::Vector3d goal_;
    std::vector<Eigen::Vector3d> obstacles_;
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<Ellipsoid> ellipsoids_;
    std::vector<Polytope> polytopes_;
    std::vector<BoundingBox> bboxes_;
    Eigen::MatrixXd y_; // Lagrangian multipliers (2 x num_ellipsoids)
    bool converged_;

    // Map data (for visualization bounds)
    double map_x_, map_y_, map_z_;

    // ============ PIPELINE FUNCTIONS ============

    // Implemented in sfc_path.cpp
    void get_path(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal,
        const std::vector<Eigen::Vector3d> &obstacles,
        const MapParams &map_params);

    // Implemented in sfc_ellipsoids.cpp
    void init_ellipsoids(double epsilon);

    // Implemented in sfc_bounding_box.cpp
    void compute_bounding_boxes(double bbox_size);

    // Implemented in sfc_admm.cpp
    void inflate_polytopes();
    void optimize_waypoints(double rho, double wc);
    void optimize_ellipsoids(double rho, double wv);
    void update_lagrangian(double rho);

    // Implemented in sfc_visualize.cpp
    void plot_obstacles();
    void plot_waypoints();
    void plot_ellipsoids();
    void plot_polytopes();
};

#endif // SAFE_FLIGHT_CORRIDOR_H