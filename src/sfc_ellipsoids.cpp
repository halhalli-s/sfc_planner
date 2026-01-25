// sfc_ellipsoids.cpp
// Initialize ellipsoids between consecutive waypoints (eq 9 from paper)

#include "safe_flight_corridor.h"
#include <cmath>
#include <iostream>

void SafeFlightCorridor::init_ellipsoids(double epsilon)
{
    ellipsoids_.clear();

    if (waypoints_.size() < 2)
    {
        std::cerr << "Error: Need at least 2 waypoints to initialize ellipsoids" << std::endl;
        return;
    }

    size_t num_ellipsoids = waypoints_.size() - 1;
    ellipsoids_.reserve(num_ellipsoids);

    for (size_t i = 0; i < num_ellipsoids; i++)
    {
        Eigen::Vector3d p_i = waypoints_[i];
        Eigen::Vector3d p_next = waypoints_[i + 1];

        // Center is midpoint between consecutive waypoints
        Eigen::Vector3d center = (p_i + p_next) / 2.0;

        // Direction vector and distance
        Eigen::Vector3d diff = p_next - p_i;
        double dist = diff.norm();

        Eigen::Matrix3d L;

        // Handle edge case: coincident waypoints
        if (dist < 1e-10)
        {
            // Degenerate case: use small sphere
            L = epsilon * Eigen::Matrix3d::Identity();
        }
        else
        {
            // Unit vector along the segment
            Eigen::Vector3d mu = diff / dist;

            // Build L*L^T matrix (eq 9 from paper)
            // - Stretched along mu by dist/2
            // - Circular cross-section with radius epsilon perpendicular to mu
            Eigen::Matrix3d mu_muT = mu * mu.transpose();
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

            Eigen::Matrix3d L_LT = (mu_muT * (dist * dist / 4.0)) +
                                   ((I - mu_muT) * (epsilon * epsilon));

            // Cholesky decomposition to get L (lower triangular)
            Eigen::LLT<Eigen::Matrix3d> llt(L_LT);

            if (llt.info() != Eigen::Success)
            {
                // Fallback if Cholesky fails
                std::cerr << "Warning: Cholesky failed for ellipsoid " << i << ", using sphere" << std::endl;
                L = epsilon * Eigen::Matrix3d::Identity();
            }
            else
            {
                L = llt.matrixL();
            }
        }

        // Create and store ellipsoid
        Ellipsoid ellipsoid;
        ellipsoid.center = center;
        ellipsoid.L = L;
        ellipsoids_.push_back(ellipsoid);
    }

    std::cout << "Initialized " << ellipsoids_.size() << " ellipsoids" << std::endl;
}

