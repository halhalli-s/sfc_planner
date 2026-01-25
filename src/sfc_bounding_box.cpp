// sfc_bounding_box.cpp
// Compute 3D bounding boxes around each ellipsoid

#include "safe_flight_corridor.h"
#include <iostream>

void SafeFlightCorridor::compute_bounding_boxes(double bbox_size)
{
    bboxes_.clear();

    if (ellipsoids_.empty())
    {
        std::cerr << "Error: No ellipsoids to compute bounding boxes for" << std::endl;
        return;
    }

    // Obstacle half-size (assuming 1x1x1 cubes)
    double obs_half = 0.5;

    for (size_t i = 0; i < ellipsoids_.size(); i++)
    {
        Eigen::Vector3d center = ellipsoids_[i].center;

        // Bounding box limits
        Eigen::Vector3d lb = center.array() - bbox_size;
        Eigen::Vector3d ub = center.array() + bbox_size;

        // Halfspace representation: A*x <= b
        // 6 faces of the bounding box:
        //   -x <= -lb(0)  =>  x >= lb(0)
        //   -y <= -lb(1)  =>  y >= lb(1)
        //   -z <= -lb(2)  =>  z >= lb(2)
        //    x <= ub(0)
        //    y <= ub(1)
        //    z <= ub(2)
        Eigen::MatrixXd A_bounds(6, 3);
        A_bounds << -1, 0, 0,
            0, -1, 0,
            0, 0, -1,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1;

        Eigen::VectorXd b_bounds(6);
        b_bounds << -lb(0), -lb(1), -lb(2), ub(0), ub(1), ub(2);

        BoundingBox bbox;
        bbox.A = A_bounds;
        bbox.b = b_bounds;
        bbox.lb = lb;
        bbox.ub = ub;

        // Find obstacles within this bounding box
        for (const auto &obs_center : obstacles_)
        {
            // Check if obstacle is within bounding box (with margin for obstacle size)
            if (obs_center(0) >= lb(0) - obs_half && obs_center(0) <= ub(0) + obs_half &&
                obs_center(1) >= lb(1) - obs_half && obs_center(1) <= ub(1) + obs_half &&
                obs_center(2) >= lb(2) - obs_half && obs_center(2) <= ub(2) + obs_half)
            {

                // 8 vertices of the obstacle cube (3 x 8 matrix)
                double x = obs_center(0);
                double y = obs_center(1);
                double z = obs_center(2);

                Eigen::MatrixXd obs_verts(3, 8);
                obs_verts << x - obs_half, x + obs_half, x + obs_half, x - obs_half,
                    x - obs_half, x + obs_half, x + obs_half, x - obs_half,
                    y - obs_half, y - obs_half, y + obs_half, y + obs_half,
                    y - obs_half, y - obs_half, y + obs_half, y + obs_half,
                    z - obs_half, z - obs_half, z - obs_half, z - obs_half,
                    z + obs_half, z + obs_half, z + obs_half, z + obs_half;

                bbox.obstacles.push_back(obs_verts);
            }
        }

        bboxes_.push_back(bbox);
    }

    std::cout << "Computed " << bboxes_.size() << " bounding boxes" << std::endl;

    // Print obstacle counts per bounding box
    for (size_t i = 0; i < bboxes_.size(); i++)
    {
        if (bboxes_[i].obstacles.size() > 0)
        {
            std::cout << "  BBox " << i << ": " << bboxes_[i].obstacles.size() << " obstacles" << std::endl;
        }
    }
}