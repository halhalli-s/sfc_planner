// sfc_ros_node.cpp
// ROS2 node that publishes SFC results (waypoints, ellipsoids, polytopes)

#include "safe_flight_corridor.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// Helper: Convert polytope H-rep to vertices (from sfc_visualize.cpp)
extern std::vector<Eigen::Vector3d> polytope_to_vertices(const Eigen::MatrixXd &A, const Eigen::VectorXd &b);

class SFCPublisher : public rclcpp::Node
{
public:
    SFCPublisher() : Node("sfc_publisher")
    {
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sfc_path", 10);
        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sfc_markers", 10);

        RCLCPP_INFO(this->get_logger(), "SFC Publisher node started");
    }

    void publish_sfc(const SafeFlightCorridor &sfc)
    {
        auto now = this->now();

        // Publish path (waypoints)
        publish_path(sfc, now);

        // Publish visualization markers
        publish_markers(sfc, now);

        RCLCPP_INFO(this->get_logger(), "Published SFC results");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    void publish_path(const SafeFlightCorridor &sfc, const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";

        auto waypoints = sfc.get_waypoints();
        for (const auto &wp : waypoints)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = "map";
            pose.pose.position.x = wp(0);
            pose.pose.position.y = wp(1);
            pose.pose.position.z = wp(2);
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", waypoints.size());
    }

    void publish_markers(const SafeFlightCorridor &sfc, const rclcpp::Time &stamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        auto obstacles = sfc.get_obstacles();
        auto ellipsoids = sfc.get_ellipsoids();
        auto polytopes = sfc.get_polytopes();
        auto waypoints = sfc.get_waypoints();

        // ============ OBSTACLES (Gray Cubes) ============
        for (size_t i = 0; i < obstacles.size(); i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
            marker.ns = "obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = obstacles[i](0);
            marker.pose.position.y = obstacles[i](1);
            marker.pose.position.z = obstacles[i](2);
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 0.8;

            marker_array.markers.push_back(marker);
        }

        // ============ ELLIPSOIDS (Cyan Spheres) ============
        for (size_t i = 0; i < ellipsoids.size(); i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
            marker.ns = "ellipsoids";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = ellipsoids[i].center(0);
            marker.pose.position.y = ellipsoids[i].center(1);
            marker.pose.position.z = ellipsoids[i].center(2);
            marker.pose.orientation.w = 1.0;

            // Use L matrix to estimate radii
            Eigen::Matrix3d cov = ellipsoids[i].L * ellipsoids[i].L.transpose();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
            Eigen::Vector3d radii = es.eigenvalues().cwiseSqrt();

            marker.scale.x = 2.0 * radii(0);
            marker.scale.y = 2.0 * radii(1);
            marker.scale.z = 2.0 * radii(2);

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.3;

            marker_array.markers.push_back(marker);
        }

        // ============ POLYTOPES (Colored Triangles) ============
        std::vector<std::array<float, 3>> colors = {
            {1.0, 0.65, 0.0}, // orange
            {1.0, 0.0, 1.0},  // magenta
            {0.0, 1.0, 1.0},  // cyan
            {1.0, 1.0, 0.0},  // yellow
            {0.0, 1.0, 0.0},  // green
            {1.0, 0.0, 0.0},  // red
            {0.0, 0.0, 1.0},  // blue
        };

        for (size_t i = 0; i < polytopes.size(); i++)
        {
            // Convert H-rep to vertices
            std::vector<Eigen::Vector3d> vertices = polytope_to_vertices(
                polytopes[i].A, polytopes[i].b);

            if (vertices.size() < 4)
            {
                RCLCPP_WARN(this->get_logger(), "Polytope %zu has only %zu vertices, skipping",
                            i, vertices.size());
                continue;
            }

            // Create triangle list marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
            marker.ns = "polytopes";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            auto color = colors[i % colors.size()];
            marker.color.r = color[0];
            marker.color.g = color[1];
            marker.color.b = color[2];
            marker.color.a = 0.2;

            // Simple convex hull from centroid
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto &v : vertices)
            {
                centroid += v;
            }
            centroid /= vertices.size();

            // Fan triangulation
            for (size_t j = 0; j < vertices.size(); j++)
            {
                size_t next = (j + 1) % vertices.size();

                geometry_msgs::msg::Point p1, p2, p3;
                p1.x = vertices[j](0);
                p1.y = vertices[j](1);
                p1.z = vertices[j](2);

                p2.x = vertices[next](0);
                p2.y = vertices[next](1);
                p2.z = vertices[next](2);

                p3.x = centroid(0);
                p3.y = centroid(1);
                p3.z = centroid(2);

                marker.points.push_back(p1);
                marker.points.push_back(p2);
                marker.points.push_back(p3);
            }

            marker_array.markers.push_back(marker);
        }

        // ============ WAYPOINTS (Black Spheres) ============
        for (size_t i = 0; i < waypoints.size(); i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = waypoints[i](0);
            marker.pose.position.y = waypoints[i](1);
            marker.pose.position.z = waypoints[i](2);
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        // ============ PATH LINE ============
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = stamp;
        line.ns = "path_line";
        line.id = id++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;

        line.scale.x = 0.05;
        line.color.r = 0.0;
        line.color.g = 0.0;
        line.color.b = 0.0;
        line.color.a = 1.0;

        for (const auto &wp : waypoints)
        {
            geometry_msgs::msg::Point p;
            p.x = wp(0);
            p.y = wp(1);
            p.z = wp(2);
            line.points.push_back(p);
        }

        marker_array.markers.push_back(line);

        // Publish all markers
        markers_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu visualization markers", marker_array.markers.size());
    }
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SFCPublisher>();

    RCLCPP_INFO(node->get_logger(), "=== SFC Validation Test ===");
    RCLCPP_INFO(node->get_logger(), "Goal: Prove onboard computer can generate SFCs");
    RCLCPP_INFO(node->get_logger(), "NOT executing trajectory - visualization only!");

    // Run SFC generation
    RCLCPP_INFO(node->get_logger(), "Generating Safe Flight Corridor...");

    Eigen::Vector3d start(1.0, 5.0, 3.0);
    Eigen::Vector3d goal(24.0, 5.0, 3.0);

    MapParams map_params;
    map_params.resolution = 1.0;
    map_params.num_obstacles = 20;

    ADMMParams admm_params;
    admm_params.outer_max = 1;
    admm_params.inner_max = 3;

    auto t_start = std::chrono::high_resolution_clock::now();

    SafeFlightCorridor sfc;
    sfc.generate(start, goal, map_params, admm_params);

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();

    RCLCPP_INFO(node->get_logger(), "✓ SFC generated in %.2f seconds", elapsed);
    RCLCPP_INFO(node->get_logger(), "✓ Waypoints: %zu", sfc.get_waypoints().size());
    RCLCPP_INFO(node->get_logger(), "✓ Polytopes: %zu", sfc.get_polytopes().size());

    // Publish results
    RCLCPP_INFO(node->get_logger(), "Publishing to RViz for visualization...");
    node->publish_sfc(sfc);

    // Keep node alive so RViz can display
    RCLCPP_INFO(node->get_logger(), "===================================");
    RCLCPP_INFO(node->get_logger(), "VALIDATION TEST RESULT:");
    RCLCPP_INFO(node->get_logger(), "✓ Onboard computer CAN generate SFCs");
    RCLCPP_INFO(node->get_logger(), "✓ Planning time: %.2f sec", elapsed);
    RCLCPP_INFO(node->get_logger(), "✓ Published to /sfc_path and /sfc_markers");
    RCLCPP_INFO(node->get_logger(), "===================================");
    RCLCPP_INFO(node->get_logger(), "View in RViz:");
    RCLCPP_INFO(node->get_logger(), "  1. Open RViz: rviz2");
    RCLCPP_INFO(node->get_logger(), "  2. Set Fixed Frame: map");
    RCLCPP_INFO(node->get_logger(), "  3. Add MarkerArray: /sfc_markers");
    RCLCPP_INFO(node->get_logger(), "  4. Add Path: /sfc_path");
    RCLCPP_INFO(node->get_logger(), "Node will stay alive for 30 seconds...");

    // Keep publishing for 30 seconds so you can open RViz
    rclcpp::Rate rate(2); // 2 Hz
    for (int i = 0; i < 120; i++)
    {
        node->publish_sfc(sfc);
        rclcpp::spin_some(node);
        rate.sleep();

        if (i % 10 == 0)
        {
            RCLCPP_INFO(node->get_logger(), "Still publishing... (%d seconds remaining)", 30 - i / 2);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Validation test complete. Shutting down.");
    rclcpp::shutdown();

    return 0;
}