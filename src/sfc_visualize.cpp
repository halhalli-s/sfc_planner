// sfc_visualize.cpp
// Visualization using libcdd for polytope conversion and Python/Matplotlib for rendering

#include "safe_flight_corridor.h"
#include <Python.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>

// CDD library for H-rep to V-rep conversion
extern "C"
{
#include <cdd/setoper.h>
#include <cdd/cdd.h>
}

// Convert polytope from H-representation (Ax <= b) to V-representation (vertices)
std::vector<Eigen::Vector3d> polytope_to_vertices(const Eigen::MatrixXd &A, const Eigen::VectorXd &b)
{
    std::vector<Eigen::Vector3d> vertices;

    int num_faces = A.rows();

    // Create CDD matrix: [b | -A] for Ax <= b format
    // CDD uses format: b - Ax >= 0, which is equivalent to Ax <= b
    dd_MatrixPtr H = dd_CreateMatrix(num_faces, 4); // 4 = 1 + 3 dimensions
    H->representation = dd_Inequality;
    H->numbtype = dd_Real;

    for (int i = 0; i < num_faces; i++)
    {
        dd_set_d(H->matrix[i][0], b(i));     // b
        dd_set_d(H->matrix[i][1], -A(i, 0)); // -A[:,0]
        dd_set_d(H->matrix[i][2], -A(i, 1)); // -A[:,1]
        dd_set_d(H->matrix[i][3], -A(i, 2)); // -A[:,2]
    }

    dd_ErrorType err = dd_NoError;
    dd_PolyhedraPtr P = dd_DDMatrix2Poly(H, &err);

    if (err == dd_NoError && P != nullptr)
    {
        dd_MatrixPtr V = dd_CopyGenerators(P);

        for (int i = 0; i < V->rowsize; i++)
        {
            // First element is 1 for vertex, 0 for ray
            double type_val = dd_get_d(V->matrix[i][0]);

            if (type_val == 1.0)
            {
                double x = dd_get_d(V->matrix[i][1]);
                double y = dd_get_d(V->matrix[i][2]);
                double z = dd_get_d(V->matrix[i][3]);
                vertices.push_back(Eigen::Vector3d(x, y, z));
            }
        }

        dd_FreeMatrix(V);
        dd_FreePolyhedra(P);
    }

    dd_FreeMatrix(H);

    return vertices;
}

// Export polytope mesh to PLY format for MeshLab
void export_polytope_ply(const std::vector<Eigen::Vector3d> &vertices,
                         const std::string &filename,
                         int r = 255, int g = 165, int b = 0)
{
    if (vertices.size() < 4)
    {
        std::cout << "Not enough vertices to export: " << filename << std::endl;
        return;
    }

    std::ofstream ply(filename);
    if (!ply.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // Compute convex hull centroid for fan triangulation
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &v : vertices)
    {
        centroid += v;
    }
    centroid /= vertices.size();

    // Simple fan triangulation from centroid
    std::vector<std::array<int, 3>> faces;
    int n = vertices.size();
    for (int i = 0; i < n; i++)
    {
        faces.push_back({i, (i + 1) % n, n}); // triangle: vertex[i], vertex[i+1], centroid
    }

    // Write PLY header
    ply << "ply\n";
    ply << "format ascii 1.0\n";
    ply << "element vertex " << (vertices.size() + 1) << "\n";
    ply << "property float x\n";
    ply << "property float y\n";
    ply << "property float z\n";
    ply << "property uchar red\n";
    ply << "property uchar green\n";
    ply << "property uchar blue\n";
    ply << "element face " << faces.size() << "\n";
    ply << "property list uchar int vertex_indices\n";
    ply << "end_header\n";

    // Write vertices
    for (const auto &v : vertices)
    {
        ply << v(0) << " " << v(1) << " " << v(2) << " "
            << r << " " << g << " " << b << "\n";
    }
    // Write centroid
    ply << centroid(0) << " " << centroid(1) << " " << centroid(2) << " "
        << r << " " << g << " " << b << "\n";

    // Write faces
    for (const auto &f : faces)
    {
        ply << "3 " << f[0] << " " << f[1] << " " << f[2] << "\n";
    }

    ply.close();
    std::cout << "Exported: " << filename << " (" << vertices.size() << " vertices, "
              << faces.size() << " faces)" << std::endl;
}

// Export obstacles as PLY cubes
void export_obstacle_ply(const Eigen::Vector3d &center, const std::string &filename, double size = 1.0)
{
    std::ofstream ply(filename);
    if (!ply.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    double s = size / 2.0;

    // 8 vertices of cube
    std::vector<Eigen::Vector3d> verts = {
        center + Eigen::Vector3d(-s, -s, -s),
        center + Eigen::Vector3d(s, -s, -s),
        center + Eigen::Vector3d(s, s, -s),
        center + Eigen::Vector3d(-s, s, -s),
        center + Eigen::Vector3d(-s, -s, s),
        center + Eigen::Vector3d(s, -s, s),
        center + Eigen::Vector3d(s, s, s),
        center + Eigen::Vector3d(-s, s, s)};

    // 12 triangular faces (2 per cube face)
    std::vector<std::array<int, 3>> faces = {
        {0, 1, 2}, {0, 2, 3}, // bottom
        {4, 6, 5},
        {4, 7, 6}, // top
        {0, 5, 1},
        {0, 4, 5}, // front
        {2, 7, 3},
        {2, 6, 7}, // back
        {0, 3, 7},
        {0, 7, 4}, // left
        {1, 6, 2},
        {1, 5, 6} // right
    };

    // Write PLY
    ply << "ply\nformat ascii 1.0\n";
    ply << "element vertex 8\n";
    ply << "property float x\nproperty float y\nproperty float z\n";
    ply << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    ply << "element face 12\n";
    ply << "property list uchar int vertex_indices\n";
    ply << "end_header\n";

    for (const auto &v : verts)
    {
        ply << v(0) << " " << v(1) << " " << v(2) << " 128 128 128\n";
    }
    for (const auto &f : faces)
    {
        ply << "3 " << f[0] << " " << f[1] << " " << f[2] << "\n";
    }

    ply.close();
}

// Export all scene to PLY files
void SafeFlightCorridor::export_to_meshlab(const std::string &output_dir)
{
    std::cout << "\n=== Exporting to MeshLab PLY format ===" << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;

    // Create output directory if it doesn't exist
    system(("mkdir -p " + output_dir).c_str());

    // Export obstacles
    std::cout << "\nExporting " << obstacles_.size() << " obstacles..." << std::endl;
    for (size_t i = 0; i < obstacles_.size(); i++)
    {
        std::stringstream filename;
        filename << output_dir << "/obstacle_" << std::setfill('0') << std::setw(3) << i << ".ply";
        export_obstacle_ply(obstacles_[i], filename.str());
    }

    // Export polytopes with different colors
    std::cout << "\nExporting " << polytopes_.size() << " polytopes..." << std::endl;
    std::vector<std::array<int, 3>> colors = {
        {255, 165, 0}, // orange
        {255, 0, 255}, // magenta
        {0, 255, 255}, // cyan
        {255, 255, 0}, // yellow
        {0, 255, 0},   // green
        {255, 0, 0},   // red
        {0, 0, 255},   // blue
        {255, 128, 0}, // dark orange
        {128, 0, 255}, // purple
        {0, 128, 255}  // light blue
    };

    for (size_t i = 0; i < polytopes_.size(); i++)
    {
        std::vector<Eigen::Vector3d> verts = polytope_to_vertices(polytopes_[i].A, polytopes_[i].b);

        if (verts.size() >= 4)
        {
            std::stringstream filename;
            filename << output_dir << "/polytope_" << std::setfill('0') << std::setw(3) << i << ".ply";

            auto color = colors[i % colors.size()];
            export_polytope_ply(verts, filename.str(), color[0], color[1], color[2]);
        }
        else
        {
            std::cout << "Warning: Polytope " << i << " has only " << verts.size()
                      << " vertices, skipping" << std::endl;
        }
    }

    // Export waypoints as small spheres (simplified as point cloud)
    std::ofstream waypoints_ply(output_dir + "/waypoints.ply");
    waypoints_ply << "ply\nformat ascii 1.0\n";
    waypoints_ply << "element vertex " << waypoints_.size() << "\n";
    waypoints_ply << "property float x\nproperty float y\nproperty float z\n";
    waypoints_ply << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    waypoints_ply << "end_header\n";
    for (const auto &wp : waypoints_)
    {
        waypoints_ply << wp(0) << " " << wp(1) << " " << wp(2) << " 0 0 0\n";
    }
    waypoints_ply.close();
    std::cout << "Exported: " << output_dir << "/waypoints.ply" << std::endl;

    std::cout << "\n✓ Export complete! Open in MeshLab:" << std::endl;
    std::cout << "  cd " << output_dir << " && meshlab *.ply" << std::endl;
    std::cout << "\nOr load all files individually in MeshLab to toggle layers.\n"
              << std::endl;
}

void SafeFlightCorridor::visualize()
{
    // Initialize Python
    if (!Py_IsInitialized())
    {
        Py_Initialize();
    }

    // Build Python script as string
    std::stringstream ss;

    ss << "import matplotlib.pyplot as plt\n";
    ss << "from mpl_toolkits.mplot3d import Axes3D\n";
    ss << "from mpl_toolkits.mplot3d.art3d import Poly3DCollection\n";
    ss << "import numpy as np\n";
    ss << "from scipy.spatial import ConvexHull\n";
    ss << "\n";
    ss << "fig = plt.figure(figsize=(14, 10))\n";
    ss << "ax = fig.add_subplot(111, projection='3d')\n";
    ss << "\n";

    // ============ HELPER FUNCTIONS ============
    ss << "# Helper function to draw a 3D cube\n";
    ss << "def draw_cube(ax, center, size=1.0, color='gray', alpha=0.6):\n";
    ss << "    s = size / 2\n";
    ss << "    x, y, z = center\n";
    ss << "    vertices = [\n";
    ss << "        [[x-s, y-s, z-s], [x+s, y-s, z-s], [x+s, y+s, z-s], [x-s, y+s, z-s]],\n";
    ss << "        [[x-s, y-s, z+s], [x+s, y-s, z+s], [x+s, y+s, z+s], [x-s, y+s, z+s]],\n";
    ss << "        [[x-s, y-s, z-s], [x+s, y-s, z-s], [x+s, y-s, z+s], [x-s, y-s, z+s]],\n";
    ss << "        [[x-s, y+s, z-s], [x+s, y+s, z-s], [x+s, y+s, z+s], [x-s, y+s, z+s]],\n";
    ss << "        [[x-s, y-s, z-s], [x-s, y+s, z-s], [x-s, y+s, z+s], [x-s, y-s, z+s]],\n";
    ss << "        [[x+s, y-s, z-s], [x+s, y+s, z-s], [x+s, y+s, z+s], [x+s, y-s, z+s]]\n";
    ss << "    ]\n";
    ss << "    ax.add_collection3d(Poly3DCollection(vertices, facecolors=color, linewidths=0.5, edgecolors='darkgray', alpha=alpha))\n";
    ss << "\n";

    ss << "# Helper function to draw ellipsoid\n";
    ss << "def draw_ellipsoid(ax, center, L, color='cyan', alpha=0.3):\n";
    ss << "    u = np.linspace(0, 2 * np.pi, 30)\n";
    ss << "    v = np.linspace(0, np.pi, 20)\n";
    ss << "    x = np.outer(np.cos(u), np.sin(v))\n";
    ss << "    y = np.outer(np.sin(u), np.sin(v))\n";
    ss << "    z = np.outer(np.ones(np.size(u)), np.cos(v))\n";
    ss << "    for i in range(len(x)):\n";
    ss << "        for j in range(len(x[0])):\n";
    ss << "            pt = L @ np.array([x[i,j], y[i,j], z[i,j]]) + center\n";
    ss << "            x[i,j], y[i,j], z[i,j] = pt\n";
    ss << "    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)\n";
    ss << "\n";

    ss << "# Helper function to draw polytope from vertices\n";
    ss << "def draw_polytope(ax, vertices, color='orange', alpha=0.2):\n";
    ss << "    if len(vertices) < 4:\n";
    ss << "        return\n";
    ss << "    try:\n";
    ss << "        hull = ConvexHull(vertices)\n";
    ss << "        for simplex in hull.simplices:\n";
    ss << "            triangle = [vertices[s] for s in simplex]\n";
    ss << "            ax.add_collection3d(Poly3DCollection([triangle], facecolors=color, linewidths=0.5, edgecolors=color, alpha=alpha))\n";
    ss << "    except Exception as e:\n";
    ss << "        print(f'Could not draw polytope: {e}')\n";
    ss << "\n";

    // ============ OBSTACLES ============
    ss << "# Draw obstacles\n";
    for (size_t i = 0; i < obstacles_.size(); i++)
    {
        ss << "draw_cube(ax, [" << obstacles_[i](0) << ", " << obstacles_[i](1) << ", " << obstacles_[i](2) << "], size=1.0, color='gray', alpha=0.6)\n";
    }
    ss << "\n";

    // ============ POLYTOPES (using libcdd conversion) ============
    ss << "# Draw polytopes\n";
    ss << "polytope_colors = plt.cm.tab10(np.linspace(0, 1, " << polytopes_.size() << "))\n";

    for (size_t i = 0; i < polytopes_.size(); i++)
    {
        // Convert H-rep to V-rep using libcdd
        std::vector<Eigen::Vector3d> verts = polytope_to_vertices(polytopes_[i].A, polytopes_[i].b);

        if (verts.size() >= 4)
        {
            ss << "poly_verts_" << i << " = np.array([";
            for (size_t j = 0; j < verts.size(); j++)
            {
                ss << "[" << verts[j](0) << ", " << verts[j](1) << ", " << verts[j](2) << "]";
                if (j < verts.size() - 1)
                    ss << ", ";
            }
            ss << "])\n";
            ss << "draw_polytope(ax, poly_verts_" << i << ", color=polytope_colors[" << i << "], alpha=0.15)\n";
        }
        else
        {
            std::cout << "Warning: Polytope " << i << " has only " << verts.size() << " vertices, skipping visualization" << std::endl;
        }
    }
    ss << "\n";

    // ============ ELLIPSOIDS ============
    ss << "# Draw ellipsoids\n";
    for (size_t i = 0; i < ellipsoids_.size(); i++)
    {
        // Skip invalid ellipsoids (check for NaN or very small values)
        if (std::isnan(ellipsoids_[i].center(0)) || std::abs(ellipsoids_[i].center(0)) < 1e-100)
        {
            std::cout << "Warning: Ellipsoid " << i << " has invalid center, skipping" << std::endl;
            continue;
        }

        ss << "center_" << i << " = np.array([" << ellipsoids_[i].center(0) << ", "
           << ellipsoids_[i].center(1) << ", " << ellipsoids_[i].center(2) << "])\n";
        ss << "L_" << i << " = np.array([";
        ss << "[" << ellipsoids_[i].L(0, 0) << ", " << ellipsoids_[i].L(0, 1) << ", " << ellipsoids_[i].L(0, 2) << "], ";
        ss << "[" << ellipsoids_[i].L(1, 0) << ", " << ellipsoids_[i].L(1, 1) << ", " << ellipsoids_[i].L(1, 2) << "], ";
        ss << "[" << ellipsoids_[i].L(2, 0) << ", " << ellipsoids_[i].L(2, 1) << ", " << ellipsoids_[i].L(2, 2) << "]])\n";
        ss << "draw_ellipsoid(ax, center_" << i << ", L_" << i << ", color='cyan', alpha=0.25)\n";
    }
    ss << "\n";

    // ============ WAYPOINTS ============
    ss << "# Draw waypoints\n";
    ss << "wp_x = [";
    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        // Skip invalid waypoints
        if (std::isnan(waypoints_[i](0)) || std::abs(waypoints_[i](0)) < 1e-100)
        {
            ss << start_(0); // Use start as fallback for invalid first waypoint
        }
        else
        {
            ss << waypoints_[i](0);
        }
        if (i < waypoints_.size() - 1)
            ss << ", ";
    }
    ss << "]\n";

    ss << "wp_y = [";
    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        if (std::isnan(waypoints_[i](1)) || std::abs(waypoints_[i](1)) < 1e-100)
        {
            ss << start_(1);
        }
        else
        {
            ss << waypoints_[i](1);
        }
        if (i < waypoints_.size() - 1)
            ss << ", ";
    }
    ss << "]\n";

    ss << "wp_z = [";
    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        if (std::isnan(waypoints_[i](2)))
        {
            ss << start_(2);
        }
        else
        {
            ss << waypoints_[i](2);
        }
        if (i < waypoints_.size() - 1)
            ss << ", ";
    }
    ss << "]\n";

    ss << "ax.plot(wp_x, wp_y, wp_z, 'k-', linewidth=2.5, label='Optimized Path', zorder=10)\n";
    ss << "ax.scatter(wp_x, wp_y, wp_z, c='black', s=60, marker='o', zorder=11)\n";
    ss << "\n";

    // ============ START AND GOAL ============
    ss << "# Draw start and goal\n";
    ss << "ax.scatter([" << start_(0) << "], [" << start_(1) << "], [" << start_(2) << "], c='green', s=250, marker='o', label='Start', zorder=12)\n";
    ss << "ax.scatter([" << goal_(0) << "], [" << goal_(1) << "], [" << goal_(2) << "], c='red', s=300, marker='*', label='Goal', zorder=12)\n";
    ss << "\n";

    // ============ FORMATTING ============
    ss << "# Formatting\n";
    ss << "ax.set_xlabel('X (m)', fontsize=12)\n";
    ss << "ax.set_ylabel('Y (m)', fontsize=12)\n";
    ss << "ax.set_zlabel('Z (m)', fontsize=12)\n";
    ss << "ax.set_xlim(0, " << map_x_ << ")\n";
    ss << "ax.set_ylim(0, " << map_y_ << ")\n";
    ss << "ax.set_zlim(0, " << map_z_ << ")\n";
    ss << "ax.set_title('3D Safe Flight Corridor - ADMM Optimization Result', fontsize=14)\n";
    ss << "ax.legend(loc='upper left')\n";
    ss << "ax.set_box_aspect([" << map_x_ << ", " << map_y_ << ", " << map_z_ << "])\n";
    ss << "plt.tight_layout()\n";
    ss << "plt.show()\n";

    // Execute Python script
    std::string script = ss.str();

    std::cout << "Launching 3D visualization with libcdd polytope conversion..." << std::endl;

    int result = PyRun_SimpleString(script.c_str());
    if (result != 0)
    {
        std::cerr << "Error executing Python visualization script" << std::endl;
        PyErr_Print();
    }

    // After visualization, print polytope analysis
    plot_polytopes();
}

// Individual plot functions for debugging
void SafeFlightCorridor::plot_obstacles()
{
    std::cout << "Obstacles: " << obstacles_.size() << std::endl;
    for (size_t i = 0; i < obstacles_.size(); i++)
    {
        std::cout << "  " << i << ": [" << obstacles_[i](0) << ", "
                  << obstacles_[i](1) << ", " << obstacles_[i](2) << "]" << std::endl;
    }
}

void SafeFlightCorridor::plot_waypoints()
{
    std::cout << "Waypoints: " << waypoints_.size() << std::endl;
    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        std::cout << "  " << i << ": [" << waypoints_[i](0) << ", "
                  << waypoints_[i](1) << ", " << waypoints_[i](2) << "]" << std::endl;
    }
}

void SafeFlightCorridor::plot_ellipsoids()
{
    std::cout << "Ellipsoids: " << ellipsoids_.size() << std::endl;
    for (size_t i = 0; i < ellipsoids_.size(); i++)
    {
        std::cout << "  " << i << ": center [" << ellipsoids_[i].center(0) << ", "
                  << ellipsoids_[i].center(1) << ", " << ellipsoids_[i].center(2) << "]" << std::endl;
    }
}

void SafeFlightCorridor::plot_polytopes()
{
    std::cout << "\n=== POLYTOPE ANALYSIS ===" << std::endl;
    std::cout << "Polytopes: " << polytopes_.size() << std::endl;

    int total_violations = 0;

    for (size_t i = 0; i < polytopes_.size(); i++)
    {
        std::vector<Eigen::Vector3d> verts = polytope_to_vertices(polytopes_[i].A, polytopes_[i].b);
        std::cout << "\nPolytope " << i << ": " << polytopes_[i].A.rows() << " faces, " << verts.size() << " vertices" << std::endl;

        // Print bounding box
        if (verts.size() > 0)
        {
            Eigen::Vector3d min_pt = verts[0];
            Eigen::Vector3d max_pt = verts[0];
            for (const auto &v : verts)
            {
                min_pt = min_pt.cwiseMin(v);
                max_pt = max_pt.cwiseMax(v);
            }
            std::cout << "  Bounds: [" << min_pt.transpose() << "] to [" << max_pt.transpose() << "]" << std::endl;
        }

        // Check if any obstacles are inside this polytope
        bool has_obstacle = false;
        for (size_t j = 0; j < obstacles_.size(); j++)
        {
            // Check if obstacle satisfies all halfspace constraints: A*x <= b
            bool inside = true;
            for (int k = 0; k < polytopes_[i].A.rows(); k++)
            {
                double val = polytopes_[i].A.row(k).dot(obstacles_[j]);
                if (val > polytopes_[i].b(k) + 1e-6) // Small tolerance
                {
                    inside = false;
                    break;
                }
            }
            if (inside)
            {
                has_obstacle = true;
                total_violations++;
                std::cout << "  ❌ WARNING: Obstacle " << j << " at ["
                          << obstacles_[j](0) << ", " << obstacles_[j](1) << ", " << obstacles_[j](2)
                          << "] is INSIDE polytope " << i << "!" << std::endl;
            }
        }
        if (!has_obstacle)
        {
            std::cout << "  ✓ Obstacle-free" << std::endl;
        }
    }

    std::cout << "\n=== SUMMARY ===" << std::endl;
    std::cout << "Total polytopes: " << polytopes_.size() << std::endl;
    std::cout << "Polytopes with obstacles: " << total_violations << std::endl;
    if (total_violations == 0)
    {
        std::cout << "✓ ALL POLYTOPES ARE OBSTACLE-FREE!" << std::endl;
    }
    else
    {
        std::cout << "❌ OPTIMIZATION FAILED - Some polytopes contain obstacles!" << std::endl;
    }
}
