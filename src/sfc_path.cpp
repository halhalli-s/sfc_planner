// sfc_path.cpp
// 3D A* path planning with waypoint simplification

#include "safe_flight_corridor.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <algorithm>

// Hash function for 3D grid coordinates
struct GridHash {
    std::size_t operator()(const Eigen::Vector3i& v) const {
        return std::hash<int>()(v.x()) ^ (std::hash<int>()(v.y()) << 1) ^ (std::hash<int>()(v.z()) << 2);
    }
};

// A* node
struct AStarNode {
    Eigen::Vector3i pos;
    double g, f;
    Eigen::Vector3i parent;
    
    bool operator>(const AStarNode& other) const {
        return f > other.f;
    }
};

// Check if point is collision free
bool is_collision_free(const Eigen::Vector3i& pos, 
                       const std::vector<std::vector<std::vector<int>>>& map,
                       int rows, int cols, int layers) {
    if (pos.x() < 0 || pos.x() >= cols || 
        pos.y() < 0 || pos.y() >= rows || 
        pos.z() < 0 || pos.z() >= layers) {
        return false;
    }
    return map[pos.y()][pos.x()][pos.z()] == 0;
}

// Check if line is collision free
bool is_line_collision_free(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                            const std::vector<std::vector<std::vector<int>>>& map,
                            double resolution, int rows, int cols, int layers) {
    double dist = (p2 - p1).norm();
    int num_checks = std::max(2, static_cast<int>(std::ceil(dist / (resolution * 0.5))));
    
    for (int i = 0; i <= num_checks; i++) {
        double t = static_cast<double>(i) / num_checks;
        Eigen::Vector3d pt = p1 + t * (p2 - p1);
        
        int gx = std::clamp(static_cast<int>(std::round(pt.x() / resolution)), 0, cols - 1);
        int gy = std::clamp(static_cast<int>(std::round(pt.y() / resolution)), 0, rows - 1);
        int gz = std::clamp(static_cast<int>(std::round(pt.z() / resolution)), 0, layers - 1);
        
        if (map[gy][gx][gz] == 1) {
            return false;
        }
    }
    return true;
}

// Inflate map for clearance
void inflate_map(std::vector<std::vector<std::vector<int>>>& map, int radius) {
    int rows = map.size();
    int cols = map[0].size();
    int layers = map[0][0].size();
    
    auto original = map;  // Copy
    
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            for (int z = 0; z < layers; z++) {
                if (original[y][x][z] == 1) {
                    // Inflate around this obstacle
                    for (int dy = -radius; dy <= radius; dy++) {
                        for (int dx = -radius; dx <= radius; dx++) {
                            for (int dz = -radius; dz <= radius; dz++) {
                                int ny = y + dy;
                                int nx = x + dx;
                                int nz = z + dz;
                                if (ny >= 0 && ny < rows && nx >= 0 && nx < cols && nz >= 0 && nz < layers) {
                                    map[ny][nx][nz] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void SafeFlightCorridor::get_path(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    const std::vector<Eigen::Vector3d>& obstacles,
    const MapParams& map_params)
{
    waypoints_.clear();
    
    double resolution = map_params.resolution;
    int cols = static_cast<int>(std::ceil(map_params.map_x / resolution));
    int rows = static_cast<int>(std::ceil(map_params.map_y / resolution));
    int layers = static_cast<int>(std::ceil(map_params.map_z / resolution));
    
    std::cout << "Grid size: " << cols << " x " << rows << " x " << layers << std::endl;
    
    // Create 3D occupancy grid
    std::vector<std::vector<std::vector<int>>> map(
        rows, std::vector<std::vector<int>>(cols, std::vector<int>(layers, 0)));
    
    // Fill obstacles
    double obs_half = map_params.obstacle_size / 2.0;
    for (const auto& obs : obstacles) {
        int x_min = std::max(0, static_cast<int>(std::floor((obs.x() - obs_half) / resolution)));
        int x_max = std::min(cols - 1, static_cast<int>(std::ceil((obs.x() + obs_half) / resolution)));
        int y_min = std::max(0, static_cast<int>(std::floor((obs.y() - obs_half) / resolution)));
        int y_max = std::min(rows - 1, static_cast<int>(std::ceil((obs.y() + obs_half) / resolution)));
        int z_min = std::max(0, static_cast<int>(std::floor((obs.z() - obs_half) / resolution)));
        int z_max = std::min(layers - 1, static_cast<int>(std::ceil((obs.z() + obs_half) / resolution)));
        
        for (int y = y_min; y <= y_max; y++) {
            for (int x = x_min; x <= x_max; x++) {
                for (int z = z_min; z <= z_max; z++) {
                    map[y][x][z] = 1;
                }
            }
        }
    }
    
    // Inflate map for clearance
    int clearance_cells = static_cast<int>(std::ceil(map_params.clearance / resolution));
    inflate_map(map, clearance_cells);
    
    // Convert start/goal to grid
    Eigen::Vector3i start_grid(
        std::clamp(static_cast<int>(std::round(start.x() / resolution)), 0, cols - 1),
        std::clamp(static_cast<int>(std::round(start.y() / resolution)), 0, rows - 1),
        std::clamp(static_cast<int>(std::round(start.z() / resolution)), 0, layers - 1)
    );
    Eigen::Vector3i goal_grid(
        std::clamp(static_cast<int>(std::round(goal.x() / resolution)), 0, cols - 1),
        std::clamp(static_cast<int>(std::round(goal.y() / resolution)), 0, rows - 1),
        std::clamp(static_cast<int>(std::round(goal.z() / resolution)), 0, layers - 1)
    );
    
    std::cout << "Start grid: [" << start_grid.x() << ", " << start_grid.y() << ", " << start_grid.z() << "]" << std::endl;
    std::cout << "Goal grid: [" << goal_grid.x() << ", " << goal_grid.y() << ", " << goal_grid.z() << "]" << std::endl;
    
    // 26-connected neighbors
    std::vector<Eigen::Vector3i> directions;
    std::vector<double> costs;
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                directions.emplace_back(dx, dy, dz);
                costs.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
            }
        }
    }
    
    // A* search
    auto heuristic = [&goal_grid](const Eigen::Vector3i& p) {
        return (p.cast<double>() - goal_grid.cast<double>()).norm();
    };
    
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_map<Eigen::Vector3i, double, GridHash> g_score;
    std::unordered_map<Eigen::Vector3i, Eigen::Vector3i, GridHash> came_from;
    std::unordered_map<Eigen::Vector3i, bool, GridHash> closed_set;
    
    AStarNode start_node;
    start_node.pos = start_grid;
    start_node.g = 0;
    start_node.f = heuristic(start_grid);
    start_node.parent = Eigen::Vector3i(-1, -1, -1);
    
    open_list.push(start_node);
    g_score[start_grid] = 0;
    
    bool found = false;
    
    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();
        
        if (closed_set[current.pos]) continue;
        closed_set[current.pos] = true;
        came_from[current.pos] = current.parent;
        
        if (current.pos == goal_grid) {
            found = true;
            break;
        }
        
        for (size_t d = 0; d < directions.size(); d++) {
            Eigen::Vector3i neighbor = current.pos + directions[d];
            
            if (!is_collision_free(neighbor, map, rows, cols, layers)) continue;
            if (closed_set[neighbor]) continue;
            
            double new_g = current.g + costs[d];
            
            if (g_score.find(neighbor) == g_score.end() || new_g < g_score[neighbor]) {
                g_score[neighbor] = new_g;
                
                AStarNode next;
                next.pos = neighbor;
                next.g = new_g;
                next.f = new_g + heuristic(neighbor);
                next.parent = current.pos;
                
                open_list.push(next);
            }
        }
    }
    
    if (!found) {
        std::cerr << "Error: No path found!" << std::endl;
        waypoints_.push_back(start);
        waypoints_.push_back(goal);
        return;
    }
    
    // Reconstruct path
    std::vector<Eigen::Vector3d> path;
    Eigen::Vector3i current = goal_grid;
    while (current.x() >= 0) {
        path.push_back(current.cast<double>() * resolution);
        current = came_from[current];
    }
    std::reverse(path.begin(), path.end());
    
    std::cout << "A* path found with " << path.size() << " points" << std::endl;
    
    // Simplify path - extract key waypoints
    std::vector<Eigen::Vector3d> key_waypoints;
    key_waypoints.push_back(path[0]);
    
    for (size_t i = 1; i < path.size(); i++) {
        if (!is_line_collision_free(key_waypoints.back(), path[i], map, resolution, rows, cols, layers)) {
            key_waypoints.push_back(path[i - 1]);
        }
    }
    key_waypoints.push_back(path.back());
    
    std::cout << "Simplified to " << key_waypoints.size() << " key waypoints" << std::endl;
    
    // Add auxiliary waypoints for long segments
    double max_segment_length = 2.0;
    waypoints_.push_back(key_waypoints[0]);
    
    for (size_t i = 0; i < key_waypoints.size() - 1; i++) {
        Eigen::Vector3d p1 = key_waypoints[i];
        Eigen::Vector3d p2 = key_waypoints[i + 1];
        double segment_length = (p2 - p1).norm();
        
        if (segment_length > max_segment_length) {
            int num_aux = static_cast<int>(std::ceil(segment_length / max_segment_length));
            for (int j = 1; j < num_aux; j++) {
                double t = static_cast<double>(j) / num_aux;
                waypoints_.push_back(p1 + t * (p2 - p1));
            }
        }
        waypoints_.push_back(p2);
    }
    
    std::cout << "Final waypoint count: " << waypoints_.size() << std::endl;
}