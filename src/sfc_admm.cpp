// sfc_admm.cpp
// ADMM optimization: inflate_polytopes, optimize_waypoints, optimize_ellipsoids, update_lagrangian

#include "safe_flight_corridor.h"
#include <casadi/casadi.hpp>
#include <iris/iris.h>
#include <iostream>

using namespace casadi;

// ============ INFLATE POLYTOPES USING IRIS ============
void SafeFlightCorridor::inflate_polytopes()
{
    polytopes_.clear();

    for (size_t i = 0; i < ellipsoids_.size(); i++)
    {
        Polytope poly;

        try
        {
            // Setup IRIS problem
            iris::IRISProblem problem(3); // 3D
            problem.setSeedPoint(ellipsoids_[i].center);

            // Set bounding box
            iris::Polyhedron bounds(bboxes_[i].A, bboxes_[i].b);
            problem.setBounds(bounds);

            // Add obstacles
            for (const auto &obs_verts : bboxes_[i].obstacles)
            {
                problem.addObstacle(obs_verts);
            }

            // IRIS options
            iris::IRISOptions options;
            options.require_containment = true;
            options.error_on_infeasible_start = false;

            // Inflate region
            iris::IRISRegion region = iris::inflate_region(problem, options);

            poly.A = region.getPolyhedron().getA();
            poly.b = region.getPolyhedron().getB();
        }
        catch (const std::exception &e)
        {
            std::cerr << "IRIS failed for ellipsoid " << i << ": " << e.what() << std::endl;
            // Fallback to bounding box
            poly.A = bboxes_[i].A;
            poly.b = bboxes_[i].b;
        }

        polytopes_.push_back(poly);
    }
}

// ============ OPTIMIZE WAYPOINTS (eq 12a, 12b, 12c) ============
void SafeFlightCorridor::optimize_waypoints(double rho, double wc)
{
    int n = waypoints_.size();
    int M = ellipsoids_.size();

    // Decision variables: 3*n (flattened waypoints)
    SX p = SX::sym("p", 3 * n);

    // Helper to get waypoint i
    auto get_waypoint = [&p, n](int i) -> SX
    {
        return p(Slice(i * 3, i * 3 + 3));
    };

    // Cost function
    SX cost = 0;

    // (12a) Path length cost
    for (int i = 0; i < M; i++)
    {
        SX p_prev = get_waypoint(i);
        SX p_curr = get_waypoint(i + 1);
        SX diff = p_curr - p_prev;
        cost += wc * dot(diff, diff);
    }

    // (12b) ADMM penalty
    for (int i = 0; i < M; i++)
    {
        Eigen::Matrix3d L = ellipsoids_[i].L;
        Eigen::Vector3d d = ellipsoids_[i].center;
        Eigen::Matrix3d L_inv_T = L.inverse().transpose();

        // Convert to CasADi SX (not DM!)
        SX L_inv_T_sx = SX::zeros(3, 3);
        SX d_sx = SX::zeros(3, 1);
        for (int r = 0; r < 3; r++)
        {
            d_sx(r) = d(r);
            for (int c = 0; c < 3; c++)
            {
                L_inv_T_sx(r, c) = L_inv_T(r, c);
            }
        }

        SX p_prev = get_waypoint(i);
        SX p_curr = get_waypoint(i + 1);

        SX h0_curr = norm_2(mtimes(L_inv_T_sx, p_curr - d_sx)) - 1;
        SX h0_prev = norm_2(mtimes(L_inv_T_sx, p_prev - d_sx)) - 1;

        SX penalty_curr = fmax(0, h0_curr) * fmax(0, h0_curr);
        SX penalty_prev = fmax(0, h0_prev) * fmax(0, h0_prev);

        double y_curr = y_(0, i);
        double y_prev = y_(1, i);

        cost += (rho / 2.0) * ((penalty_curr + y_curr) * (penalty_curr + y_curr) +
                               (penalty_prev + y_prev) * (penalty_prev + y_prev));
    }

    // Constraints
    std::vector<SX> g_vec;

    // (12c) p_i in P_i ∩ P_{i+1}
    for (int i = 0; i < M - 1; i++)
    {
        SX p_i = get_waypoint(i + 1);

        // In P_i
        Eigen::MatrixXd A_i = polytopes_[i].A;
        Eigen::VectorXd b_i = polytopes_[i].b;
        for (int j = 0; j < A_i.rows(); j++)
        {
            SX constraint = A_i(j, 0) * p_i(0) + A_i(j, 1) * p_i(1) + A_i(j, 2) * p_i(2) - b_i(j);
            g_vec.push_back(constraint);
        }

        // In P_{i+1}
        Eigen::MatrixXd A_ip1 = polytopes_[i + 1].A;
        Eigen::VectorXd b_ip1 = polytopes_[i + 1].b;
        for (int j = 0; j < A_ip1.rows(); j++)
        {
            SX constraint = A_ip1(j, 0) * p_i(0) + A_ip1(j, 1) * p_i(1) + A_ip1(j, 2) * p_i(2) - b_ip1(j);
            g_vec.push_back(constraint);
        }
    }

    int num_ineq = g_vec.size();

    // Fixed start and goal
    SX p_start = get_waypoint(0);
    SX p_goal = get_waypoint(n - 1);

    for (int j = 0; j < 3; j++)
    {
        g_vec.push_back(p_start(j) - start_(j));
        g_vec.push_back(p_goal(j) - goal_(j));
    }

    int num_eq = 6;

    // Stack constraints
    SX g = vertcat(g_vec);

    // Setup NLP
    SXDict nlp = {{"x", p}, {"f", cost}, {"g", g}};

    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.max_iter"] = 1000;

    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Initial guess
    DM p0 = DM::zeros(3 * n);
    for (int i = 0; i < n; i++)
    {
        p0(i * 3) = waypoints_[i](0);
        p0(i * 3 + 1) = waypoints_[i](1);
        p0(i * 3 + 2) = waypoints_[i](2);
    }

    // Bounds
    DM lbg = DM::zeros(num_ineq + num_eq);
    DM ubg = DM::zeros(num_ineq + num_eq);
    for (int i = 0; i < num_ineq; i++)
    {
        lbg(i) = -DM::inf();
        ubg(i) = 0;
    }

    // Solve
    DMDict args = {{"x0", p0}, {"lbg", lbg}, {"ubg", ubg}};
    DMDict res = solver(args);

    // Extract solution
    DM p_opt = res["x"];
    for (int i = 0; i < n; i++)
    {
        waypoints_[i](0) = static_cast<double>(p_opt(i * 3));
        waypoints_[i](1) = static_cast<double>(p_opt(i * 3 + 1));
        waypoints_[i](2) = static_cast<double>(p_opt(i * 3 + 2));
    }

    double final_cost = static_cast<double>(res["f"]);
    std::cout << "Waypoint optimization done. Cost: " << final_cost << std::endl;
}

// ============ OPTIMIZE ELLIPSOIDS (eq 14a, 14b, 14c) ============
void SafeFlightCorridor::optimize_ellipsoids(double rho, double wv)
{
    int M = ellipsoids_.size();

    for (int i = 0; i < M; i++)
    {
        // Decision variables: L (6 elements) + d (3 elements)
        SX L11 = SX::sym("L11");
        SX L21 = SX::sym("L21");
        SX L22 = SX::sym("L22");
        SX L31 = SX::sym("L31");
        SX L32 = SX::sym("L32");
        SX L33 = SX::sym("L33");
        SX d = SX::sym("d", 3);

        SX L = SX::zeros(3, 3);
        L(0, 0) = L11;
        L(1, 0) = L21;
        L(1, 1) = L22;
        L(2, 0) = L31;
        L(2, 1) = L32;
        L(2, 2) = L33;

        // Waypoints for this ellipsoid - convert to SX
        Eigen::Vector3d p_prev_eig = waypoints_[i];
        Eigen::Vector3d p_curr_eig = waypoints_[i + 1];

        SX p_prev = SX::zeros(3, 1);
        SX p_curr = SX::zeros(3, 1);
        for (int j = 0; j < 3; j++)
        {
            p_prev(j) = p_prev_eig(j);
            p_curr(j) = p_curr_eig(j);
        }

        // Cost (14a): -wv * log(det(L))
        // For lower triangular: det(L) = L11 * L22 * L33
        SX cost = -wv * log(L11 * L22 * L33);

        // Cost (14b): ADMM penalty
        SX L_inv = inv(L);
        SX L_inv_T = L_inv.T();

        SX h0_curr = norm_2(mtimes(L_inv_T, p_curr - d)) - 1;
        SX h0_prev = norm_2(mtimes(L_inv_T, p_prev - d)) - 1;

        SX penalty_curr = fmax(0, h0_curr) * fmax(0, h0_curr);
        SX penalty_prev = fmax(0, h0_prev) * fmax(0, h0_prev);

        double y_curr = y_(0, i);
        double y_prev = y_(1, i);

        cost += (rho / 2.0) * ((penalty_curr + y_curr) * (penalty_curr + y_curr) +
                               (penalty_prev + y_prev) * (penalty_prev + y_prev));

        // Constraints (14c): ||a_j^T * L||_2 + a_j^T * d <= b_j
        Eigen::MatrixXd A_i = polytopes_[i].A;
        Eigen::VectorXd b_i = polytopes_[i].b;
        int num_faces = A_i.rows();

        std::vector<SX> g_vec;
        for (int j = 0; j < num_faces; j++)
        {
            // Convert a_j to SX
            SX a_j = SX::zeros(1, 3);
            a_j(0) = A_i(j, 0);
            a_j(1) = A_i(j, 1);
            a_j(2) = A_i(j, 2);

            SX constraint = norm_2(mtimes(a_j, L).T()) + mtimes(a_j, d) - b_i(j);
            g_vec.push_back(constraint);
        }

        SX g = vertcat(g_vec);

        // Setup NLP
        SX opt_vars = vertcat(std::vector<SX>{L11, L21, L22, L31, L32, L33, d});

        SXDict nlp = {{"x", opt_vars}, {"f", cost}, {"g", g}};

        Dict opts;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.max_iter"] = 1000;

        Function solver = nlpsol("solver", "ipopt", nlp, opts);

        // Initial guess
        Eigen::Matrix3d L_curr = ellipsoids_[i].L;
        Eigen::Vector3d d_curr = ellipsoids_[i].center;

        DM x0 = DM::zeros(9);
        x0(0) = L_curr(0, 0);
        x0(1) = L_curr(1, 0);
        x0(2) = L_curr(1, 1);
        x0(3) = L_curr(2, 0);
        x0(4) = L_curr(2, 1);
        x0(5) = L_curr(2, 2);
        x0(6) = d_curr(0);
        x0(7) = d_curr(1);
        x0(8) = d_curr(2);

        // Bounds
        DM lbg = -DM::inf(num_faces);
        DM ubg = DM::zeros(num_faces);

        // L11, L22, L33 > 0
        DM lbx = DM::zeros(9);
        DM ubx = DM::inf(9);
        lbx(0) = 1e-6; // L11
        lbx(1) = -DM::inf();
        lbx(2) = 1e-6; // L22
        lbx(3) = -DM::inf();
        lbx(4) = -DM::inf();
        lbx(5) = 1e-6; // L33
        lbx(6) = -DM::inf();
        lbx(7) = -DM::inf();
        lbx(8) = -DM::inf();

        // Solve
        DMDict args = {{"x0", x0}, {"lbg", lbg}, {"ubg", ubg}, {"lbx", lbx}, {"ubx", ubx}};
        DMDict res = solver(args);

        // Extract solution
        DM x_opt = res["x"];

        ellipsoids_[i].L(0, 0) = static_cast<double>(x_opt(0));
        ellipsoids_[i].L(1, 0) = static_cast<double>(x_opt(1));
        ellipsoids_[i].L(1, 1) = static_cast<double>(x_opt(2));
        ellipsoids_[i].L(2, 0) = static_cast<double>(x_opt(3));
        ellipsoids_[i].L(2, 1) = static_cast<double>(x_opt(4));
        ellipsoids_[i].L(2, 2) = static_cast<double>(x_opt(5));
        ellipsoids_[i].L(0, 1) = 0;
        ellipsoids_[i].L(0, 2) = 0;
        ellipsoids_[i].L(1, 2) = 0;

        ellipsoids_[i].center(0) = static_cast<double>(x_opt(6));
        ellipsoids_[i].center(1) = static_cast<double>(x_opt(7));
        ellipsoids_[i].center(2) = static_cast<double>(x_opt(8));
    }

    std::cout << "Ellipsoid optimization done." << std::endl;
}

// ============ UPDATE LAGRANGIAN MULTIPLIERS ============
void SafeFlightCorridor::update_lagrangian(double rho)
{
    int M = ellipsoids_.size();

    for (int i = 0; i < M; i++)
    {
        Eigen::Matrix3d L = ellipsoids_[i].L;
        Eigen::Vector3d d = ellipsoids_[i].center;
        Eigen::Matrix3d L_inv_T = L.inverse().transpose();

        Eigen::Vector3d p_prev = waypoints_[i];
        Eigen::Vector3d p_curr = waypoints_[i + 1];

        double h0_curr = (L_inv_T * (p_curr - d)).norm() - 1;
        double h0_prev = (L_inv_T * (p_prev - d)).norm() - 1;

        double penalty_curr = std::max(0.0, h0_curr) * std::max(0.0, h0_curr);
        double penalty_prev = std::max(0.0, h0_prev) * std::max(0.0, h0_prev);

        y_(0, i) += penalty_curr;
        y_(1, i) += penalty_prev;
    }

    std::cout << "Lagrangian multipliers updated." << std::endl;
}