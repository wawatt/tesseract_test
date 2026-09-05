#include "vamp_r2000ic/vamp_r2000ic_planner.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

int main(int argc, char** argv) {
    std::cout << "=========================================================" << std::endl;
    std::cout << "   Fanuc R-2000iC/165F VAMP SIMD Accelerated Planning    " << std::endl;
    std::cout << "=========================================================" << std::endl;

    vamp_r2000ic::VampR2000icPlanner planner;
    if (!planner.init()) {
        std::cerr << "Failed to initialize planner!" << std::endl;
        return -1;
    }
    std::cout << "Planner successfully initialized with 6-DOF R-2000iC StateSpace." << std::endl;

    // -------------------------------------------------------------
    // 1. Benchmark: SIMD Forward Kinematics & Collision Throughput
    // -------------------------------------------------------------
    std::cout << "\n[1] Running SIMD Forward Kinematics & Collision Benchmark..." << std::endl;
    // Add a test obstacle box in the workspace
    planner.addObstacleBox("table", 1.5, 0.0, 0.5, 0.8, 1.2, 0.6);

    const int NUM_ITERATIONS = 100000;
    std::vector<double> test_joints = {0.2, 0.3, -0.4, 0.1, 0.5, -0.2};

    auto t_start = std::chrono::high_resolution_clock::now();
    int col_count = 0;
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Vary joint 1 slightly to simulate real sampling
        test_joints[0] = 0.2 + (i % 100) * 0.005;
        if (planner.checkCollision(test_joints)) {
            col_count++;
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    double us_per_check = (total_ms * 1000.0) / NUM_ITERATIONS;

    std::cout << "  Iterations: " << NUM_ITERATIONS << std::endl;
    std::cout << "  Total Time: " << std::fixed << std::setprecision(2) << total_ms << " ms" << std::endl;
    std::cout << "  Speed:      " << std::fixed << std::setprecision(3) << us_per_check << " microseconds per check" << std::endl;
    std::cout << "  Throughput: " << std::fixed << std::setprecision(0) << (NUM_ITERATIONS / (total_ms / 1000.0)) << " checks/sec" << std::endl;

    // -------------------------------------------------------------
    // 2. Motion Planning Test: Free Space without Obstacles
    // -------------------------------------------------------------
    std::cout << "\n[2] Planning Free Space Motion (No Obstacle)..." << std::endl;
    planner.clearObstacles();

    std::vector<double> start_q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> goal_q = {0.8, 0.5, -0.5, 0.4, 0.6, -0.3};
    std::vector<std::vector<double>> trajectory;

    auto t_plan0 = std::chrono::high_resolution_clock::now();
    bool ok0 = planner.planFreespace(start_q, goal_q, trajectory, 5.0, 0.02, 0.025, "RRTConnect");
    auto t_plan1 = std::chrono::high_resolution_clock::now();
    double plan_ms0 = std::chrono::duration<double, std::milli>(t_plan1 - t_plan0).count();

    if (ok0) {
        std::cout << "  -> Plan succeeded in: " << plan_ms0 << " ms" << std::endl;
        std::cout << "  -> Trajectory waypoints: " << trajectory.size() << std::endl;
    } else {
        std::cerr << "  -> Planning failed!" << std::endl;
    }

    // -------------------------------------------------------------
    // 3. Motion Planning Test: Obstacle Avoidance (RRTConnect)
    // -------------------------------------------------------------
    std::cout << "\n[3] Planning Obstacle Avoidance Motion (with Obstacle Box blocking direct path)..." << std::endl;
    // Place obstacle directly in front of arm
    planner.addObstacleBox("blocker", 1.2, 0.2, 1.0, 0.6, 0.6, 0.6);

    auto t_obs0 = std::chrono::high_resolution_clock::now();
    bool ok_obs = planner.planFreespace(start_q, goal_q, trajectory, 5.0, 0.02, 0.025, "RRTConnect");
    auto t_obs1 = std::chrono::high_resolution_clock::now();
    double obs_ms = std::chrono::duration<double, std::milli>(t_obs1 - t_obs0).count();

    if (ok_obs) {
        std::cout << "  -> Obstacle Avoidance succeeded in: " << obs_ms << " ms" << std::endl;
        std::cout << "  -> Trajectory waypoints: " << trajectory.size() << std::endl;
        std::cout << "  -> First 3 waypoints:" << std::endl;
        for (size_t i = 0; i < std::min<size_t>(3, trajectory.size()); ++i) {
            std::cout << "     WP " << i << ": [ ";
            for (double j : trajectory[i]) std::cout << std::fixed << std::setprecision(3) << j << " ";
            std::cout << "]" << std::endl;
        }
    } else {
        std::cerr << "  -> Planning failed!" << std::endl;
    }

    // -------------------------------------------------------------
    // 4. Motion Planning Test: Optimization with RRT*
    // -------------------------------------------------------------
    std::cout << "\n[4] Planning with RRT* (Optimal Planner)..." << std::endl;
    auto t_star0 = std::chrono::high_resolution_clock::now();
    bool ok_star = planner.planFreespace(start_q, goal_q, trajectory, 2.0, 0.05, 0.025, "RRTstar");
    auto t_star1 = std::chrono::high_resolution_clock::now();
    double star_ms = std::chrono::duration<double, std::milli>(t_star1 - t_star0).count();

    if (ok_star) {
        std::cout << "  -> RRT* Succeeded in: " << star_ms << " ms" << std::endl;
        std::cout << "  -> Trajectory waypoints: " << trajectory.size() << std::endl;
    } else {
        std::cerr << "  -> RRT* Planning failed!" << std::endl;
    }

    std::cout << "\nBenchmark completed successfully!" << std::endl;
    return 0;
}
