#include <iostream>
#include <vector>
#include "mpc_gimbal_planner/mpc_view_planner.h"

using namespace mpc_gimbal_planner;

int main() {
    std::cout << "=== MPC View Planner Unit Test ===" << std::endl;

    // 初始化规划器
    MPCViewPlanner planner(10, 0.1);  // horizon=10, dt=0.1s

    // 设置代价函数权重
    planner.setWeights(1.0,   // w_tracking (视角追踪)
                      0.5,   // w_smoothness (视角平滑)
                      0.2);  // w_control (防止控制剧烈)

    // 设置约束
    planner.setConstraints(2.0,   // max_pan_rate (rad/s)
                          2.0,   // max_tilt_rate (rad/s)
                          1.0,   // max_pan_accel (rad/s²)
                          1.0);  // max_tilt_accel (rad/s²)

    // 设置相机参数
    planner.setCameraParams(1470.0, 1470.0,  // fx, fy
                           480.0, 360.0,     // cx, cy
                           960, 720);         // width, height

    std::cout << "Planner initialized with:" << std::endl;
    std::cout << "  Horizon: " << planner.getHorizon() << " steps" << std::endl;
    std::cout << "  Dt: " << planner.getDt() << "s" << std::endl;

    // 创建测试轨迹
    std::cout << "\n=== Test Scenario ===" << std::endl;
    std::cout << "Target moving from (5, 0, 0) to (2, 2, -1) m" << std::endl;

    std::vector<TrajectoryPoint> trajectory;

    // 生成10步预测轨迹 (从(5,0,0)到(2,2,-1))
    for (int i = 0; i < 10; ++i) {
        TrajectoryPoint pt;
        pt.timestamp = i * 0.1;
        pt.x = 5.0 - (i / 10.0) * 3.0;  // 5 -> 2
        pt.y = (i / 10.0) * 2.0;        // 0 -> 2
        pt.z = -(i / 10.0) * 1.0;       // 0 -> -1
        pt.vx = -3.0;  // 速度持续指向新方向
        pt.vy = 2.0;
        pt.vz = -1.0;
        pt.confidence = 0.9;

        trajectory.push_back(pt);
    }

    planner.updateTrajectory(trajectory);
    std::cout << "Trajectory updated with " << trajectory.size() << " points" << std::endl;

    // 初始云台位置
    double current_pan = 0.0;   // 0 rad (正前方)
    double current_tilt = 0.0;  // 0 rad (水平)

    // 运行5个规划步骤
    std::cout << "\n=== MPC Planning Steps ===" << std::endl;
    std::cout << "Initial gimbal: pan=" << current_pan << " tilt=" << current_tilt << std::endl;

    for (int step = 0; step < 5; ++step) {
        GimbalCommand cmd = planner.solve(current_pan, current_tilt);

        std::cout << "\nStep " << step << ":" << std::endl;
        std::cout << "  Command:" << std::endl;
        std::cout << "    Pan: " << cmd.pan << " rad (" << (cmd.pan * 180 / M_PI) << "°)" << std::endl;
        std::cout << "    Tilt: " << cmd.tilt << " rad (" << (cmd.tilt * 180 / M_PI) << "°)" << std::endl;
        std::cout << "    Pan rate: " << cmd.pan_rate << " rad/s" << std::endl;
        std::cout << "    Tilt rate: " << cmd.tilt_rate << " rad/s" << std::endl;

        // 更新当前位置（模拟云台执行指令）
        current_pan = cmd.pan;
        current_tilt = cmd.tilt;
    }

    std::cout << "\n=== Final Gimbal Pose ===" << std::endl;

    // 计算最后一个轨迹点相对于最终云台位置的角度
    const auto& last_pt = trajectory.back();
    double pan_to_target = std::atan2(last_pt.y, last_pt.x);
    double tilt_to_target = std::atan2(-last_pt.z, std::sqrt(last_pt.x*last_pt.x + last_pt.y*last_pt.y));

    std::cout << "Target in gimbal frame:" << std::endl;
    std::cout << "  Pan to target: " << pan_to_target << " rad (" << (pan_to_target * 180 / M_PI) << "°)" << std::endl;
    std::cout << "  Tilt to target: " << tilt_to_target << " rad (" << (tilt_to_target * 180 / M_PI) << "°)" << std::endl;

    std::cout << "Gimbal final pose:" << std::endl;
    std::cout << "  Pan: " << current_pan << " rad (" << (current_pan * 180 / M_PI) << "°)" << std::endl;
    std::cout << "  Tilt: " << current_tilt << " rad (" << (current_tilt * 180 / M_PI) << "°)" << std::endl;

    double pan_error = std::abs(pan_to_target - current_pan);
    double tilt_error = std::abs(tilt_to_target - current_tilt);

    std::cout << "\nTracking error:" << std::endl;
    std::cout << "  Pan error: " << pan_error << " rad (" << (pan_error * 180 / M_PI) << "°)" << std::endl;
    std::cout << "  Tilt error: " << tilt_error << " rad (" << (tilt_error * 180 / M_PI) << "°)" << std::endl;

    if (pan_error < 0.2 && tilt_error < 0.2) {
        std::cout << "\n✓ Test PASSED: Gimbal successfully tracked target" << std::endl;
    } else {
        std::cout << "\n✗ Test WARNING: Large tracking error" << std::endl;
    }

    return 0;
}
