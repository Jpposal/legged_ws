#include <iostream>
#include <array>
#include <Eigen/Dense>
#include "my_robot_controllers/convert.h"




std::array<double, NUM_WHEELS> calculateCurrents(const Eigen::VectorXd& tau) {
    std::array<double, NUM_WHEELS> currents;

    // 提取力和力矩
    double Fx = tau(0);
    double Fy = tau(1); 
    double Fz = tau(2);
    double Mx = tau(3);
    double My = tau(4);
    double Mz = tau(5);

    for (int i = 0; i < NUM_WHEELS; ++i) {
        double dx = wheelPositions[i][0];
        double dy = wheelPositions[i][1];

        // Fx 和 Fy 是作用在小车上的力，需要分解成每个轮子的推力
        double F_wheel_x = Fx / NUM_WHEELS;
        double F_wheel_y = Fy / NUM_WHEELS;

        // Mz 是作用在小车上的旋转力矩，需要转换成每个轮子的推力
        double F_wheel_rot = Mz/4 * (dx * wheelDirections[i][1] - dy * wheelDirections[i][0]) / ((abs(dx) + abs(dy))*(abs(dx) + abs(dy)));

        // 每个轮子的总推力
        double F_wheel_total = (F_wheel_x * wheelDirections[i][0]  + F_wheel_y * wheelDirections[i][1] + F_wheel_rot)*2;

        // 根据总推力计算电流
        currents[i] = F_wheel_total / k;

    }
    // double F_wheel_1=k*currents[0];
    // double F_wheel_2=k*currents[1];
    // double F_wheel_3=k*currents[2];
    // double F_wheel_4=k*currents[3];
    //     ROS_WARN_STREAM("F_wheel_1: " << F_wheel_1 << ", success pass update");
    //     ROS_WARN_STREAM("F_wheel_2: " << F_wheel_2 << ", success pass update");
    //     ROS_WARN_STREAM("F_wheel_3: " << F_wheel_3 << ", success pass update");
    //     ROS_WARN_STREAM("F_wheel_4: " << F_wheel_4 << ", success pass update");
    return currents;
}

// int main() {
//     // 示例力和力矩输入
//     Eigen::VectorXd tau(6);
//     tau << 10.0, 0.0, 0.0, 0.0, 0.0, 5.0;

//     // 计算电流
//     std::array<double, NUM_WHEELS> currents = calculateCurrents(tau);

//     // 输出结果
//     for (int i = 0; i < NUM_WHEELS; ++i) {
//         std::cout << "Wheel " << i + 1 << " current: " << currents[i] << std::endl;
//     }

//     return 0;
// }
