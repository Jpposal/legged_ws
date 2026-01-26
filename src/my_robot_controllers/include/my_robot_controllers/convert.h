#ifndef _CONVERT_H_
#define _CONVERT_H_
#include <iostream>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>  
#include "ros/ros.h"

constexpr int NUM_WHEELS = 4;
constexpr double k_I = 0.35714286; // 电机的力矩常数
constexpr double r = 0.07625; // 轮子的半径，单位为米
constexpr double k = k_I / r; //

// 轮子的几何位置（单位：米）
constexpr double wheelPositions[NUM_WHEELS][2] = {
    { 0.17, 0.185 },  // 左前
    { -0.17, 0.185 }, // 右前
    { 0.17, -0.185 }, // 左后
    { -0.17, -0.185 } // 右后
};

// 轮子的方向，均为正前方
constexpr double wheelDirections[NUM_WHEELS][2] = {
    { -1.0, 1.0 },  // 左前
    { 1.0, 1.0 },  // 右前
    { 1.0, 1.0 },  // 左后
    { -1.0, 1.0 }   // 右后
};


std::array<double, NUM_WHEELS> calculateCurrents(const Eigen::VectorXd& tau);


#endif