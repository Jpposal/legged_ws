#ifndef STATE_H
#define STATE_H

#include <Eigen/Core>

using namespace Eigen;

struct State {
        Vector3d x;
        Vector3d v;
        Vector3d w;
        Matrix3d R;
        VectorXd dq;
        VectorXd ddq;
    };

struct Target {
        Vector3d al_d;
        Vector3d w_d;
        Vector3d x_d;
        Vector3d v_d;
        Vector3d a_d;
        Matrix3d R_d;
    };

//系统的状态误差结构体
struct Error {
        Vector3d v_r;
        Vector3d a_r;
        Vector3d w_r;
        Vector3d al_r;
        VectorXd dq_r;
        Matrix3d R_e;
    };

//机器人状态结构体
struct State_Robot {
        Eigen::MatrixXd grasp_matrix;
        Eigen::MatrixXd inv_M;

        // estimated 
        Eigen::Vector3d x_d_hat;
        Eigen::Vector3d v_d_hat;
        Eigen::Vector3d a_d_hat;
        Eigen::Matrix3d R_d_hat;
        Eigen::Vector3d w_d_hat;
        Eigen::VectorXd hat_o;
        Eigen::VectorXd dhat_o;

        // reference 
        Eigen::Vector3d v_r;
        Eigen::Vector3d a_r;
        Eigen::Vector3d w_r;
        Eigen::Vector3d al_r;
        Eigen::VectorXd dq_r;

        Eigen::Vector3d v_r_hat;
        Eigen::Vector3d w_r_hat;
        Eigen::Vector3d a_r_hat;
        Eigen::Vector3d al_r_hat;
        Eigen::VectorXd dqr_hat;
        
        // error
        Eigen::Vector3d x_e;
        Eigen::Matrix3d R_e;
        Eigen::Vector3d v_e;
        Eigen::Vector3d w_e;
        Eigen::VectorXd dq_e;
        Eigen::VectorXd s;
        Eigen::VectorXd s_hat;
        
        Eigen::MatrixXd Y;
        Eigen::MatrixXd Yr;
        Eigen::MatrixXd Yr_hat;
    };

#endif