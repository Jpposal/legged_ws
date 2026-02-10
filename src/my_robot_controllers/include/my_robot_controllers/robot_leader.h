#ifndef ROBOT_LEADER_H
#define ROBOT_LEADER_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include "my_robot_controllers/funs.h"
#include "state.h"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;


//机器人类
//成员：private：比例参数、机器人位置r,public:机器人状态
//方法：机器人构造函数，机器人状态更新函数
class Robot_Leader{
public:
    Robot_Leader() {
        state_robot.grasp_matrix.resize(6,6); 
        state_robot.inv_M.resize(6,6); 
        
        state_robot.dq_r.resize(6,1);
        state_robot.dqr_hat.resize(6,1);
        state_robot.dq_e.resize(6,1);
        state_robot.s.resize(6,1);
        state_robot.s_hat.resize(6,1);
        state_robot.Y.resize(6,10);
        state_robot.Yr.resize(6,10);
        state_robot.Yr_hat.resize(6,10);

        F.resize(6,1);
        tau.resize(6,1);
        state_robot.hat_o.resize(10);
        state_robot.dhat_o.resize(10);
        state_robot.hat_o.setZero();
    } // 添加无参数构造函数
    ~Robot_Leader() {}
    void initial(Vector3d r_);
    void robotupdate(Target target,State state,double dt);
    void DREM(Target target,State state,double dt,double t);
    Eigen::VectorXd F;
    Eigen::VectorXd tau;


private:
    Vector3d r;
    State_Robot state_robot;

    struct DREM_vars{
        // DREM
        Eigen::MatrixXd Yp;
        Eigen::VectorXd taup;
        Eigen::MatrixXd Yf,tauf,taue;
        double phi;
        Eigen::MatrixXd y_p11,y_p12,y_p22,y_p23;
        Eigen::MatrixXd barYp,bartaup;
    DREM_vars(){
        Yp.setZero(6,10);
        taup.setZero(6);
        Yf.setZero(10,10);
        tauf.setZero(10,1);
        taue.resize(10,1);
        
        y_p11.setZero(3,1);
        y_p12.setZero(3,3);
        y_p22.setZero(3,3);
        y_p23.setZero(3,6);
        barYp.setZero(6,10);
        bartaup.setZero(6,1);
    };
        ~DREM_vars(){};
    };

    struct Gains {
            double K_i, h,lmd,Kx,Kv,Ka,lmd_p,lmd_a,lmd_b,rho;   
            Eigen::MatrixXd Gamma;                              // gains 改了，记得检查 /tagg 
            Gains(){
                Gamma.setZero(10,10);
                Gamma.diagonal() << 500, 0, 0, 0, 500, 0, 0, 500, 0, 500; 
                
                // Gamma.setZero(); // Set Gamma to 0 to freeze adaptation
                K_i=500;
                h=30;
                lmd=2;
                Kx=30;
                Kv=30;
                Ka=1;
                lmd_p=1;
                lmd_a=1.2;
                lmd_b=0.3;
                rho=2;
            }
            ~Gains(){}
        };
    Gains gain;
    DREM_vars drem;
   
};


#endif