#include "my_robot_controllers/robot_leader.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
const Eigen::Matrix3d zero_3 = Eigen::MatrixXd::Zero(3,3);



void Robot_Leader::initial(Vector3d r_){
        r=r_;
}


//更新机器人相关状态
//输入：搬运系统线性化矩阵、搬运系统复合速度状态误差s、搬运系统旋转矩阵R,离散时间差dt
void Robot_Leader::robotupdate(Target target,State state,double dt){//传入target、state、dt和t

    state_robot.grasp_matrix << I_3, zero_3,
                    skew3(state.R*r), I_3;  // / 注意：有旋转矩阵
    state_robot.inv_M << I_3, zero_3,
             -skew3(state.R*r), I_3;

    state.dq << state.v, state.w;
    state_robot.v_r = target.v_d - gain.lmd*(state.x - target.x_d);
    state_robot.a_r = target.a_d - gain.lmd*(state.v - target.v_d);
    state_robot.w_r = target.w_d - gain.lmd*state.R*vee(Pa(target.R_d.transpose()*state.R));         
    state_robot.R_e = target.R_d.transpose()*state.R;
    state_robot.al_r = target.al_d - gain.lmd*skew3(state.w)*state.R*vee(Pa(state_robot.R_e)) 
                        - gain.lmd*state.R*vee(Pa(skew3(target.R_d.transpose()*(state.w - target.w_d))*state_robot.R_e));
    state_robot.dq_r << state_robot.v_r,state_robot.w_r;
    state_robot.s = state.dq - state_robot.dq_r;

    Vector3d Yr11,Yr21;
    Matrix3d Yr12,Yr22;
    MatrixXd Yr13(3,6),Yr23(3,6);


    Yr11 = state_robot.a_r;
    Yr12 = -skew3(state_robot.al_r)*state.R-skew3(state.w)*skew3(state_robot.w_r)*state.R;
    Yr13 = MatrixXd::Zero(3,6);
    Yr21 = Vector3d::Zero();
    Yr22 = skew3(state_robot.a_r)*state.R+skew3(state.w)*skew3(state_robot.v_r)*state.R-skew3(state_robot.w_r)*skew3(state.v)*state.R;
    Yr23 = state.R*L_fun(state.R.transpose()*state_robot.al_r)+skew3(state.w)*state.R*L_fun(state.R.transpose()*state_robot.w_r);
    state_robot.Yr <<  Yr11,Yr12,Yr13,
                Yr21,Yr22,Yr23;

    F = state_robot.Yr*state_robot.hat_o-gain.K_i*state_robot.s-gain.h*state.dq.transpose()*state.dq*state_robot.s;   
    tau = state_robot.inv_M*F;

}

void Robot_Leader::DREM(Target target,State state,double dt,double t){//传入target、state和dt
    Vector3d Y11,Y21;
    Matrix3d Y12,Y22;
    MatrixXd Y13(3,6),Y23(3,6);

    // matlab的test2
    Eigen::Vector3d Yp11,Yp21,dy_p11;
    Eigen::Matrix3d Yp12,dy_p12;
    Eigen::Matrix3d Yp22,dy_p22;
    Eigen::MatrixXd Yp13(3,6),Yp23(3,6),dy_p23(3,6);
    Eigen::MatrixXd dbarYp(6,10);
    Eigen::VectorXd dtaup(6,1),dbartaup(6,1);


    Yp11 = gain.lmd_p*state.v-pow(gain.lmd_p,2)*exp(-gain.lmd_p*t)*drem.y_p11;
    Yp12 = -gain.lmd_p*skew3(state.w)*state.R+pow(gain.lmd_p,2)*exp(-gain.lmd_p*t)*drem.y_p12;
    Yp13.setZero();
    Yp21.setZero();
    Yp22 = gain.lmd_p*skew3(state.v)*state.R-exp(-gain.lmd_p*t)*drem.y_p22;
    Yp23 = gain.lmd_p*state.R*L_fun(state.R.transpose()*state.w)-pow(gain.lmd_p,2)*exp(-gain.lmd_p*t)*drem.y_p23;
    
    drem.Yp << Yp11,Yp12,Yp13,      
            Yp21,Yp22,Yp23;
    
    
    drem.Yf <<  drem.Yp,
                drem.barYp.row(0),
                drem.barYp.block(3,0,3,10);      // 报错   改为上一种形式。   block(Index startRow, Index startCol, Index blockRows, Index blockCols) 

    drem.tauf << drem.taup,
                drem.bartaup.row(0),
                drem.bartaup.block(3,0,3,1);
    
    drem.phi = drem.Yf.determinant();
    drem.taue = drem.Yf.adjoint()*drem.tauf;

    // filter 动力学 
    // Yp是Y的滤波结果，用来避免利用到加速度，因此下面四行是为了计算Yp中的积分项。
    dy_p11 = exp(gain.lmd_p*t)*state.v;
    dy_p12 = exp(gain.lmd_p*t)*skew3(state.w)*state.R;
    dy_p22 = pow(gain.lmd_p,2)*exp(gain.lmd_p*t)*skew3(state.v)*state.R + gain.lmd_p*exp(gain.lmd_p*t)*skew3(state.v)*skew3(state.w)*state.R;
    dy_p23 = exp(gain.lmd_p*t)*state.R*L_fun(state.R.transpose()*state.w);
    // 对应Yp的taup的动力学
    dtaup = -gain.lmd_p*drem.taup + gain.lmd_p*F;     
    
    // 下面两行是补充的4行的动力学
    dbarYp = -gain.lmd_a*drem.barYp + gain.lmd_b*drem.Yp;           
    dbartaup = -gain.lmd_a*drem.bartaup + gain.lmd_b*drem.taup;     
    
    // DREM的参数 update
    Eigen::VectorXd y1(10);
    double rho;
    rho = 1/(1e-6+pow(drem.phi,2));

    y1 = DREM_sim(drem.phi,state_robot.hat_o,drem.taue,rho,dt);
    // [FIX] Disable parameter adaptation for now to test pure PD/Tracking performance
    // state_robot.hat_o = -gain.Gamma*state_robot.Yr.transpose()*state_robot.s*dt + y1;  


    // dynamics
    target.R_d = expm(dt*skew3(target.w_d))*target.R_d;     // SO(3)的期望轨迹 
    
    // DREM update
    drem.y_p11 += dy_p11*dt;
    drem.y_p12 += dy_p12*dt;
    drem.y_p22 += dy_p22*dt;
    drem.y_p23 += dy_p23*dt;
    drem.taup += dtaup*dt;
    drem.barYp += dbarYp*dt;
    drem.bartaup += dbartaup*dt;
}


