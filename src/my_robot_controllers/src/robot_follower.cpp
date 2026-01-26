#include "my_robot_controllers/robot_follower.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
const Eigen::Matrix3d zero_3 = Eigen::Matrix3d::Zero();




void Robot_Follower::initial(Vector3d r_, Vector3d x_init, Matrix3d R_init){
        r=r_;
        state_robot.x_d_hat = x_init;
        state_robot.R_d_hat = R_init;
        state_robot.v_d_hat.setZero();
        state_robot.w_d_hat.setZero();
}

void Robot_Follower::robotupdate(Vector3d w_d,Vector3d a_d,Vector3d al_d,State state,double dt,double T){//传入some target、state、dt和t

    state_robot.grasp_matrix << I_3, zero_3,
                    skew3(state.R*r), I_3;  // / 注意：有旋转矩阵
    state_robot.inv_M << I_3, zero_3,
             -skew3(state.R*r), I_3;

    state.dq << state.v, state.w;

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



    state_robot.w_d_hat = w_d-gain.lmd*state_robot.R_d_hat*vee(Pa(state.R.transpose()*state_robot.R_d_hat));
    state_robot.a_d_hat = -gain.Kx*(state_robot.x_d_hat-state.x)-gain.Kv*(state_robot.v_d_hat-state.v)+a_d;

    state_robot.a_r_hat = state_robot.a_d_hat-gain.lmd*(state.v-state_robot.v_d_hat);
    state_robot.v_r_hat = state_robot.v_d_hat-gain.lmd*(state.x-state_robot.x_d_hat);
    state_robot.w_r_hat = state_robot.w_d_hat-gain.lmd*state.R*vee(Pa(state_robot.R_d_hat.transpose()*state.R));
    state_robot.al_r_hat = al_d-gain.lmd*(skew3(state_robot.w_d_hat)*state_robot.R_d_hat-skew3(state.w)*state.R)*vee(Pa(state.R.transpose()*state_robot.R_d_hat))
                    -gain.lmd*(state_robot.R_d_hat-state.R)*vee(Pa(state.R.transpose()*skew3(state_robot.w_d_hat-state.w)*state_robot.R_d_hat));  

    Vector3d Yr_h11,Yr_h21;
    Matrix3d Yr_h12,Yr_h22;
    MatrixXd Yr_h13(3,6),Yr_h23(3,6);
    
    Yr_h11 = state_robot.a_r_hat;
    Yr_h12 = -skew3(state_robot.al_r_hat)*state.R-skew3(state.w)*skew3(state_robot.w_r_hat)*state.R;
    Yr_h13 = MatrixXd::Zero(3,6);
    Yr_h21 = Vector3d::Zero();
    Yr_h22 = skew3(state_robot.a_r_hat)*state.R+skew3(state.w)*skew3(state_robot.v_r_hat)*state.R-skew3(state_robot.w_r_hat)*skew3(state.v)*state.R;
    Yr_h23 = state.R*L_fun(state.R.transpose()*state_robot.al_r_hat)+skew3(state.w)*state.R*L_fun(state.R.transpose()*state_robot.w_r_hat);
    state_robot.Yr_hat <<  Yr_h11,Yr_h12,Yr_h13,
                Yr_h21,Yr_h22,Yr_h23;

    state_robot.dqr_hat << state_robot.v_r_hat,state_robot.w_r_hat;
    state_robot.s_hat = state.dq - state_robot.dqr_hat;



    F = state_robot.Yr_hat*state_robot.hat_o-gain.K_i*state_robot.s_hat;   
    tau = state_robot.inv_M*F;

}

void Robot_Follower::DREM(State state,double dt,double t){//传入target、state和dt
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
    state_robot.hat_o = -gain.Gamma*state_robot.Yr.transpose()*state_robot.s*dt + y1;  

    // dynamics 
    state_robot.R_d_hat = expm(dt*skew3(state_robot.w_d_hat))*state_robot.R_d_hat;   
    state_robot.v_d_hat += state_robot.a_d_hat*dt;
    state_robot.x_d_hat += state_robot.v_d_hat*dt;

    // DREM update
    drem.y_p11 += dy_p11*dt;
    drem.y_p12 += dy_p12*dt;
    drem.y_p22 += dy_p22*dt;
    drem.y_p23 += dy_p23*dt;
    drem.taup += dtaup*dt;
    drem.barYp += dbarYp*dt;
    drem.bartaup += dbartaup*dt;
}






