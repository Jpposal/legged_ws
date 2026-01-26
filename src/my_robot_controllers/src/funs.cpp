#include "my_robot_controllers/funs.h"

//计算矩阵的反对称部分
Matrix3d Pa(Matrix3d A){
    Matrix3d Af=(A - A.transpose())/2;
    return Af;

}

//向量转反对称矩阵



Matrix3d skew3(Eigen::Vector3d _w)
{
    Eigen::Matrix3d wx;
    wx <<  0, -_w(2), _w(1),
            _w(2), 0, -_w(0),
            -_w(1), _w(0), 0;
    
    return wx;
};

//反对称矩阵转向量
Vector3d vee(Matrix3d A){
    Eigen::VectorXd vec = Eigen::VectorXd::Zero(3);
    vec(2) = A(0, 1);
    vec(1) = A(0, 2);
    vec(0) = A(1, 2);
    return vec;

}

//计算L_fun矩阵
MatrixXd L_fun(Eigen::Vector3d v){
    Eigen::MatrixXd R_out;
    R_out.resize(3,6);
    R_out <<    v(0),v(1),v(2),0,0,0,
                0,v(0),0,v(1),v(2),0,
                0,0,v(0),0,v(1),v(2);
    return R_out;
}

Eigen::MatrixXd expm(Eigen::Matrix3d R){
    Eigen::Matrix3d R_out;
    R_out = R.exp();
    return R_out;
}

Eigen::VectorXd DREM_sim(double _phi,Eigen::VectorXd _hat_o,Eigen::VectorXd _taue,double _gain,double _dt){
    Eigen::VectorXd drem_out(10);
    Eigen::VectorXd _dhat_o(10);
    for (int i = 0; i < _hat_o.rows(); i++){
        _dhat_o(i) = -_gain * _phi * (_phi*_hat_o(i)-_taue(i)); // 注意有负号
        drem_out(i) = _hat_o(i) + _dhat_o(i)*_dt;          
    };
    return drem_out;
}