#ifndef _FUNS_H_
#define _FUNS_H_

#include <Eigen/Core>
#include <Eigen/Dense>  
#include <unsupported/Eigen/MatrixFunctions>    // 矩阵指数函数
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>  
// #include <eigen3/unsupported/Eigen/MatrixFunctions>    // 矩阵指数函数



using namespace Eigen;

Matrix3d Pa(Matrix3d A);
Matrix3d skew3(Vector3d v);
Vector3d vee(Matrix3d A);
MatrixXd L_fun(Vector3d v);
MatrixXd expm(Eigen::Matrix3d R);
VectorXd DREM_sim(double _phi,Eigen::VectorXd _hat_o,Eigen::VectorXd _taue,double _gain,double _dt);


#endif //_FUNS_H_