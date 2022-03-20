#include "../Eigen/Dense"
#include "../Eigen/StdVector"

#include <iostream>
#include <math.h>
#include <vector>


#define PI 3.141592653589793

//using namespace std;

using Input = Eigen::Vector2d;
using State = Eigen::Vector3d;
using Obs = Eigen::Matrix<double,6,1>;
using State_Var = Eigen::Matrix3d;
using Gain = Eigen::Matrix<double,3,6>;
using MotorPos = std::vector<Eigen::Vector2d>;
using MotorMove = std::vector<Eigen::Vector2d>;
using vector2d = Eigen::Vector2d;//2D vector
using vector3d = Eigen::Vector3d;//3D vector
using matrix2d = Eigen::Matrix2d;
using matrix3d = Eigen::Matrix3d; 
using matrix6d = Eigen::Matrix<double,6,6>;


#include "../others/dynamical_system.hh"
#include "../header/Encoder.hh"
#include "../header/input_estimate.hh"
#include "../header/EKF.hh"


