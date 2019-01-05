#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  int size = estimations.size();
  VectorXd result(4);
  result << 0,0,0,0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0){
    return result;
  }
  
  for(int i=0; i<size; i++){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    result += residual;
  }
  result /= size;
  result = result.array().sqrt();
  
  return result;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd result(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  

  float pxy2 = px*px + py*py;
  float pxy2_sqrt = sqrt(pxy2);
  float pxy2_3 = pow(pxy2, 3);
  float pxy2_3_sqrt = sqrt(pxy2_3);
  
  if (fabs(pxy2) < 0.0001){
    cout << "Tools::CalculateJacobian divide by zero" << endl;
    return result;
  }
  
  result(0,0) = px / pxy2_sqrt;
  result(0,1) = py / pxy2_sqrt;
  result(0,2) = 0;
  result(0,3) = 0;
  result(1,0) = - py / pxy2;
  result(1,1) = px / pxy2;
  result(1,2) = 0;
  result(1,3) = 0;
  result(2,0) = (py*(vx*py - vy*px)) / pxy2_3_sqrt;
  result(2,1) = (px*(vy*px - vx*py)) / pxy2_3_sqrt;
  result(2,2) = px / pxy2_sqrt;
  result(2,3) = py / pxy2_sqrt;
  
  return result;
}
