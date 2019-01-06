#include "tools.h"
#include <iostream>

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
   cout << "calculating size in rmse"<<endl;
  size_t N = estimations.size();
  cout << "initializing rmse vector"<<endl;
  VectorXd rmse = VectorXd::Zero(N);

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
   cout << "not invalid estimation" << endl;
  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  cout << "calculate mean"<<endl;
  rmse = rmse/estimations.size();

  // calculate the squared root
  cout << "calculate sqrt"<<endl;
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
   
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   float px,py,vx,vy;
   px = x_state(0);
   py = x_state(1);
   vx = x_state(2);
   vy = x_state(3);
   
   float rho_squared = px*px + py*py;
   float rho = sqrt(rho_squared);

   MatrixXd H_jacobian(3,4);
   
   // check division by zero
  if (fabs(rho_squared) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return H_jacobian;
  }
   cout<<"Jacobian:"<<endl;
   H_jacobian << px/rho, py/rho, 0, 0,
		  -1*py/rho_squared, px/rho_squared, 0, 0,
		  py*(vx*py-vy*px)/(rho_squared*rho),px*(vy*px-vx*py)/(rho_squared*rho),px/rho,py/rho;
		  
	return H_jacobian;
}
