#include "kalman_filter.h"
#include <iostream>
#define PI 3.14159265
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   P_ = F_ * P_ * (F_.transpose()) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * (H_.transpose()) + R_;
  MatrixXd K = P_ * (H_.transpose()) * (S.inverse());

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   float rho_squared = x_(0)*x_(0) + x_(1)*x_(1);
   float phi,rho_dot;
   
   if(x_(0) == 0)
	   phi = PI/2;
   else
	   phi = atan2(x_(1),x_(0));
       //while(phi < -PI || phi > PI)
		 //  phi+=(2*PI);
   
   if (fabs(rho_squared) < 0.0001) {
    cout << "UpdateEKF - Error - Division by Zero" << endl;
    rho_dot = 0;
  }
  else
	  rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/sqrt(rho_squared);
  
  VectorXd z_pred = VectorXd(3);
  //cout<<"zpred:"<<endl;
  z_pred << sqrt(rho_squared),phi,rho_dot;
  
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * (H_.transpose()) + R_;
  MatrixXd K = P_ * (H_.transpose()) * (S.inverse());

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
   
}
