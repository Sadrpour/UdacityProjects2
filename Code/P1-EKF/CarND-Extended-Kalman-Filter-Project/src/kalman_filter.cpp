#include "kalman_filter.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include "tools.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    VectorXd h(3);
    float p1 = sqrt(x_(0)*x_(0) +  x_(1)*x_(1));
    float p2 = atan2(x_(1),x_(0));

    if (fabs(p2) > M_PI)
      {
        p2 -= round(p2 / (2.0d * M_PI)) * (2.0d * M_PI);
      }

    // if(p2>3.14 || p2<-3.14){cout << "p2 is : " << p2 << endl;}
    float p3 = (x_(0)*x_(2) +  x_(1)*x_(3))/p1;
    h << p1,p2,p3;
    //cout << "here1" << endl;
    // h << sqrt(x_(0)*x_(0) +  x_(1)*x_(1)),atan2(x_(1),x_(0)),(x_(0)*x_(2) +  x_(1)*x_(3))/sqrt(x_(0)*x_(0) +  x_(1)*x_(1));
    // cout << "h is" << h << endl;
    VectorXd z_pred = h;
    // VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
