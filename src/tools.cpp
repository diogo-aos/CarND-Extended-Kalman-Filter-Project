#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
	rmse << 0,0,0,0;
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
    cout << "CalculateRMSE () - Error - estimation vector empty or size mismatch" << endl;
	  return rmse;
  }

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
	    VectorXd residual = estimations[i] - ground_truth[i];
	    residual = residual.array() * residual.array();
	    rmse += residual;
	}
	rmse = rmse / estimations.size();
	rmse =  rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if (px==0 || py==0) {
    cout << "CalculateJacobian () - Error - Division by zero" << endl;
    return Hj;
  }

  float pxpy2 = px*px + py*py;
  float sq_pxpy2 = sqrt(pxpy2);
	//compute the Jacobian matrix
		Hj << px / sq_pxpy2, py / sq_pxpy2, 0, 0,
	        -py / pxpy2,   px/pxpy2,      0, 0,
	        py * (vx * py - vy * px) / (sq_pxpy2 * pxpy2), px * (vy * px - vx * py) / (sq_pxpy2 * pxpy2), px / sq_pxpy2, py / sq_pxpy2;

	return Hj;
}
