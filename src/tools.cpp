#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd RMSE(4);
	RMSE << 0, 0, 0, 0;

	// error handling
	if (estimations.size() == 0) {
		std::cout << "Estimations vector is empty" << std::endl;
		return RMSE;
	}

	if (estimations.size() != ground_truth.size()) {
		std::cout << "Estimations and ground_truth vectors must be the same size" << std::endl;
		return RMSE;
	}

	// sum squared residuals
	for (unsigned int i=0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];
		VectorXd square = residual.array().square();
		RMSE += square;
	}

	// calculate mean
	RMSE /= estimations.size();

	// calculate square root
	RMSE = RMSE.array().sqrt();
	
	return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3, 4);
	// state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// check for zero division
	if (px==0 && py==0) {
		std::cout << "CalculateJacobian() - Divide by 0 error" << std::endl;
		return Hj;
	}

	// intermediate variables
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = c1 * c2;

	Hj << px / c2, py / c2, 0, 0,
		-(py / c1), px / c1, 0, 0,
		py*(vx*py - vy * px) / c3, px*(px*vy - py * vx) / c3, px / c2, py / c2;
	
	return Hj;
}
