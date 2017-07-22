#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    // Calculate the RMSE here.
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.size() != ground_truth.size() || estimations.size() == 0){
        cout << "Invalid estimation / ground_truth data" << endl;
        return rmse;
    }

    // accumulate squared residuals
    for(int i=0; i < estimations.size(); i++){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    // calculate mean
    rmse = rmse/estimations.size();

    // calculate square-root
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    // Calculate a Jacobian here.
    MatrixXd Hj(3,4);
    Hj.fill(0.0);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //check division by zero
    if(fabs(c1) < 0.0001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    float c4 = vx*py - vy*px;

    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*c4/c3, -px*c4/c3, px/c2, py/c2;

    return Hj;
}