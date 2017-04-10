#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    // Initialize vector to accumulate rmse.
    VectorXd rmse = VectorXd::Zero(4);
    for (int i = 0; i < estimations.size(); ++i) {
        // Compute residual for sample i.
        VectorXd residual = ground_truth[i] - estimations[i];
        // Square residual.
        residual = residual.array() * residual.array();
        // Accumulate.
        rmse += residual;
    }
    // Compute Mean Square Error (MSE).
    rmse /= estimations.size();
    // Compute square root of mean squared error (RMSE).
    return rmse.array().sqrt();

}
