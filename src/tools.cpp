#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    // Vector to store rmse for each estimated variable.
    VectorXd rmse(estimations[0].size());
    for (int i=0; i < estimations.size(); i++) {
        // Compute error between estimate and truth.
        VectorXd diff = estimations[i] - ground_truth[i];
        // Accumulate squared error
        rmse = rmse.array() + diff.array().square();
    }
    // Compute mean squared error
    rmse = rmse / estimations.size();

    // Compute root means sqaured error
    rmse = rmse.array().sqrt();

    return rmse;

}
