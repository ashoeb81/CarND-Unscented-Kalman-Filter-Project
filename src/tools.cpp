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
    VectorXd rmse(estimations(0).size());
    for (int i=0; i < estimations.size(); i++) {
        VectorXd diff = estimations(i) - ground_truth(i);
        dif
    }
}
