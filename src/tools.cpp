#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

   VectorXd rmse(4);
   rmse << 0,0,0,0;

   // Check the validity of the following inputs:
   // 1) The estimation vector size should not be zero
   if (0 == estimations.size())
   {
      return rmse;
   }
   // 2) The estimation vector size should equal ground truth vector size
   if (estimations.size() != ground_truth.size())
   {
      return rmse;
   }

   // Accumulate squared residuals
   VectorXd cumulative_squared_error(4);
   cumulative_squared_error << 0, 0, 0, 0;
   for (int i=0; i < estimations.size(); ++i) {
      VectorXd error = estimations[i] - ground_truth[i];
      VectorXd squared_error = error.array() * error.array();
      cumulative_squared_error += squared_error;
   }

   // Calculate the mean
   float n = static_cast<float>(estimations.size());
   VectorXd mean_squarred_error(4);
   mean_squarred_error = (1 / n) * cumulative_squared_error;

   // Calculate the squared root
   rmse = mean_squarred_error.array().sqrt();

   // Return the result
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
   MatrixXd Hj(3,4);
   
   // Recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // Check division by zero
   if (px == 0 && py == 0)
   {
      return Hj;
   }
   
   // Compute the Jacobian matrix
   float q1 = (px * px) + (py * py);
   float q2 = sqrt(q1);
   float q3 = q1 * q2;

   Hj << (px / q2), (py / q2), 0, 0,
         (- py / q1), (px / q1), 0, 0,
         (py * (vx * py - vy * px) / q3), (px * (vy * px - vx * py) / q3), (px / q2), (py / q2);

   return Hj;
}
