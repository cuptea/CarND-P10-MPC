#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// TODO: Set the timestep length and duration
const size_t N =20; // number of timesteps
const double dt = 0.1; // timestep evaluation frequency

const double Lf = 2.67; // this is the length from front to Center-of-Gravity (CoG)

const double ref_cte = 0; // reference cross track error
const double ref_epsi = 0; // reference orientation error
const double ref_v = 40; // reference velocity

// indices of the start of each variable in the input to the solver
const int x_start = 0;
const int y_start = x_start + N;
const int psi_start = y_start + N;
const int v_start = psi_start + N;
const int cte_start = v_start + N;
const int epsi_start = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start = delta_start + N - 1;

// weights for cost computations
const double w_cte = 100.0;
const double w_epsi = 100.0;
const double w_v = 0.1;
const double w_delta = 1.0;
const double w_a = 1.0;
const double w_d_delta = 10.0;
const double w_d_a = 2.0;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
