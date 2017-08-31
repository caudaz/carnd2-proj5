// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
//#include "Dense" // For UDACITY

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {

  //TODO complete the next_state calculation ...
  float x = state[0];
  float y = state[1];
  float psi = state[2];
  float v = state[3];
  
  x = x + v * cos(psi) * dt;
  y = y + v * sin(psi) * dt;
  psi = psi + v / Lf * actuators[0] * dt;
  v = v + actuators[1] * dt;

  Eigen::VectorXd next_state(state.size());
  next_state  << x, y, psi, v;  

  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}