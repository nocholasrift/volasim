/*
 * @file:   RungeKutta.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class implementing basic 3rd and 4th order runge-kutta ODE solver
 */

#ifndef RUNGEKUTTA_H
#define RUNGEKUTTA_H

#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <iostream>

namespace amrl {

/// Numerical ODE solver using Runge-Kutta method
/// Class assumes system of first order ODEs
/// The solver format has been slightly modified from general RK formula as it's intended
/// to be used for simulation of controlled dynamic systems where
/// ODE input is a state variable (x) AND an input force (u) --> dx/dt = f(x, u)
/// Given:
///     State variable:             x
///     Input force:                u
///     Time-step:                  dt
///     Initial condition:          x0    = x(t0)
///     System of first order ODEs: dx/dt = f(x, u)
/// Result of RK method:
///     State one step forward:     x1    = x(t0 + dt)
///
/// Explicit RK formula used:
///    x1 = x0 + dt*SUM(b_i*f_i) for i = [1, ..., s]
/// Where:
///    f_1 = f(x0, u)
///    f_i = f(x0 + dt*SUM(a_ij*f_j), u) for j = [1, ..., i-1]
/// 
/// Values of s, b_i, a_ij depend on choice of RK solver type and
/// were found in reference text listed below.
/// 
/// Reference:
///  Book Title: "Numerical Methods for Differential Equations: A Computational Approach"
///  Author: John R. Dormand
template <int N, int M>
class RungeKutta
{
public:
  // Syntax Convenience for state and input value
  using X_t = Eigen::Vector<double, N>;
  using U_t = Eigen::Vector<double, M>;

  /// Options for RK solver
  enum class SolverType {
    kThirdOrder,
    kFourthOrderClassic,
    kFourthOrderOptimal
  };

  /// Constructor
  /// @param  ode First order diffential equation for x'.(x'=f(x, u))
  /// @param  type Option for what order of solver and solver coefficients
  RungeKutta(
    std::function<X_t(const X_t&, const U_t&)> &&ode,
    const SolverType type = SolverType::kFourthOrderClassic);

  // Delete copy constructor. Can't guarentee
  // ODE function can always be copied/moved
  RungeKutta(const RungeKutta &rhs) = delete;

  /// Default destructor
  ~RungeKutta(void) = default;

  /// Calculate system one time step forward
  /// @param  x0 Current state of the system. x(t0)
  /// @param  u  Input command at current time. u(t0)
  /// @param  dt Time step to calculate system forward. dt
  /// @return State of the system one time step forward. x(t0+dt)
  X_t step(const X_t &x0, const U_t &u, const double dt) const;

private:

  // State differential equation. x'(t) = f(x, u)
  std::function<X_t(const X_t&, const U_t&)> _ode_func;

  /// Number of stages. Depends on RK solver type
  int _s;

  /// Coefficients for RK formula
  Eigen::MatrixXd _a;
  Eigen::VectorXd _b;
};

template<int N, int M>
RungeKutta<N, M>::RungeKutta(
    std::function<RungeKutta<N, M>::X_t(const RungeKutta<N, M>::X_t&, const U_t&)> &&ode,
    const RungeKutta<N, M>::SolverType type)
: _ode_func(std::move(ode))
{
  switch(type) {
    case SolverType::kThirdOrder:
      _s = 3;
      _b.resize(3);
      _b(0) = 1.0/6.0;
      _b(1) = 2.0/3.0;
      _b(2) = 1.0/6.0;

      _a = Eigen::MatrixXd::Zero(3, 2);
      _a(1, 0) =  0.5;
      _a(2, 0) = -1.0;
      _a(2, 1) =  2.0;
      break;
    case SolverType::kFourthOrderClassic:
      _s = 4;
      _b.resize(4);
      _b(0) = 1.0/6.0;
      _b(1) = 1.0/3.0;
      _b(2) = 1.0/3.0;
      _b(3) = 1.0/6.0;

      _a = Eigen::MatrixXd::Zero(4, 3);
      _a(1, 0) = 0.5;
      _a(2, 1) = 0.5;
      _a(3, 2) = 1.0;
      break;
    case SolverType::kFourthOrderOptimal:
      _s = 4;
      _b.resize(4);
      _b(0) =  0.17476028;
      _b(1) = -0.55148066;
      _b(2) =  1.20553560;
      _b(3) =  0.17118478;

      _a = Eigen::MatrixXd::Zero(4, 3);
      _a(1, 0) =  0.4;
      _a(2, 0) =  0.29697761;
      _a(2, 1) =  0.15875964;
      _a(3, 0) =  0.21810040;
      _a(3, 1) = -3.05096516;
      _a(3, 2) =  3.83286476;
      break;
  }
}

template<int N, int M>
typename RungeKutta<N, M>::X_t RungeKutta<N, M>::step(
    const X_t &x0, const U_t &u, const double dt) const
{
  std::vector<X_t> func_evals; // Store evaluation of x' function evaluations f_j
  X_t dx = X_t::Zero(N);       // Delta from RK step (1 for each state variable)

  // Loop number of stages for specified RK type
  for(int i = 0; i < _s; ++i) {
    X_t x_stage = x0; // State to plug into ODE for this stage
    for(int j = 0; j < i; ++j) {
      if(_a(i, j) != 0.0) {
        for(int k = 0; k < N; ++k)
          x_stage[k] += dt*_a(i, j)*func_evals[j][k];
      }
    }

    // Calculate state derivative from ode
    func_evals.push_back(_ode_func(x_stage, u));

    // Add stage function evaluation to dx
    for(int j = 0; j < N; ++j) {
      dx[j] += dt*_b[i]*func_evals[i][j];
    }
  }

  return x0 + dx;
}

} // namespace amrl

#endif

