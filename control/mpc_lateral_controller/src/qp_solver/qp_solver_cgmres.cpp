// Copyright 2018-2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpc_lateral_controller/qp_solver/qp_solver_cgmres.hpp"

#include "cgmres/horizon.hpp"
#include "cgmres/single_shooting_cgmres_solver.hpp"
#include "cgmres/zero_horizon_ocp_solver.hpp"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
QPSolverCGMRES::QPSolverCGMRES(const rclcpp::Logger & logger) : logger_{logger}
{
  // Define the solver settings.
  settings_.sampling_time = 0.001;  // sampling period
  settings_.zeta = 1000;
  settings_.finite_difference_epsilon = 1e-08;
  // For initialization.
  settings_.max_iter = 50;
  settings_.opterr_tol = 1e-06;

  // Define the initial time and initial state.
  // state は 横偏差、ヨー角、ステアリング角度の3つ
  // const double t0 = 0;

  // mpc.set_uc(initializer.ucopt());
  // mpc.init_dummy_mu();
  // Perform a numerical simulation.
  // const double tsim = 10;
  // const double sampling_time = settings.sampling_time;
  // const unsigned int sim_steps = std::floor(tsim / sampling_time);

  // double t = t0;
  // cgmres::VectorX x = x0;
  // cgmres::VectorX dx = cgmres::VectorX::Zero(x0.size());
}

bool QPSolverCGMRES::solveCGMRES(
  const Eigen::VectorXd & x0, const double prediction_dt, Eigen::VectorXd & u)
{
  // // Define the horizon.
  const double alpha = 0.0;
  cgmres::Horizon horizon(prediction_dt, alpha);

  // Define the C/GMRES solver.
  constexpr int N = 100;
  constexpr int kmax = 5;
  cgmres::SingleShootingCGMRESSolver<cgmres::OCP_lateral_control, N, kmax> mpc(
    ocp_, horizon, settings_);

  cgmres::Vector<3> x;
  x << x0(0), x0(1), x0(2);

  // Initialize the solution of the C/GMRES method.
  constexpr int kmax_init = 1;
  cgmres::ZeroHorizonOCPSolver<cgmres::OCP_lateral_control, kmax_init> initializer(ocp_, settings_);
  cgmres::Vector<1> uc0;
  uc0 << 0.0;  // 初期値
  initializer.set_uc(uc0);
  const double t0 = 0.0;
  initializer.solve(t0, x);
  initializer.ucopt();
  u = initializer.uopt();
  RCLCPP_DEBUG(logger_, "u = %f", u(0));
  return true;
}

bool QPSolverCGMRES::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
  const Eigen::VectorXd & ub_a, Eigen::VectorXd & u)
{
  const Eigen::Index raw_a = a.rows();
  const Eigen::Index col_a = a.cols();
  const Eigen::Index dim_u = ub.size();
  Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(dim_u, dim_u);

  // convert matrix to vector for cgmressolver
  std::vector<double> f(&f_vec(0), f_vec.data() + f_vec.cols() * f_vec.rows());

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;

  for (int i = 0; i < dim_u; ++i) {
    lower_bound.push_back(lb(i));
    upper_bound.push_back(ub(i));
  }

  for (int i = 0; i < col_a; ++i) {
    lower_bound.push_back(lb_a(i));
    upper_bound.push_back(ub_a(i));
  }

  Eigen::MatrixXd cgmresA = Eigen::MatrixXd(dim_u + col_a, raw_a);
  cgmresA << Identity, a;

  /* execute optimization */
  auto result = cgmressolver_.optimize(h_mat, cgmresA, f, lower_bound, upper_bound);

  std::vector<double> U_cgmres = std::get<0>(result);
  u = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    &U_cgmres[0], static_cast<Eigen::Index>(U_cgmres.size()), 1);

  const int status_val = std::get<3>(result);
  if (status_val != 1) {
    RCLCPP_WARN(logger_, "optimization failed : %s", cgmressolver_.getStatusMessage().c_str());
    return false;
  }
  const auto has_nan =
    std::any_of(U_cgmres.begin(), U_cgmres.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
    return false;
  }

  // polish status: successful (1), unperformed (0), (-1) unsuccessful
  int status_polish = std::get<2>(result);
  if (status_polish == -1 || status_polish == 0) {
    const auto s = (status_polish == 0) ? "Polish process is not performed in cgmres."
                                        : "Polish process failed in cgmres.";
    RCLCPP_INFO(logger_, "%s The required accuracy is met, but the solution can be inaccurate.", s);
    return true;
  }
  return true;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
