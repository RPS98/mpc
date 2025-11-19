// Copyright 2025 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file spline_example.cpp
 *
 * @brief Hermite Spline Example with lookup table arc-length reparametrization
 *
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <vector>
#include <numeric>
#include <algorithm>

#include "spline/spline.hpp"
#include "spline/trajectory_generator.hpp"

class CsvLogger
{
public:
  explicit CsvLogger(const std::string & filename)
  : file_(filename)
  {
    if (!file_.is_open()) {
      std::cerr << "Failed to open CSV file: " << filename << std::endl;
      return;
    }

    // Set fixed format with reasonable precision for floating-point values
    file_ << std::fixed << std::setprecision(6);

    // CSV header: theta, position (px,py,pz), velocity (vx,vy,vz)
    file_ << "theta,px,py,pz,vx,vy,vz\n";
  }

  ~CsvLogger()
  {
    if (file_.is_open()) {
      file_.close();
    }
  }

  void logSample(
    double theta,
    const Eigen::Vector3d & position,
    const Eigen::Vector3d & velocity)
  {
    if (!file_.is_open()) {
      return;
    }

    file_ << theta << ","
          << position.x() << "," << position.y() << "," << position.z() << ","
          << velocity.x() << "," << velocity.y() << "," << velocity.z() << "\n";
  }

private:
  std::ofstream file_;
};

int main()
{
  std::cout << "Spline Example - Hermite Spline Evaluation" << std::endl;

  // Create setpoints for the spline
  double speed = 6.0;  // m/s
  std::vector<spline::Setpoint> setpoints;
  setpoints.emplace_back(
    "gate01",
    Eigen::Vector3d(12.5, 2.0, 1.45),
    Eigen::Vector3d(cos(3.14159), sin(3.14159), 0.0) * speed);
  setpoints.emplace_back(
    "gate02",
    Eigen::Vector3d(6.5, 6.0, 1.45),
    Eigen::Vector3d(cos(2.35619), sin(2.35619), 0.0) * speed);
  setpoints.emplace_back(
    "gate03",
    Eigen::Vector3d(5.5, 14.0, 1.45),
    Eigen::Vector3d(cos(2.0944), sin(2.0944), 0.0) * speed);
  setpoints.emplace_back(
    "gate04",
    Eigen::Vector3d(2.5, 24.0, 1.45),
    Eigen::Vector3d(cos(1.5708), sin(1.5708), 0.0) * speed);
  setpoints.emplace_back(
    "gate05",
    Eigen::Vector3d(7.5, 30.0, 1.45),
    Eigen::Vector3d(cos(-0.174533), sin(-0.174533), 0.0) * speed);
  setpoints.emplace_back(
    "gate06",
    Eigen::Vector3d(12.2, 22.0, 1.45),
    Eigen::Vector3d(cos(0.0), sin(0.0), 0.0) * speed);
  setpoints.emplace_back(
    "gate07_splitup",
    Eigen::Vector3d(17.5, 30.0, 4.15),
    Eigen::Vector3d(cos(1.39626), sin(1.39626), 0.0) * speed);
  setpoints.emplace_back(
    "gate08",
    Eigen::Vector3d(18.5, 22.0, 1.45),
    Eigen::Vector3d(cos(-1.39626), sin(-1.39626), 0.0) * speed);
  setpoints.emplace_back(
    "gate09",
    Eigen::Vector3d(20.5, 14.0, 1.45),
    Eigen::Vector3d(cos(-1.74533), sin(-1.74533), 0.0) * speed);
  setpoints.emplace_back(
    "gate10_ladderup",
    Eigen::Vector3d(18.5, 6.0, 4.15),
    Eigen::Vector3d(cos(-2.35619), sin(-2.35619), 0.0) * speed);
  setpoints.emplace_back(
    "gate10_ladderdown",
    Eigen::Vector3d(18.5, 6.0, 1.45),
    Eigen::Vector3d(cos(-2.35619), sin(-2.35619), 0.0) * speed);
  setpoints.emplace_back(
    "end_point",
    Eigen::Vector3d(12.5, 2.0, 1.45),
    Eigen::Vector3d(cos(3.14159), sin(3.14159), 0.0) * speed);

  // Create spline
  spline::HermiteSpline spline(setpoints);

  // Logger
  CsvLogger spline_logger("spline_theta.csv");

  // Evaluate
  double t = 0.0;
  double dt = 0.01;
  double t_max = spline.getTmax();
  while (t <= t_max) {
    auto [position, velocity] = spline.evaluateWithDerivative(t);

    spline_logger.logSample(t, position, velocity);
    t += dt;
  }

  // Reparametrization by arc length
  int n_samples = setpoints.size() * 100;
  int poly_degree = 5;
  spline::ArcLengthReparametrizationResult reparam = spline::computeArcLengthReparametrization(
    spline, n_samples);

  // Logger
  CsvLogger arclength_logger("spline_arc_length.csv");

  // Evaluate
  double s = 0.0;
  double ds = 0.01;
  double s_max = reparam.total_length;
  while (s <= s_max) {
    auto [position, velocity] = spline::evaluateArcLengthSplineWithDerivative(s, reparam, spline);

    arclength_logger.logSample(s, position, velocity);
    s += ds;
  }

  // Trayectory Generator example
  spline::TrajectoryGenerator traj_gen(setpoints, 4, 400);

  // Logger
  CsvLogger traj_logger("spline_trajectory_generator.csv");

  // Evaluate
  Eigen::Vector3d position;
  Eigen::Vector3d derivative;
  s = 0.0;
  ds = 0.01;
  double s_global = 0.0;
  while (!traj_gen.isFinished(s)) {
    auto [reparam_result, updated_s] = traj_gen.evaluateSpline(s, position, derivative);
    s = updated_s;

    traj_logger.logSample(s_global, position, derivative);
    s += ds;
    s_global += ds;
  }

  return 0;
}
