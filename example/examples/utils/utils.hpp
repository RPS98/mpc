// Copyright 2024 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names of its
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
 * @file example_utils.hpp
 *
 * Acados MPC examples using Acados Sim solver utility functions implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef EXAMPLE_UTILS_HPP_
#define EXAMPLE_UTILS_HPP_

#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace acados_mpc {
namespace acados_mpc_examples {

inline Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
  // Calculate half angles
  const double roll_half  = roll * 0.5;
  const double pitch_half = pitch * 0.5;
  const double yaw_half   = yaw * 0.5;

  // Calculate sine and cosine of the half angles
  const double sr = std::sin(roll_half);
  const double cr = std::cos(roll_half);
  const double sp = std::sin(pitch_half);
  const double cp = std::cos(pitch_half);
  const double sy = std::sin(yaw_half);
  const double cy = std::cos(yaw_half);

  // Calculate quaternion components
  const double w = cr * cp * cy + sr * sp * sy;
  const double x = sr * cp * cy - cr * sp * sy;
  const double y = cr * sp * cy + sr * cp * sy;
  const double z = cr * cp * sy - sr * sp * cy;

  // Return the quaternion
  return Eigen::Quaterniond(w, x, y, z).normalized();
}

inline Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();

  // Roll (x-axis rotation)
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll      = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const double sinp = 2.0 * (w * y - z * x);
  double pitch      = 0.0;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(std::acos(-1.0) / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  // Yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw       = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

inline Eigen::Vector4d quaternionToVector(const Eigen::Quaterniond& q) {
  return {q.w(), q.x(), q.y(), q.z()};
}

inline Eigen::Quaterniond computePathFacing(const Eigen::Vector3d& velocity) {
  const double yaw   = std::atan2(velocity.y(), velocity.x());
  const double pitch = 0.0;
  const double roll  = 0.0;

  return eulerToQuaternion(roll, pitch, yaw);
}

class CsvLogger {
public:
  explicit CsvLogger(const std::string& file_name) : file_name_(file_name) {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time,"
             "x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,"
             "x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,"
             "vx_ref,vy_ref,vz_ref,thrust,wx,wy,wz"
          << std::endl;
  }

  ~CsvLogger() { close(); }

  void addDouble(const double data, const bool add_final_comma = true) {
    if (std::isnan(data)) {
      throw std::invalid_argument("Data is nan");
    }
    file_ << data;
    if (add_final_comma) {
      file_ << ",";
    }
  }

  template <typename Derived>
  void addVectorRow(const Eigen::MatrixBase<Derived>& data, const bool add_final_comma = true) {
    for (Eigen::Index index = 0; index < data.size(); ++index) {
      addDouble(data(index), add_final_comma || index < data.size() - 1);
    }
  }

  void save(const double time,
            const Eigen::Vector3d& state_position,
            const Eigen::Quaterniond& state_orientation,
            const Eigen::Vector3d& state_velocity,
            const Eigen::Vector3d& reference_position,
            const Eigen::Quaterniond& reference_orientation,
            const Eigen::Vector3d& reference_velocity,
            const double control_thrust,
            const Eigen::Vector3d& control_angular_velocity) {
    addDouble(time);

    // State
    addVectorRow(state_position);
    addVectorRow(quaternionToVector(state_orientation));
    addVectorRow(quaternionToEuler(state_orientation));
    addVectorRow(state_velocity);

    // Reference
    addVectorRow(reference_position);
    addVectorRow(quaternionToVector(reference_orientation));
    addVectorRow(quaternionToEuler(reference_orientation));
    addVectorRow(reference_velocity);

    // Control
    addDouble(control_thrust);
    addVectorRow(control_angular_velocity, false);

    // End line
    file_ << std::endl;
  }

  void close() {
    if (file_.is_open()) {
      file_.close();
    }
  }

private:
  std::string file_name_;
  std::ofstream file_;
};

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

#endif  // EXAMPLE_UTILS_HPP_
