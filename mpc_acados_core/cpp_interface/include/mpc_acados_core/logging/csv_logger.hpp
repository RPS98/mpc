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
 * @file csv_logger.hpp
 *
 * CSV logger and orientation helpers shared by MPC example runners.
 *
 * Writes a fixed 37-column schema that matches the layout expected by the
 * plotting utilities in ``mpc_acados_core.plotting.plot_results``. Helper
 * functions to convert between quaternions and Euler angles using Eigen types
 * are also provided because they are needed both by the logger and by the
 * runners that produce reference orientations.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef MPC_ACADOS_CORE_LOGGING_CSV_LOGGER_HPP_
#define MPC_ACADOS_CORE_LOGGING_CSV_LOGGER_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace mpc_acados_core {
namespace logging {

inline constexpr const char* kSimulatorLogsDir = "simulator_logs";

/// Resolve plain file names into ``simulator_logs/<name>``.
inline std::filesystem::path normalizeLogPath(const std::string& file_name) {
  std::filesystem::path file_path(file_name);
  if (!file_path.has_parent_path()) {
    file_path = std::filesystem::path(kSimulatorLogsDir) / file_path;
  }
  return file_path;
}

/// Convert a quaternion to ZYX Euler angles (roll, pitch, yaw).
inline Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double roll      = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp  = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  const double yaw       = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

/// Convert ZYX Euler angles (roll, pitch, yaw) to a unit quaternion.
inline Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
  const double sr = std::sin(roll * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cy = std::cos(yaw * 0.5);

  return Eigen::Quaterniond(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
      .normalized();
}

/// Quaternion facing the given planar direction vector.
inline Eigen::Quaterniond computePathFacing(const Eigen::Vector3d& direction) {
  const double yaw = std::atan2(direction.y(), direction.x());
  return eulerToQuaternion(0.0, 0.0, yaw);
}

/**
 * @brief CSV logger for MPC + MAV Simulator integrated runners.
 *
 * Columns (37 total):
 *   time,
 *   x, y, z, qw, qx, qy, qz, roll, pitch, yaw,
 *   vx, vy, vz, wx, wy, wz,
 *   x_ref, y_ref, z_ref,
 *   qw_ref, qx_ref, qy_ref, qz_ref, roll_ref, pitch_ref, yaw_ref,
 *   thrust, wx_cmd, wy_cmd, wz_cmd,
 *   motor_w0, motor_w1, motor_w2, motor_w3,
 *   controller_solve_time_us, waypoint_index, hover_active, max_speed
 */
class CsvLogger {
public:
  explicit CsvLogger(const std::string& file_name) {
    const std::filesystem::path file_path = normalizeLogPath(file_name);
    file_name_                            = file_path.string();

    if (file_path.has_parent_path()) {
      std::error_code error;
      std::filesystem::create_directories(file_path.parent_path(), error);
      if (error) {
        throw std::runtime_error("Could not create log directory '" +
                                 file_path.parent_path().string() + "': " + error.message());
      }
    }

    std::cout << "Saving to file: " << file_name_ << std::endl;
    file_ = std::ofstream(file_name_, std::ofstream::out | std::ofstream::trunc);
    if (!file_.is_open()) {
      throw std::runtime_error("Could not open file: " + file_name_);
    }
    file_ << "time,"
             "x,y,z,qw,qx,qy,qz,roll,pitch,yaw,"
             "vx,vy,vz,wx,wy,wz,"
             "x_ref,y_ref,z_ref,"
             "qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,"
             "thrust,wx_cmd,wy_cmd,wz_cmd,"
             "motor_w0,motor_w1,motor_w2,motor_w3,"
             "controller_solve_time_us,waypoint_index,hover_active,max_speed"
          << std::endl;
  }

  ~CsvLogger() { close(); }

  void save(const double time,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& linear_velocity,
            const Eigen::Vector3d& angular_velocity,
            const Eigen::Vector3d& reference_position,
            const Eigen::Quaterniond& reference_orientation,
            const double thrust,
            const Eigen::Vector3d& command_angular_velocity,
            const Eigen::Matrix<double, 4, 1>& motor_w,
            const double controller_solve_time_us,
            const int waypoint_index,
            const bool hover_active,
            const double max_speed) {
    const Eigen::Vector3d euler     = quaternionToEuler(orientation);
    const Eigen::Vector3d euler_ref = quaternionToEuler(reference_orientation);

    // Time
    write(time);
    // State
    writeVector(position);
    write(orientation.w());
    write(orientation.x());
    write(orientation.y());
    write(orientation.z());
    writeVector(euler);
    writeVector(linear_velocity);
    writeVector(angular_velocity);
    // Reference
    writeVector(reference_position);
    write(reference_orientation.w());
    write(reference_orientation.x());
    write(reference_orientation.y());
    write(reference_orientation.z());
    writeVector(euler_ref);
    // Actuation
    write(thrust);
    writeVector(command_angular_velocity);
    // Motor state
    writeVector(motor_w);
    // Control metadata
    write(controller_solve_time_us);
    write(static_cast<double>(waypoint_index));
    write(hover_active ? 1.0 : 0.0);
    write(max_speed, false);
    file_ << "\n";
  }

  void close() {
    if (file_.is_open()) {
      file_.close();
    }
  }

private:
  void write(const double value, const bool comma = true) {
    file_ << value;
    if (comma) {
      file_ << ",";
    }
  }

  template <typename Derived>
  void writeVector(const Eigen::MatrixBase<Derived>& v, const bool trailing_comma = true) {
    for (Eigen::Index i = 0; i < v.size(); ++i) {
      const bool last = (i == v.size() - 1);
      write(v(i), trailing_comma || !last);
    }
  }

  std::string file_name_;
  std::ofstream file_;
};

/// Render a simple progress bar to stdout. ``progress`` in [0, 1].
inline void printProgress(const double progress) {
  constexpr int kWidth = 60;
  const int pos        = static_cast<int>(kWidth * std::clamp(progress, 0.0, 1.0));

  std::cout << "\r[";
  for (int i = 0; i < kWidth; ++i) {
    std::cout << (i < pos ? '=' : (i == pos ? '>' : ' '));
  }
  std::cout << "] " << static_cast<int>(progress * 100.0) << "%" << std::flush;
}

/// Arithmetic mean. Returns 0 if ``values`` is empty.
inline double computeMean(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

/**
 * @brief Compute desired orientation for the current waypoint.
 *
 * When ``path_facing`` is enabled the drone yaws to face the direction of
 * travel. Yaw is held unchanged when the drone is within 0.1 m of the
 * waypoint to avoid discontinuities near the goal.
 *
 * @param waypoint            Target waypoint position (world frame, m).
 * @param current_position    Current drone position (world frame, m).
 * @param current_orientation Current drone orientation.
 * @param path_facing         Whether to align yaw with the direction of travel.
 * @return Desired orientation quaternion.
 */
inline Eigen::Quaterniond getDesiredOrientation(const Eigen::Vector3d& waypoint,
                                                const Eigen::Vector3d& current_position,
                                                const Eigen::Quaterniond& current_orientation,
                                                const bool path_facing) {
  if (!path_facing) {
    return Eigen::Quaterniond::Identity();
  }
  const Eigen::Vector3d diff = waypoint - current_position;
  if (diff.head<2>().norm() < 0.1) {
    return current_orientation;
  }
  return computePathFacing(diff);
}

}  // namespace logging
}  // namespace mpc_acados_core

#endif  // MPC_ACADOS_CORE_LOGGING_CSV_LOGGER_HPP_
