// Copyright 2024 Universidad Politécnica de Madrid
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
 * @file acados_mpc_datatype.hpp
 *
 * Acados MPC data types definition.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_
#define ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_

#include <mpc_generated_code/acados_sim_solver_mpc.h>
#include <mpc_generated_code/acados_solver_mpc.h>
#include <mpc_generated_code/mpc_model/mpc_model.h>

#include <array>
#include <stdexcept>
#include <string>

namespace acados_mpc {

/**
 * @brief State x
 */
struct State {
  static constexpr size_t Nx = MPC_NX;
  std::array<double, MPC_NX> data;
  std::size_t size = MPC_NX;

  /**
   * @brief Constructor
   */
  State();

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);
};

/**
 * @brief Control u
 */
struct Control {
  static constexpr size_t Nu = MPC_NU;
  std::array<double, MPC_NU> data;
  std::size_t size = MPC_NU;

  /**
   * @brief Constructor
   */
  Control();

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);
};

/**
 * @brief Reference yref
 */
struct Reference {
  static constexpr size_t Nyref = (MPC_NX + MPC_NU);
  std::array<double, MPC_N*(MPC_NX + MPC_NU)> data;
  std::size_t size = MPC_N * (MPC_NX + MPC_NU);

  /**
   * @brief Constructor
   */
  Reference();

  /**
   * @brief Get the data at index
   *
   * @param index index of the stage.
   * @return double* data.
   */
  double* get_data(const int index);

  /**
   * @brief Get the data at index
   *
   * @param index index of the stage.
   * @return const double* data.
   */
  const double* get_data(const int index) const;

  /**
   * @brief Get the data at index
   *
   * @param index index of the stage.
   * @return State state.
   */
  State get_state(const int index) const;

  /**
   * @brief Set the data at index
   *
   * @param index index of the stage.
   * @param state state.
   */
  void set_data(const int index, const double value);

  /**
   * @brief Set the data at index of the stage and value
   *
   * @param ref_index index of the stage.
   * @param value_index index of the value.
   * @param state state.
   */
  void set_data(const int ref_index, const int value_index, const double value);

  /**
   * @brief Set the state at index
   *
   * @param index index of the stage.
   * @param state state.
   * @param control control.
   */
  void set_state(const int index, const State& state, const Control& control = Control());
};

/**
 * @brief MPCGains
 *
 * Gains Q, R and Qe for the MPC.
 */
struct MPCGains {
  static constexpr size_t Nq  = MPC_NX;
  static constexpr size_t Nqe = MPC_NX;
  static constexpr size_t Nr  = MPC_NU;
  std::array<double, (MPC_NX + MPC_NU) * (MPC_NX + MPC_NU)> W;
  std::array<double, MPC_NX * MPC_NX> We;

  /**
   * @brief Constructor
   */
  MPCGains();

  /**
   * @brief Get the W matrix
   *
   * @return double* W.
   */
  double* get_W();

  /**
   * @brief Get the W matrix
   *
   * @return const double* W.
   */
  const double* get_W() const;

  /**
   * @brief Get the We matrix
   *
   * @return double* We.
   */
  double* get_We();

  /**
   * @brief Get the We matrix
   *
   * @return const double* We.
   */
  const double* get_We() const;

  /**
   * @brief Set the W matrix at index
   *
   * @param index index of the diagonal.
   * @param value value.
   */
  void set_W(const int index, const double value);

  /**
   * @brief Set the We matrix at index
   *
   * @param index index of the diagonal.
   * @param value value.
   */
  void set_We(const int index, const double value);

  /**
   * @brief Set the Q, Qe and R matrix
   *
   * @param index index of the diagonal.
   * @param value value.
   */
  void set_gains(const MPCGains& gains);

  /**
   * @brief Set the Q matrix at index
   *
   * @param index index of the diagonal.
   * @param value value.
   */
  void set_Q(const int index, const double value);

  /**
   * @brief Set the Q matrix
   *
   * @param Q std::array<double, MPC_NX> Q.
   */
  void set_Q(const std::array<double, MPC_NX>& Q);

  /**
   * @brief Set the R matrix at index
   *
   * @param index index of the diagonal.
   * @param value value.
   */
  void set_R(const int index, const double value);

  /**
   * @brief Set the R matrix
   *
   * @param R std::array<double, MPC_NU> R.
   */
  void set_R(const std::array<double, MPC_NU>& R);

  /**
   * @brief Set the Qe matrix at index
   *
   * @param index index of the diagonal.
   * @param value value.
   */
  void set_Q_end(const int index, const double value);

  /**
   * @brief Set the Qe matrix
   *
   * @param Qe std::array<double, MPC_NX> Qe.
   */
  void set_Q_end(const std::array<double, MPC_NX>& Qe);
};

/**
 * @brief MPCBounds
 *
 * Bounds lbu and ubu for the MPC.
 */
struct MPCBounds {
  static constexpr size_t Nu = MPC_NU;
  std::array<double, MPC_NU> lbu;
  std::array<double, MPC_NU> ubu;

  /**
   * @brief Constructor
   */
  MPCBounds();

  /**
   * @brief Get the lbu array
   *
   * @return double* lbu.
   */
  double* get_lbu();

  /**
   * @brief Get the lbu array
   *
   * @return const double* lbu.
   */
  const double* get_lbu() const;

  /**
   * @brief Get the ubu array
   *
   * @return double* ubu.
   */
  double* get_ubu();

  /**
   * @brief Get the ubu array
   *
   * @return const double* ubu.
   */
  const double* get_ubu() const;

  /**
   * @brief Set the bounds
   *
   * @param bounds bounds.
   */
  void set_bounds(const MPCBounds& bounds);

  /**
   * @brief Set the lbu
   *
   * @param lbu lbu.
   */
  void set_lbu(const std::array<double, MPC_NU>& lbu);

  /**
   * @brief Set the lbu at index
   *
   * @param index index.
   * @param value value.
   */
  void set_lbu(const int index, const double value);

  /**
   * @brief Set the ubu
   *
   * @param ubu ubu.
   */
  void set_ubu(const std::array<double, MPC_NU>& ubu);

  /**
   * @brief Set the ubu at index
   *
   * @param index index.
   * @param value value.
   */
  void set_ubu(const int index, const double value);
};

/**
 * @brief MPCOnlineParams
 *
 * Online parameters p for the MPC.
 */
struct MPCOnlineParams {
  static constexpr size_t Np = MPC_NP;
  std::array<double, MPC_NP> data;
  std::size_t size = MPC_NP;

  /**
   * @brief Constructor
   */
  MPCOnlineParams();

  /**
   * @brief Get the data
   *
   * @return double* data.
   */
  double* get_data();

  /**
   * @brief Get the data
   *
   * @return const double* data.
   */
  const double* get_data() const;

  /**
   * @brief Get the data at index
   *
   * @param index index.
   * @return double* data.
   */
  double* get_data(const int index);

  /**
   * @brief Get the data at index
   *
   * @param index index.
   * @return const double* data.
   */
  const double* get_data(const int index) const;

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_online_params(const MPCOnlineParams& params);

  /**
   * @brief Set the data
   *
   * @param data data.
   */
  void set_data(const std::array<double, MPC_NP>& data);

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);
};

}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_
