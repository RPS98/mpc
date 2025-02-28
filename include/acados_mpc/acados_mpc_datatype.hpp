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
  static const std::size_t size = MPC_NX;

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
  static const std::size_t size = MPC_NU;

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
  static constexpr size_t Nyref = MPC_NY;
  std::array<double, MPC_N * MPC_NY> data;
  static const std::size_t size = MPC_N * MPC_NY;

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

struct ReferenceEnd {
  std::array<double, MPC_NYN> data;
  static const std::size_t size = MPC_NYN;

  /**
   * @brief Constructor
   */
  ReferenceEnd();

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
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);
};

/**
 * @brief Gains
 *
 * Gains Q, R and Qe for the MPC.
 */
struct Gains {
  static constexpr size_t Nq  = MPC_NYN;
  static constexpr size_t Nqe = MPC_NYN;
  static constexpr size_t Nr  = (MPC_NY - MPC_NYN);
  std::array<double, MPC_NY * MPC_NY> W;
  std::array<double, MPC_NYN * MPC_NYN> We;

  /**
   * @brief Constructor
   */
  Gains();

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
   * @brief Get the Q matrix
   *
   * @return std::array<double, Gains::Nq> Copy of Q.
   */
  std::array<double, Gains::Nq> get_Q() const;

  /**
   * @brief Get the Qe matrix
   *
   * @return std::array<double, Gains::Nqe> Copy of Qe.
   */
  std::array<double, Gains::Nqe> get_Q_end() const;

  /**
   * @brief Get the R matrix
   *
   * @return std::array<double, Gains::Nr> Copy of R.
   */
  std::array<double, Gains::Nr> get_R() const;

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
  void set_gains(const Gains& gains);

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
   * @param Q std::array<double, Gains::Nq> Q.
   */
  void set_Q(const std::array<double, Gains::Nq>& Q);

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
   * @param R std::array<double, Gains::Nr> R.
   */
  void set_R(const std::array<double, Gains::Nr>& R);

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
   * @param Qe std::array<double, Gains::Nqe> Qe.
   */
  void set_Q_end(const std::array<double, Gains::Nqe>& Qe);
};

/**
 * @brief Bounds
 *
 * Bounds lbu and ubu for the MPC.
 */
struct Bounds {
  static constexpr size_t Nu = MPC_NU;
  std::array<double, MPC_NU> lbu;
  std::array<double, MPC_NU> ubu;

  /**
   * @brief Constructor
   */
  Bounds();

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
   * @brief Get the lbu array
   *
   * @return std::array<double, MPC_NU> lbu.
   */
  std::array<double, MPC_NU> get_lbu_array() const;

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
   * @brief Get the ubu array
   *
   * @return std::array<double, MPC_NU> ubu.
   */
  std::array<double, MPC_NU> get_ubu_array() const;

  /**
   * @brief Set the bounds
   *
   * @param bounds bounds.
   */
  void set_bounds(const Bounds& bounds);

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
 * @brief OnlineParams
 *
 * Online parameters p for the MPC.
 */
struct OnlineParams {
  static constexpr size_t Np = MPC_NP;
  std::array<double, (MPC_N + 1) * MPC_NP> data;
  static const std::size_t size_n = (MPC_N + static_cast<size_t>(1));
  static const std::size_t size   = size_n * MPC_NP;

  /**
   * @brief Constructor
   */
  OnlineParams();

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
   * @brief Get the online parameters
   *
   * @return std::array<double, MPC_NP> Copy of online parameters.
   */
  std::array<double, MPC_NP> get_online_params(const int index = 0) const;

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_online_params(const OnlineParams& params);

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
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
};

}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_
