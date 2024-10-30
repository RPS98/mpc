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
 * @file acados_mpc.hpp
 *
 * Acados MPC class definition.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef ACADOS_MPC_ACADOS_MPC_HPP_
#define ACADOS_MPC_ACADOS_MPC_HPP_

#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <mpc_generated_code/acados_sim_solver_mpc.h>
#include <mpc_generated_code/acados_solver_mpc.h>
#include <mpc_generated_code/mpc_model/mpc_model.h>

#include <array>
#include <iostream>
#include <stdexcept>

#include "acados_mpc/acados_mpc_datatype.hpp"

namespace acados_mpc {

/**
 * @brief MPCData
 *
 * Data structure to hold the MPC data.
 *
 * @param state state.
 * @param control control.
 * @param reference reference.
 * @param reference_end reference_end.
 */
struct MPCData {
  State state;
  Control control;
  Reference reference;
  State reference_end;
};

/**
 * @brief MPC class
 *
 * MPC class to solve the MPC using acados.
 */
class MPC {
public:
  /**
   * @brief Constructor
   *
   * @param data MPCData pointer.
   */
  MPC();

  /**
   * @brief Destructor
   */
  ~MPC();

  /**
   * @brief Solve the MPC
   *
   * Return status:
   *  ACADOS_SUCCESS = 0
   *  ACADOS_NAN_DETECTED = 1
   *  ACADOS_MAXITER = 2
   *  ACADOS_MINSTEP = 3
   *  ACADOS_QP_FAILURE = 4
   *  ACADOS_READY = 5
   *  ACADOS_UNBOUNDED = 6
   *
   * @return int status.
   */
  int solve();

  // Getters

  /**
   * @brief Get the number of prediction steps.
   */
  inline int get_prediction_steps() const { return MPC_N; }

  /**
   * @brief Get the prediction time horizon in seconds.
   *
   * It is the prediction steps multiplied by the prediction time step.
   */
  inline double get_prediction_time_horizon() const { return MPC_N * *nlp_in_->Ts; }

  /**
   * @brief Get the prediction time step in seconds.
   */
  inline double get_prediction_time_step() const { return *nlp_in_->Ts; }

  /**
   * @brief Get the MPCData pointer to modify the data.
   */
  MPCData* get_data() { return &mpc_data_; }

  /**
   * @brief Get the MPCGains pointer to modify the gains.
   *
   * update_gains() must be called to update the gains.
   */
  MPCGains* get_gains() { return &gains_; }

  /**
   * @brief Get the MPCBounds pointer to modify the bounds.
   *
   * update_bounds() must be called to update the bounds.
   */
  MPCBounds* get_bounds() { return &bounds_; }

  /**
   * @brief Get the MPCOnlineParams pointer to modify the online parameters.
   *
   * update_online_params() must be called to update the online parameters.
   */
  MPCOnlineParams* get_online_params() { return &p_params_; }

  // Setters

  /**
   * @brief Update the gains Q, R and Qe.
   *
   * It uses the MPCGains pointer to update the gains.
   * It can be accessed using get_gains().
   */
  void update_gains();

  /**
   * @brief Update the bounds lbx and ubx.
   *
   * It uses the MPCBounds pointer to update the bounds.
   * It can be accessed using get_bounds().
   */
  void update_bounds();

  /**
   * @brief Update the online parameters p.
   *
   * It uses the MPCOnlineParams pointer to update the online parameters.
   * It can be accessed using get_online_params().
   */
  void update_online_params();

private:
  /**
   * @brief Initialize the solver
   */
  void initializeSolver();

  /**
   * @brief Set the solver state x0
   */
  void setSolverState();

  /**
   * @brief Set the solver reference yref
   */
  void setSolverRefence();

  /**
   * @brief Set the solver reference yref_N
   */
  void setSolverRefenceEnd();

  /**
   * @brief Validate the status
   *
   * @param status status.
   */
  inline void validateStatus(const int status) {
    if (status) {
      std::cerr << "acados_create() returned status " << status << std::endl;
    }
  }

private:
  // acados
  mpc_solver_capsule* capsule_ = nullptr;
  ocp_nlp_in* nlp_in_          = nullptr;
  ocp_nlp_out* nlp_out_        = nullptr;
  ocp_nlp_solver* nlp_solver_  = nullptr;
  ocp_nlp_config* nlp_config_  = nullptr;
  ocp_nlp_dims* nlp_dims_      = nullptr;

  // Internal variables
  int status_;
  double prediction_time_step_;
  MPCData mpc_data_         = MPCData();
  MPCGains gains_           = MPCGains();
  MPCBounds bounds_         = MPCBounds();
  MPCOnlineParams p_params_ = MPCOnlineParams();
};
}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_HPP_
