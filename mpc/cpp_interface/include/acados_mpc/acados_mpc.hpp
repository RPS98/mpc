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
 * @author Rafael Perez-Segui, Carmen De Rojas Pita-Romero <r.psegui@upm.es> <c.derojas@upm.es>
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

static constexpr int P_IDX_MASS    = 0;
static constexpr int P_IDX_D_ORIENTATION = 1;
static constexpr int P_IDX_G_CONTOUR_ERROR    = 5;
static constexpr int P_IDX_G_LAG_ERROR    = 8;
static constexpr int P_IDX_G_ORIENTATION    = 9;
static constexpr int P_IDX_G_ACTUATION    = 12;
static constexpr int P_IDX_G_THETA_VELOCITY    = 16;
static constexpr int P_IDX_G_PROGRESS = 17;
static constexpr int P_IDX_S1_P = 18;
static constexpr int P_IDX_S1_M = 21;
static constexpr int P_IDX_S2_P = 24;
static constexpr int P_IDX_S2_M = 27;
static constexpr int P_IDX_S3_P = 30;
static constexpr int P_IDX_S3_M = 33;
static constexpr int P_IDX_S_LENGTH = 36;
static constexpr int P_IDX_S_POLY_COEFFS = 37;

/**
 * @brief MPCData
 *
 * Data structure to hold the MPC data.
 *
 * @param state state.
 * @param p_params online parameters.
 * @param actuation actuation.
 */
struct MPCData {
  State state;
  OnlineParams p_params;
  Actuation actuation;
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
   */
  MPC();

  /**
   * @brief Destructor
   */
  ~MPC();

  /**
   * @brief Solve the MPC
   *
   * MPCData must be set before calling this function.
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
   * @brief Get the ActuationBounds pointer to modify the actuation_bounds.
   *
   * update_actuation_bounds() must be called to update the actuation_bounds.
   */
  ActuationBounds* get_actuation_bounds() { return &actuation_bounds_; }

  /**
   * @brief Get the StateBounds pointer to modify the state_bounds.
   *
   * update_state_bounds() must be called to update the state_bounds.
   */
  StateBounds* get_state_bounds() { return &state_bounds_; }

  /**
   * @brief Get the SoftStateBounds pointer to modify the soft_state_bounds.
   *
   * update_soft_state_bounds() must be called to update the soft_state_bounds.
   */
  SoftStateBounds* get_soft_state_bounds() { return &soft_state_bounds_; }

  /**
   * @brief Get the SlackWeights pointer to modify the slack_weights.
   *
   * update_slack_weights() must be called to update the slack_weights.
   */
  SlackWeights* get_slack_weights() { return &slack_weights_; }

  /**
   * @brief Get the SlackWeightsEnd pointer to modify the slack_weights_end.
   *
   * update_slack_weights_end() must be called to update the slack_weights_end.
   */
  SlackWeightsEnd* get_slack_weights_end() { return &slack_weights_end_; }

  // Setters

   /**
   * @brief Set the solver online mass parameter of p
   */
  void setSolverOnlineMassParams(const double value);

  /**
   * @brief Set the solver online contouring error gain parameter of p
   */
  void setSolverOnlineContourErrorGainParams(const std::array<double, 3> value);

  /**
   * @brief Set the solver online lag error gain parameter of p
   */
  void setSolverOnlineLagErrorGainParams(const double value);

  /** 
   * @brief Set the solver online orientation error gain parameter of p
  */
  void setSolverOnlineOrientationErrorGainParams(const std::array<double, 3> value);

  /** 
  * @brief Set the solver online actuation error gain parameter of p
  */
  void setSolverOnlineActuationGainParams(const std::array<double, 4> value);

  /** 
  * @brief Set the solver online theta velocity gain parameter of p
  */
  void setSolverOnlineThetaVelocityGainParams(const double value);

  /**
   * @brief Set the solver online progress gain parameter of p
   */
  void setSolverOnlineProgressGainParams(const double value);

  /**
   * @brief Set the solver online s1_p parameters of p
   */
  void setSolverOnlineS1PParams(const std::array<double, 3> value);

  /**
   * @brief Set the solver online s1_m parameters of p
   */
  void setSolverOnlineS1MParams(const std::array<double, 3> value);

  /**
   * @brief Set the solver online s2_p parameters of p
   */
  void setSolverOnlineS2PParams(const std::array<double, 3> value);

  /**
   * @brief Set the solver online s2_m parameters of p
   */
  void setSolverOnlineS2MParams(const std::array<double, 3> value);

  /** 
   * @brief Set the solver online s3_p parameters of p
   */
  void setSolverOnlineS3PParams(const std::array<double, 3> value);

  /**
   * @brief Set the solver online s3_m parameters of p
   */
  void setSolverOnlineS3MParams(const std::array<double, 3> value);

  /**
   * @brief Set the solver online s_length parameters of p
   */
  void setSolverOnlineSLengthParams(const double value);

  /**
   * @brief Set the solver online s_poly_coeffs parameters of p
   */
  void setSolverOnlineSPolyCoeffsParams(const std::array<double, 6> value);

  /**
   * @brief Update the actuation_bounds lbx and ubx.
   *
   * It uses the ActuationBounds pointer to update the actuation_bounds.
   * It can be accessed using get_actuation_bounds().
   */
  void update_actuation_bounds();

  /**
   * @brief Update the state_bounds lbx and ubx.
   *
   * It uses the StateBounds pointer to update the state_bounds.
   * It can be accessed using get_state_bounds().
   */
  void update_state_bounds();

  /** 
   * @brief Update the parameters p.
   * It uses the OnlineParams pointer in MPCData to update the parameters p.
   */
  void update_online_parameters();

  /**
   * @brief Update the soft_state_bounds lsbx and usbx.
   *
   * It uses the SoftStateBounds pointer to update the soft_state_bounds.
   * It can be accessed using get_soft_state_bounds().
   */
  void update_soft_state_bounds();

  /**
   * @brief Update the slack_weights Zl, Zu, zl, zu.
   *
   * It uses the SlackWeights pointer to update the slack_weights.
   * It can be accessed using get_slack_weights().
   */
  void update_slack_weights();

  /**
   * @brief Update the slack_weights_end Zl_e, Zu_e, zl_e, zu_e.
   *
   * It uses the SlackWeightsEnd pointer to update the slack_weights_end.
   * It can be accessed using get_slack_weights_end().
   */
  void update_slack_weights_end();

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
   * @brief Set the solver online parameters p
   */
  void setSolverOnlineParams();

 
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

  // Dynamic input
  MPCData mpc_data_ = MPCData();

  // Parameters
  ActuationBounds actuation_bounds_  = ActuationBounds();
  StateBounds state_bounds_          = StateBounds();
  SoftStateBounds soft_state_bounds_ = SoftStateBounds();
  SlackWeights slack_weights_        = SlackWeights();
  SlackWeightsEnd slack_weights_end_ = SlackWeightsEnd();
};
}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_HPP_
