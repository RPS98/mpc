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
 * @brief AcadosSolverPointers
 *
 * Data structure to hold the acados solver pointers.
 *
 * @param capsule Acados solver capsule.
 * @param nlp_in Acados NLP input.
 * @param nlp_out Acados NLP output.
 * @param nlp_solver Acados NLP solver.
 * @param nlp_config Acados NLP configuration.
 * @param nlp_dims Acados NLP dimensions.
 */
struct AcadosSolverPointers {
  mpc_solver_capsule* capsule = nullptr;
  ocp_nlp_in* nlp_in          = nullptr;
  ocp_nlp_out* nlp_out        = nullptr;
  ocp_nlp_solver* nlp_solver  = nullptr;
  ocp_nlp_config* nlp_config  = nullptr;
  ocp_nlp_dims* nlp_dims      = nullptr;
};

/**
 * @brief MPCData
 *
 * Data structure to hold the MPC data.
 *
 * @param state state.
 * @param actuation actuation.
 * @param p_params online parameters.
 * @param reference reference.
 * @param reference_end reference_end.
 */
struct MPCData {
  State state;
  Actuation actuation;
  OnlineParameters p_params;
  Reference reference;
  ReferenceEnd reference_end;
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
  inline int getPredictionSteps() const { return MPC_N; }

  /**
   * @brief Get the prediction time horizon in seconds.
   *
   * It is the prediction steps multiplied by the prediction time step.
   */
  inline double getPredictionTimeHorizon() const { return MPC_N * *acados_pointers_.nlp_in->Ts; }

  /**
   * @brief Get the prediction time step in seconds.
   */
  inline double getPredictionTimeStep() const { return *acados_pointers_.nlp_in->Ts; }

  /**
   * @brief Get the MPCData pointer to modify the data.
   */
  MPCData* getData() { return &mpc_data_; }

  /**
   * @brief Get the OnlineParameters pointer to modify online parameters.
   */
  OnlineParameters* getParameters() { return &mpc_data_.p_params; }

  /**
   * @brief Set all online parameters at once.
   *
   * @param params online parameters to copy.
   */
  void setParameters(const OnlineParameters& params) { mpc_data_.p_params.setParameters(params); }

  /**
   * @brief Set one stage parameter vector or broadcast it to all stages.
   *
   * @param params stage parameters to copy.
   * @param stage stage index, or -1 to broadcast.
   */
  void setParameters(const Parameters& params, const int stage = -1) {
    mpc_data_.p_params.setParameters(params, stage);
  }

  /**
   * @brief Get the Gains pointer to modify the gains.
   *
   * updateGains() must be called to update the gains.
   */
  Gains* getGains() { return &gains_; }

  /**
   * @brief Get the ActuationBounds pointer to modify the actuation_bounds.
   *
   * updateActuationBounds() must be called to update the actuation_bounds.
   */
  ActuationBounds* getActuationBounds() { return &actuation_bounds_; }

  /**
   * @brief Get the StateBounds pointer to modify the state_bounds.
   *
   * updateStateBounds() must be called to update the state_bounds.
   */
  StateBounds* getStateBounds() { return &state_bounds_; }

  /**
   * @brief Get the SoftStateBounds pointer to modify the soft_state_bounds.
   *
   * updateSoftStateBounds() must be called to update the soft_state_bounds.
   */
  SoftStateBounds* getSoftStateBounds() { return &soft_state_bounds_; }

  /**
   * @brief Get the SlackWeights pointer to modify the slack_weights.
   *
   * updateSlackWeights() must be called to update the slack_weights.
   */
  SlackWeights* getSlackWeights() { return &slack_weights_; }

  /**
   * @brief Get the SlackWeightsEnd pointer to modify the slack_weights_end.
   *
   * updateSlackWeightsEnd() must be called to update the slack_weights_end.
   */
  SlackWeightsEnd* getSlackWeightsEnd() { return &slack_weights_end_; }

  /**
   * @brief Get the NonlinearConstraintBounds pointer to modify the nonlinear_constraint_bounds.
   *
   * updateNonlinearConstraintBounds() must be called to apply changes to the solver.
   */
  NonlinearConstraintBounds* getNonlinearConstraintBounds() {
    return &nonlinear_constraint_bounds_;
  }

  /**
   * @brief Get the SoftNonlinearConstraintBounds pointer to modify soft nonlinear bounds.
   *
   * updateSoftNonlinearConstraintBounds() must be called to apply changes to the solver.
   */
  SoftNonlinearConstraintBounds* getSoftNonlinearConstraintBounds() {
    return &soft_nonlinear_constraint_bounds_;
  }

  /**
   * @brief Get the AcadosSolverPointers to access internal acados structures.
   *
   * Allows direct access to acados solver internals for advanced operations.
   */
  const AcadosSolverPointers* getAcadosSolverPointers() const { return &acados_pointers_; }

  /**
   * @brief Update the time step used in the prediction model.
   *
   * @param time_step time step in seconds.
   */
  void updateTimeStep(const double time_step);

  /**
   * @brief Update the time steps used in the prediction model.
   *
   * @param time_steps array of time steps in seconds.
   */
  void updateTimeStep(std::array<double, MPC_N> time_steps);

  /**
   * @brief Update the gains Q, R and Qe.
   *
   * It uses the Gains pointer to update the gains.
   * It can be accessed using getGains().
   */
  void updateGains();

  /**
   * @brief Update the actuation_bounds lbx and ubx.
   *
   * It uses the ActuationBounds pointer to update the actuation_bounds.
   * It can be accessed using getActuationBounds().
   */
  void updateActuationBounds();

  /**
   * @brief Update the state_bounds lbx and ubx.
   *
   * It uses the StateBounds pointer to update the state_bounds.
   * It can be accessed using getStateBounds().
   */
  void updateStateBounds();

  /**
   * @brief Update the soft_state_bounds lsbx and usbx.
   *
   * It uses the SoftStateBounds pointer to update the soft_state_bounds.
   * It can be accessed using getSoftStateBounds().
   */
  void updateSoftStateBounds();

  /**
   * @brief Update the slack_weights Zl, Zu, zl, zu.
   *
   * It uses the SlackWeights pointer to update the slack_weights.
   * It can be accessed using getSlackWeights().
   */
  void updateSlackWeights();

  /**
   * @brief Update the slack_weights_end Zl_e, Zu_e, zl_e, zu_e.
   *
   * It uses the SlackWeightsEnd pointer to update the slack_weights_end.
   * It can be accessed using getSlackWeightsEnd().
   */
  void updateSlackWeightsEnd();

  /**
   * @brief Update the nonlinear constraint bounds lh and uh.
   *
   * It uses the NonlinearConstraintBounds pointer to update the bounds.
   * It can be accessed using getNonlinearConstraintBounds().
   */
  void updateNonlinearConstraintBounds();

  /**
   * @brief Update the soft nonlinear constraint bounds lsh and ush.
   *
   * It uses the SoftNonlinearConstraintBounds pointer to update the bounds.
   * It can be accessed using getSoftNonlinearConstraintBounds().
   */
  void updateSoftNonlinearConstraintBounds();

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
   * @brief Set the solver online parameters p
   */
  void setSolverParameters();

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
  AcadosSolverPointers acados_pointers_;

  // Internal variables
  int status_ = 0;
  std::array<double, MPC_N> prediction_time_steps_{};

  // Dynamic input
  MPCData mpc_data_ = MPCData();

  // Parameters
  Gains gains_                                                    = Gains();
  ActuationBounds actuation_bounds_                               = ActuationBounds();
  StateBounds state_bounds_                                       = StateBounds();
  SoftStateBounds soft_state_bounds_                              = SoftStateBounds();
  SlackWeights slack_weights_                                     = SlackWeights();
  SlackWeightsEnd slack_weights_end_                              = SlackWeightsEnd();
  NonlinearConstraintBounds nonlinear_constraint_bounds_          = NonlinearConstraintBounds();
  SoftNonlinearConstraintBounds soft_nonlinear_constraint_bounds_ = SoftNonlinearConstraintBounds();
};
}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_HPP_
