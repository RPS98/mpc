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
 * @file acados_mpc.cpp
 *
 * Acados MPC class implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "acados_mpc/acados_mpc.hpp"

namespace acados_mpc {

MPC::MPC() { initializeSolver(); }

MPC::~MPC() {
  ocp_nlp_dims_destroy(acados_pointers_.nlp_dims);
  ocp_nlp_config_destroy(acados_pointers_.nlp_config);
  ocp_nlp_solver_destroy(acados_pointers_.nlp_solver);
  ocp_nlp_out_destroy(acados_pointers_.nlp_out);
  ocp_nlp_in_destroy(acados_pointers_.nlp_in);
  mpc_acados_free_capsule(acados_pointers_.capsule);
}

void MPC::initializeSolver() {
  // Acados capsule
  acados_pointers_.capsule = mpc_acados_create_capsule();

  // Create acados solver
  status_ = mpc_acados_create(acados_pointers_.capsule);
  validateStatus(status_);

  // Get acados structs
  acados_pointers_.nlp_in     = mpc_acados_get_nlp_in(acados_pointers_.capsule);
  acados_pointers_.nlp_out    = mpc_acados_get_nlp_out(acados_pointers_.capsule);
  acados_pointers_.nlp_solver = mpc_acados_get_nlp_solver(acados_pointers_.capsule);
  acados_pointers_.nlp_config = mpc_acados_get_nlp_config(acados_pointers_.capsule);
  acados_pointers_.nlp_dims   = mpc_acados_get_nlp_dims(acados_pointers_.capsule);
}

void MPC::setSolverState() {
  status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                          acados_pointers_.nlp_in, acados_pointers_.nlp_out, 0,
                                          "lbx", mpc_data_.state.data.data());
  validateStatus(status_);
  status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                          acados_pointers_.nlp_in, acados_pointers_.nlp_out, 0,
                                          "ubx", mpc_data_.state.data.data());
  validateStatus(status_);
}

void MPC::setSolverRefence() {
  for (int i = 0; i < MPC_N; i++) {
    status_ =
        ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                               acados_pointers_.nlp_in, i, "yref", mpc_data_.reference.getData(i));
    validateStatus(status_);
  }
}

void MPC::setSolverRefenceEnd() {
  status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                   acados_pointers_.nlp_in, MPC_N, "yref",
                                   mpc_data_.reference_end.data.data());
  validateStatus(status_);
}

void MPC::setSolverParameters() {
  // Apply the stage-wise online parameter vector across the horizon.
  for (int i = 0; i < OnlineParameters::Nstages; i++) {
    status_ = mpc_acados_update_params(acados_pointers_.capsule, i, mpc_data_.p_params.getData(i),
                                       OnlineParameters::Np);
    validateStatus(status_);
  }
}

int MPC::solve() {
  // Set solver state and reference
  setSolverState();
  setSolverRefence();
  setSolverRefenceEnd();
  setSolverParameters();

  // Solve OCP
  status_ = mpc_acados_solve(acados_pointers_.capsule);
  validateStatus(status_);

  // Get solution
  ocp_nlp_out_get(acados_pointers_.nlp_config, acados_pointers_.nlp_dims, acados_pointers_.nlp_out,
                  0, "u", mpc_data_.actuation.data.data());

  if (status_ != 0) {
    std::cerr << "MPC solver returned status " << status_ << std::endl;
  }

  return status_;
}

void MPC::updateTimeStep(const double time_step) {
  for (int i = 0; i <= MPC_N; i++) {
    prediction_time_steps_[i] = time_step;
  }
  mpc_acados_update_time_steps(acados_pointers_.capsule, getPredictionSteps(),
                               prediction_time_steps_.data());
}

void MPC::updateTimeStep(const std::array<double, MPC_N> time_steps) {
  prediction_time_steps_ = time_steps;
  mpc_acados_update_time_steps(acados_pointers_.capsule, getPredictionSteps(),
                               prediction_time_steps_.data());
}

void MPC::updateGains() {
  // weight matrix at intermediate shooting nodes (1 to N-1)
  for (int i = 0; i < MPC_N; i++) {
    status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                     acados_pointers_.nlp_in, i, "W", gains_.getW());
    validateStatus(status_);
  }

  // weight matrix at terminal shooting node (N)
  status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                   acados_pointers_.nlp_in, MPC_N, "W", gains_.getWe());
  validateStatus(status_);
}

void MPC::updateActuationBounds() {
  // lower actuation_bounds on u at shooting nodes (0 to N-1)
  // upper actuation_bounds on u at shooting nodes (0 to N-1)
  for (int i = 0; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                            acados_pointers_.nlp_in, acados_pointers_.nlp_out, i,
                                            "lbu", actuation_bounds_.lbu.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                            acados_pointers_.nlp_in, acados_pointers_.nlp_out, i,
                                            "ubu", actuation_bounds_.ubu.data());
    validateStatus(status_);
  }
}

void MPC::updateStateBounds() {
  if (MPC_NBX == 0) {
    return;
  }

  // lower state_bounds on u at shooting nodes (0 to N-1)
  // upper state_bounds on u at shooting nodes (0 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                            acados_pointers_.nlp_in, acados_pointers_.nlp_out, i,
                                            "lbx", state_bounds_.lbx.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                            acados_pointers_.nlp_in, acados_pointers_.nlp_out, i,
                                            "ubx", state_bounds_.ubx.data());
    validateStatus(status_);
  }
}

void MPC::updateSoftStateBounds() {
  if (MPC_NSBX == 0) {
    return;
  }

  // lower soft state bounds at shooting nodes (1 to N-1)
  // upper soft state bounds at shooting nodes (1 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                            acados_pointers_.nlp_in, acados_pointers_.nlp_out, i,
                                            "lsbx", soft_state_bounds_.lsbx.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                            acados_pointers_.nlp_in, acados_pointers_.nlp_out, i,
                                            "usbx", soft_state_bounds_.usbx.data());
    validateStatus(status_);
  }

  // lower soft state bounds at terminal shooting node (N)
  // upper soft state bounds at terminal shooting node (N)
  status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                          acados_pointers_.nlp_in, acados_pointers_.nlp_out, MPC_N,
                                          "lsbx", soft_state_bounds_.lsbx.data());
  validateStatus(status_);
  status_ = ocp_nlp_constraints_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                          acados_pointers_.nlp_in, acados_pointers_.nlp_out, MPC_N,
                                          "usbx", soft_state_bounds_.usbx.data());
  validateStatus(status_);
}

void MPC::updateSlackWeights() {
  if (MPC_NSBX == 0) {
    return;
  }

  // slack weights at shooting nodes (1 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                     acados_pointers_.nlp_in, i, "Zl", slack_weights_.Zl.data());
    validateStatus(status_);
    status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                     acados_pointers_.nlp_in, i, "Zu", slack_weights_.Zu.data());
    validateStatus(status_);
    status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                     acados_pointers_.nlp_in, i, "zl", slack_weights_.zl.data());
    validateStatus(status_);
    status_ = ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                                     acados_pointers_.nlp_in, i, "zu", slack_weights_.zu.data());
    validateStatus(status_);
  }
}

void MPC::updateSlackWeightsEnd() {
  if (MPC_NSBXN == 0) {
    return;
  }

  // slack weights at terminal shooting node (N)
  status_ =
      ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                             acados_pointers_.nlp_in, MPC_N, "Zl", slack_weights_end_.Zl_e.data());
  validateStatus(status_);
  status_ =
      ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                             acados_pointers_.nlp_in, MPC_N, "Zu", slack_weights_end_.Zu_e.data());
  validateStatus(status_);
  status_ =
      ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                             acados_pointers_.nlp_in, MPC_N, "zl", slack_weights_end_.zl_e.data());
  validateStatus(status_);
  status_ =
      ocp_nlp_cost_model_set(acados_pointers_.nlp_config, acados_pointers_.nlp_dims,
                             acados_pointers_.nlp_in, MPC_N, "zu", slack_weights_end_.zu_e.data());
  validateStatus(status_);
}

}  // namespace acados_mpc
