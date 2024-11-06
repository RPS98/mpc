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
  ocp_nlp_dims_destroy(nlp_dims_);
  ocp_nlp_config_destroy(nlp_config_);
  ocp_nlp_solver_destroy(nlp_solver_);
  ocp_nlp_out_destroy(nlp_out_);
  ocp_nlp_in_destroy(nlp_in_);
  mpc_acados_free_capsule(capsule_);
}

void MPC::initializeSolver() {
  // Acados capsule
  capsule_ = mpc_acados_create_capsule();

  // Create acados solver
  status_ = mpc_acados_create(capsule_);
  validateStatus(status_);

  // Get acados structs
  nlp_in_     = mpc_acados_get_nlp_in(capsule_);
  nlp_out_    = mpc_acados_get_nlp_out(capsule_);
  nlp_solver_ = mpc_acados_get_nlp_solver(capsule_);
  nlp_config_ = mpc_acados_get_nlp_config(capsule_);
  nlp_dims_   = mpc_acados_get_nlp_dims(capsule_);
}

void MPC::setSolverState() {
  status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx",
                                          mpc_data_.state.data.data());
  validateStatus(status_);
  status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx",
                                          mpc_data_.state.data.data());
  validateStatus(status_);
}

void MPC::setSolverRefence() {
  for (int i = 0; i < MPC_N; i++) {
    status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref",
                                     mpc_data_.reference.get_data(i));
    validateStatus(status_);
  }
}

void MPC::setSolverRefenceEnd() {
  status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_N, "yref",
                                   mpc_data_.reference_end.data.data());
  validateStatus(status_);
}

void MPC::setSolverOnlineParams() {
  // initial values for parameter vector - can be updated stagewise
  for (int i = 0; i <= MPC_N + 1; i++) {
    ocp_nlp_in_set(nlp_config_, nlp_dims_, nlp_in_, i, "parameter_values",
                   mpc_data_.p_params.get_data());
  }
}

int MPC::solve() {
  // Set solver state and reference
  setSolverState();
  setSolverRefence();
  setSolverRefenceEnd();
  setSolverOnlineParams();

  // Solve OCP
  status_ = mpc_acados_solve(capsule_);
  validateStatus(status_);

  // Get solution
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", mpc_data_.control.data.data());

  return status_;
}

void MPC::update_gains() {
  // weight matrix at intermediate shooting nodes (1 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", gains_.get_W());
    validateStatus(status_);
  }

  // weight matrix at terminal shooting node (N)
  status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_N, "W", gains_.get_We());
}

void MPC::update_bounds() {
  // lower bounds on u at shooting nodes (0 to N-1)
  // upper bounds on u at shooting nodes (0 to N-1)
  for (int i = 0; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "lbu",
                                            bounds_.lbu.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "ubu",
                                            bounds_.ubu.data());
    validateStatus(status_);
  }
}

}  // namespace acados_mpc
