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
 * @author Rafael Perez-Segui, Carmen De Rojas Pita-Romero <r.psegui@upm.es> <c.derojas@upm.es>
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
  status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx",
                                          mpc_data_.state.data.data());
  validateStatus(status_);
  status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx",
                                          mpc_data_.state.data.data());
  validateStatus(status_);
}

void MPC::setSolverOnlineParams() {
  // initial values for parameter vector - can be updated stagewise
  for (int i = 0; i <= MPC_N; i++) {
    ocp_nlp_in_set(nlp_config_, nlp_dims_, nlp_in_, i, "parameter_values",
                   mpc_data_.p_params.get_data(i));
  }
}

void MPC::setSolverOnlineMassParams(const double value) {
  mpc_data_.p_params.set_data(MPC_NP, P_IDX_MASS, value);
}

void MPC::setSolverOnlineContourErrorGainParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_G_CONTOUR_ERROR; i< P_IDX_G_CONTOUR_ERROR +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_G_CONTOUR_ERROR]);
  }
}

void MPC::setSolverOnlineLagErrorGainParams(const double value) {
  mpc_data_.p_params.set_data(MPC_NP, P_IDX_G_LAG_ERROR, value);
}

void MPC::setSolverOnlineOrientationErrorGainParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_D_ORIENTATION; i< P_IDX_D_ORIENTATION +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_D_ORIENTATION]);
  }
}

void MPC::setSolverOnlineActuationGainParams(const std::array<double, 4> value) {
  for ( int i = P_IDX_G_ACTUATION; i< P_IDX_G_ACTUATION +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_G_ACTUATION]);
  }
}

void MPC::setSolverOnlineThetaVelocityGainParams(const double value) {
  mpc_data_.p_params.set_data(MPC_NP, P_IDX_G_THETA_VELOCITY, value);
}

void MPC::setSolverOnlineProgressGainParams(const double value) {
  mpc_data_.p_params.set_data(MPC_NP, P_IDX_G_PROGRESS, value);
}

void MPC::setSolverOnlineS1PParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_S1_P; i< P_IDX_S1_P +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S1_P]);
  }
}

void MPC::setSolverOnlineS1MParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_S1_M; i< P_IDX_S1_M +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S1_M]);
  }
}

void MPC::setSolverOnlineS2PParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_S2_P; i< P_IDX_S2_P +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S2_P]);
  }
}

void MPC::setSolverOnlineS2MParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_S2_M; i< P_IDX_S2_M +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S2_M]);
  }
}

void MPC::setSolverOnlineS3PParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_S3_P; i< P_IDX_S3_P +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S3_P]);
  }
}

void MPC::setSolverOnlineS3MParams(const std::array<double, 3> value) {
  for ( int i = P_IDX_S3_M; i< P_IDX_S3_M +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S3_M]);
  }
}

void MPC::setSolverOnlineSLengthParams(const double value) {
  mpc_data_.p_params.set_data(MPC_NP, P_IDX_S_LENGTH, value);
}

void MPC::setSolverOnlineSPolyCoeffsParams(const std::array<double, 6> value) {
  for ( int i = P_IDX_S_POLY_COEFFS; i< P_IDX_S_POLY_COEFFS +value.size(); i++ ){
  mpc_data_.p_params.set_data(MPC_NP, i, value[i - P_IDX_S_POLY_COEFFS]);
  }
}

int MPC::solve() {
  // Set solver state and reference
  setSolverState();
  setSolverOnlineParams();

  // Solve OCP
  status_ = mpc_acados_solve(capsule_);
  validateStatus(status_);

  // Get solution
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", mpc_data_.actuation.data.data());

  if (status_ != 0) {
    std::cerr << "MPC solver returned status " << status_ << std::endl;
  }

  return status_;
}


void MPC::update_online_parameters() {
  // ToDo(Carmendrpr): check if this is correct
  for (int i = 0; i <= MPC_N; i++) {
  ocp_nlp_in_set(nlp_config_, nlp_dims_, nlp_in_, i, "parameter_values",
                   mpc_data_.p_params.get_data());
    validateStatus(status_);
  }
}

void MPC::update_actuation_bounds() {
  // lower actuation_bounds on u at shooting nodes (0 to N-1)
  // upper actuation_bounds on u at shooting nodes (0 to N-1)
  for (int i = 0; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lbu",
                                            actuation_bounds_.lbu.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "ubu",
                                            actuation_bounds_.ubu.data());
    validateStatus(status_);
  }
}

void MPC::update_state_bounds() {
  // lower state_bounds on u at shooting nodes (0 to N-1)
  // upper state_bounds on u at shooting nodes (0 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lbx",
                                            state_bounds_.lbx.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "ubx",
                                            state_bounds_.ubx.data());
    validateStatus(status_);
  }
}

void MPC::update_soft_state_bounds() {
  // lower soft state bounds at shooting nodes (1 to N-1)
  // upper soft state bounds at shooting nodes (1 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lsbx",
                                            soft_state_bounds_.lsbx.data());
    validateStatus(status_);
    status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "usbx",
                                            soft_state_bounds_.usbx.data());
    validateStatus(status_);
  }

  // lower soft state bounds at terminal shooting node (N)
  // upper soft state bounds at terminal shooting node (N)
  status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, MPC_N, "lsbx",
                                          soft_state_bounds_.lsbx.data());
  validateStatus(status_);
  status_ = ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, MPC_N, "usbx",
                                          soft_state_bounds_.usbx.data());
  validateStatus(status_);
}

void MPC::update_slack_weights() {
  // slack weights at shooting nodes (1 to N-1)
  for (int i = 1; i < MPC_N; i++) {
    status_ =
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "Zl", slack_weights_.Zl.data());
    validateStatus(status_);
    status_ =
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "Zu", slack_weights_.Zu.data());
    validateStatus(status_);
    status_ =
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "zl", slack_weights_.zl.data());
    validateStatus(status_);
    status_ =
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "zu", slack_weights_.zu.data());
    validateStatus(status_);
  }
}

void MPC::update_slack_weights_end() {
  // slack weights at terminal shooting node (N)
  status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_N, "Zl",
                                   slack_weights_end_.Zl_e.data());
  validateStatus(status_);
  status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_N, "Zu",
                                   slack_weights_end_.Zu_e.data());
  validateStatus(status_);
  status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_N, "zl",
                                   slack_weights_end_.zl_e.data());
  validateStatus(status_);
  status_ = ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_N, "zu",
                                   slack_weights_end_.zu_e.data());
  validateStatus(status_);
}

}  // namespace acados_mpc
