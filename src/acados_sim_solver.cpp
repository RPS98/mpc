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
 * @file acados_sim_solver.cpp
 *
 * Acados SIM solver class implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "acados_mpc/acados_sim_solver.hpp"

namespace acados_mpc {

MPCSimSolver::MPCSimSolver() { initializeSolver(); }

MPCSimSolver::~MPCSimSolver() {}

void MPCSimSolver::initializeSolver() {
  capsule_ = mpc_acados_sim_solver_create_capsule();
  status_  = mpc_acados_sim_create(capsule_);
  validateStatus(status_);

  // Get acados structs
  sim_in_  = mpc_acados_get_sim_in(capsule_);
  sim_out_ = mpc_acados_get_sim_out(capsule_);
}

int MPCSimSolver::solve(MPCData *data) {
  // Set initial state
  for (int i = 0; i < MPC_NX; i++) {
    sim_in_->x[i] = data->state.data[i];
  }

  // Set control input
  for (int i = 0; i < MPC_NU; i++) {
    sim_in_->u[i] = data->control.data[i];
  }

  // Solve
  status_ = mpc_acados_sim_solve(capsule_);
  validateStatus(status_);

  // Get solution
  for (int i = 0; i < MPC_NX; i++) {
    data->state.data[i] = sim_out_->xn[i];
  }

  return status_;
}

}  // namespace acados_mpc
