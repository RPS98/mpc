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
 * @file acados_sim_solver.hpp
 *
 * Acados SIM solver class definition.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef ACADOS_MPC_ACADOS_SIM_SOLVER_HPP_
#define ACADOS_MPC_ACADOS_SIM_SOLVER_HPP_

#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <mpc_generated_code/acados_sim_solver_mpc.h>
#include <mpc_generated_code/acados_solver_mpc.h>
#include <mpc_generated_code/mpc_model/mpc_model.h>

#include <array>
#include <iostream>
#include <stdexcept>

#include "acados_mpc/acados_mpc.hpp"

namespace acados_mpc {

/**
 * @brief MPCSimSolver
 *
 * MPC simulation solver class using acados sim solver.
 */
class MPCSimSolver {
public:
  /**
   * @brief Constructor
   */
  MPCSimSolver();

  /**
   * @brief Destructor
   */
  ~MPCSimSolver();

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
   * @param data MPCData pointer.
   * @return int status.
   */
  int solve(MPCData *data);

private:
  /**
   * @brief Initialize the solver
   */
  void initializeSolver();

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
  mpc_sim_solver_capsule *capsule_;
  sim_in *sim_in_;
  sim_out *sim_out_;

  // Internal variables
  int status_;
};
}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_SIM_SOLVER_HPP_
