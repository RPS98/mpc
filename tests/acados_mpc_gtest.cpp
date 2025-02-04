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
 * @file acados_mpc_gtest.cpp
 *
 * Acados MPC gtest tests.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <gtest/gtest.h>
#include <memory>
#include "acados_mpc/acados_mpc.hpp"
#include "acados_mpc/acados_mpc_datatype.hpp"
#include "acados_mpc/acados_sim_solver.hpp"

namespace acados_mpc {

TEST(acados_mpc, test_acados_mpc) {
  MPCData mpc_data;
  EXPECT_NO_THROW(MPC());
  auto mpc = MPC();

  EXPECT_NO_THROW(mpc.solve());
  EXPECT_NO_THROW(mpc.get_prediction_steps());
  EXPECT_NO_THROW(mpc.get_prediction_time_horizon());
  EXPECT_NO_THROW(mpc.get_prediction_time_step());
  EXPECT_NO_THROW(mpc.get_data());
  EXPECT_NO_THROW(mpc.get_gains());
  EXPECT_NO_THROW(mpc.get_bounds());
  EXPECT_NO_THROW(mpc.update_gains());
  EXPECT_NO_THROW(mpc.update_bounds());
}

TEST(acados_mpc, test_acados_datatypes) {
  EXPECT_NO_THROW(State());
  auto state = State();
  EXPECT_NO_THROW(state.set_data(0, 0.0));

  EXPECT_NO_THROW(Control());
  auto control = Control();
  EXPECT_NO_THROW(control.set_data(0, 0.0));

  EXPECT_NO_THROW(Reference());
  auto reference = Reference();
  EXPECT_NO_THROW(reference.get_data(0));
  EXPECT_NO_THROW(reference.get_state(0));
  EXPECT_NO_THROW(reference.set_data(0, 0.0));
  EXPECT_NO_THROW(reference.set_state(0, State(), Control()));

  EXPECT_NO_THROW(ReferenceEnd());
  auto reference_end = ReferenceEnd();
  EXPECT_NO_THROW(reference_end.get_data());
  EXPECT_NO_THROW(reference_end.set_data(0, 0.0));

  EXPECT_NO_THROW(Gains());
  auto gains = Gains();
  EXPECT_NO_THROW(gains.get_W());
  EXPECT_NO_THROW(gains.get_We());
  EXPECT_NO_THROW(gains.get_Q());
  EXPECT_NO_THROW(gains.get_Q_end());
  EXPECT_NO_THROW(gains.get_R());
  EXPECT_NO_THROW(gains.set_W(0, 0.0));
  EXPECT_NO_THROW(gains.set_We(0, 0.0));
  EXPECT_NO_THROW(gains.set_Q(0, 0.0));
  EXPECT_NO_THROW(gains.set_R(0, 0.0));
  EXPECT_NO_THROW(gains.set_Q_end(0, 0.0));

  EXPECT_NO_THROW(Bounds());
  auto bounds = Bounds();
  EXPECT_NO_THROW(bounds.get_lbu());
  EXPECT_NO_THROW(bounds.get_ubu());
  EXPECT_NO_THROW(bounds.set_lbu(0, 0.0));
  EXPECT_NO_THROW(bounds.set_ubu(0, 0.0));

  EXPECT_NO_THROW(OnlineParams());
  auto p_params = OnlineParams();
  EXPECT_NO_THROW(p_params.get_data());
  EXPECT_NO_THROW(p_params.get_data(0));
  EXPECT_NO_THROW(p_params.set_data(0, 0.0));
}

TEST(acados_mpc, test_acados_sim_solver) {
  EXPECT_NO_THROW(MPCSimSolver());
  auto sim_solver = MPCSimSolver();
  auto mpc_data   = MPCData();
  EXPECT_NO_THROW(sim_solver.solve(&mpc_data));
}
}  // namespace acados_mpc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
