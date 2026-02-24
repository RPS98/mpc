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

TEST(acadosMpc, testAcadosMpc) {
  MPCData mpc_data;
  EXPECT_NO_THROW(MPC());
  auto mpc = MPC();

  EXPECT_NO_THROW(mpc.solve());
  EXPECT_NO_THROW(mpc.getPredictionSteps());
  EXPECT_NO_THROW(mpc.getPredictionTimeHorizon());
  EXPECT_NO_THROW(mpc.getPredictionTimeStep());
  EXPECT_NO_THROW(mpc.getData());
  EXPECT_NO_THROW(mpc.getOnlineParams());
  auto online_params = OnlineParams();
  online_params.setMass(1.5);
  EXPECT_NO_THROW(mpc.setOnlineParams(online_params));
  EXPECT_DOUBLE_EQ(mpc.getOnlineParams()->getMass(), 1.5);
  EXPECT_NO_THROW(mpc.getGains());
  EXPECT_NO_THROW(mpc.getActuationBounds());
  EXPECT_NO_THROW(mpc.getStateBounds());
  EXPECT_NO_THROW(mpc.updateGains());
  EXPECT_NO_THROW(mpc.updateActuationBounds());
  EXPECT_NO_THROW(mpc.updateStateBounds());
}

TEST(acadosMpc, testAcadosDatatypes) {
  EXPECT_NO_THROW(State());
  EXPECT_EQ(State::position_offset, 0u);
  EXPECT_EQ(State::position_length, 3u);
  EXPECT_EQ(State::orientation_offset, 3u);
  EXPECT_EQ(State::orientation_length, 4u);
  EXPECT_EQ(State::linear_velocity_offset, 7u);
  EXPECT_EQ(State::linear_velocity_length, 3u);
  auto state = State();
  EXPECT_NO_THROW(state.setPosition(std::array<double, 3>{1.0, 2.0, 3.0}));
  EXPECT_NO_THROW(state.setOrientation(std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
  EXPECT_NO_THROW(state.setLinearVelocity(std::array<double, 3>{4.0, 5.0, 6.0}));
  auto position        = state.getPosition();
  auto orientation     = state.getOrientation();
  auto linear_velocity = state.getLinearVelocity();
  EXPECT_DOUBLE_EQ(position[0], 1.0);
  EXPECT_DOUBLE_EQ(position[1], 2.0);
  EXPECT_DOUBLE_EQ(position[2], 3.0);
  EXPECT_DOUBLE_EQ(orientation[0], 1.0);
  EXPECT_DOUBLE_EQ(orientation[1], 0.0);
  EXPECT_DOUBLE_EQ(orientation[2], 0.0);
  EXPECT_DOUBLE_EQ(orientation[3], 0.0);
  EXPECT_DOUBLE_EQ(linear_velocity[0], 4.0);
  EXPECT_DOUBLE_EQ(linear_velocity[1], 5.0);
  EXPECT_DOUBLE_EQ(linear_velocity[2], 6.0);
  EXPECT_DOUBLE_EQ(state.data[0], 1.0);
  EXPECT_DOUBLE_EQ(state.data[1], 2.0);
  EXPECT_DOUBLE_EQ(state.data[2], 3.0);
  EXPECT_DOUBLE_EQ(state.data[7], 4.0);
  EXPECT_DOUBLE_EQ(state.data[8], 5.0);
  EXPECT_DOUBLE_EQ(state.data[9], 6.0);
  EXPECT_NO_THROW(state.setData(0, 0.0));
  EXPECT_DOUBLE_EQ(state.data[0], 0.0);

  EXPECT_NO_THROW(Actuation());
  EXPECT_EQ(Actuation::thrust_offset, 0u);
  EXPECT_EQ(Actuation::thrust_length, 1u);
  EXPECT_EQ(Actuation::angular_velocity_offset, 1u);
  EXPECT_EQ(Actuation::angular_velocity_length, 3u);
  auto actuation = Actuation();
  EXPECT_NO_THROW(actuation.setThrust(9.81));
  EXPECT_NO_THROW(actuation.setAngularVelocity(std::array<double, 3>{0.1, 0.2, 0.3}));
  auto thrust           = actuation.getThrust();
  auto angular_velocity = actuation.getAngularVelocity();
  EXPECT_DOUBLE_EQ(thrust, 9.81);
  EXPECT_DOUBLE_EQ(angular_velocity[0], 0.1);
  EXPECT_DOUBLE_EQ(angular_velocity[1], 0.2);
  EXPECT_DOUBLE_EQ(angular_velocity[2], 0.3);
  EXPECT_DOUBLE_EQ(actuation.data[0], 9.81);
  EXPECT_DOUBLE_EQ(actuation.data[1], 0.1);
  EXPECT_DOUBLE_EQ(actuation.data[2], 0.2);
  EXPECT_DOUBLE_EQ(actuation.data[3], 0.3);
  EXPECT_NO_THROW(actuation.setData(0, 0.0));
  EXPECT_DOUBLE_EQ(actuation.data[0], 0.0);

  EXPECT_NO_THROW(Gains());
  auto gains = Gains();
  EXPECT_NO_THROW(gains.getW());
  EXPECT_NO_THROW(gains.getWe());
  EXPECT_NO_THROW(gains.getQ());
  EXPECT_NO_THROW(gains.getQEnd());
  EXPECT_NO_THROW(gains.getR());
  EXPECT_NO_THROW(gains.setW(0, 0.0));
  EXPECT_NO_THROW(gains.setWe(0, 0.0));
  EXPECT_NO_THROW(gains.setQ(0, 0.0));
  EXPECT_NO_THROW(gains.setR(0, 0.0));
  EXPECT_NO_THROW(gains.setQEnd(0, 0.0));

  EXPECT_NO_THROW(ActuationBounds());
  auto actuation_bounds = ActuationBounds();
  EXPECT_NO_THROW(actuation_bounds.getLbu());
  EXPECT_NO_THROW(actuation_bounds.getLbuArray());
  EXPECT_NO_THROW(actuation_bounds.getUbu());
  EXPECT_NO_THROW(actuation_bounds.getUbuArray());
  EXPECT_NO_THROW(actuation_bounds.setLbu(0, 0.0));
  EXPECT_NO_THROW(actuation_bounds.setUbu(0, 0.0));

  EXPECT_NO_THROW(StateBounds());
  auto state_bounds = StateBounds();
  EXPECT_NO_THROW(state_bounds.getLbx());
  EXPECT_NO_THROW(state_bounds.getLbxArray());
  EXPECT_NO_THROW(state_bounds.getUbx());
  EXPECT_NO_THROW(state_bounds.getUbxArray());
  EXPECT_NO_THROW(state_bounds.setLbx(0, 0.0));
  EXPECT_NO_THROW(state_bounds.setUbx(0, 0.0));

  EXPECT_NO_THROW(OnlineParams());
  EXPECT_EQ(OnlineParams::mass_offset, 0u);
  EXPECT_EQ(OnlineParams::mass_length, 1u);
  EXPECT_EQ(OnlineParams::desired_position_offset, 1u);
  EXPECT_EQ(OnlineParams::desired_position_length, 3u);
  auto p_params = OnlineParams();
  EXPECT_NO_THROW(p_params.getData());
  EXPECT_NO_THROW(p_params.getOnlineParams());
  EXPECT_NO_THROW(p_params.setMass(2.0));
  EXPECT_NO_THROW(p_params.setDesiredPosition(std::array<double, 3>{1.0, 2.0, 3.0}));
  EXPECT_NO_THROW(p_params.setDesiredOrientation(std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
  EXPECT_NO_THROW(p_params.setExternalForce(std::array<double, 3>{0.1, 0.2, 0.3}));
  EXPECT_DOUBLE_EQ(p_params.getMass(), 2.0);
  auto desired_position = p_params.getDesiredPosition();
  EXPECT_DOUBLE_EQ(desired_position[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_position[1], 2.0);
  EXPECT_DOUBLE_EQ(desired_position[2], 3.0);
  EXPECT_NO_THROW(p_params.setData(0, 0.0));
}

TEST(acadosMpc, testAcadosSimSolver) {
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
