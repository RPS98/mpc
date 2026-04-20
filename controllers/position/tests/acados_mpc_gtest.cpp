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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
  EXPECT_NO_THROW(MPC());
  auto mpc = MPC();

  EXPECT_NO_THROW(mpc.solve());
  EXPECT_NO_THROW(mpc.getPredictionSteps());
  EXPECT_NO_THROW(mpc.getPredictionTimeHorizon());
  EXPECT_NO_THROW(mpc.getPredictionTimeStep());
  EXPECT_NO_THROW(mpc.getData());
  EXPECT_NO_THROW(mpc.getParameters());

  auto online_params = OnlineParameters();
  if constexpr (OnlineParameters::size > 0) {
    EXPECT_NO_THROW(online_params.setData(0, 1.5));
    EXPECT_EQ(OnlineParameters::Nstages, static_cast<size_t>(MPC_N + 1));
    EXPECT_NO_THROW(mpc.setParameters(online_params));
    EXPECT_DOUBLE_EQ(mpc.getParameters()->getData()[0], 1.5);
  } else {
    EXPECT_NO_THROW(mpc.setParameters(online_params));
  }

  EXPECT_NO_THROW(mpc.getGains());
  EXPECT_NO_THROW(mpc.getActuationBounds());
  EXPECT_NO_THROW(mpc.getStateBounds());
  EXPECT_NO_THROW(mpc.getSoftStateBounds());
  EXPECT_NO_THROW(mpc.getSlackWeights());
  EXPECT_NO_THROW(mpc.getSlackWeightsEnd());
  EXPECT_NO_THROW(mpc.getNonlinearConstraintBounds());
  EXPECT_NO_THROW(mpc.getSoftNonlinearConstraintBounds());
  EXPECT_NO_THROW(mpc.updateGains());
  EXPECT_NO_THROW(mpc.updateActuationBounds());
  EXPECT_NO_THROW(mpc.updateStateBounds());
  EXPECT_NO_THROW(mpc.updateSoftStateBounds());
  EXPECT_NO_THROW(mpc.updateSlackWeights());
  EXPECT_NO_THROW(mpc.updateSlackWeightsEnd());
  EXPECT_NO_THROW(mpc.updateNonlinearConstraintBounds());
  EXPECT_NO_THROW(mpc.updateSoftNonlinearConstraintBounds());
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
  EXPECT_NO_THROW(state.setPosition(
      std::array<double, State::position_length>{1.0, 2.0, 3.0}));
  EXPECT_NO_THROW(state.setOrientation(
      std::array<double, State::orientation_length>{1.0, 2.0, 3.0, 4.0}));
  EXPECT_NO_THROW(state.setLinearVelocity(
      std::array<double, State::linear_velocity_length>{1.0, 2.0, 3.0}));
  auto state_position = state.getPosition();
  EXPECT_DOUBLE_EQ(state_position[0], 1.0);
  EXPECT_DOUBLE_EQ(state.data[State::position_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(state_position[1], 2.0);
  EXPECT_DOUBLE_EQ(state.data[State::position_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(state_position[2], 3.0);
  EXPECT_DOUBLE_EQ(state.data[State::position_offset + 2], 3.0);
  auto state_orientation = state.getOrientation();
  EXPECT_DOUBLE_EQ(state_orientation[0], 1.0);
  EXPECT_DOUBLE_EQ(state.data[State::orientation_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(state_orientation[1], 2.0);
  EXPECT_DOUBLE_EQ(state.data[State::orientation_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(state_orientation[2], 3.0);
  EXPECT_DOUBLE_EQ(state.data[State::orientation_offset + 2], 3.0);
  EXPECT_DOUBLE_EQ(state_orientation[3], 4.0);
  EXPECT_DOUBLE_EQ(state.data[State::orientation_offset + 3], 4.0);
  auto state_linear_velocity = state.getLinearVelocity();
  EXPECT_DOUBLE_EQ(state_linear_velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(state.data[State::linear_velocity_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(state_linear_velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(state.data[State::linear_velocity_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(state_linear_velocity[2], 3.0);
  EXPECT_DOUBLE_EQ(state.data[State::linear_velocity_offset + 2], 3.0);
  EXPECT_NO_THROW(state.setData(0, 0.0));
  EXPECT_DOUBLE_EQ(state.data[0], 0.0);

  EXPECT_NO_THROW(Actuation());
  EXPECT_EQ(Actuation::thrust_offset, 0u);
  EXPECT_EQ(Actuation::thrust_length, 1u);
  EXPECT_EQ(Actuation::angular_velocity_offset, 1u);
  EXPECT_EQ(Actuation::angular_velocity_length, 3u);
  auto actuation = Actuation();
  EXPECT_NO_THROW(actuation.setThrust(1.0));
  EXPECT_NO_THROW(actuation.setAngularVelocity(
      std::array<double, Actuation::angular_velocity_length>{1.0, 2.0, 3.0}));
  auto actuation_thrust = actuation.getThrust();
  EXPECT_DOUBLE_EQ(actuation_thrust, 1.0);
  EXPECT_DOUBLE_EQ(actuation.data[Actuation::thrust_offset], 1.0);
  auto actuation_angular_velocity = actuation.getAngularVelocity();
  EXPECT_DOUBLE_EQ(actuation_angular_velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(actuation.data[Actuation::angular_velocity_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(actuation_angular_velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(actuation.data[Actuation::angular_velocity_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(actuation_angular_velocity[2], 3.0);
  EXPECT_DOUBLE_EQ(actuation.data[Actuation::angular_velocity_offset + 2], 3.0);
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
  if constexpr (ActuationBounds::Nu > 0) {
    EXPECT_NO_THROW(actuation_bounds.setLbu(0, 0.0));
    EXPECT_NO_THROW(actuation_bounds.setUbu(0, 0.0));
  }

  EXPECT_NO_THROW(StateBounds());
  auto state_bounds = StateBounds();
  EXPECT_NO_THROW(state_bounds.getLbx());
  EXPECT_NO_THROW(state_bounds.getLbxArray());
  EXPECT_NO_THROW(state_bounds.getUbx());
  EXPECT_NO_THROW(state_bounds.getUbxArray());
  if constexpr (StateBounds::Nx > 0) {
    EXPECT_NO_THROW(state_bounds.setLbx(0, 0.0));
    EXPECT_NO_THROW(state_bounds.setUbx(0, 0.0));
  }

  EXPECT_NO_THROW(Parameters());
  EXPECT_EQ(Parameters::mass_offset, 0u);
  EXPECT_EQ(Parameters::mass_length, 1u);
  EXPECT_EQ(Parameters::desired_position_offset, 1u);
  EXPECT_EQ(Parameters::desired_position_length, 3u);
  EXPECT_EQ(Parameters::desired_orientation_offset, 4u);
  EXPECT_EQ(Parameters::desired_orientation_length, 4u);
  EXPECT_EQ(Parameters::external_force_offset, 8u);
  EXPECT_EQ(Parameters::external_force_length, 3u);
  EXPECT_EQ(Parameters::Q_offset, 11u);
  EXPECT_EQ(Parameters::Q_length, 9u);
  EXPECT_EQ(Parameters::Qe_offset, 20u);
  EXPECT_EQ(Parameters::Qe_length, 9u);
  EXPECT_EQ(Parameters::R_offset, 29u);
  EXPECT_EQ(Parameters::R_length, 4u);
  auto p_params = Parameters();
  EXPECT_NO_THROW(p_params.getData());
  EXPECT_NO_THROW(p_params.getParameters());
  EXPECT_NO_THROW(p_params.setMass(1.0));
  EXPECT_NO_THROW(p_params.setDesiredPosition(
      std::array<double, Parameters::desired_position_length>{1.0, 2.0, 3.0}));
  EXPECT_NO_THROW(p_params.setDesiredOrientation(
      std::array<double, Parameters::desired_orientation_length>{1.0, 2.0, 3.0,
                                                                 4.0}));
  EXPECT_NO_THROW(p_params.setExternalForce(
      std::array<double, Parameters::external_force_length>{1.0, 2.0, 3.0}));
  EXPECT_NO_THROW(p_params.setQ(std::array<double, Parameters::Q_length>{
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0}));
  EXPECT_NO_THROW(p_params.setQe(std::array<double, Parameters::Qe_length>{
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0}));
  EXPECT_NO_THROW(p_params.setR(
      std::array<double, Parameters::R_length>{1.0, 2.0, 3.0, 4.0}));
  auto parameters_mass = p_params.getMass();
  EXPECT_DOUBLE_EQ(parameters_mass, 1.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::mass_offset], 1.0);
  auto parameters_desired_position = p_params.getDesiredPosition();
  EXPECT_DOUBLE_EQ(parameters_desired_position[0], 1.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::desired_position_offset + 0],
                   1.0);
  EXPECT_DOUBLE_EQ(parameters_desired_position[1], 2.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::desired_position_offset + 1],
                   2.0);
  EXPECT_DOUBLE_EQ(parameters_desired_position[2], 3.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::desired_position_offset + 2],
                   3.0);
  auto parameters_desired_orientation = p_params.getDesiredOrientation();
  EXPECT_DOUBLE_EQ(parameters_desired_orientation[0], 1.0);
  EXPECT_DOUBLE_EQ(
      p_params.getData()[Parameters::desired_orientation_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(parameters_desired_orientation[1], 2.0);
  EXPECT_DOUBLE_EQ(
      p_params.getData()[Parameters::desired_orientation_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(parameters_desired_orientation[2], 3.0);
  EXPECT_DOUBLE_EQ(
      p_params.getData()[Parameters::desired_orientation_offset + 2], 3.0);
  EXPECT_DOUBLE_EQ(parameters_desired_orientation[3], 4.0);
  EXPECT_DOUBLE_EQ(
      p_params.getData()[Parameters::desired_orientation_offset + 3], 4.0);
  auto parameters_external_force = p_params.getExternalForce();
  EXPECT_DOUBLE_EQ(parameters_external_force[0], 1.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::external_force_offset + 0],
                   1.0);
  EXPECT_DOUBLE_EQ(parameters_external_force[1], 2.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::external_force_offset + 1],
                   2.0);
  EXPECT_DOUBLE_EQ(parameters_external_force[2], 3.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::external_force_offset + 2],
                   3.0);
  auto parameters_Q = p_params.getQ();
  EXPECT_DOUBLE_EQ(parameters_Q[0], 1.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(parameters_Q[1], 2.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(parameters_Q[2], 3.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 2], 3.0);
  EXPECT_DOUBLE_EQ(parameters_Q[3], 4.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 3], 4.0);
  EXPECT_DOUBLE_EQ(parameters_Q[4], 5.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 4], 5.0);
  EXPECT_DOUBLE_EQ(parameters_Q[5], 6.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 5], 6.0);
  EXPECT_DOUBLE_EQ(parameters_Q[6], 7.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 6], 7.0);
  EXPECT_DOUBLE_EQ(parameters_Q[7], 8.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 7], 8.0);
  EXPECT_DOUBLE_EQ(parameters_Q[8], 9.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Q_offset + 8], 9.0);
  auto parameters_Qe = p_params.getQe();
  EXPECT_DOUBLE_EQ(parameters_Qe[0], 1.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[1], 2.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[2], 3.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 2], 3.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[3], 4.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 3], 4.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[4], 5.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 4], 5.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[5], 6.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 5], 6.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[6], 7.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 6], 7.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[7], 8.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 7], 8.0);
  EXPECT_DOUBLE_EQ(parameters_Qe[8], 9.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::Qe_offset + 8], 9.0);
  auto parameters_R = p_params.getR();
  EXPECT_DOUBLE_EQ(parameters_R[0], 1.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::R_offset + 0], 1.0);
  EXPECT_DOUBLE_EQ(parameters_R[1], 2.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::R_offset + 1], 2.0);
  EXPECT_DOUBLE_EQ(parameters_R[2], 3.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::R_offset + 2], 3.0);
  EXPECT_DOUBLE_EQ(parameters_R[3], 4.0);
  EXPECT_DOUBLE_EQ(p_params.getData()[Parameters::R_offset + 3], 4.0);
  EXPECT_NO_THROW(p_params.setData(0, 0.0));
  EXPECT_DOUBLE_EQ(p_params.getData()[0], 0.0);

  EXPECT_NO_THROW(NonlinearConstraintBounds());
  auto nonlinear_constraint_bounds = NonlinearConstraintBounds();
  EXPECT_NO_THROW(nonlinear_constraint_bounds.getLh());
  EXPECT_NO_THROW(nonlinear_constraint_bounds.getUh());
  EXPECT_NO_THROW(nonlinear_constraint_bounds.getLhArray());
  EXPECT_NO_THROW(nonlinear_constraint_bounds.getUhArray());
  if constexpr (NonlinearConstraintBounds::Nh > 0) {
    EXPECT_NO_THROW(nonlinear_constraint_bounds.setLh(0, 0.0));
    EXPECT_NO_THROW(nonlinear_constraint_bounds.setUh(0, 0.0));
  }

  EXPECT_NO_THROW(SoftNonlinearConstraintBounds());
  auto soft_nonlinear_constraint_bounds = SoftNonlinearConstraintBounds();
  EXPECT_NO_THROW(soft_nonlinear_constraint_bounds.getLsh());
  EXPECT_NO_THROW(soft_nonlinear_constraint_bounds.getUsh());
  EXPECT_NO_THROW(soft_nonlinear_constraint_bounds.getLshArray());
  EXPECT_NO_THROW(soft_nonlinear_constraint_bounds.getUshArray());
  if constexpr (SoftNonlinearConstraintBounds::Nsh > 0) {
    EXPECT_NO_THROW(soft_nonlinear_constraint_bounds.setLsh(0, 0.0));
    EXPECT_NO_THROW(soft_nonlinear_constraint_bounds.setUsh(0, 0.0));
  }
}

TEST(acadosMpc, testAcadosSimSolver) {
  EXPECT_NO_THROW(MPCSimSolver());
  auto sim_solver = MPCSimSolver();
  auto mpc_data = MPCData();
  EXPECT_NO_THROW(sim_solver.solve(&mpc_data));
}
} // namespace acados_mpc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
