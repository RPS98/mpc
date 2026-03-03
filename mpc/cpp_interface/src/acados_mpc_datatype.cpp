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
 * @file acados_mpc_datatype.hpp
 *
 * Acados MPC data types implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "acados_mpc/acados_mpc_datatype.hpp"

namespace acados_mpc {

#ifdef ENABLE_CHECKS
#  define CHECK_MPC_INDEX(index, max_size) checkIndex(index, max_size)
#else
#  define CHECK_MPC_INDEX(index, max_size) (void)0
#endif

inline void checkIndex(const int index, const int max_size) {
  if (index < 0 || index >= max_size) {
    throw std::out_of_range("Index out of range.");
  }
}

State::State() {
  data.fill(0.0);
  data[position_offset + 0]        = 0.0;
  data[position_offset + 1]        = 0.0;
  data[position_offset + 2]        = 0.0;
  data[orientation_offset + 0]     = 1.0;
  data[orientation_offset + 1]     = 0.0;
  data[orientation_offset + 2]     = 0.0;
  data[orientation_offset + 3]     = 0.0;
  data[linear_velocity_offset + 0] = 0.0;
  data[linear_velocity_offset + 1] = 0.0;
  data[linear_velocity_offset + 2] = 0.0;
}

void State::setData(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}
void State::setPosition(const std::array<double, State::position_length>& value) {
  for (size_t i = 0; i < value.size(); ++i) {
    setData(static_cast<int>(position_offset + i), value[i]);
  }
}

std::array<double, State::position_length> State::getPosition() const {
  std::array<double, State::position_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = data[position_offset + i];
  }
  return value;
}

void State::setOrientation(const std::array<double, State::orientation_length>& value) {
  for (size_t i = 0; i < value.size(); ++i) {
    setData(static_cast<int>(orientation_offset + i), value[i]);
  }
}

std::array<double, State::orientation_length> State::getOrientation() const {
  std::array<double, State::orientation_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = data[orientation_offset + i];
  }
  return value;
}

void State::setLinearVelocity(const std::array<double, State::linear_velocity_length>& value) {
  for (size_t i = 0; i < value.size(); ++i) {
    setData(static_cast<int>(linear_velocity_offset + i), value[i]);
  }
}

std::array<double, State::linear_velocity_length> State::getLinearVelocity() const {
  std::array<double, State::linear_velocity_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = data[linear_velocity_offset + i];
  }
  return value;
}

Actuation::Actuation() {
  data.fill(0.0);
  data[thrust_offset]               = 0.0;
  data[angular_velocity_offset + 0] = 0.0;
  data[angular_velocity_offset + 1] = 0.0;
  data[angular_velocity_offset + 2] = 0.0;
}

void Actuation::setData(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  data[index] = value;
}
void Actuation::setThrust(const double value) { setData(thrust_offset, value); }

double Actuation::getThrust() const { return data[thrust_offset]; }

void Actuation::setAngularVelocity(
    const std::array<double, Actuation::angular_velocity_length>& value) {
  for (size_t i = 0; i < value.size(); ++i) {
    setData(static_cast<int>(angular_velocity_offset + i), value[i]);
  }
}

std::array<double, Actuation::angular_velocity_length> Actuation::getAngularVelocity() const {
  std::array<double, Actuation::angular_velocity_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = data[angular_velocity_offset + i];
  }
  return value;
}

Reference::Reference() { data.fill(0.0); }

double* Reference::getData(const int index) {
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * MPC_NY];
}

const double* Reference::getData(const int index) const {
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * MPC_NY];
}

State Reference::getState(const int index) const {
  CHECK_MPC_INDEX(index, MPC_N);
  State state;
  for (int i = 0; i < MPC_NX; i++) {
    state.data[i] = getData(index)[i];
  }
  return state;
}

void Reference::setData(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

void Reference::setData(const int ref_index, const int value_index, const double value) {
  CHECK_MPC_INDEX(ref_index, MPC_N);
  CHECK_MPC_INDEX(value_index, MPC_NY);
  data[ref_index * MPC_NY + value_index] = value;
}

void Reference::setState(const int index, const State& state, const Actuation& actuation) {
  CHECK_MPC_INDEX(index, MPC_N);

  int row_index = index * MPC_NY;
  for (int i = 0; i < MPC_NX; i++) {
    setData(row_index + i, state.data[i]);
  }
  for (int i = 0; i < MPC_NU; i++) {
    setData(row_index + MPC_NX + i, actuation.data[i]);
  }
}

ReferenceEnd::ReferenceEnd() { data.fill(0.0); }

double* ReferenceEnd::getData() { return data.data(); }

const double* ReferenceEnd::getData() const { return data.data(); }

void ReferenceEnd::setData(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

Gains::Gains() {
  W.fill(0.0);
  We.fill(0.0);
}

double* Gains::getW() { return W.data(); }

const double* Gains::getW() const { return W.data(); }

double* Gains::getWe() { return We.data(); }

const double* Gains::getWe() const { return We.data(); }

std::array<double, Gains::Nq> Gains::getQ() const {
  std::array<double, Gains::Nq> Q;
  for (size_t i = 0; i < Nq; ++i) {
    Q[i] = W[i * MPC_NY + i];
  }
  return Q;
}

std::array<double, Gains::Nqe> Gains::getQEnd() const {
  std::array<double, Gains::Nqe> Qe;
  for (size_t i = 0; i < Nqe; ++i) {
    Qe[i] = We[i * MPC_NYN + i];
  }
  return Qe;
}

std::array<double, Gains::Nr> Gains::getR() const {
  std::array<double, Gains::Nr> R;
  for (size_t i = 0; i < Nr; ++i) {
    auto index = (MPC_NYN + i) * MPC_NY + (MPC_NYN + i);
    R[i]       = W[index];
  }
  return R;
}

void Gains::setW(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NY);
  W[index * MPC_NY + index] = value;
}

void Gains::setWe(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NYN);
  We[index * MPC_NYN + index] = value;
}

void Gains::setGains(const Gains& gains) {
  for (size_t i = 0; i < W.size(); ++i) {
    W[i] = gains.W[i];
  }

  for (size_t i = 0; i < We.size(); ++i) {
    We[i] = gains.We[i];
  }
}

void Gains::setQ(const int index, const double value) {
  CHECK_MPC_INDEX(index, Gains::Nq);
  setW(index, value);
}

void Gains::setQ(const std::array<double, Gains::Nq>& Q) {
  for (size_t i = 0; i < Q.size(); ++i) {
    setQ(i, Q[i]);
  }
}

void Gains::setR(const int index, const double value) {
  CHECK_MPC_INDEX(index, Gains::Nr);
  setW(MPC_NYN + index, value);
}

void Gains::setR(const std::array<double, Gains::Nr>& R) {
  for (size_t i = 0; i < R.size(); ++i) {
    setR(i, R[i]);
  }
}

void Gains::setQEnd(const int index, const double value) { setWe(index, value); }

void Gains::setQEnd(const std::array<double, Gains::Nqe>& Qe) {
  for (size_t i = 0; i < Qe.size(); ++i) {
    setQEnd(i, Qe[i]);
  }
}

ActuationBounds::ActuationBounds() {
  lbu.fill(0.0);
  ubu.fill(0.0);
}

double* ActuationBounds::getLbu() { return lbu.data(); }

const double* ActuationBounds::getLbu() const { return lbu.data(); }

std::array<double, MPC_NU> ActuationBounds::getLbuArray() const { return lbu; }

double* ActuationBounds::getUbu() { return ubu.data(); }

const double* ActuationBounds::getUbu() const { return ubu.data(); }

std::array<double, MPC_NU> ActuationBounds::getUbuArray() const { return ubu; }

void ActuationBounds::setBounds(const ActuationBounds& bounds) {
  for (size_t i = 0; i < lbu.size(); ++i) {
    lbu[i] = bounds.lbu[i];
  }

  for (size_t i = 0; i < ubu.size(); ++i) {
    ubu[i] = bounds.ubu[i];
  }
}

void ActuationBounds::setLbu(const std::array<double, MPC_NU>& lbu) {
  for (size_t i = 0; i < lbu.size(); ++i) {
    setLbu(i, lbu[i]);
  }
}

void ActuationBounds::setLbu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  lbu[index] = value;
}

void ActuationBounds::setUbu(const std::array<double, MPC_NU>& ubu) {
  for (size_t i = 0; i < ubu.size(); ++i) {
    setUbu(i, ubu[i]);
  }
}

void ActuationBounds::setUbu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  ubu[index] = value;
}

StateBounds::StateBounds() {
  lbx.fill(0.0);
  ubx.fill(0.0);
}

double* StateBounds::getLbx() { return lbx.data(); }

const double* StateBounds::getLbx() const { return lbx.data(); }

std::array<double, MPC_NBX> StateBounds::getLbxArray() const { return lbx; }

double* StateBounds::getUbx() { return ubx.data(); }

const double* StateBounds::getUbx() const { return ubx.data(); }

std::array<double, MPC_NBX> StateBounds::getUbxArray() const { return ubx; }

void StateBounds::setBounds(const StateBounds& bounds) {
  for (size_t i = 0; i < lbx.size(); ++i) {
    lbx[i] = bounds.lbx[i];
  }

  for (size_t i = 0; i < ubx.size(); ++i) {
    ubx[i] = bounds.ubx[i];
  }
}

void StateBounds::setLbx(const std::array<double, MPC_NBX>& lbx) {
  for (size_t i = 0; i < lbx.size(); ++i) {
    setLbx(i, lbx[i]);
  }
}

void StateBounds::setLbx(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NX);
  lbx[index] = value;
}

void StateBounds::setUbx(const std::array<double, MPC_NBX>& ubx) {
  for (size_t i = 0; i < ubx.size(); ++i) {
    setUbx(i, ubx[i]);
  }
}

void StateBounds::setUbx(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NX);
  ubx[index] = value;
}

Parameters::Parameters() {
  data.fill(0.0);
  data[mass_offset]                    = 1.0;
  data[desired_orientation_offset + 0] = 1.0;
  data[desired_orientation_offset + 1] = 0.0;
  data[desired_orientation_offset + 2] = 0.0;
  data[desired_orientation_offset + 3] = 0.0;
  data[external_force_offset + 0]      = 0.0;
  data[external_force_offset + 1]      = 0.0;
  data[external_force_offset + 2]      = 0.0;
}

double* Parameters::getData() { return data.data(); }

const double* Parameters::getData() const { return data.data(); }

std::array<double, MPC_NP> Parameters::getParameters() const {
  std::array<double, MPC_NP> params;
  for (size_t i = 0; i < MPC_NP; ++i) {
    params[i] = data[i];
  }
  return params;
}

void Parameters::setParameters(const Parameters& params) {
  for (size_t i = 0; i < data.size(); ++i) {
    setData(i, params.data[i]);
  }
}

void Parameters::setData(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}
void Parameters::setMass(const double value) { setData(mass_offset, value); }

double Parameters::getMass() const { return data[mass_offset]; }

void Parameters::setDesiredOrientation(
    const std::array<double, Parameters::desired_orientation_length>& value) {
  for (size_t i = 0; i < value.size(); ++i) {
    setData(static_cast<int>(desired_orientation_offset + i), value[i]);
  }
}

std::array<double, Parameters::desired_orientation_length> Parameters::getDesiredOrientation()
    const {
  std::array<double, Parameters::desired_orientation_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = data[desired_orientation_offset + i];
  }
  return value;
}

void Parameters::setExternalForce(
    const std::array<double, Parameters::external_force_length>& value) {
  for (size_t i = 0; i < value.size(); ++i) {
    setData(static_cast<int>(external_force_offset + i), value[i]);
  }
}

std::array<double, Parameters::external_force_length> Parameters::getExternalForce() const {
  std::array<double, Parameters::external_force_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = data[external_force_offset + i];
  }
  return value;
}

OnlineParameters::OnlineParameters() {
  data.fill(0.0);
  Parameters default_params;
  setParameters(default_params);
}

double* OnlineParameters::getData() { return data.data(); }

const double* OnlineParameters::getData() const { return data.data(); }

double* OnlineParameters::getData(const int stage) {
  CHECK_MPC_INDEX(stage, Nstages);
  return &data[stage * Np];
}

const double* OnlineParameters::getData(const int stage) const {
  CHECK_MPC_INDEX(stage, Nstages);
  return &data[stage * Np];
}

OnlineParameters::StageParameters OnlineParameters::getParameters(const int stage) const {
  CHECK_MPC_INDEX(stage, Nstages);
  StageParameters params;
  for (size_t i = 0; i < Np; ++i) {
    params.data[i] = data[stage * Np + i];
  }
  return params;
}

std::array<double, OnlineParameters::Nstages * MPC_NP> OnlineParameters::getOnlineParameters()
    const {
  return data;
}

void OnlineParameters::setParameters(const OnlineParameters& params) {
  for (size_t i = 0; i < data.size(); ++i) {
    setData(i, params.data[i]);
  }
}

void OnlineParameters::setParameters(const StageParameters& params, const int stage) {
  if (stage < 0) {
    for (size_t stage_index = 0; stage_index < Nstages; ++stage_index) {
      setParameters(params, static_cast<int>(stage_index));
    }
    return;
  }
  CHECK_MPC_INDEX(stage, Nstages);
  for (size_t i = 0; i < params.size; ++i) {
    setData(stage, static_cast<int>(i), params.data[i]);
  }
}

void OnlineParameters::setData(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

void OnlineParameters::setData(const int stage, const int index, const double value) {
  CHECK_MPC_INDEX(stage, Nstages);
  CHECK_MPC_INDEX(index, Np);
  data[stage * Np + index] = value;
}
void OnlineParameters::setMass(const double value, const int stage) {
  if (stage < 0) {
    for (size_t stage_index = 0; stage_index < Nstages; ++stage_index) {
      setMass(value, static_cast<int>(stage_index));
    }
    return;
  }
  setData(stage, mass_offset, value);
}

double OnlineParameters::getMass(const int stage) const {
  CHECK_MPC_INDEX(stage, Nstages);
  return getData(stage)[mass_offset];
}

void OnlineParameters::setDesiredOrientation(
    const std::array<double, OnlineParameters::desired_orientation_length>& value,
    const int stage) {
  if (stage < 0) {
    for (size_t stage_index = 0; stage_index < Nstages; ++stage_index) {
      setDesiredOrientation(value, static_cast<int>(stage_index));
    }
    return;
  }
  CHECK_MPC_INDEX(stage, Nstages);
  for (size_t i = 0; i < value.size(); ++i) {
    setData(stage, static_cast<int>(desired_orientation_offset + i), value[i]);
  }
}

std::array<double, OnlineParameters::desired_orientation_length>
OnlineParameters::getDesiredOrientation(const int stage) const {
  CHECK_MPC_INDEX(stage, Nstages);
  std::array<double, OnlineParameters::desired_orientation_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = getData(stage)[desired_orientation_offset + i];
  }
  return value;
}

void OnlineParameters::setExternalForce(
    const std::array<double, OnlineParameters::external_force_length>& value,
    const int stage) {
  if (stage < 0) {
    for (size_t stage_index = 0; stage_index < Nstages; ++stage_index) {
      setExternalForce(value, static_cast<int>(stage_index));
    }
    return;
  }
  CHECK_MPC_INDEX(stage, Nstages);
  for (size_t i = 0; i < value.size(); ++i) {
    setData(stage, static_cast<int>(external_force_offset + i), value[i]);
  }
}

std::array<double, OnlineParameters::external_force_length> OnlineParameters::getExternalForce(
    const int stage) const {
  CHECK_MPC_INDEX(stage, Nstages);
  std::array<double, OnlineParameters::external_force_length> value = {};
  for (size_t i = 0; i < value.size(); ++i) {
    value[i] = getData(stage)[external_force_offset + i];
  }
  return value;
}

SoftStateBounds::SoftStateBounds() {
  lsbx.fill(0.0);
  usbx.fill(0.0);
}

double* SoftStateBounds::getLsbx() { return lsbx.data(); }

const double* SoftStateBounds::getLsbx() const { return lsbx.data(); }

std::array<double, MPC_NSBX> SoftStateBounds::getLsbxArray() const { return lsbx; }

double* SoftStateBounds::getUsbx() { return usbx.data(); }

const double* SoftStateBounds::getUsbx() const { return usbx.data(); }

std::array<double, MPC_NSBX> SoftStateBounds::getUsbxArray() const { return usbx; }

void SoftStateBounds::setBounds(const SoftStateBounds& bounds) {
  for (size_t i = 0; i < lsbx.size(); ++i) {
    lsbx[i] = bounds.lsbx[i];
  }

  for (size_t i = 0; i < usbx.size(); ++i) {
    usbx[i] = bounds.usbx[i];
  }
}

void SoftStateBounds::setLsbx(const std::array<double, MPC_NSBX>& lsbx) {
  for (size_t i = 0; i < lsbx.size(); ++i) {
    setLsbx(i, lsbx[i]);
  }
}

void SoftStateBounds::setLsbx(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  lsbx[index] = value;
}

void SoftStateBounds::setUsbx(const std::array<double, MPC_NSBX>& usbx) {
  for (size_t i = 0; i < usbx.size(); ++i) {
    setUsbx(i, usbx[i]);
  }
}

void SoftStateBounds::setUsbx(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  usbx[index] = value;
}

SlackWeights::SlackWeights() {
  Zl.fill(0.0);
  Zu.fill(0.0);
  zl.fill(0.0);
  zu.fill(0.0);
}

double* SlackWeights::getZl() { return Zl.data(); }

const double* SlackWeights::getZl() const { return Zl.data(); }

std::array<double, MPC_NSBX> SlackWeights::getZlArray() const { return Zl; }

double* SlackWeights::getZu() { return Zu.data(); }

const double* SlackWeights::getZu() const { return Zu.data(); }

std::array<double, MPC_NSBX> SlackWeights::getZuArray() const { return Zu; }

double* SlackWeights::getzl() { return zl.data(); }

const double* SlackWeights::getzl() const { return zl.data(); }

std::array<double, MPC_NSBX> SlackWeights::getzlArray() const { return zl; }

double* SlackWeights::getzu() { return zu.data(); }

const double* SlackWeights::getzu() const { return zu.data(); }

std::array<double, MPC_NSBX> SlackWeights::getzuArray() const { return zu; }

void SlackWeights::setWeights(const SlackWeights& weights) {
  for (size_t i = 0; i < Zl.size(); ++i) {
    Zl[i] = weights.Zl[i];
  }

  for (size_t i = 0; i < Zu.size(); ++i) {
    Zu[i] = weights.Zu[i];
  }

  for (size_t i = 0; i < zl.size(); ++i) {
    zl[i] = weights.zl[i];
  }

  for (size_t i = 0; i < zu.size(); ++i) {
    zu[i] = weights.zu[i];
  }
}

void SlackWeights::setZl(const std::array<double, MPC_NSBX>& Zl) {
  for (size_t i = 0; i < Zl.size(); ++i) {
    setZl(i, Zl[i]);
  }
}

void SlackWeights::setZl(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zl[index] = value;
}

void SlackWeights::setZu(const std::array<double, MPC_NSBX>& Zu) {
  for (size_t i = 0; i < Zu.size(); ++i) {
    setZu(i, Zu[i]);
  }
}

void SlackWeights::setZu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zu[index] = value;
}

void SlackWeights::setzl(const std::array<double, MPC_NSBX>& zl) {
  for (size_t i = 0; i < zl.size(); ++i) {
    setzl(i, zl[i]);
  }
}

void SlackWeights::setzl(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zl[index] = value;
}

void SlackWeights::setzu(const std::array<double, MPC_NSBX>& zu) {
  for (size_t i = 0; i < zu.size(); ++i) {
    setzu(i, zu[i]);
  }
}

void SlackWeights::setzu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zu[index] = value;
}

SlackWeightsEnd::SlackWeightsEnd() {
  Zl_e.fill(0.0);
  Zu_e.fill(0.0);
  zl_e.fill(0.0);
  zu_e.fill(0.0);
}

double* SlackWeightsEnd::getZlE() { return Zl_e.data(); }

const double* SlackWeightsEnd::getZlE() const { return Zl_e.data(); }

std::array<double, MPC_NSBX> SlackWeightsEnd::getZlEArray() const { return Zl_e; }

double* SlackWeightsEnd::getZuE() { return Zu_e.data(); }

const double* SlackWeightsEnd::getZuE() const { return Zu_e.data(); }

std::array<double, MPC_NSBX> SlackWeightsEnd::getZuEArray() const { return Zu_e; }

double* SlackWeightsEnd::getzlE() { return zl_e.data(); }

const double* SlackWeightsEnd::getzlE() const { return zl_e.data(); }

std::array<double, MPC_NSBX> SlackWeightsEnd::getzlEArray() const { return zl_e; }

double* SlackWeightsEnd::getzuE() { return zu_e.data(); }

const double* SlackWeightsEnd::getzuE() const { return zu_e.data(); }

std::array<double, MPC_NSBX> SlackWeightsEnd::getzuEArray() const { return zu_e; }

void SlackWeightsEnd::setWeights(const SlackWeightsEnd& weights) {
  for (size_t i = 0; i < Zl_e.size(); ++i) {
    Zl_e[i] = weights.Zl_e[i];
  }

  for (size_t i = 0; i < Zu_e.size(); ++i) {
    Zu_e[i] = weights.Zu_e[i];
  }

  for (size_t i = 0; i < zl_e.size(); ++i) {
    zl_e[i] = weights.zl_e[i];
  }

  for (size_t i = 0; i < zu_e.size(); ++i) {
    zu_e[i] = weights.zu_e[i];
  }
}

void SlackWeightsEnd::setZlE(const std::array<double, MPC_NSBX>& Zl_e) {
  for (size_t i = 0; i < Zl_e.size(); ++i) {
    setZlE(i, Zl_e[i]);
  }
}

void SlackWeightsEnd::setZlE(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zl_e[index] = value;
}

void SlackWeightsEnd::setZuE(const std::array<double, MPC_NSBX>& Zu_e) {
  for (size_t i = 0; i < Zu_e.size(); ++i) {
    setZuE(i, Zu_e[i]);
  }
}

void SlackWeightsEnd::setZuE(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zu_e[index] = value;
}

void SlackWeightsEnd::setzlE(const std::array<double, MPC_NSBX>& zl_e) {
  for (size_t i = 0; i < zl_e.size(); ++i) {
    setzlE(i, zl_e[i]);
  }
}

void SlackWeightsEnd::setzlE(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zl_e[index] = value;
}

void SlackWeightsEnd::setzuE(const std::array<double, MPC_NSBX>& zu_e) {
  for (size_t i = 0; i < zu_e.size(); ++i) {
    setzuE(i, zu_e[i]);
  }
}

void SlackWeightsEnd::setzuE(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zu_e[index] = value;
}

}  // namespace acados_mpc
