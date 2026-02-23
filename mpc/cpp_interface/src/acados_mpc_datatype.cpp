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

namespace acados_mpc
{

#ifdef ENABLE_CHECKS
#  define CHECK_MPC_INDEX(index, max_size) check_index(index, max_size)
#else
#  define CHECK_MPC_INDEX(index, max_size) (void)0
#endif

inline void check_index(const int index, const int max_size)
{
  if (index < 0 || index >= max_size) {
    throw std::out_of_range("Index out of range.");
  }
}

State::State()
{
  data.fill(0.0);
  data[3] = 1.0;  // Quaternion w
}

void State::set_data(const int index, const double value)
{
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

Actuation::Actuation() {data.fill(0.0);}

void Actuation::set_data(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NU);
  data[index] = value;
}

Reference::Reference() {data.fill(0.0);}

double * Reference::get_data(const int index)
{
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * MPC_NY];
}

const double * Reference::get_data(const int index) const
{
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * MPC_NY];
}

State Reference::get_state(const int index) const
{
  CHECK_MPC_INDEX(index, MPC_N);
  State state;
  for (int i = 0; i < MPC_NX; i++) {
    state.data[i] = get_data(index)[i];
  }
  return state;
}

void Reference::set_data(const int index, const double value)
{
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

void Reference::set_data(const int ref_index, const int value_index, const double value)
{
  CHECK_MPC_INDEX(ref_index, MPC_N);
  CHECK_MPC_INDEX(value_index, MPC_NY);
  data[ref_index * MPC_NY + value_index] = value;
}

void Reference::set_state(const int index, const State & state, const Actuation & actuation)
{
  CHECK_MPC_INDEX(index, MPC_N);

  int row_index = index * MPC_NY;
  for (int i = 0; i < MPC_NX; i++) {
    set_data(row_index + i, state.data[i]);
  }
  for (int i = 0; i < MPC_NU; i++) {
    set_data(row_index + MPC_NX + i, actuation.data[i]);
  }
}

ReferenceEnd::ReferenceEnd() {data.fill(0.0);}

double * ReferenceEnd::get_data() {return data.data();}

const double * ReferenceEnd::get_data() const {return data.data();}

void ReferenceEnd::set_data(const int index, const double value)
{
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}


Gains::Gains()
{
  W.fill(0.0);
  We.fill(0.0);
}

double * Gains::get_W() {return W.data();}

const double * Gains::get_W() const {return W.data();}

double * Gains::get_We() {return We.data();}

const double * Gains::get_We() const {return We.data();}

std::array<double, Gains::Nq> Gains::get_Q() const
{
  std::array<double, Gains::Nq> Q;
  for (size_t i = 0; i < Nq; ++i) {
    Q[i] = W[i * MPC_NY + i];
  }
  return Q;
}

std::array<double, Gains::Nqe> Gains::get_Q_end() const
{
  std::array<double, Gains::Nqe> Qe;
  for (size_t i = 0; i < Nqe; ++i) {
    Qe[i] = We[i * MPC_NYN + i];
  }
  return Qe;
}

std::array<double, Gains::Nr> Gains::get_R() const
{
  std::array<double, Gains::Nr> R;
  for (size_t i = 0; i < Nr; ++i) {
    auto index = (MPC_NYN + i) * MPC_NY + (MPC_NYN + i);
    R[i] = W[index];
  }
  return R;
}

void Gains::set_W(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NY);
  W[index * MPC_NY + index] = value;
}

void Gains::set_We(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NYN);
  We[index * MPC_NYN + index] = value;
}

void Gains::set_gains(const Gains & gains)
{
  for (size_t i = 0; i < W.size(); ++i) {
    W[i] = gains.W[i];
  }

  for (size_t i = 0; i < We.size(); ++i) {
    We[i] = gains.We[i];
  }
}

void Gains::set_Q(const int index, const double value)
{
  CHECK_MPC_INDEX(index, Gains::Nq);
  set_W(index, value);
}

void Gains::set_Q(const std::array<double, Gains::Nq> & Q)
{
  for (size_t i = 0; i < Q.size(); ++i) {
    set_Q(i, Q[i]);
  }
}

void Gains::set_R(const int index, const double value)
{
  CHECK_MPC_INDEX(index, Gains::Nr);
  set_W(MPC_NYN + index, value);
}

void Gains::set_R(const std::array<double, Gains::Nr> & R)
{
  for (size_t i = 0; i < R.size(); ++i) {
    set_R(i, R[i]);
  }
}

void Gains::set_Q_end(const int index, const double value) {set_We(index, value);}

void Gains::set_Q_end(const std::array<double, Gains::Nqe> & Qe)
{
  for (size_t i = 0; i < Qe.size(); ++i) {
    set_Q_end(i, Qe[i]);
  }
}

ActuationBounds::ActuationBounds()
{
  lbu.fill(0.0);
  ubu.fill(0.0);
}

double * ActuationBounds::get_lbu() {return lbu.data();}

const double * ActuationBounds::get_lbu() const {return lbu.data();}

std::array<double, MPC_NU> ActuationBounds::get_lbu_array() const {return lbu;}

double * ActuationBounds::get_ubu() {return ubu.data();}

const double * ActuationBounds::get_ubu() const {return ubu.data();}

std::array<double, MPC_NU> ActuationBounds::get_ubu_array() const {return ubu;}

void ActuationBounds::set_bounds(const ActuationBounds & bounds)
{
  for (size_t i = 0; i < lbu.size(); ++i) {
    lbu[i] = bounds.lbu[i];
  }

  for (size_t i = 0; i < ubu.size(); ++i) {
    ubu[i] = bounds.ubu[i];
  }
}

void ActuationBounds::set_lbu(const std::array<double, MPC_NU> & lbu)
{
  for (size_t i = 0; i < lbu.size(); ++i) {
    set_lbu(i, lbu[i]);
  }
}

void ActuationBounds::set_lbu(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NU);
  lbu[index] = value;
}

void ActuationBounds::set_ubu(const std::array<double, MPC_NU> & ubu)
{
  for (size_t i = 0; i < ubu.size(); ++i) {
    set_ubu(i, ubu[i]);
  }
}

void ActuationBounds::set_ubu(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NU);
  ubu[index] = value;
}

StateBounds::StateBounds()
{
  lbx.fill(0.0);
  ubx.fill(0.0);
}

double * StateBounds::get_lbx() {return lbx.data();}

const double * StateBounds::get_lbx() const {return lbx.data();}

std::array<double, MPC_NBX> StateBounds::get_lbx_array() const {return lbx;}

double * StateBounds::get_ubx() {return ubx.data();}

const double * StateBounds::get_ubx() const {return ubx.data();}

std::array<double, MPC_NBX> StateBounds::get_ubx_array() const {return ubx;}

void StateBounds::set_bounds(const StateBounds & bounds)
{
  for (size_t i = 0; i < lbx.size(); ++i) {
    lbx[i] = bounds.lbx[i];
  }

  for (size_t i = 0; i < ubx.size(); ++i) {
    ubx[i] = bounds.ubx[i];
  }
}

void StateBounds::set_lbx(const std::array<double, MPC_NBX> & lbx)
{
  for (size_t i = 0; i < lbx.size(); ++i) {
    set_lbx(i, lbx[i]);
  }
}

void StateBounds::set_lbx(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NX);
  lbx[index] = value;
}

void StateBounds::set_ubx(const std::array<double, MPC_NBX> & ubx)
{
  for (size_t i = 0; i < ubx.size(); ++i) {
    set_ubx(i, ubx[i]);
  }
}

void StateBounds::set_ubx(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NX);
  ubx[index] = value;
}

OnlineParams::OnlineParams() {data.fill(0.0);}

double * OnlineParams::get_data() {return data.data();}

const double * OnlineParams::get_data() const {return data.data();}

double * OnlineParams::get_data(const int index)
{
  CHECK_MPC_INDEX(index, size_n);
  return &data[index * Np];
}

const double * OnlineParams::get_data(const int index) const
{
  CHECK_MPC_INDEX(index, size_n);
  return &data[index * Np];
}

std::array<double, MPC_NP> OnlineParams::get_online_params(const int index) const
{
  CHECK_MPC_INDEX(index, size_n);
  std::array<double, MPC_NP> params;
  for (size_t i = 0; i < MPC_NP; ++i) {
    params[i] = get_data(index)[i];
  }
  return params;
}

void OnlineParams::set_online_params(const OnlineParams & params)
{
  for (size_t i = 0; i < data.size(); ++i) {
    set_data(i, params.data[i]);
  }
}

void OnlineParams::set_data(const int index, const double value)
{
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

void OnlineParams::set_data(const int index, const int value_index, const double value)
{
  CHECK_MPC_INDEX(index, size);
  CHECK_MPC_INDEX(value_index, Np);
  data[index * MPC_NP + value_index] = value;
}

SoftStateBounds::SoftStateBounds()
{
  lsbx.fill(0.0);
  usbx.fill(0.0);
}

double * SoftStateBounds::get_lsbx() {return lsbx.data();}

const double * SoftStateBounds::get_lsbx() const {return lsbx.data();}

std::array<double, MPC_NSBX> SoftStateBounds::get_lsbx_array() const {return lsbx;}

double * SoftStateBounds::get_usbx() {return usbx.data();}

const double * SoftStateBounds::get_usbx() const {return usbx.data();}

std::array<double, MPC_NSBX> SoftStateBounds::get_usbx_array() const {return usbx;}

void SoftStateBounds::set_bounds(const SoftStateBounds & bounds)
{
  for (size_t i = 0; i < lsbx.size(); ++i) {
    lsbx[i] = bounds.lsbx[i];
  }

  for (size_t i = 0; i < usbx.size(); ++i) {
    usbx[i] = bounds.usbx[i];
  }
}

void SoftStateBounds::set_lsbx(const std::array<double, MPC_NSBX> & lsbx)
{
  for (size_t i = 0; i < lsbx.size(); ++i) {
    set_lsbx(i, lsbx[i]);
  }
}

void SoftStateBounds::set_lsbx(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  lsbx[index] = value;
}

void SoftStateBounds::set_usbx(const std::array<double, MPC_NSBX> & usbx)
{
  for (size_t i = 0; i < usbx.size(); ++i) {
    set_usbx(i, usbx[i]);
  }
}

void SoftStateBounds::set_usbx(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  usbx[index] = value;
}

SlackWeights::SlackWeights()
{
  Zl.fill(0.0);
  Zu.fill(0.0);
  zl.fill(0.0);
  zu.fill(0.0);
}

double * SlackWeights::get_Zl() {return Zl.data();}

const double * SlackWeights::get_Zl() const {return Zl.data();}

std::array<double, MPC_NSBX> SlackWeights::get_Zl_array() const {return Zl;}

double * SlackWeights::get_Zu() {return Zu.data();}

const double * SlackWeights::get_Zu() const {return Zu.data();}

std::array<double, MPC_NSBX> SlackWeights::get_Zu_array() const {return Zu;}

double * SlackWeights::get_zl() {return zl.data();}

const double * SlackWeights::get_zl() const {return zl.data();}

std::array<double, MPC_NSBX> SlackWeights::get_zl_array() const {return zl;}

double * SlackWeights::get_zu() {return zu.data();}

const double * SlackWeights::get_zu() const {return zu.data();}

std::array<double, MPC_NSBX> SlackWeights::get_zu_array() const {return zu;}

void SlackWeights::set_weights(const SlackWeights & weights)
{
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

void SlackWeights::set_Zl(const std::array<double, MPC_NSBX> & Zl)
{
  for (size_t i = 0; i < Zl.size(); ++i) {
    set_Zl(i, Zl[i]);
  }
}

void SlackWeights::set_Zl(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zl[index] = value;
}

void SlackWeights::set_Zu(const std::array<double, MPC_NSBX> & Zu)
{
  for (size_t i = 0; i < Zu.size(); ++i) {
    set_Zu(i, Zu[i]);
  }
}

void SlackWeights::set_Zu(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zu[index] = value;
}

void SlackWeights::set_zl(const std::array<double, MPC_NSBX> & zl)
{
  for (size_t i = 0; i < zl.size(); ++i) {
    set_zl(i, zl[i]);
  }
}

void SlackWeights::set_zl(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zl[index] = value;
}

void SlackWeights::set_zu(const std::array<double, MPC_NSBX> & zu)
{
  for (size_t i = 0; i < zu.size(); ++i) {
    set_zu(i, zu[i]);
  }
}

void SlackWeights::set_zu(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zu[index] = value;
}

SlackWeightsEnd::SlackWeightsEnd()
{
  Zl_e.fill(0.0);
  Zu_e.fill(0.0);
  zl_e.fill(0.0);
  zu_e.fill(0.0);
}

double * SlackWeightsEnd::get_Zl_e() {return Zl_e.data();}

const double * SlackWeightsEnd::get_Zl_e() const {return Zl_e.data();}

std::array<double, MPC_NSBX> SlackWeightsEnd::get_Zl_e_array() const {return Zl_e;}

double * SlackWeightsEnd::get_Zu_e() {return Zu_e.data();}

const double * SlackWeightsEnd::get_Zu_e() const {return Zu_e.data();}

std::array<double, MPC_NSBX> SlackWeightsEnd::get_Zu_e_array() const {return Zu_e;}

double * SlackWeightsEnd::get_zl_e() {return zl_e.data();}

const double * SlackWeightsEnd::get_zl_e() const {return zl_e.data();}

std::array<double, MPC_NSBX> SlackWeightsEnd::get_zl_e_array() const {return zl_e;}

double * SlackWeightsEnd::get_zu_e() {return zu_e.data();}

const double * SlackWeightsEnd::get_zu_e() const {return zu_e.data();}

std::array<double, MPC_NSBX> SlackWeightsEnd::get_zu_e_array() const {return zu_e;}

void SlackWeightsEnd::set_weights(const SlackWeightsEnd & weights)
{
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

void SlackWeightsEnd::set_Zl_e(const std::array<double, MPC_NSBX> & Zl_e)
{
  for (size_t i = 0; i < Zl_e.size(); ++i) {
    set_Zl_e(i, Zl_e[i]);
  }
}

void SlackWeightsEnd::set_Zl_e(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zl_e[index] = value;
}

void SlackWeightsEnd::set_Zu_e(const std::array<double, MPC_NSBX> & Zu_e)
{
  for (size_t i = 0; i < Zu_e.size(); ++i) {
    set_Zu_e(i, Zu_e[i]);
  }
}

void SlackWeightsEnd::set_Zu_e(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  Zu_e[index] = value;
}

void SlackWeightsEnd::set_zl_e(const std::array<double, MPC_NSBX> & zl_e)
{
  for (size_t i = 0; i < zl_e.size(); ++i) {
    set_zl_e(i, zl_e[i]);
  }
}

void SlackWeightsEnd::set_zl_e(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zl_e[index] = value;
}

void SlackWeightsEnd::set_zu_e(const std::array<double, MPC_NSBX> & zu_e)
{
  for (size_t i = 0; i < zu_e.size(); ++i) {
    set_zu_e(i, zu_e[i]);
  }
}

void SlackWeightsEnd::set_zu_e(const int index, const double value)
{
  CHECK_MPC_INDEX(index, MPC_NSBX);
  zu_e[index] = value;
}

}  // namespace acados_mpc
