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
#  define CHECK_MPC_INDEX(index, max_size) check_index(index, max_size)
#else
#  define CHECK_MPC_INDEX(index, max_size) (void)0
#endif

inline void check_index(const int index, const int max_size) {
  if (index < 0 || index >= max_size) {
    throw std::out_of_range("Index out of range.");
  }
}

State::State() {
  data.fill(0.0);
  data[3] = 1.0;  // Quaternion w
}

void State::set_data(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

Control::Control() { data.fill(0.0); }

void Control::set_data(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  data[index] = value;
}

Reference::Reference() {
  data.fill(0.0);
  set_state(0, State());
}

double *Reference::get_data(const int index) {
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * (MPC_NX + MPC_NU)];
}

const double *Reference::get_data(const int index) const {
  CHECK_MPC_INDEX(index, MPC_N);
  State state;
  for (int i = 0; i < MPC_NX; i++) {
    state.data[i] = data[index * (MPC_NX + MPC_NU) + i];
  }
  return &data[index * (MPC_NX + MPC_NU)];
}

State Reference::get_state(const int index) const {
  CHECK_MPC_INDEX(index, MPC_N);
  State state;
  for (int i = 0; i < MPC_NX; i++) {
    state.data[i] = get_data(index)[i];
  }
  return state;
}

void Reference::set_data(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

void Reference::set_data(const int ref_index, const int value_index, const double value) {
  CHECK_MPC_INDEX(ref_index, MPC_N);
  CHECK_MPC_INDEX(value_index, (MPC_NX + MPC_NU));
  data[ref_index * (MPC_NX + MPC_NU) + value_index] = value;
}

void Reference::set_state(const int index, const State &state, const Control &control) {
  CHECK_MPC_INDEX(index, MPC_N);

  int row_index = index * (MPC_NX + MPC_NU);
  for (int i = 0; i < MPC_NX; i++) {
    set_data(row_index + i, state.data[i]);
  }
  for (int i = 0; i < MPC_NU; i++) {
    set_data(row_index + MPC_NX + i, control.data[i]);
  }
}

MPCGains::MPCGains() {
  W.fill(0.0);
  We.fill(0.0);
}

double *MPCGains::get_W() { return W.data(); }

const double *MPCGains::get_W() const { return W.data(); }

double *MPCGains::get_We() { return We.data(); }

const double *MPCGains::get_We() const { return We.data(); }

void MPCGains::set_W(const int index, const double value) {
  CHECK_MPC_INDEX(index, (MPC_NX + MPC_NU));
  W[index * (MPC_NX + MPC_NU) + index] = value;
}

void MPCGains::set_We(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NX);
  We[index * MPC_NX + index] = value;
}

void MPCGains::set_gains(const MPCGains &gains) {
  for (size_t i = 0; i < W.size(); ++i) {
    W[i] = gains.W[i];
  }

  for (size_t i = 0; i < We.size(); ++i) {
    We[i] = gains.We[i];
  }
}

void MPCGains::set_Q(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NX);
  set_W(index, value);
}

void MPCGains::set_Q(const std::array<double, MPC_NX> &Q) {
  for (size_t i = 0; i < Q.size(); ++i) {
    set_Q(i, Q[i]);
  }
}

void MPCGains::set_R(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  set_W(MPC_NX + index, value);
}

void MPCGains::set_R(const std::array<double, MPC_NU> &R) {
  for (size_t i = 0; i < R.size(); ++i) {
    set_R(i, R[i]);
  }
}

void MPCGains::set_Q_end(const int index, const double value) { set_We(index, value); }

void MPCGains::set_Q_end(const std::array<double, MPC_NX> &Qe) {
  for (size_t i = 0; i < Qe.size(); ++i) {
    set_Q_end(i, Qe[i]);
  }
}

MPCBounds::MPCBounds() {
  lbu.fill(0.0);
  ubu.fill(0.0);
}

double *MPCBounds::get_lbu() { return lbu.data(); }

const double *MPCBounds::get_lbu() const { return lbu.data(); }

double *MPCBounds::get_ubu() { return ubu.data(); }

const double *MPCBounds::get_ubu() const { return ubu.data(); }

void MPCBounds::set_bounds(const MPCBounds &bounds) {
  for (size_t i = 0; i < lbu.size(); ++i) {
    lbu[i] = bounds.lbu[i];
  }

  for (size_t i = 0; i < ubu.size(); ++i) {
    ubu[i] = bounds.ubu[i];
  }
}

void MPCBounds::set_lbu(const std::array<double, MPC_NU> &lbu) {
  for (size_t i = 0; i < lbu.size(); ++i) {
    set_lbu(i, lbu[i]);
  }
}

void MPCBounds::set_lbu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  lbu[index] = value;
}

void MPCBounds::set_ubu(const std::array<double, MPC_NU> &ubu) {
  for (size_t i = 0; i < ubu.size(); ++i) {
    set_ubu(i, ubu[i]);
  }
}

void MPCBounds::set_ubu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  ubu[index] = value;
}

MPCOnlineParams::MPCOnlineParams() { data.fill(0.0); }

double *MPCOnlineParams::get_data() { return data.data(); }

const double *MPCOnlineParams::get_data() const { return data.data(); }

double *MPCOnlineParams::get_data(const int index) {
  CHECK_MPC_INDEX(index, MPC_NP);
  return &data[index];
}

const double *MPCOnlineParams::get_data(const int index) const {
  CHECK_MPC_INDEX(index, MPC_NP);
  return &data[index];
}

void MPCOnlineParams::set_online_params(const MPCOnlineParams &params) {
  for (size_t i = 0; i < data.size(); ++i) {
    set_data(i, params.data[i]);
  }
}

void MPCOnlineParams::set_data(const std::array<double, MPC_NP> &data) {
  for (size_t i = 0; i < data.size(); ++i) {
    set_data(i, data[i]);
  }
}

void MPCOnlineParams::set_data(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NP);
  data[index] = value;
}

}  // namespace acados_mpc
