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

Reference::Reference() { data.fill(0.0); }

double *Reference::get_data(const int index) {
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * MPC_NY];
}

const double *Reference::get_data(const int index) const {
  CHECK_MPC_INDEX(index, MPC_N);
  return &data[index * MPC_NY];
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
  CHECK_MPC_INDEX(value_index, MPC_NY);
  data[ref_index * MPC_NY + value_index] = value;
}

void Reference::set_state(const int index, const State &state, const Control &control) {
  CHECK_MPC_INDEX(index, MPC_N);

  int row_index = index * MPC_NY;
  for (int i = 0; i < MPC_NX; i++) {
    set_data(row_index + i, state.data[i]);
  }
  for (int i = 0; i < MPC_NU; i++) {
    set_data(row_index + MPC_NX + i, control.data[i]);
  }
}

ReferenceEnd::ReferenceEnd() { data.fill(0.0); }

double *ReferenceEnd::get_data() { return data.data(); }

const double *ReferenceEnd::get_data() const { return data.data(); }

void ReferenceEnd::set_data(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

Gains::Gains() {
  W.fill(0.0);
  We.fill(0.0);
}

double *Gains::get_W() { return W.data(); }

const double *Gains::get_W() const { return W.data(); }

double *Gains::get_We() { return We.data(); }

const double *Gains::get_We() const { return We.data(); }

std::array<double, Gains::Nq> Gains::get_Q() const {
  std::array<double, Gains::Nq> Q;
  for (size_t i = 0; i < Nq; ++i) {
    Q[i] = W[i * MPC_NY + i];
  }
  return Q;
}

std::array<double, Gains::Nqe> Gains::get_Q_end() const {
  std::array<double, Gains::Nqe> Qe;
  for (size_t i = 0; i < Nqe; ++i) {
    Qe[i] = We[i * MPC_NYN + i];
  }
  return Qe;
}

std::array<double, Gains::Nr> Gains::get_R() const {
  std::array<double, Gains::Nr> R;
  for (size_t i = 0; i < Nr; ++i) {
    R[i] = W[MPC_NYN + i * MPC_NY + MPC_NYN + i];
  }
  return R;
}

void Gains::set_W(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NY);
  W[index * MPC_NY + index] = value;
}

void Gains::set_We(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NYN);
  We[index * MPC_NYN + index] = value;
}

void Gains::set_gains(const Gains &gains) {
  for (size_t i = 0; i < W.size(); ++i) {
    W[i] = gains.W[i];
  }

  for (size_t i = 0; i < We.size(); ++i) {
    We[i] = gains.We[i];
  }
}

void Gains::set_Q(const int index, const double value) {
  CHECK_MPC_INDEX(index, Gains::Nq);
  set_W(index, value);
}

void Gains::set_Q(const std::array<double, Gains::Nq> &Q) {
  for (size_t i = 0; i < Q.size(); ++i) {
    set_Q(i, Q[i]);
  }
}

void Gains::set_R(const int index, const double value) {
  CHECK_MPC_INDEX(index, Gains::Nr);
  set_W(MPC_NYN + index, value);
}

void Gains::set_R(const std::array<double, Gains::Nr> &R) {
  for (size_t i = 0; i < R.size(); ++i) {
    set_R(i, R[i]);
  }
}

void Gains::set_Q_end(const int index, const double value) { set_We(index, value); }

void Gains::set_Q_end(const std::array<double, Gains::Nqe> &Qe) {
  for (size_t i = 0; i < Qe.size(); ++i) {
    set_Q_end(i, Qe[i]);
  }
}

Bounds::Bounds() {
  lbu.fill(0.0);
  ubu.fill(0.0);
}

double *Bounds::get_lbu() { return lbu.data(); }

const double *Bounds::get_lbu() const { return lbu.data(); }

std::array<double, MPC_NU> Bounds::get_lbu_array() const { return lbu; }

double *Bounds::get_ubu() { return ubu.data(); }

const double *Bounds::get_ubu() const { return ubu.data(); }

std::array<double, MPC_NU> Bounds::get_ubu_array() const { return ubu; }

void Bounds::set_bounds(const Bounds &bounds) {
  for (size_t i = 0; i < lbu.size(); ++i) {
    lbu[i] = bounds.lbu[i];
  }

  for (size_t i = 0; i < ubu.size(); ++i) {
    ubu[i] = bounds.ubu[i];
  }
}

void Bounds::set_lbu(const std::array<double, MPC_NU> &lbu) {
  for (size_t i = 0; i < lbu.size(); ++i) {
    set_lbu(i, lbu[i]);
  }
}

void Bounds::set_lbu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  lbu[index] = value;
}

void Bounds::set_ubu(const std::array<double, MPC_NU> &ubu) {
  for (size_t i = 0; i < ubu.size(); ++i) {
    set_ubu(i, ubu[i]);
  }
}

void Bounds::set_ubu(const int index, const double value) {
  CHECK_MPC_INDEX(index, MPC_NU);
  ubu[index] = value;
}

OnlineParams::OnlineParams() { data.fill(0.0); }

double *OnlineParams::get_data() { return data.data(); }

const double *OnlineParams::get_data() const { return data.data(); }

double *OnlineParams::get_data(const int index) {
  CHECK_MPC_INDEX(index, size_n);
  return &data[index * Np];
}

const double *OnlineParams::get_data(const int index) const {
  CHECK_MPC_INDEX(index, size_n);
  return &data[index * Np];
}

std::array<double, MPC_NP> OnlineParams::get_online_params(const int index) const {
  CHECK_MPC_INDEX(index, size_n);
  std::array<double, MPC_NP> params;
  for (size_t i = 0; i < MPC_NP; ++i) {
    params[i] = get_data(index)[i];
  }
  return params;
}

void OnlineParams::set_online_params(const OnlineParams &params) {
  for (size_t i = 0; i < data.size(); ++i) {
    set_data(i, params.data[i]);
  }
}

void OnlineParams::set_data(const int index, const double value) {
  CHECK_MPC_INDEX(index, size);
  data[index] = value;
}

void OnlineParams::set_data(const int index, const int value_index, const double value) {
  CHECK_MPC_INDEX(index, size);
  CHECK_MPC_INDEX(value_index, Np);
  data[index * MPC_NP + value_index] = value;
}

}  // namespace acados_mpc
