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
 * @file acados_mpc_benchmark.cpp
 *
 * Acados MPC benchmark tests.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <benchmark/benchmark.h>
#include <exception>
#include <memory>
#include "acados_mpc/acados_mpc.hpp"

static void BM_TEST_INIT(benchmark::State &state) {
  // Perform setup here
  acados_mpc::MPC mpc;
  for (auto _ : state) {
    mpc.solve();
  }
}
BENCHMARK(BM_TEST_INIT)->Threads(1)->Repetitions(10);

static void BM_TEST_UpdateGains(benchmark::State &state) {
  // Perform setup here
  acados_mpc::MPC mpc;
  acados_mpc::MPCGains gains;
  acados_mpc::MPCBounds bounds;
  acados_mpc::MPCOnlineParams p_params;
  for (auto _ : state) {
    mpc.get_gains()->set_gains(gains);
    mpc.get_bounds()->set_bounds(bounds);
    mpc.get_online_params()->set_online_params(p_params);
    mpc.update_bounds();
    mpc.update_gains();
    mpc.update_online_params();
  }
}
BENCHMARK(BM_TEST_UpdateGains)->Threads(1)->Repetitions(10);

BENCHMARK_MAIN();
