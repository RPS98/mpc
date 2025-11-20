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
 * Acados MPC data types definition.
 *
 * @author Rafael Perez-Segui, Carmen De Rojas Pita-Romero <r.psegui@upm.es> <c.derojas@upm.es>
 *  */

#ifndef ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_
#define ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_

#include <mpc_generated_code/acados_sim_solver_mpc.h>
#include <mpc_generated_code/acados_solver_mpc.h>
#include <mpc_generated_code/mpc_model/mpc_model.h>

#include <array>
#include <stdexcept>
#include <string>

namespace acados_mpc
{

/**
 * @brief State x
 */
struct State
{
  static constexpr size_t Nx = MPC_NX;
  std::array<double, MPC_NX> data;
  static const std::size_t size = MPC_NX;

  /**
   * @brief Constructor
   */
  State();

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);
};

/**
 * @brief Actuation u
 */
struct Actuation
{
  static constexpr size_t Nu = MPC_NU;
  std::array<double, MPC_NU> data;
  static const std::size_t size = MPC_NU;

  /**
   * @brief Constructor
   */
  Actuation();

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);
};

/**
 * @brief ActuationBounds
 *
 * ActuationBounds lbu and ubu for the MPC.
 */
struct ActuationBounds
{
  static constexpr size_t Nu = MPC_NU;
  std::array<double, MPC_NU> lbu;
  std::array<double, MPC_NU> ubu;

  /**
   * @brief Constructor
   */
  ActuationBounds();

  /**
   * @brief Get the lbu array
   *
   * @return double* lbu.
   */
  double * get_lbu();

  /**
   * @brief Get the lbu array
   *
   * @return const double* lbu.
   */
  const double * get_lbu() const;

  /**
   * @brief Get the lbu array
   *
   * @return std::array<double, MPC_NU> lbu.
   */
  std::array<double, MPC_NU> get_lbu_array() const;

  /**
   * @brief Get the ubu array
   *
   * @return double* ubu.
   */
  double * get_ubu();

  /**
   * @brief Get the ubu array
   *
   * @return const double* ubu.
   */
  const double * get_ubu() const;

  /**
   * @brief Get the ubu array
   *
   * @return std::array<double, MPC_NU> ubu.
   */
  std::array<double, MPC_NU> get_ubu_array() const;

  /**
   * @brief Set the bounds
   *
   * @param bounds bounds.
   */
  void set_bounds(const ActuationBounds & bounds);

  /**
   * @brief Set the lbu
   *
   * @param lbu lbu.
   */
  void set_lbu(const std::array<double, MPC_NU> & lbu);

  /**
   * @brief Set the lbu at index
   *
   * @param index index.
   * @param value value.
   */
  void set_lbu(const int index, const double value);

  /**
   * @brief Set the ubu
   *
   * @param ubu ubu.
   */
  void set_ubu(const std::array<double, MPC_NU> & ubu);

  /**
   * @brief Set the ubu at index
   *
   * @param index index.
   * @param value value.
   */
  void set_ubu(const int index, const double value);
};

/**
 * @brief StateBounds
 *
 * StateBounds lbx and ubx for the MPC.
 */
struct StateBounds
{
  static constexpr size_t Nx = MPC_NBX;
  std::array<double, MPC_NBX> lbx;
  std::array<double, MPC_NBX> ubx;

  /**
   * @brief Constructor
   */
  StateBounds();

  /**
   * @brief Get the lbx array
   *
   * @return double* lbx.
   */
  double * get_lbx();

  /**
   * @brief Get the lbx array
   *
   * @return const double* lbx.
   */
  const double * get_lbx() const;

  /**
   * @brief Get the lbx array
   *
   * @return std::array<double, MPC_NBX> lbx.
   */
  std::array<double, MPC_NBX> get_lbx_array() const;

  /**
   * @brief Get the ubx array
   *
   * @return double* ubx.
   */
  double * get_ubx();

  /**
   * @brief Get the ubx array
   *
   * @return const double* ubx.
   */
  const double * get_ubx() const;

  /**
   * @brief Get the ubx array
   *
   * @return std::array<double, MPC_NBX> ubx.
   */
  std::array<double, MPC_NBX> get_ubx_array() const;

  /**
   * @brief Set the bounds
   *
   * @param bounds bounds.
   */
  void set_bounds(const StateBounds & bounds);

  /**
   * @brief Set the lbx
   *
   * @param lbx lbx.
   */
  void set_lbx(const std::array<double, MPC_NBX> & lbx);

  /**
   * @brief Set the lbx at index
   *
   * @param index index.
   * @param value value.
   */
  void set_lbx(const int index, const double value);

  /**
   * @brief Set the ubx
   *
   * @param ubx ubx.
   */
  void set_ubx(const std::array<double, MPC_NBX> & ubx);

  /**
   * @brief Set the ubx at index
   *
   * @param index index.
   * @param value value.
   */
  void set_ubx(const int index, const double value);
};

/**
 * @brief OnlineParams
 *
 * Online parameters p for the MPC.
 */
struct OnlineParams
{
  static constexpr size_t Np = MPC_NP;
  std::array<double, (MPC_N + 1) * MPC_NP> data;
  static const std::size_t size_n = (MPC_N + static_cast<size_t>(1));
  static const std::size_t size = size_n * MPC_NP;

  /**
   * @brief Constructor
   */
  OnlineParams();

  /**
   * @brief Get the data
   *
   * @return double* data.
   */
  double * get_data();

  /**
   * @brief Get the data
   *
   * @return const double* data.
   */
  const double * get_data() const;

  /**
   * @brief Get the data at index
   *
   * @param index index of the stage.
   * @return double* data.
   */
  double * get_data(const int index);

  /**
   * @brief Get the data at index
   *
   * @param index index of the stage.
   * @return const double* data.
   */
  const double * get_data(const int index) const;

  /**
   * @brief Get the online parameters
   *
   * @return std::array<double, MPC_NP> Copy of online parameters.
   */
  std::array<double, MPC_NP> get_online_params(const int index = 0) const;

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_online_params(const OnlineParams & params);

  /**
   * @brief Set the data at index
   *
   * @param index index.
   * @param value value.
   */
  void set_data(const int index, const double value);

  /**
   * @brief Set the data at index of the stage and value
   *
   * @param ref_index index of the stage.
   * @param value_index index of the value.
   * @param state state.
   */
  void set_data(const int ref_index, const int value_index, const double value);
};

/**
 * @brief SoftStateBounds
 *
 * Soft state bounds lsbx and usbx for the MPC.
 */
struct SoftStateBounds
{
  static constexpr size_t Nsbx = MPC_NSBX;
  std::array<double, MPC_NSBX> lsbx;
  std::array<double, MPC_NSBX> usbx;

  /**
   * @brief Constructor
   */
  SoftStateBounds();

  /**
   * @brief Get the lsbx array
   *
   * @return double* lsbx.
   */
  double * get_lsbx();

  /**
   * @brief Get the lsbx array
   *
   * @return const double* lsbx.
   */
  const double * get_lsbx() const;

  /**
   * @brief Get the lsbx array
   *
   * @return std::array<double, MPC_NSBX> lsbx.
   */
  std::array<double, MPC_NSBX> get_lsbx_array() const;

  /**
   * @brief Get the usbx array
   *
   * @return double* usbx.
   */
  double * get_usbx();

  /**
   * @brief Get the usbx array
   *
   * @return const double* usbx.
   */
  const double * get_usbx() const;

  /**
   * @brief Get the usbx array
   *
   * @return std::array<double, MPC_NSBX> usbx.
   */
  std::array<double, MPC_NSBX> get_usbx_array() const;

  /**
   * @brief Set the bounds
   *
   * @param bounds bounds.
   */
  void set_bounds(const SoftStateBounds & bounds);

  /**
   * @brief Set the lsbx
   *
   * @param lsbx lsbx.
   */
  void set_lsbx(const std::array<double, MPC_NSBX> & lsbx);

  /**
   * @brief Set the lsbx at index
   *
   * @param index index.
   * @param value value.
   */
  void set_lsbx(const int index, const double value);

  /**
   * @brief Set the usbx
   *
   * @param usbx usbx.
   */
  void set_usbx(const std::array<double, MPC_NSBX> & usbx);

  /**
   * @brief Set the usbx at index
   *
   * @param index index.
   * @param value value.
   */
  void set_usbx(const int index, const double value);
};

/**
 * @brief SlackWeights
 *
 * Slack weights Zl, Zu, zl, zu for the MPC soft constraints.
 */
struct SlackWeights
{
  static constexpr size_t Nsbx = MPC_NSBX;
  std::array<double, MPC_NSBX> Zl;  // Diagonal Hessian weights for lower slack variables
  std::array<double, MPC_NSBX> Zu;  // Diagonal Hessian weights for upper slack variables
  std::array<double, MPC_NSBX> zl;  // Linear weights for lower slack variables
  std::array<double, MPC_NSBX> zu;  // Linear weights for upper slack variables

  /**
   * @brief Constructor
   */
  SlackWeights();

  /**
   * @brief Get the Zl array
   *
   * @return double* Zl.
   */
  double * get_Zl();

  /**
   * @brief Get the Zl array
   *
   * @return const double* Zl.
   */
  const double * get_Zl() const;

  /**
   * @brief Get the Zl array
   *
   * @return std::array<double, MPC_NSBX> Zl.
   */
  std::array<double, MPC_NSBX> get_Zl_array() const;

  /**
   * @brief Get the Zu array
   *
   * @return double* Zu.
   */
  double * get_Zu();

  /**
   * @brief Get the Zu array
   *
   * @return const double* Zu.
   */
  const double * get_Zu() const;

  /**
   * @brief Get the Zu array
   *
   * @return std::array<double, MPC_NSBX> Zu.
   */
  std::array<double, MPC_NSBX> get_Zu_array() const;

  /**
   * @brief Get the zl array
   *
   * @return double* zl.
   */
  double * get_zl();

  /**
   * @brief Get the zl array
   *
   * @return const double* zl.
   */
  const double * get_zl() const;

  /**
   * @brief Get the zl array
   *
   * @return std::array<double, MPC_NSBX> zl.
   */
  std::array<double, MPC_NSBX> get_zl_array() const;

  /**
   * @brief Get the zu array
   *
   * @return double* zu.
   */
  double * get_zu();

  /**
   * @brief Get the zu array
   *
   * @return const double* zu.
   */
  const double * get_zu() const;

  /**
   * @brief Get the zu array
   *
   * @return std::array<double, MPC_NSBX> zu.
   */
  std::array<double, MPC_NSBX> get_zu_array() const;

  /**
   * @brief Set the weights
   *
   * @param weights weights.
   */
  void set_weights(const SlackWeights & weights);

  /**
   * @brief Set the Zl
   *
   * @param Zl Zl.
   */
  void set_Zl(const std::array<double, MPC_NSBX> & Zl);

  /**
   * @brief Set the Zl at index
   *
   * @param index index.
   * @param value value.
   */
  void set_Zl(const int index, const double value);

  /**
   * @brief Set the Zu
   *
   * @param Zu Zu.
   */
  void set_Zu(const std::array<double, MPC_NSBX> & Zu);

  /**
   * @brief Set the Zu at index
   *
   * @param index index.
   * @param value value.
   */
  void set_Zu(const int index, const double value);

  /**
   * @brief Set the zl
   *
   * @param zl zl.
   */
  void set_zl(const std::array<double, MPC_NSBX> & zl);

  /**
   * @brief Set the zl at index
   *
   * @param index index.
   * @param value value.
   */
  void set_zl(const int index, const double value);

  /**
   * @brief Set the zu
   *
   * @param zu zu.
   */
  void set_zu(const std::array<double, MPC_NSBX> & zu);

  /**
   * @brief Set the zu at index
   *
   * @param index index.
   * @param value value.
   */
  void set_zu(const int index, const double value);
};

/**
 * @brief SlackWeightsEnd
 *
 * Slack weights Zl_e, Zu_e, zl_e, zu_e for the terminal MPC soft constraints.
 */
struct SlackWeightsEnd
{
  static constexpr size_t Nsbx_e = MPC_NSBX;
  std::array<double, MPC_NSBX> Zl_e;  // Diagonal Hessian weights for lower slack variables
  std::array<double, MPC_NSBX> Zu_e;  // Diagonal Hessian weights for upper slack variables
  std::array<double, MPC_NSBX> zl_e;  // Linear weights for lower slack variables
  std::array<double, MPC_NSBX> zu_e;  // Linear weights for upper slack variables

  /**
   * @brief Constructor
   */
  SlackWeightsEnd();

  /**
   * @brief Get the Zl_e array
   *
   * @return double* Zl_e.
   */
  double * get_Zl_e();

  /**
   * @brief Get the Zl_e array
   *
   * @return const double* Zl_e.
   */
  const double * get_Zl_e() const;

  /**
   * @brief Get the Zl_e array
   *
   * @return std::array<double, MPC_NSBX> Zl_e.
   */
  std::array<double, MPC_NSBX> get_Zl_e_array() const;

  /**
   * @brief Get the Zu_e array
   *
   * @return double* Zu_e.
   */
  double * get_Zu_e();

  /**
   * @brief Get the Zu_e array
   *
   * @return const double* Zu_e.
   */
  const double * get_Zu_e() const;

  /**
   * @brief Get the Zu_e array
   *
   * @return std::array<double, MPC_NSBX> Zu_e.
   */
  std::array<double, MPC_NSBX> get_Zu_e_array() const;

  /**
   * @brief Get the zl_e array
   *
   * @return double* zl_e.
   */
  double * get_zl_e();

  /**
   * @brief Get the zl_e array
   *
   * @return const double* zl_e.
   */
  const double * get_zl_e() const;

  /**
   * @brief Get the zl_e array
   *
   * @return std::array<double, MPC_NSBX> zl_e.
   */
  std::array<double, MPC_NSBX> get_zl_e_array() const;

  /**
   * @brief Get the zu_e array
   *
   * @return double* zu_e.
   */
  double * get_zu_e();

  /**
   * @brief Get the zu_e array
   *
   * @return const double* zu_e.
   */
  const double * get_zu_e() const;

  /**
   * @brief Get the zu_e array
   *
   * @return std::array<double, MPC_NSBX> zu_e.
   */
  std::array<double, MPC_NSBX> get_zu_e_array() const;

  /**
   * @brief Set the weights
   *
   * @param weights weights.
   */
  void set_weights(const SlackWeightsEnd & weights);

  /**
   * @brief Set the Zl_e
   *
   * @param Zl_e Zl_e.
   */
  void set_Zl_e(const std::array<double, MPC_NSBX> & Zl_e);

  /**
   * @brief Set the Zl_e at index
   *
   * @param index index.
   * @param value value.
   */
  void set_Zl_e(const int index, const double value);

  /**
   * @brief Set the Zu_e
   *
   * @param Zu_e Zu_e.
   */
  void set_Zu_e(const std::array<double, MPC_NSBX> & Zu_e);

  /**
   * @brief Set the Zu_e at index
   *
   * @param index index.
   * @param value value.
   */
  void set_Zu_e(const int index, const double value);

  /**
   * @brief Set the zl_e
   *
   * @param zl_e zl_e.
   */
  void set_zl_e(const std::array<double, MPC_NSBX> & zl_e);

  /**
   * @brief Set the zl_e at index
   *
   * @param index index.
   * @param value value.
   */
  void set_zl_e(const int index, const double value);

  /**
   * @brief Set the zu_e
   *
   * @param zu_e zu_e.
   */
  void set_zu_e(const std::array<double, MPC_NSBX> & zu_e);

  /**
   * @brief Set the zu_e at index
   *
   * @param index index.
   * @param value value.
   */
  void set_zu_e(const int index, const double value);
};

}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_DATATYPE_HPP_
