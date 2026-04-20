#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Solver configuration dataclasses."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass
import numpy as np


def _check_index(index: int, max_size: int, index_name: str = 'index') -> None:
    """
    Validate an index in [0, max_size).

    :param index: Index value
    :type index: int
    :param max_size: Maximum allowed size
    :type max_size: int
    :param index_name: Name to use in the error message
    :type index_name: str
    :return: None
    :rtype: None
    """
    if index < 0 or index >= max_size:
        raise IndexError(f"{index_name}={index} is out of range [0, {max_size}).")


def _check_vector_size(name: str, value: np.ndarray, expected_size: int) -> np.ndarray:
    """
    Validate and normalize a 1D numpy array.

    :param name: Variable name for error messages
    :type name: str
    :param value: Input vector
    :type value: np.ndarray
    :param expected_size: Expected vector length
    :type expected_size: int
    :return: Normalized 1D array
    :rtype: np.ndarray
    """
    vector = np.asarray(value)
    if vector.shape != (expected_size,):
        raise ValueError(
            f"Size mismatch: {name} has shape {vector.shape}, "
            f"but expected size is ({expected_size},).")
    return vector


def _check_matrix_size(name: str, value: np.ndarray, expected_shape: tuple[int, int]) -> np.ndarray:
    """
    Validate and normalize a 2D numpy array.

    :param name: Variable name for error messages
    :type name: str
    :param value: Input matrix
    :type value: np.ndarray
    :param expected_shape: Expected matrix shape
    :type expected_shape: tuple[int, int]
    :return: Normalized 2D array
    :rtype: np.ndarray
    """
    matrix = np.asarray(value)
    if matrix.shape != expected_shape:
        raise ValueError(
            f"Size mismatch: {name} has shape {matrix.shape}, "
            f"but expected shape is {expected_shape}.")
    return matrix


class Reference:
    """Reference yref."""

    def __init__(self, mpc_n: int, mpc_ny: int, x_size: int, u_size: int) -> None:
        """
        Initialize the reference y_ref.

        :param mpc_n: Size of the prediction horizon
        :type mpc_n: int
        :param mpc_ny: Size of the reference state
        :type mpc_ny: int
        :param x_size: Size of the state vector
        :type x_size: int
        :param u_size: Size of the actuation vector
        :type u_size: int
        :return: None
        :rtype: None
        """
        self.N = mpc_n
        self.w_size = mpc_ny
        self.x_size = int(x_size)
        self.u_size = int(u_size)

        # Data (same concept as C++ `Reference::data`)
        self.data = np.zeros((self.N, self.w_size))
        self.y_ref = self.data

    def get_data(self, index: int) -> np.ndarray:
        """
        Get the data at stage index.

        :param index: index of the stage
        :type index: int
        :return: stage reference vector
        :rtype: np.ndarray
        """
        _check_index(index, self.N)
        return self.data[index, :]

    def set_data(self, *args) -> None:
        """
        Set reference data.

        :param args: Either ``(index, value)`` or ``(ref_index, value_index, value)``
        :type args: tuple
        :return: None
        :rtype: None
        """
        if len(args) == 2:
            index, value = args
            _check_index(index, self.N * self.w_size)
            ref_index = index // self.w_size
            value_index = index % self.w_size
            self.data[ref_index, value_index] = value
            return

        if len(args) == 3:
            ref_index, value_index, value = args
            _check_index(ref_index, self.N, 'ref_index')
            _check_index(value_index, self.w_size, 'value_index')
            self.data[ref_index, value_index] = value
            return

        raise TypeError("set_data expects (index, value) or (ref_index, value_index, value).")
    
    def set_y_ref(self, y_ref: np.ndarray, stage: int = -1) -> None:
        """
        Set the reference state of the MPC problem.
        
        If shape is y_ref_size, set it for all stages.
        If shape is (N+1, y_ref_size), set it for each stage.

        :param y_ref: Reference state array
        :type y_ref: np.ndarray
        :param stage: Stage index (default: -1 for all stages)
        :type stage: int
        :return: None
        :rtype: None
        """
        # Check size of y_ref
        if y_ref.shape[0] == self.w_size:
            # If y_ref is given for all stages, set it for each stage
            if stage == -1:
                for stage_i in range(self.N):
                    self.y_ref[stage_i, :] = y_ref
            else:
                self.y_ref[stage, :] = y_ref
            return
        elif y_ref.shape[0] == self.N and y_ref.shape[1] == self.w_size:
            # If y_ref is given for each stage, set it for each stage
            if stage == -1:
                for stage_i in range(self.N):
                    self.y_ref[stage_i, :] = y_ref[stage_i, :]
            else:
                self.y_ref[stage, :] = y_ref[stage, :]
            return
        else:
            raise ValueError(
                f"Size mismatch: y_ref has shape {y_ref.shape}, "
                f"but expected size is either ({self.w_size},) or ({self.N}, {self.w_size}).")


class ReferenceEnd:
    """Reference y_ref_end for MPC."""

    def __init__(self, mpc_nyn: int) -> None:
        """Initialize the reference y_ref_e.
        
        :param mpc_nyn: Size of the reference terminal state
        :type mpc_nyn: int
        :return: None
        :rtype: None
        """
        self.we_size = mpc_nyn

        # Data (same concept as C++ `ReferenceEnd::data`)
        self.data = np.zeros(self.we_size)
        self.y_ref_e = self.data

    def get_data(self) -> np.ndarray:
        """
        Get the terminal reference data.

        :return: terminal reference vector
        :rtype: np.ndarray
        """
        return self.data

    def set_y_ref_e(self, y_ref_e: np.ndarray) -> None:
        """
        Set the reference terminal state of the MPC problem.

        :param y_ref_e: Reference terminal state array
        :type y_ref_e: np.ndarray
        :return: None
        :rtype: None
        """
        # Check size of y_ref_e
        if y_ref_e.shape[0] != self.we_size:
            raise ValueError(
                f"Size mismatch: y_ref_e has shape {y_ref_e.shape}, "
                f"but expected size is ({self.we_size},).")
        self.data[:] = y_ref_e

    def set_data(self, index: int, value: float) -> None:
        """
        Set the data at index.

        :param index: index
        :type index: int
        :param value: value
        :type value: float
        :return: None
        :rtype: None
        """
        _check_index(index, self.we_size)
        self.data[index] = value


@dataclass
class Gains:
    """
    Cost for the MPC problem.
    
    W: Weight for internal stages (stage cost)
    We: Weight for end stages (terminal cost)
    """
    W: np.ndarray  # Weight for internal stages
    We: np.ndarray  # Weight for end stages

    def __init__(self, mpc_ny: int, mpc_nyn: int) -> None:
        """
        Initialize the gains W and We.
        
        :param mpc_ny: Size of the reference state for internal stages
        :type mpc_ny: int
        :param mpc_nyn: Size of the reference state for end stages
        :type mpc_nyn: int
        """
        self.q_size = mpc_nyn
        self.qe_size = mpc_nyn
        self.u_size = mpc_ny - mpc_nyn
        
        self.w_size = mpc_ny
        self.we_size = self.q_size
        self.W = np.zeros((self.w_size, self.w_size))
        self.We = np.zeros((self.we_size, self.we_size))
    
    def get_W(self) -> np.ndarray:
        """
        Get the W matrix.

        :return: W
        :rtype: np.ndarray
        """
        return self.W

    def get_We(self) -> np.ndarray:
        """
        Get the We matrix.

        :return: We
        :rtype: np.ndarray
        """
        return self.We

    def set_W(self, W_or_index, value: float = None) -> None:
        """
        Set W matrix.

        :param W_or_index: Full W matrix or diagonal index
        :type W_or_index: Union[np.ndarray, int]
        :param value: Diagonal value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.W = _check_matrix_size('W', W_or_index, self.W.shape)
            return

        index = int(W_or_index)
        _check_index(index, self.w_size)
        self.W[index, index] = value
    
    def set_We(self, We_or_index, value: float = None) -> None:
        """
        Set We matrix.

        :param We_or_index: Full We matrix or diagonal index
        :type We_or_index: Union[np.ndarray, int]
        :param value: Diagonal value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.We = _check_matrix_size('We', We_or_index, self.We.shape)
            return

        index = int(We_or_index)
        _check_index(index, self.we_size)
        self.We[index, index] = value

    def set_gains(self, gains: 'Gains') -> None:
        """
        Set Q, R and Qe from another Gains object.

        :param gains: gains
        :type gains: Gains
        :return: None
        :rtype: None
        """
        if gains.W.shape != self.W.shape or gains.We.shape != self.We.shape:
            raise ValueError(
                f"Size mismatch: gains has W shape {gains.W.shape} and We shape {gains.We.shape}, "
                f"but expected {self.W.shape} and {self.We.shape}.")
        self.W = gains.W.copy()
        self.We = gains.We.copy()
    
    def set_Q(self, Q_or_index, value: float = None) -> None:
        """
        Set Q matrix.

        :param Q_or_index: Q diagonal index, Q diagonal vector, or full Q matrix
        :type Q_or_index: Union[int, np.ndarray]
        :param value: Diagonal value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is not None:
            index = int(Q_or_index)
            _check_index(index, self.q_size)
            self.set_W(index, value)
            return

        Q = np.asarray(Q_or_index)
        if Q.shape == (self.q_size,):
            for i, q_i in enumerate(Q):
                self.set_Q(i, q_i)
            return
        if Q.shape == (self.q_size, self.q_size):
            self.W[:self.q_size, :self.q_size] = Q
            return

        raise ValueError(
            f"Size mismatch: Q has shape {Q.shape}, "
            f"but expected ({self.q_size},) or ({self.q_size}, {self.q_size}).")
    
    def set_R(self, R_or_index, value: float = None) -> None:
        """
        Set R matrix.

        :param R_or_index: R diagonal index, R diagonal vector, or full R matrix
        :type R_or_index: Union[int, np.ndarray]
        :param value: Diagonal value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is not None:
            index = int(R_or_index)
            _check_index(index, self.u_size)
            self.set_W(self.q_size + index, value)
            return

        R = np.asarray(R_or_index)
        if R.shape == (self.u_size,):
            for i, r_i in enumerate(R):
                self.set_R(i, r_i)
            return
        if R.shape == (self.u_size, self.u_size):
            self.W[self.q_size:, self.q_size:] = R
            return

        raise ValueError(
            f"Size mismatch: R has shape {R.shape}, "
            f"but expected ({self.u_size},) or ({self.u_size}, {self.u_size}).")
    
    def set_Qe(self, Qe_or_index, value: float = None) -> None:
        """
        Set Qe matrix.

        :param Qe_or_index: Qe diagonal index, Qe diagonal vector, or full Qe matrix
        :type Qe_or_index: Union[int, np.ndarray]
        :param value: Diagonal value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is not None:
            index = int(Qe_or_index)
            _check_index(index, self.qe_size)
            self.set_We(index, value)
            return

        Qe = np.asarray(Qe_or_index)
        if Qe.shape == (self.qe_size,):
            for i, qe_i in enumerate(Qe):
                self.set_Qe(i, qe_i)
            return
        if Qe.shape == self.We.shape:
            self.We = Qe
            return

        raise ValueError(
            f"Size mismatch: Qe has shape {Qe.shape}, "
            f"but expected ({self.qe_size},) or {self.We.shape}.")

    def get_Q(self) -> np.ndarray:
        """
        Get the diagonal of the Q matrix.

        :return: Copy of Q diagonal
        :rtype: np.ndarray
        """
        return np.diag(self.W[:self.q_size, :self.q_size]).copy()

    def get_Qe(self) -> np.ndarray:
        """
        Get the diagonal of the Qe matrix.

        :return: Copy of Qe diagonal
        :rtype: np.ndarray
        """
        return np.diag(self.We).copy()

    def get_R(self) -> np.ndarray:
        """
        Get the diagonal of the R matrix.

        :return: Copy of R diagonal
        :rtype: np.ndarray
        """
        return np.diag(self.W[self.q_size:, self.q_size:]).copy()


class ActuationBounds:
    """ActuationBounds lbu and ubu for the MPC."""
    lbu: np.ndarray
    ubu: np.ndarray
    
    def __init__(self, mpc_nu: int) -> None:
        """Initialize the actuation bounds lbu and ubu.
        
        :param mpc_nu: Size of the control input
        :type mpc_nu: int
        :return: None
        :rtype: None
        """
        self.u_size = mpc_nu
        
        # Data
        self.lbu = np.zeros(self.u_size)
        self.ubu = np.zeros(self.u_size)
        # Backward-compatible aliases
        self.u_min = self.lbu
        self.u_max = self.ubu

    def get_lbu(self) -> np.ndarray:
        """
        Get the lbu array.

        :return: lbu
        :rtype: np.ndarray
        """
        return self.lbu

    def get_ubu(self) -> np.ndarray:
        """
        Get the ubu array.

        :return: ubu
        :rtype: np.ndarray
        """
        return self.ubu

    def set_bounds(self, bounds: 'ActuationBounds') -> None:
        """
        Set the bounds.

        :param bounds: bounds
        :type bounds: ActuationBounds
        :return: None
        :rtype: None
        """
        if bounds.u_size != self.u_size:
            raise ValueError(
                f"Size mismatch: bounds has u_size={bounds.u_size}, "
                f"but expected u_size={self.u_size}.")
        self.lbu[:] = bounds.lbu
        self.ubu[:] = bounds.ubu

    def set_lbu(self, lbu_or_index, value: float = None) -> None:
        """
        Set lbu.

        :param lbu_or_index: Full lbu vector or index
        :type lbu_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.lbu[:] = _check_vector_size('lbu', lbu_or_index, self.u_size)
            return

        index = int(lbu_or_index)
        _check_index(index, self.u_size)
        self.lbu[index] = value

    def set_ubu(self, ubu_or_index, value: float = None) -> None:
        """
        Set ubu.

        :param ubu_or_index: Full ubu vector or index
        :type ubu_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.ubu[:] = _check_vector_size('ubu', ubu_or_index, self.u_size)
            return

        index = int(ubu_or_index)
        _check_index(index, self.u_size)
        self.ubu[index] = value
    
    def set_u_bounds(self, u_min: np.ndarray, u_max: np.ndarray) -> None:
        """
        Set the control input bounds.

        :param u_min: Minimum control input
        :type u_min: np.ndarray
        :param u_max: Maximum control input
        :type u_max: np.ndarray
        :return: None
        :rtype: None
        """
        self.set_lbu(u_min)
        self.set_ubu(u_max)


class StateBounds:
    """
    StateBounds lbx and ubx for the MPC.
    """

    lbx: np.ndarray
    ubx: np.ndarray

    def __init__(self, mpc_nsbx: int) -> None:
        """
        Initialize the state bounds lbx and ubx.

        :param mpc_nsbx: Size of the bounded state vector
        :type mpc_nsbx: int
        :return: None
        :rtype: None
        """
        self.bx_size = mpc_nsbx
        self.lbx = np.zeros(self.bx_size)
        self.ubx = np.zeros(self.bx_size)

    def get_lbx(self) -> np.ndarray:
        """
        Get the lbx array.

        :return: lbx
        :rtype: np.ndarray
        """
        return self.lbx

    def get_ubx(self) -> np.ndarray:
        """
        Get the ubx array.

        :return: ubx
        :rtype: np.ndarray
        """
        return self.ubx

    def set_bounds(self, bounds: 'StateBounds') -> None:
        """
        Set the bounds.

        :param bounds: bounds
        :type bounds: StateBounds
        :return: None
        :rtype: None
        """
        if bounds.bx_size != self.bx_size:
            raise ValueError(
                f"Size mismatch: bounds has bx_size={bounds.bx_size}, "
                f"but expected bx_size={self.bx_size}.")
        self.lbx = bounds.lbx.copy()
        self.ubx = bounds.ubx.copy()

    def set_lbx(self, lbx_or_index, value: float = None) -> None:
        """
        Set lbx.

        :param lbx_or_index: Full lbx vector or index
        :type lbx_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            lbx = _check_vector_size('lbx', lbx_or_index, self.bx_size)
            self.lbx[:] = lbx
            return

        index = int(lbx_or_index)
        _check_index(index, self.bx_size)
        self.lbx[index] = value

    def set_lbx_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_lbx(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_lbx(index, value)

    def set_ubx(self, ubx_or_index, value: float = None) -> None:
        """
        Set ubx.

        :param ubx_or_index: Full ubx vector or index
        :type ubx_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            ubx = _check_vector_size('ubx', ubx_or_index, self.bx_size)
            self.ubx[:] = ubx
            return

        index = int(ubx_or_index)
        _check_index(index, self.bx_size)
        self.ubx[index] = value

    def set_ubx_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_ubx(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_ubx(index, value)


class NonlinearConstraintBounds:
    """
    Nonlinear constraint bounds lh and uh for the MPC.

    These are the lower and upper bounds on the nonlinear inequality
    constraints h(x, u) at intermediate shooting nodes (1 to N-1) and
    at the terminal node (N).
    """
    lh: np.ndarray
    uh: np.ndarray

    def __init__(self, mpc_nh: int) -> None:
        """
        Initialize the nonlinear constraint bounds lh and uh.

        :param mpc_nh: Number of nonlinear constraints
        :type mpc_nh: int
        :return: None
        :rtype: None
        """
        self.nh_size = mpc_nh
        self.lh = np.zeros(self.nh_size)
        self.uh = np.zeros(self.nh_size)

    def get_lh(self) -> np.ndarray:
        """
        Get the lh array.

        :return: lh
        :rtype: np.ndarray
        """
        return self.lh

    def get_uh(self) -> np.ndarray:
        """
        Get the uh array.

        :return: uh
        :rtype: np.ndarray
        """
        return self.uh

    def set_lh(self, lh_or_index, value: float = None) -> None:
        """
        Set lh.

        :param lh_or_index: Full lh vector or index
        :type lh_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.lh[:] = _check_vector_size('lh', lh_or_index, self.nh_size)
            return
        index = int(lh_or_index)
        _check_index(index, self.nh_size)
        self.lh[index] = value

    def set_uh(self, uh_or_index, value: float = None) -> None:
        """
        Set uh.

        :param uh_or_index: Full uh vector or index
        :type uh_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.uh[:] = _check_vector_size('uh', uh_or_index, self.nh_size)
            return
        index = int(uh_or_index)
        _check_index(index, self.nh_size)
        self.uh[index] = value

    def set_bounds(self, lh: np.ndarray, uh: np.ndarray) -> None:
        """
        Set both lower and upper nonlinear constraint bounds at once.

        :param lh: Lower bounds vector
        :type lh: np.ndarray
        :param uh: Upper bounds vector
        :type uh: np.ndarray
        :return: None
        :rtype: None
        """
        self.set_lh(lh)
        self.set_uh(uh)


class SoftNonlinearConstraintBounds:
    """
    Soft nonlinear constraint bounds lsh and ush for the MPC.
    """
    lsh: np.ndarray
    ush: np.ndarray

    def __init__(self, mpc_nsh: int) -> None:
        """
        Initialize the soft nonlinear constraint bounds lsh and ush.

        :param mpc_nsh: Number of soft nonlinear constraints
        :type mpc_nsh: int
        :return: None
        :rtype: None
        """
        self.sh_size = mpc_nsh
        self.lsh = np.zeros(self.sh_size)
        self.ush = np.zeros(self.sh_size)

    def get_lsh(self) -> np.ndarray:
        """
        Get the lsh array.

        :return: lsh
        :rtype: np.ndarray
        """
        return self.lsh

    def get_ush(self) -> np.ndarray:
        """
        Get the ush array.

        :return: ush
        :rtype: np.ndarray
        """
        return self.ush

    def set_lsh(self, lsh_or_index, value: float = None) -> None:
        """
        Set lsh.

        :param lsh_or_index: Full lsh vector or index
        :type lsh_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.lsh[:] = _check_vector_size('lsh', lsh_or_index, self.sh_size)
            return

        index = int(lsh_or_index)
        _check_index(index, self.sh_size)
        self.lsh[index] = value

    def set_ush(self, ush_or_index, value: float = None) -> None:
        """
        Set ush.

        :param ush_or_index: Full ush vector or index
        :type ush_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.ush[:] = _check_vector_size('ush', ush_or_index, self.sh_size)
            return

        index = int(ush_or_index)
        _check_index(index, self.sh_size)
        self.ush[index] = value

    def set_bounds(self, lsh: np.ndarray, ush: np.ndarray) -> None:
        """
        Set both lower and upper soft nonlinear bounds at once.

        :param lsh: Lower soft nonlinear bounds vector
        :type lsh: np.ndarray
        :param ush: Upper soft nonlinear bounds vector
        :type ush: np.ndarray
        :return: None
        :rtype: None
        """
        self.set_lsh(lsh)
        self.set_ush(ush)


class SoftStateBounds:
    """
    Soft state bounds lsbx and usbx for the MPC.
    """

    lsbx: np.ndarray
    usbx: np.ndarray

    def __init__(self, mpc_nsbx: int) -> None:
        """
        Initialize the soft state bounds lsbx and usbx.

        :param mpc_nsbx: Size of the soft bounded state vector
        :type mpc_nsbx: int
        :return: None
        :rtype: None
        """
        self.sbx_size = mpc_nsbx
        self.lsbx = np.zeros(self.sbx_size)
        self.usbx = np.zeros(self.sbx_size)

    def get_lsbx(self) -> np.ndarray:
        """
        Get the lsbx array.
    
        :return: lsbx
        :rtype: np.ndarray
        """
        return self.lsbx

    def get_usbx(self) -> np.ndarray:
        """
        Get the usbx array.

        :return: usbx
        :rtype: np.ndarray
        """
        return self.usbx

    def set_bounds(self, bounds: 'SoftStateBounds') -> None:
        """
        Set the bounds.

        :param bounds: bounds
        :type bounds: SoftStateBounds
        :return: None
        :rtype: None
        """
        if bounds.sbx_size != self.sbx_size:
            raise ValueError(
                f"Size mismatch: bounds has sbx_size={bounds.sbx_size}, "
                f"but expected sbx_size={self.sbx_size}.")
        self.lsbx = bounds.lsbx.copy()
        self.usbx = bounds.usbx.copy()

    def set_lsbx(self, lsbx_or_index, value: float = None) -> None:
        """
        Set lsbx.

        :param lsbx_or_index: Full lsbx vector or index
        :type lsbx_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            lsbx = _check_vector_size('lsbx', lsbx_or_index, self.sbx_size)
            self.lsbx[:] = lsbx
            return

        index = int(lsbx_or_index)
        _check_index(index, self.sbx_size)
        self.lsbx[index] = value

    def set_lsbx_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_lsbx(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_lsbx(index, value)

    def set_usbx(self, usbx_or_index, value: float = None) -> None:
        """
        Set usbx.

        :param usbx_or_index: Full usbx vector or index
        :type usbx_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            usbx = _check_vector_size('usbx', usbx_or_index, self.sbx_size)
            self.usbx[:] = usbx
            return

        index = int(usbx_or_index)
        _check_index(index, self.sbx_size)
        self.usbx[index] = value

    def set_usbx_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_usbx(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_usbx(index, value)


class SlackWeights:
    """
    Slack weights Zl, Zu, zl, zu for the MPC soft constraints.
    """

    Zl: np.ndarray
    Zu: np.ndarray
    zl: np.ndarray
    zu: np.ndarray

    def __init__(self, mpc_nsbx: int) -> None:
        """
        Initialize slack weights Zl, Zu, zl, zu.

        :param mpc_nsbx: Size of the slack vectors
        :type mpc_nsbx: int
        :return: None
        :rtype: None
        """
        self.sbx_size = mpc_nsbx
        self.Zl = np.zeros(self.sbx_size)
        self.Zu = np.zeros(self.sbx_size)
        self.zl = np.zeros(self.sbx_size)
        self.zu = np.zeros(self.sbx_size)

    def get_Zl(self) -> np.ndarray:
        """
        Get the Zl array.

        :return: Zl
        :rtype: np.ndarray
        """
        return self.Zl

    def get_Zu(self) -> np.ndarray:
        """
        Get the Zu array.

        :return: Zu
        :rtype: np.ndarray
        """
        return self.Zu

    def get_zl(self) -> np.ndarray:
        """
        Get the zl array.

        :return: zl
        :rtype: np.ndarray
        """
        return self.zl

    def get_zu(self) -> np.ndarray:
        """
        Get the zu array.

        :return: zu
        :rtype: np.ndarray
        """
        return self.zu

    def set_weights(self, weights: 'SlackWeights') -> None:
        """
        Set the weights.

        :param weights: weights
        :type weights: SlackWeights
        :return: None
        :rtype: None
        """
        if weights.sbx_size != self.sbx_size:
            raise ValueError(
                f"Size mismatch: weights has sbx_size={weights.sbx_size}, "
                f"but expected sbx_size={self.sbx_size}.")
        self.Zl = weights.Zl.copy()
        self.Zu = weights.Zu.copy()
        self.zl = weights.zl.copy()
        self.zu = weights.zu.copy()

    def set_Zl(self, Zl_or_index, value: float = None) -> None:
        """
        Set Zl.

        :param Zl_or_index: Full Zl vector or index
        :type Zl_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.Zl[:] = _check_vector_size('Zl', Zl_or_index, self.sbx_size)
            return

        index = int(Zl_or_index)
        _check_index(index, self.sbx_size)
        self.Zl[index] = value

    def set_Zl_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_Zl(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_Zl(index, value)

    def set_Zu(self, Zu_or_index, value: float = None) -> None:
        """
        Set Zu.

        :param Zu_or_index: Full Zu vector or index
        :type Zu_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.Zu[:] = _check_vector_size('Zu', Zu_or_index, self.sbx_size)
            return

        index = int(Zu_or_index)
        _check_index(index, self.sbx_size)
        self.Zu[index] = value

    def set_Zu_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_Zu(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_Zu(index, value)

    def set_zl(self, zl_or_index, value: float = None) -> None:
        """
        Set zl.

        :param zl_or_index: Full zl vector or index
        :type zl_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.zl[:] = _check_vector_size('zl', zl_or_index, self.sbx_size)
            return

        index = int(zl_or_index)
        _check_index(index, self.sbx_size)
        self.zl[index] = value

    def set_zl_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_zl(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_zl(index, value)

    def set_zu(self, zu_or_index, value: float = None) -> None:
        """
        Set zu.

        :param zu_or_index: Full zu vector or index
        :type zu_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.zu[:] = _check_vector_size('zu', zu_or_index, self.sbx_size)
            return

        index = int(zu_or_index)
        _check_index(index, self.sbx_size)
        self.zu[index] = value

    def set_zu_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_zu(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_zu(index, value)


class SlackWeightsEnd:
    """
    Slack weights Zl_e, Zu_e, zl_e, zu_e for the terminal MPC soft constraints.
    """

    Zl_e: np.ndarray
    Zu_e: np.ndarray
    zl_e: np.ndarray
    zu_e: np.ndarray

    def __init__(self, mpc_nsbx: int) -> None:
        """
        Initialize terminal slack weights Zl_e, Zu_e, zl_e, zu_e.

        :param mpc_nsbx: Size of the terminal slack vectors
        :type mpc_nsbx: int
        :return: None
        :rtype: None
        """
        self.sbx_e_size = mpc_nsbx
        self.Zl_e = np.zeros(self.sbx_e_size)
        self.Zu_e = np.zeros(self.sbx_e_size)
        self.zl_e = np.zeros(self.sbx_e_size)
        self.zu_e = np.zeros(self.sbx_e_size)

    def get_Zl_e(self) -> np.ndarray:
        """
        Get the Zl_e array.

        :return: Zl_e
        :rtype: np.ndarray
        """
        return self.Zl_e

    def get_Zu_e(self) -> np.ndarray:
        """
        Get the Zu_e array.

        :return: Zu_e
        :rtype: np.ndarray
        """
        return self.Zu_e

    def get_zl_e(self) -> np.ndarray:
        """
        Get the zl_e array.

        :return: zl_e
        :rtype: np.ndarray
        """
        return self.zl_e

    def get_zu_e(self) -> np.ndarray:
        """
        Get the zu_e array.

        :return: zu_e
        :rtype: np.ndarray
        """
        return self.zu_e

    def set_weights(self, weights: 'SlackWeightsEnd') -> None:
        """
        Set the weights.

        :param weights: weights
        :type weights: SlackWeightsEnd
        :return: None
        :rtype: None
        """
        if weights.sbx_e_size != self.sbx_e_size:
            raise ValueError(
                f"Size mismatch: weights has sbx_e_size={weights.sbx_e_size}, "
                f"but expected sbx_e_size={self.sbx_e_size}.")
        self.Zl_e = weights.Zl_e.copy()
        self.Zu_e = weights.Zu_e.copy()
        self.zl_e = weights.zl_e.copy()
        self.zu_e = weights.zu_e.copy()

    def set_Zl_e(self, Zl_e_or_index, value: float = None) -> None:
        """
        Set Zl_e.

        :param Zl_e_or_index: Full Zl_e vector or index
        :type Zl_e_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.Zl_e[:] = _check_vector_size('Zl_e', Zl_e_or_index, self.sbx_e_size)
            return

        index = int(Zl_e_or_index)
        _check_index(index, self.sbx_e_size)
        self.Zl_e[index] = value

    def set_Zl_e_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_Zl_e(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_Zl_e(index, value)

    def set_Zu_e(self, Zu_e_or_index, value: float = None) -> None:
        """
        Set Zu_e.

        :param Zu_e_or_index: Full Zu_e vector or index
        :type Zu_e_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.Zu_e[:] = _check_vector_size('Zu_e', Zu_e_or_index, self.sbx_e_size)
            return

        index = int(Zu_e_or_index)
        _check_index(index, self.sbx_e_size)
        self.Zu_e[index] = value

    def set_Zu_e_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_Zu_e(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_Zu_e(index, value)

    def set_zl_e(self, zl_e_or_index, value: float = None) -> None:
        """
        Set zl_e.

        :param zl_e_or_index: Full zl_e vector or index
        :type zl_e_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.zl_e[:] = _check_vector_size('zl_e', zl_e_or_index, self.sbx_e_size)
            return

        index = int(zl_e_or_index)
        _check_index(index, self.sbx_e_size)
        self.zl_e[index] = value

    def set_zl_e_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_zl_e(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_zl_e(index, value)

    def set_zu_e(self, zu_e_or_index, value: float = None) -> None:
        """
        Set zu_e.

        :param zu_e_or_index: Full zu_e vector or index
        :type zu_e_or_index: Union[np.ndarray, int]
        :param value: Value when setting by index
        :type value: float, optional
        :return: None
        :rtype: None
        """
        if value is None:
            self.zu_e[:] = _check_vector_size('zu_e', zu_e_or_index, self.sbx_e_size)
            return

        index = int(zu_e_or_index)
        _check_index(index, self.sbx_e_size)
        self.zu_e[index] = value

    def set_zu_e_at(self, index: int, value: float) -> None:
        """
        Backward-compatible alias of ``set_zu_e(index, value)``.

        :param index: Index
        :type index: int
        :param value: Value
        :type value: float
        :return: None
        :rtype: None
        """
        self.set_zu_e(index, value)


if __name__ == '__main__':
    pass
