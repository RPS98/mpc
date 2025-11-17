#!/usr/bin/env python3
"""
Hermite Spline and Arc Length Reparametrization.

This module provides:
1. HermiteSpline: Creates a Hermite spline from points and tangents
2. compute_arc_length_reparametrization: Computes arc length reparametrization parameters

Example:
    >>> import numpy as np
    >>> from hermite_spline import HermiteSpline, compute_arc_length_reparametrization
    >>> 
    >>> # Step 1: Create Hermite spline
    >>> points = [np.array([0, 0, 1]), np.array([1, 0, 1]), ...]
    >>> tangents = [np.array([1, 0, 0]), np.array([1, 0, 0]), ...]
    >>> spline = HermiteSpline(points, tangents)
    >>> 
    >>> # Step 2: Compute arc length reparametrization parameters
    >>> params = compute_arc_length_reparametrization(spline)
    >>> np.savez('spline_params.npz', **params)
"""

import numpy as np
from typing import List, Tuple, Dict


class HermiteSpline:
    """
    Piecewise cubic Hermite spline in 3D.
    
    Given N points and their tangent vectors, creates a smooth curve
    through the points with specified tangents.
    
    Attributes:
        points (List[np.ndarray]): Control points (N, 3)
        tangents (List[np.ndarray]): Tangent vectors at control points (N, 3)
        ti (np.ndarray): Parameter values at control points (N,)
    """
    
    def __init__(
        self,
        points: List[np.ndarray],
        tangents: List[np.ndarray],
        ti: List[float] = None
    ):
        """
        Initialize Hermite spline from points and tangents.
        
        Args:
            points: List of N numpy arrays (3,) with control points
            tangents: List of N numpy arrays (3,) with tangent vectors
            ti: Optional list of parameter values. If None, uses [0, 1, 2, ..., N-1]
        
        Example:
            >>> points = [np.array([0, 0, 1]), np.array([1, 0, 1])]
            >>> tangents = [np.array([1, 0, 0]), np.array([1, 0, 0])]
            >>> spline = HermiteSpline(points, tangents)
        """
        # Validate and store points
        self.points = [np.asarray(p, dtype=float).flatten() for p in points]
        self.tangents = [np.asarray(m, dtype=float).flatten() for m in tangents]
        
        # Validate
        assert len(self.points) == len(self.tangents), \
            "Number of points and tangents must match"
        assert len(self.points) >= 2, "Need at least 2 points"
        
        for i, p in enumerate(self.points):
            assert p.shape == (3,), f"Point {i} must be 3D, got shape {p.shape}"
        for i, m in enumerate(self.tangents):
            assert m.shape == (3,), f"Tangent {i} must be 3D, got shape {m.shape}"
        
        # Set parameter values
        if ti is None:
            self.ti = np.arange(len(self.points), dtype=float)
        else:
            self.ti = np.asarray(ti, dtype=float)
            assert len(self.ti) == len(self.points), \
                "Length of ti must match number of points"
    
    def _hermite_basis(self, s: float) -> Tuple[float, float, float, float]:
        """
        Compute Hermite basis functions at s ∈ [0, 1].
        
        :param s: Normalized parameter in [0, 1]
        :type s: float
        :return: Tuple of four basis functions (h00, h10, h01, h11)
        :rtype: Tuple[float, float, float, float]
        """
        h00 = 2 * s**3 - 3 * s**2 + 1
        h10 = s**3 - 2 * s**2 + s
        h01 = -2 * s**3 + 3 * s**2
        h11 = s**3 - s**2
        return h00, h10, h01, h11
    
    def _hermite_basis_derivative(self, s: float) -> Tuple[float, float, float, float]:
        """
        Compute derivatives of Hermite basis functions at s ∈ [0, 1].
        
        :param s: Normalized parameter in [0, 1]
        :type s: float
        :return: Tuple of four basis function derivatives (h00', h10', h01', h11')
        :rtype: Tuple[float, float, float, float]
        """
        h00_d = 6 * s**2 - 6 * s
        h10_d = 3 * s**2 - 4 * s + 1
        h01_d = -6 * s**2 + 6 * s
        h11_d = 3 * s**2 - 2 * s
        return h00_d, h10_d, h01_d, h11_d
    
    def _find_segment(self, t: float) -> int:
        """
        Find which segment contains parameter t.
        
        :param t: Parameter value to search for
        :type t: float
        :return: Index of the segment containing t
        :rtype: int
        """
        for i in range(len(self.ti) - 1):
            if t <= self.ti[i + 1]:
                return i
        return len(self.ti) - 2  # Last segment
    
    def evaluate(self, t: float) -> np.ndarray:
        """
        Evaluate spline position at parameter t.
        
        :param t: Parameter value
        :type t: float
        :return: Position as numpy array (3,)
        :rtype: np.ndarray
        """
        # Clamp to valid range
        t = np.clip(t, self.ti[0], self.ti[-1])
        
        # Find segment
        idx = self._find_segment(t)
        
        # Get segment data
        p0 = self.points[idx]
        p1 = self.points[idx + 1]
        m0 = self.tangents[idx]
        m1 = self.tangents[idx + 1]
        t0 = self.ti[idx]
        t1 = self.ti[idx + 1]
        
        # Normalize to [0, 1]
        dt = t1 - t0
        s = (t - t0) / dt
        s = np.clip(s, 0.0, 1.0)
        
        # Evaluate Hermite polynomial
        h00, h10, h01, h11 = self._hermite_basis(s)
        return h00 * p0 + h10 * m0 * dt + h01 * p1 + h11 * m1 * dt
    
    def evaluate_derivative(self, t: float) -> np.ndarray:
        """
        Evaluate spline derivative dp/dt at parameter t.
        
        :param t: Parameter value
        :type t: float
        :return: Derivative as numpy array (3,)
        :rtype: np.ndarray
        """
        # Clamp to valid range
        t = np.clip(t, self.ti[0], self.ti[-1])
        
        # Find segment
        idx = self._find_segment(t)
        
        # Get segment data
        p0 = self.points[idx]
        p1 = self.points[idx + 1]
        m0 = self.tangents[idx]
        m1 = self.tangents[idx + 1]
        t0 = self.ti[idx]
        t1 = self.ti[idx + 1]
        
        # Normalize to [0, 1]
        dt = t1 - t0
        s = (t - t0) / dt
        s = np.clip(s, 0.0, 1.0)
        
        # Evaluate derivative
        h00_d, h10_d, h01_d, h11_d = self._hermite_basis_derivative(s)
        dp_ds = h00_d * p0 + h10_d * m0 * dt + h01_d * p1 + h11_d * m1 * dt
        
        # Chain rule: dp/dt = dp/ds * ds/dt
        ds_dt = 1.0 / dt
        return dp_ds * ds_dt
    
    def get_parameters(self) -> Dict:
        """
        Get spline parameters.
        
        :return: Dictionary with 'points', 'tangents', 'ti'
        :rtype: Dict
        """
        return {
            'points': np.array(self.points),    # (N, 3)
            'tangents': np.array(self.tangents), # (N, 3)
            'ti': self.ti                        # (N,)
        }


def compute_arc_length_reparametrization(
    spline: HermiteSpline,
    n_samples: int = 200,
    poly_degree: int = 5
) -> Dict:
    """
    Compute arc length reparametrization parameters for a Hermite spline.
    
    This function takes a HermiteSpline and computes the arc length parametrization,
    returning all parameters needed for symbolic evaluation.
    
    Steps:
    1. Sample parameter t uniformly
    2. Evaluate ||dp/dt|| at each sample
    3. Integrate using trapezoidal rule to get s(t)
    4. Fit polynomial to approximate inverse t(s)
    
    :param spline: HermiteSpline object to reparametrize
    :type spline: HermiteSpline
    :param n_samples: Number of samples for numerical integration (default 200)
    :type n_samples: int
    :param poly_degree: Degree of polynomial to approximate t(s) (default 5)
    :type poly_degree: int
    :return: Dictionary with points, tangents, ti, poly_coeffs, and total_length
    :rtype: Dict
    
    Example:
        >>> spline = HermiteSpline(points, tangents)
        >>> params = compute_arc_length_reparametrization(spline)
        >>> np.savez('spline_params.npz', **params)
    """
    # Sample parameter t uniformly
    t_min = spline.ti[0]
    t_max = spline.ti[-1]
    t_samples = np.linspace(t_min, t_max, n_samples)
    
    # Evaluate ||dp/dt|| at each sample point
    norms = np.array([
        np.linalg.norm(spline.evaluate_derivative(t))
        for t in t_samples
    ])
    
    # Trapezoidal integration: s(t) = ∫||dp/dt|| dt
    dt = t_samples[1] - t_samples[0]
    arc_lengths = np.zeros(n_samples)
    arc_lengths[1:] = np.cumsum(0.5 * (norms[:-1] + norms[1:]) * dt)
    
    # Total arc length
    total_length = arc_lengths[-1]
    
    # Fit polynomial t(s_normalized) where s_normalized = s / L ∈ [0, 1]
    s_normalized = arc_lengths / total_length
    poly_coeffs = np.polyfit(s_normalized, t_samples, poly_degree)
    
    # Get spline parameters
    spline_params = spline.get_parameters()
    
    return {
        'points': spline_params['points'],       # (N, 3)
        'tangents': spline_params['tangents'],   # (N, 3)
        'ti': spline_params['ti'],               # (N,)
        'poly_coeffs': poly_coeffs,              # (degree+1,)
        'total_length': total_length             # scalar
    }

def get_segment_from_arc_length(
    s_eval: float,
    ti: np.ndarray,
    total_length: float,
    poly_coeffs: np.ndarray
) -> Tuple[int, float, float]:
    """
    Determine which spline segment corresponds to a given arc length s.
    
    This function takes an arc length value s and returns:
    1. The segment index (which pair of control points)
    2. The parameter t corresponding to s
    3. The normalized parameter s in [0, 1]
    
    :param s_eval: Arc length value to query
    :type s_eval: float
    :param ti: Parameter values at control points (shape (N,))
    :type ti: np.ndarray
    :param total_length: Total arc length of the spline (scalar)
    :type total_length: float
    :param poly_coeffs: Polynomial coefficients for t(s_normalized) (shape (degree+1,))
    :type poly_coeffs: np.ndarray
    :return: Tuple of (segment_index, t_value, s_normalized) where:
             - segment_index: int index of the segment [0, N-2]
             - t_value: parameter t corresponding to s_eval
             - s_normalized: normalized arc length s_eval/total_length in [0, 1]
    :rtype: Tuple[int, float, float]
    
    Example:
        >>> params = compute_arc_length_reparametrization(spline)
        >>> s_eval = 0.5 * params['total_length']  # Midpoint
        >>> segment_idx, t_val, s_norm = get_segment_from_arc_length(
        ...     s_eval,
        ...     params['ti'],
        ...     params['total_length'],
        ...     params['poly_coeffs']
        ... )
        >>> print(f"Arc length {s_eval:.2f} is in segment {segment_idx} at t={t_val:.2f}")
    """
    # Normalize s to [0, 1]
    s_norm = s_eval / total_length
    
    # Clamp s_norm to [0, 1]
    s_norm = np.clip(s_norm, 0.0, 1.0)
    
    # Get number of coefficients
    n_coeffs = len(poly_coeffs)
    
    # Polynomial approximation: t(s_norm) = sum(c_i * s_norm^i)
    # poly_coeffs are from np.polyfit, so highest degree first
    t_of_s = 0.0
    for i in range(n_coeffs):
        t_of_s += poly_coeffs[i] * s_norm**(n_coeffs - 1 - i)
    
    # Clamp t to valid range
    t_min = ti[0]
    t_max = ti[-1]
    t_of_s = np.clip(t_of_s, t_min, t_max)
    
    # Find which segment contains this t value
    segment_idx = 0
    for i in range(len(ti) - 1):
        if t_of_s <= ti[i + 1]:
            segment_idx = i
            break
    else:
        segment_idx = len(ti) - 2  # Last segment
    
    return segment_idx, t_of_s, s_norm



if __name__ == "__main__":
    """Example usage and validation."""
    import matplotlib.pyplot as plt
    
    def plot_3d_trajectory(ax, positions, points):
        """
e points: List[np.ndarray]
        """
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'b-', linewidth=2, label='Spline')
        pts_array = np.array(points)
        ax.scatter(pts_array[:, 0], pts_array[:, 1], pts_array[:, 2],
                   c='red', s=100, marker='o', label='Control points')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Hermite Spline')
        ax.legend()
        ax.grid(True)
    
    def plot_position_vs_parameter(ax, t_vals, positions):
        """
        Plot X, Y, Z position components vs parameter t.
        
        :param ax: Matplotlib axes
        :param t_vals: Array of parameter values
        :type t_vals: np.ndarray
        :param positions: Array of positions (N, 3)
        :type positions: np.ndarray
        """
        ax.plot(t_vals, positions[:, 0], 'r-', label='X')
        ax.plot(t_vals, positions[:, 1], 'g-', label='Y')
        ax.plot(t_vals, positions[:, 2], 'b-', label='Z')
        ax.set_xlabel('Parameter t')
        ax.set_ylabel('Position')
        ax.set_title('Position vs Parameter t')
        ax.legend()
        ax.grid(True)
    
    def plot_velocity_components(ax, t_vals, velocities):
        """
        Plot velocity components (dX/dt, dY/dt, dZ/dt) and magnitude vs parameter t.
        
        :param ax: Matplotlib axes
        :param t_vals: Array of parameter values
        :type t_vals: np.ndarray
        :param velocities: Array of velocities (N, 3)
        :type velocities: np.ndarray
        """
        ax.plot(t_vals, velocities[:, 0], 'r-', label='dX/dt', linewidth=1.5)
        ax.plot(t_vals, velocities[:, 1], 'g-', label='dY/dt', linewidth=1.5)
        ax.plot(t_vals, velocities[:, 2], 'b-', label='dZ/dt', linewidth=1.5)
        
        # Velocity magnitude
        vel_mag = np.linalg.norm(velocities, axis=1)
        ax.plot(t_vals, vel_mag, 'm--', linewidth=2, label='||dp/dt||')
        ax.axhline(y=np.mean(vel_mag), color='k', linestyle=':', 
                   alpha=0.5, label=f'Mean: {np.mean(vel_mag):.3f}')
        
        ax.set_xlabel('Parameter t')
        ax.set_ylabel('Velocity')
        ax.set_title('Velocity Components and Magnitude\n(Non-uniform speed)')
        ax.legend()
        ax.grid(True)
    
    print("=== Creating Hermite Spline ===")
    
    # Define example waypoints (same as in spline_evaluation.py)
    points = [
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
        np.array([2.0, 0.0, 1.0]),
        np.array([4.0, 1.0, 1.0]),
    ]
    
    # Define tangent vectors at each waypoint
    tangents = [
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    ]
    
    # Create spline
    spline = HermiteSpline(points, tangents)
    print(f"Spline created with {len(points)} control points")
    print(f"Parameter range: [{spline.ti[0]}, {spline.ti[-1]}]")
    
    print("\n=== Computing Arc Length Reparametrization ===")
    
    # Compute arc length reparametrization parameters
    params = compute_arc_length_reparametrization(spline, n_samples=200, poly_degree=5)
    print(f"Total arc length: {params['total_length']:.4f}")
    print(f"Polynomial degree: {len(params['poly_coeffs']) - 1}")
    
    print("\n=== Parameters for Symbolic Evaluation ===")
    print(f"Points shape: {params['points'].shape}")
    print(f"Tangents shape: {params['tangents'].shape}")
    print(f"Ti shape: {params['ti'].shape}")
    print(f"Poly coeffs shape: {params['poly_coeffs'].shape}")
    print(f"Total length: {params['total_length']:.4f}")

    s_eval = 0.1 * params['total_length']  # Midpoint
    segment_idx, t_val, s_norm = get_segment_from_arc_length(s_eval, params['ti'],
        params['total_length'],
        params['poly_coeffs']
        )
    print(f"Arc length {s_eval:.2f} is in segment {segment_idx} at t={t_val:.2f}")
    
    # Save parameters
    np.savez('spline_params.npz', **params)
    print("\nParameters saved to 'spline_params.npz'")
    
    print("\n=== Sampling and Plotting ===")
    
    # Sample in parameter space
    t_vals = np.linspace(spline.ti[0], spline.ti[-1], 100)
    positions_t = np.array([spline.evaluate(t) for t in t_vals])
    velocities_t = np.array([spline.evaluate_derivative(t) for t in t_vals])
    
    # Plot
    fig = plt.figure(figsize=(15, 5))
    
    # 3D trajectory
    ax1 = fig.add_subplot(131, projection='3d')
    plot_3d_trajectory(ax1, positions_t, points)
    
    # Position vs parameter
    ax2 = fig.add_subplot(132)
    plot_position_vs_parameter(ax2, t_vals, positions_t)
    
    # Velocity components and magnitude
    ax3 = fig.add_subplot(133)
    plot_velocity_components(ax3, t_vals, velocities_t)
    
    plt.tight_layout()
    plt.show()
    
    print("\n=== Done! ===")
    print("\nNext step: Use these parameters with spline_evaluation.py")
    print("to create symbolic CasADi functions for Acados/MPCC")
