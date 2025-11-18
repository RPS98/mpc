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
"""Spline trajectory generator."""

__authors__ = 'Carmen De Rojas Pita-Romero'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from hermite_spline import HermiteSpline, compute_arc_length_reparametrization
from typing import List, Tuple, Dict
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from spline_evaluation import evaluate_arc_length_spline


@dataclass
class Setpoint:
    """Setpoint dataclass with id, position and tangent."""
    id: int
    position: np.ndarray
    tangent: np.ndarray


class Path:
    """Path class to manage waypoints with an index."""
    
    def __init__(self, setpoints: List[Setpoint]):
        """
        Initialize path with setpoints.
        
        :param setpoints: list of Setpoint objects
        """
        self.setpoints = setpoints
        self.current_index = 0
    
    def get_waypoints(self, num: int) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        Get the next num waypoints and tangents starting from current index.
        
        :param num: number of waypoints to retrieve
        :return: tuple of (waypoints list, tangents list)
        """
        wps = []
        tgs = []
        
        for i in range(num):
            idx = self.current_index + i
            if idx >= len(self.setpoints):
                break
            wps.append(self.setpoints[idx].position)
            tgs.append(self.setpoints[idx].tangent)
        
        return wps, tgs
    
    def advance(self) -> bool:
        """
        Advance the current index by one.
        
        :return: True if advanced successfully, False if at the end
        """
        if self.current_index + 1 < len(self.setpoints):
            self.current_index += 1
            return True
        return False
    
    def get_current_setpoint(self) -> Setpoint:
        """Get the current setpoint."""
        if self.current_index >= len(self.setpoints):
            return None
        return self.setpoints[self.current_index]
    
    def __len__(self) -> int:
        """Return the total number of setpoints."""
        return len(self.setpoints)
    
    def remaining_waypoints(self) -> int:
        """Return the number of waypoints remaining from current index."""
        return len(self.setpoints) - self.current_index



class SplineTrajectoryGenerator:
    """Spline trajectory generator base class."""
        
    def __init__(
        self,
        setpoints: List[Setpoint],
        num_wp: int,
        spline_samples: int = 200,
        spline_degree: int = 5,
        plot_on_regeneration: bool = False,
    ):
        """
        :param setpoints: list of Setpoint objects
        :param num_wp: number of waypoints to use for spline generation
        :param spline_samples: number of samples for arc length reparametrization
        :param spline_degree: degree of polynomial for arc length reparametrization
        :param plot_on_regeneration: if True, plot spline every time it's regenerated
        """
        # Global parameters
        self.num_wp = num_wp
        self.spline = None
        self.spline_samples = spline_samples
        self.spline_degree = spline_degree
        self.spline_reparametrization = None
        self.plot_on_regeneration = plot_on_regeneration
        self.regeneration_count = 0

        # Create Path object to manage waypoints
        self.path = Path(setpoints)
        self.generate_spline()

    def generate_spline(self):
        # Get next num_wp from Path
        wps, tgs = self.path.get_waypoints(self.num_wp)
        self.spline = HermiteSpline(wps, tgs)
        self.spline_reparametrization = compute_arc_length_reparametrization(
            self.spline, n_samples=self.spline_samples, poly_degree=self.spline_degree)
        
        # Plot if enabled
        if self.plot_on_regeneration:
            self._plot_spline_regeneration()
    
    def _plot_spline_regeneration(self):
        """Plot the current spline in 3D and by axes."""
        self.regeneration_count += 1
        
        # Sample the spline along its arc length
        total_length = self.spline_reparametrization['total_length']
        s_values = np.linspace(0, total_length, 100)
        
        positions = []
        for s in s_values:
            pos, _, _ = self.evaluate_arc_length_spline(s)
            positions.append(pos)
        
        positions = np.array(positions)
        
        # Get current waypoints for visualization
        wps, tgs = self.path.get_waypoints(self.num_wp)
        wps = np.array(wps)
        
        # Create NEW figure with subplots (each regeneration gets its own figure)
        fig = plt.figure(num=f'Regeneration #{self.regeneration_count}', figsize=(16, 10))
        fig.suptitle(f'Spline Regeneration #{self.regeneration_count} | Arc Length: {total_length:.3f} m | Path Index: {self.path.current_index}',
                     fontsize=14, fontweight='bold')
        
        # 3D plot
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                'b-', linewidth=2, label='Spline Trajectory')
        ax1.scatter(wps[:, 0], wps[:, 1], wps[:, 2],
                   color='red', s=100, label='Waypoints', zorder=5, edgecolors='black')
        
        # Add tangent vectors at waypoints
        for i, (wp, tg) in enumerate(zip(wps, tgs)):
            ax1.quiver(wp[0], wp[1], wp[2],
                      tg[0]*0.3, tg[1]*0.3, tg[2]*0.3,
                      color='green', arrow_length_ratio=0.3, linewidth=2, alpha=0.6)
        
        # Mark start and end
        ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
                   color='lime', s=150, marker='o', label='Start', zorder=6, edgecolors='black')
        ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
                   color='orange', s=150, marker='s', label='End', zorder=6, edgecolors='black')
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.legend(loc='upper left', fontsize=8)
        ax1.set_title('3D View', fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # X-Y projection
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Trajectory')
        ax2.scatter(wps[:, 0], wps[:, 1],
                   color='red', s=100, label='Waypoints', zorder=5, edgecolors='black')
        for wp, tg in zip(wps, tgs):
            ax2.arrow(wp[0], wp[1], tg[0]*0.3, tg[1]*0.3,
                     head_width=0.15, head_length=0.15, fc='green', ec='green', alpha=0.6)
        ax2.scatter(positions[0, 0], positions[0, 1],
                   color='lime', s=150, marker='o', label='Start', zorder=6, edgecolors='black')
        ax2.scatter(positions[-1, 0], positions[-1, 1],
                   color='orange', s=150, marker='s', label='End', zorder=6, edgecolors='black')
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Y [m]')
        ax2.legend(loc='best', fontsize=8)
        ax2.set_title('X-Y Projection', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # X-Z projection
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=2, label='Trajectory')
        ax3.scatter(wps[:, 0], wps[:, 2],
                   color='red', s=100, label='Waypoints', zorder=5, edgecolors='black')
        ax3.scatter(positions[0, 0], positions[0, 2],
                   color='lime', s=150, marker='o', label='Start', zorder=6, edgecolors='black')
        ax3.scatter(positions[-1, 0], positions[-1, 2],
                   color='orange', s=150, marker='s', label='End', zorder=6, edgecolors='black')
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.legend(loc='best', fontsize=8)
        ax3.set_title('X-Z Projection', fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')
        
        # Y-Z projection
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(positions[:, 1], positions[:, 2], 'b-', linewidth=2, label='Trajectory')
        ax4.scatter(wps[:, 1], wps[:, 2],
                   color='red', s=100, label='Waypoints', zorder=5, edgecolors='black')
        ax4.scatter(positions[0, 1], positions[0, 2],
                   color='lime', s=150, marker='o', label='Start', zorder=6, edgecolors='black')
        ax4.scatter(positions[-1, 1], positions[-1, 2],
                   color='orange', s=150, marker='s', label='End', zorder=6, edgecolors='black')
        ax4.set_xlabel('Y [m]')
        ax4.set_ylabel('Z [m]')
        ax4.legend(loc='best', fontsize=8)
        ax4.set_title('Y-Z Projection', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)  # Small pause to allow the figure to render

    
    def evaluate_arc_length_spline(
        self, s: float
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Evaluate the spline at arc length s.
        
        :param s: Arc length value to evaluate (must be >= 0)
        :return: Tuple of (position, derivative, theta) where:
                 - position: np.ndarray (3,) - position at arc length s
                 - derivative: np.ndarray (3,) - tangent vector at arc length s
                 - theta: float - normalized arc length parameter [0, 1]
        """
        position, derivative, theta = evaluate_arc_length_spline(
            s,
            self.spline_reparametrization['points'],
            self.spline_reparametrization['tangents'],
            self.spline_reparametrization['ti'],
            self.spline_reparametrization['total_length'],
            self.spline_reparametrization['poly_coeffs']
        )
        return position, derivative, theta
    
    def evaluate_spline(self, s: float) -> Tuple[float, Dict]:
        """
        Evaluate the spline at arc length s and regenerate if needed.
        
        This method evaluates the spline at the given arc length s. If s has passed
        the first segment (corresponding to the first waypoint), the spline is 
        regenerated starting from the next waypoint in the path. The arc length s
        is reset to 0 when regeneration occurs.
        
        Workflow:
        1. Evaluates the spline at arc length s
        2. Checks if s has passed the first segment (segment_idx > 0)
        3. If true and more waypoints are available:
           - Advances the path index
           - Regenerates the spline with the next num_wp waypoints
           - Resets s to 0
        4. Returns the adjusted s and spline evaluation results
        
        :param s: Arc length value to evaluate (must be >= 0)
        :return: Tuple of (new_s, spline_params) where:
                 - new_s: float, adjusted arc length (0 if spline regenerated, s otherwise)
                 - spline_params: dict containing:
                   * 'position': np.ndarray (3,) - position at arc length s
                   * 'derivative': np.ndarray (3,) - tangent vector at arc length s
                   * 'reparametrization': dict - current spline reparametrization parameters
                     containing 'points', 'tangents', 'ti', 'poly_coeffs', 'total_length'
        
        Example:
            >>> spline_gen = SplineTrajectoryGenerator(setpoints, num_wp=3)
            >>> s = 0.0
            >>> for i in range(10):
            >>>     s += 0.5  # Increment arc length
            >>>     new_s, params = spline_gen.evaluate_spline(s)
            >>>     print(f"Position: {params['position']}")
            >>>     s = new_s  # Update s in case of regeneration
        """
        # Evaluate spline at current arc length s
        pos, vel, theta = self.evaluate_arc_length_spline(s)
        
        # Check if we've passed the first segment (first waypoint)
        new_s = s
        if theta >= 1.0:
            # Advance to next waypoint
            if self.path.advance():
                # Regenerate spline with new waypoints
                self.generate_spline()
                # Reset s to 0 for the new spline
                new_s = 0.0
                # Regenerate evaluation at s=0
                pos, vel, theta = self.evaluate_arc_length_spline(new_s)
            else:
                # No more waypoints available, keep current spline
                # Keep s as is (will be clamped to spline length)
                pass
        
        # Return adjusted s and spline parameters
        spline_params = {
            'position': pos,
            'derivative': vel,
            'reparametrization': self.spline_reparametrization
        }
        
        return new_s, spline_params


def main():
    """Demo: Evaluate trajectory until no waypoints remain and plot in 3D."""
    
    # Enable interactive mode for non-blocking plots
    # plt.ion()
    
    # Create list of setpoints based on gates_config.yaml (SEASON 2 TRACK)
    # Format: [x, y, z, yaw] -> tangent perpendicular to yaw angle
    # tangent = [cos(yaw), sin(yaw), 0]
    speed = 6.0  # m/s
    setpoints = [
        # gate01: [12.5, 2.0, 1.45, 3.14159]
        Setpoint(id=1, position=np.array([12.5, 2.0, 1.45]), 
                 tangent=speed * np.array([-1.0, 0.0, 0.0])),  # cos(π) = -1, sin(π) = 0
        
        # gate02: [6.5, 6.0, 1.45, 2.35619]
        Setpoint(id=2, position=np.array([6.5, 6.0, 1.45]), 
                 tangent=speed * np.array([-0.707, 0.707, 0.0])),  # cos(3π/4) ≈ -0.707, sin(3π/4) ≈ 0.707
        
        # gate03: [5.5, 14.0, 1.45, 2.0944]
        Setpoint(id=3, position=np.array([5.5, 14.0, 1.45]), 
                 tangent=speed * np.array([-0.5, 0.866, 0.0])),  # cos(2π/3) = -0.5, sin(2π/3) ≈ 0.866
        
        # gate04: [2.5, 24.0, 1.45, 1.5708]
        Setpoint(id=4, position=np.array([2.5, 24.0, 1.45]), 
                 tangent=speed * np.array([0.0, 1.0, 0.0])),  # cos(π/2) = 0, sin(π/2) = 1
        
        # gate05: [7.5, 30.0, 1.45, -0.174533]
        Setpoint(id=5, position=np.array([7.5, 30.0, 1.45]), 
                 tangent=speed * np.array([0.985, -0.174, 0.0])),  # cos(-π/18) ≈ 0.985, sin(-π/18) ≈ -0.174
        
        # gate06: [12.2, 22.0, 1.45, 0.0]
        Setpoint(id=6, position=np.array([12.2, 22.0, 1.45]), 
                 tangent=speed * np.array([1.0, 0.0, 0.0])),  # cos(0) = 1, sin(0) = 0
        
        # gate07_splitup: [17.5, 30.0, 4.15, 1.39626]
        Setpoint(id=7, position=np.array([17.5, 30.0, 4.15]), 
                 tangent=speed * np.array([0.174, 0.985, 0.0])),  # cos(1.39626) ≈ 0.174, sin(1.39626) ≈ 0.985
        
        # gate08: [18.5, 22.0, 1.45, -1.39626]
        Setpoint(id=8, position=np.array([18.5, 22.0, 1.45]), 
                 tangent=speed * np.array([0.174, -0.985, 0.0])),  # cos(-1.39626) ≈ 0.174, sin(-1.39626) ≈ -0.985
        
        # gate09: [20.5, 14.0, 1.45, -1.74533]
        Setpoint(id=9, position=np.array([20.5, 14.0, 1.45]), 
                 tangent=speed * np.array([-0.174, -0.985, 0.0])),  # cos(-π+0.4) ≈ -0.174, sin(-π+0.4) ≈ -0.985
        
        # gate10_ladderup: [18.5, 6.0, 4.15, -2.35619]
        Setpoint(id=10, position=np.array([18.5, 6.0, 4.15]), 
                 tangent=speed * np.array([-0.707, -0.707, 0.0])),  # cos(-3π/4) ≈ -0.707, sin(-3π/4) ≈ -0.707
    ]
    
    # Create spline generator with 3 waypoints per spline and plot on regeneration
    spline_gen = SplineTrajectoryGenerator(setpoints, num_wp=5, plot_on_regeneration=True)
    
    print("=== Spline Trajectory Evaluation Demo ===")
    print(f"Total waypoints: {len(setpoints)}")
    print(f"Waypoints per spline: {spline_gen.num_wp}")
    print(f"Initial spline length: {spline_gen.spline_reparametrization['total_length']:.4f}\n")
    
    # Collect trajectory by incrementing arc length
    trajectory = []
    s = 0.0
    ds = 0.01  # Arc length increment
    regeneration_count = 0
    iteration = 0
    
    # Evaluate trajectory until we reach the end
    # Continue while we haven't reached the final spline's end
    reached_end = False
    while not reached_end:
        # Evaluate spline at current arc length
        prev_s = s
        new_s, params = spline_gen.evaluate_spline(s)
        
        # Store position
        trajectory.append(params['position'])
        
        # Check if spline was regenerated
        if new_s < prev_s:
            regeneration_count += 1
            print(f"Iteration {iteration}: Spline regenerated (#{regeneration_count})")
            print(f"  - Arc length reset from {prev_s:.2f} to {new_s:.2f}")
            print(f"  - Path index: {spline_gen.path.current_index}")
            print(f"  - New spline length: {params['reparametrization']['total_length']:.4f}\n")
        
        # Update arc length
        s = new_s + ds
        
        # Check if we've reached the end (no more waypoints to regenerate and s exceeds spline length)
        if spline_gen.path.remaining_waypoints() < spline_gen.num_wp:
            remaining_length = params['reparametrization']['total_length']
            if s > remaining_length:
                print(f"Reached end of trajectory at iteration {iteration}")
                print(f"Final position: {params['position']}")
                print(f"Total trajectory points: {len(trajectory)}\n")
                reached_end = True
        
        iteration += 1
    
    trajectory = np.array(trajectory)
    
    # Create 3D visualization
    fig = plt.figure(figsize=(14, 6))
    
    # 3D plot
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
             'b-', linewidth=2, alpha=0.7, label='Evaluated Trajectory')
    
    # Plot waypoints
    ax1.scatter([sp.position[0] for sp in setpoints],
                [sp.position[1] for sp in setpoints],
                [sp.position[2] for sp in setpoints],
                color='red', s=100, label='Waypoints', zorder=5, edgecolors='black')
    
    # Add tangent vectors at waypoints
    for sp in setpoints:
        ax1.quiver(sp.position[0], sp.position[1], sp.position[2],
                   sp.tangent[0]*0.3, sp.tangent[1]*0.3, sp.tangent[2]*0.3,
                   color='green', arrow_length_ratio=0.3, linewidth=2, alpha=0.6)
    
    # Mark start and end points
    ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2],
                color='lime', s=200, marker='o', label='Start', zorder=6, edgecolors='black')
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2],
                color='orange', s=200, marker='s', label='End', zorder=6, edgecolors='black')
    
    ax1.set_xlabel('X [m]', fontsize=10)
    ax1.set_ylabel('Y [m]', fontsize=10)
    ax1.set_zlabel('Z [m]', fontsize=10)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.set_title('3D Trajectory with Spline Regeneration', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # 2D XY projection
    ax2 = fig.add_subplot(122)
    ax2.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
    ax2.scatter([sp.position[0] for sp in setpoints],
                [sp.position[1] for sp in setpoints],
                color='red', s=100, label='Waypoints', zorder=5, edgecolors='black')
    
    # Add tangent vectors in 2D
    for sp in setpoints:
        ax2.arrow(sp.position[0], sp.position[1],
                  sp.tangent[0]*0.3, sp.tangent[1]*0.3,
                  head_width=0.15, head_length=0.15, fc='green', ec='green', alpha=0.6)
    
    # Mark start and end
    ax2.scatter(trajectory[0, 0], trajectory[0, 1],
                color='lime', s=200, marker='o', label='Start', zorder=6, edgecolors='black')
    ax2.scatter(trajectory[-1, 0], trajectory[-1, 1],
                color='orange', s=200, marker='s', label='End', zorder=6, edgecolors='black')
    
    ax2.set_xlabel('X [m]', fontsize=10)
    ax2.set_ylabel('Y [m]', fontsize=10)
    ax2.legend(loc='upper left', fontsize=9)
    ax2.set_title('XY Projection', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    print("Displaying 3D trajectory plot...")
    plt.show()

if __name__ == "__main__":
    main()