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

from hermite_spline import HermiteSpline, compute_arc_length_reparametrization, get_segment_from_arc_length
from typing import List, Tuple, Dict
import numpy as np
import matplotlib.pyplot as plt

class SplineTrajectoryGenerator:
    """Spline trajectory generator base class."""
        
    def __init__(
        self,
        wp: list[str, np.ndarray, np.ndarray],
        origin: np.ndarray,
    ):
        # Global parameters
        self.waypoints = wp # id, position, tg
        self.drone_pose = origin # position, tg
        self.theta = None
        self.spline_reparametrization = None
        self.theta_intervals = None
        self.first_wp=True

    def first_spline(self):
        wps = [self.drone_pose['position']] + [wp['position'] for wp in self.waypoints[:3]]
        tgs = [self.drone_pose['tg']] + [wp['tg'] for wp in self.waypoints[:3]]
        self.generate_path(wps, tgs)

    def generate_path(self, point: list[np.ndarray],tg: list[np.ndarray]):
        print(f"Generating spline with points: {point} and tangents: {tg}\n")
        spline = HermiteSpline(point, tg)
        self.spline_reparametrization = compute_arc_length_reparametrization(spline, n_samples=200, poly_degree=5)
    

    
    def check_status_trajectory(self, s_eval,)-> bool:
        segment_idx, t_val, s_norm = get_segment_from_arc_length(s_eval, self.spline_reparametrization['ti'],
            self.spline_reparametrization['total_length'],
            self.spline_reparametrization['poly_coeffs']
            )
        print(f"Segment idx: {segment_idx}, t_val: {t_val}, s_norm: {s_norm}\n")
        if segment_idx !=0:
            print('Regenerating spline...\n')
            wps = [wp['position'] for wp in self.waypoints[:4]]
            tgs = [wp['tg'] for wp in self.waypoints[:4]]
            self.generate_path(wps, tgs)
            if self.first_wp:
                self.first_wp=False
                return True
            first_wp = self.waypoints.pop(0) 
            self.waypoints.append(first_wp)

            return True
        return False

def main():
    drone_pose = {
        'position': np.array([0.0, 0.0, 0.0]),
        'tg': np.array([1.0, 0.0, 0.0])  
    }

    waypoints = [
        {'id': 'wp0', 'position': [0.0, 0.0, 0.5], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp1', 'position': [1.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp2', 'position': [2.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp3', 'position': [3.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp4', 'position': [4.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp5', 'position': [5.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp6', 'position': [6.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp7', 'position': [7.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp8', 'position': [8.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
        {'id': 'wp9', 'position': [9.0, 0.0, 1.0], 'tg': [1.0, 0.0, 0.0]},
    ]

    spline_generator = SplineTrajectoryGenerator(waypoints, drone_pose)
    
    spline_generator.first_spline()

    trajectory_points = [drone_pose['position']]
    loop_count = 0
    s_eval = 0.0
    loop_count_wp = 0
    
    while True:
        s_eval = s_eval + spline_generator.spline_reparametrization['total_length'] * 0.1
        print(f"Evaluating s_eval: {s_eval}\n")
        print(f"arc_lenght: {spline_generator.spline_reparametrization['total_length']}\n")
        if spline_generator.check_status_trajectory(s_eval):
            s_eval = 0.0
            trajectory_points.append(spline_generator.waypoints[0]['position']) 
            loop_count_wp += 1
        loop_count += 1
        if loop_count > 200 or loop_count_wp >= 14:  
            break
    trajectory_points = np.array(trajectory_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], label='Trajectory')
    ax.scatter(drone_pose['position'][0], drone_pose['position'][1], drone_pose['position'][2], color='red', label='Initial Position')
    ax.scatter(
        [wp['position'][0] for wp in waypoints],
        [wp['position'][1] for wp in waypoints],
        [wp['position'][2] for wp in waypoints],
        color='green', label='Waypoints'
    )
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('SplineTrajectoryGenerator')
    plt.show()

if __name__ == "__main__":
    main()