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
        wp: dict,
        origin: dict,
        num_wp: int,
        cyclic: bool,
    ):
        """
        :param wp: dict of waypoints and tangents
        :param origin: dict position and tangent of the drone"""
        # Global parameters

        self.drone_pose = origin 
        self.num_wp = num_wp
        self.cyclic =cyclic
        self.theta = None
        self.spline_reparametrization = None
        self.theta_intervals = None

        # Add key to waypoints dict
        self.waypoints = {
            f'wp_{i}': {'waypoints': pos, 'waypoints_tg': tg}
            for i, (pos, tg) in enumerate(zip(wp['waypoints'], wp['waypoints_tg']))
        }
        
    def first_spline(self):

        """Generate the first spline trajectory."""

        wps = [self.drone_pose['waypoints']] + [self.waypoints[f'wp_{i}']['waypoints'] for i in range(self.num_wp-1 )]
        tgs = [self.drone_pose['waypoints_tg']] + [self.waypoints[f'wp_{i}']['waypoints_tg'] for i in range(self.num_wp -1)]
        self.generate_spline(wps, tgs)

    def generate_spline(self, point: list[np.ndarray],tg: list[np.ndarray]):

        """Generate spline trajectory from waypoints and tangents."""

        # print(f"Generating spline with points: {point} and tangents: {tg}\n")
        spline = HermiteSpline(point, tg)
        self.spline_reparametrization = compute_arc_length_reparametrization(spline, n_samples=200, poly_degree=5)
        print(f"Spline generated with total length: {self.spline_reparametrization['total_length']}\n")
        print(f"Spline generated with total length: {self.spline_reparametrization['ti']}\n")
    
    def check_status_trajectory(self, s_eval,)-> bool:

        """Check the status of the trajectory and regenerate if needed."""

        segment_idx, t_val, s_norm = get_segment_from_arc_length(s_eval, self.spline_reparametrization['ti'],
            self.spline_reparametrization['total_length'],
            self.spline_reparametrization['poly_coeffs']
            )
        if segment_idx !=0:
            print(f"Segment idx: {segment_idx}, t_val: {t_val}, s_norm: {s_norm}\n")
            print('Regenerating spline...\n')
            wps = [data['waypoints'] for data in list(self.waypoints.values())[:self.num_wp ]]
            tgs = [data['waypoints_tg'] for data in list(self.waypoints.values())[:self.num_wp ]]
            if not self.cyclic:
                if len(self.waypoints) < self.num_wp:
                    return False
            self.generate_spline(wps, tgs)

            # Remove the first waypoint and add it to the end
            first_wp_key, first_wp_value = list(self.waypoints.items())[0]
            self.waypoints.pop(first_wp_key)

            if  self.cyclic:
                self.waypoints[first_wp_key] = first_wp_value
                return True
            return True
        return False

def main():
    drone_pose = {
        'waypoints': np.array([0.0, 0.0, 0.0]),
        'waypoints_tg': np.array([1.0, 0.0, 0.0])  
    }

    waypoints = {
        'waypoints': [
            [0.0, 0.0, 0.5],
            [1.0, 0.0, 1.0],
            [2.0, 0.0, 1.0],
            [3.0, 0.0, 1.0],
            [4.0, 0.0, 1.0],
            [5.0, 0.0, 1.0],
            [6.0, 0.0, 1.0],
            [7.0, 0.0, 1.0],
            [8.0, 0.0, 1.0],
            [9.0, 0.0, 1.0],
        ],
        'waypoints_tg': [
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ]
    }

    spline_generator = SplineTrajectoryGenerator(waypoints, drone_pose,3,True)
    spline_generator.first_spline()

    trajectory_points = [drone_pose['waypoints']]
    loop_count = 0
    s_eval = 0.0
    loop_count_wp = 0
    while True:
        s_eval = s_eval + spline_generator.spline_reparametrization['total_length'] * 0.1
        if spline_generator.check_status_trajectory(s_eval):
            s_eval = 0.0
            first_wp_key = list(spline_generator.waypoints.keys())[0]
            trajectory_points.append(spline_generator.waypoints[first_wp_key]['waypoints'])
            loop_count_wp += 1
        loop_count += 1
        if loop_count > 100 or loop_count_wp >= 14:  
            break
    trajectory_points = np.array(trajectory_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], label='Trajectory')
    ax.scatter(drone_pose['waypoints'][0], drone_pose['waypoints'][1], drone_pose['waypoints'][2], color='red', label='Initial Position')
    ax.scatter( 
    [wp[0] for wp in waypoints['waypoints']],
    [wp[1] for wp in waypoints['waypoints']],
    [wp[2] for wp in waypoints['waypoints']],
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