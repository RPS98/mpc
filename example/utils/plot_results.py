# Copyright https://github.com/enhatem/quadrotor_mpc_acados
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import matplotlib.pyplot as plt
import numpy as np


def plotDrone3D(ax, X, q):
    """
    Plot a 3D representation of a drone's position and orientation.

    :param ax: Matplotlib 3D axis object.
    :param X: Position [x, y, z].
    :param q: Quaternion [qw, qx, qy, qz].
    """
    length = 0.046  # arm length
    r = 0.02   # rotor length
    heading_length = 0.2

    x, y, z = X
    qw, qx, qy, qz = q

    # Rotation matrix based on quaternion
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

    # Position of rotors and the center of the body
    c1 = np.array([x, y, z]) + R @ np.array([heading_length, 0, 0])
    q1 = np.array([x, y, z]) + R @ np.array([length, length, 0])
    q2 = np.array([x, y, z]) + R @ np.array([-length, -length, 0])
    q3 = np.array([x, y, z]) + R @ np.array([length, -length, 0])
    q4 = np.array([x, y, z]) + R @ np.array([-length, length, 0])

    # Rotor end points
    r1, r2, r3, r4 = [q + R @ np.array([0, 0, r]) for q in [q1, q2, q3, q4]]

    # Plot drone structure
    ax.plot3D([q1[0], q2[0]], [q1[1], q2[1]], [q1[2], q2[2]], 'k')
    ax.plot3D([q3[0], q4[0]], [q3[1], q4[1]], [q3[2], q4[2]], 'k')
    for q, r in zip([q1, q2, q3, q4], [r1, r2, r3, r4]):
        ax.plot3D([q[0], r[0]], [q[1], r[1]], [q[2], r[2]], 'r')
    ax.plot3D([x, c1[0]], [y, c1[1]], [z, c1[2]], '-', color='green', label='heading')


def axisEqual3D(ax):
    """
    Set equal scaling for 3D axis.

    :param ax: Matplotlib 3D axis object.
    """
    extents = np.array([getattr(ax, f'get_{dim}lim')() for dim in 'xyz'])
    sz = extents[:, 1] - extents[:, 0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize / 2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, f'set_{dim}lim')(ctr - r, ctr + r)


def plotSim3D(simX, ref_traj):
    """
    Plot the simulation and reference trajectory in 3D, along with drone snapshots.

    :param simX: Simulation states (position and quaternion).
    :param ref_traj: Reference trajectory states.
    """
    ax = plt.axes(projection='3d')
    x, y, z = simX[:, 0], simX[:, 1], simX[:, 2]
    x_ref, y_ref, z_ref = ref_traj[:, 0], ref_traj[:, 1], ref_traj[:, 2]

    ax.plot3D(x, y, z, label='meas')
    ax.plot3D(x_ref, y_ref, z_ref, label='ref')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title('Performed Trajectory')
    ax.legend()

    NUM_STEPS = simX.shape[0]
    MEAS_EVERY_STEPS = 60

    # Plot initial drone position
    X0 = simX[0, :3]
    q0 = simX[0, 3:7]
    plotDrone3D(ax, X0, q0)

    # Plot drone positions at intervals
    for step in range(NUM_STEPS):
        if step != 0 and step % MEAS_EVERY_STEPS == 0:
            X = simX[step, :3]
            q = simX[step, 3:7]
            plotDrone3D(ax, X, q)

    axisEqual3D(ax)
    plt.show()
