import matplotlib.pyplot as plt
import numpy as np


def plotDrone3D(ax,X,q):
    
    l= 0.046 # arm length
    r = 0.02 # rotor length
    # l = 0.1
    # r = 0.04

    x = X[0]
    y = X[1]
    z = X[2]

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    R = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])

    c1 = np.array([x,y,z]) + R @ np.array([r,0,0])
    q1 = np.array([x,y,z]) + R @ np.array([l,l,0])
    q2 = np.array([x,y,z]) + R @ np.array([-l,-l,0])
    q3 = np.array([x,y,z]) + R @ np.array([l,-l,0])
    q4 = np.array([x,y,z]) + R @ np.array([-l,l,0])

    r1 = q1 + R @ np.array([0,0,r])
    r2 = q2 + R @ np.array([0,0,r])
    r3 = q3 + R @ np.array([0,0,r])
    r4 = q4 + R @ np.array([0,0,r])

    ax.plot3D([q1[0], q2[0]], [q1[1], q2[1]], [q1[2], q2[2]], 'k')
    ax.plot3D([q3[0], q4[0]], [q3[1], q4[1]], [q3[2], q4[2]], 'k')
    ax.plot3D([q1[0], r1[0]], [q1[1], r1[1]], [q1[2], r1[2]], 'r')
    ax.plot3D([q2[0], r2[0]], [q2[1], r2[1]], [q2[2], r2[2]], 'r')
    ax.plot3D([q3[0], r3[0]], [q3[1], r3[1]], [q3[2], r3[2]], 'r')
    ax.plot3D([q4[0], r4[0]], [q4[1], r4[1]], [q4[2], r4[2]], 'r')
    ax.plot3D([x, c1[0]], [y, c1[1]], [z, c1[2]], '-', color='orange', label='heading')


def axisEqual3D(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


def plotSim3D(simX, ref_traj, save=False):
    
    # extracting initial position
    # x_start = simX[0][0]
    # y_start = simX[0][1]
    # z_start = simX[0][2]

    # extracting final position
    # x_end = simX[-1][0]
    # y_end = simX[-1][1]
    # z_end = simX[-1][2]

    plt.style.use('seaborn')

    x = simX[:,0]
    y = simX[:,1]
    z = simX[:,2]

    x_ref = ref_traj[:,0]
    y_ref = ref_traj[:,1]
    z_ref = ref_traj[:,2]

    fig, ax = plt.subplots()
    plt.title('Reference trajectory')
    ax = plt.axes(projection = "3d")
    ax.plot3D(x, y, z, label='meas')
    ax.plot3D(x_ref, y_ref, z_ref, label='ref')
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    # fig, ax = plt.subplots()

    # ax.plot(simX[:,0], simX[:,1], label='traj')
    # ax.plot(ref_traj[0:Nsim,0], ref_traj[0:Nsim,1], '--', label='ref_traj')

    # plotting initial and final position
    # ax.plot(y_start,z_start,'.', color='red')
    # ax.text(y_start,z_start,'start', color='red')
    # ax.plot(y_end,z_end,'.', color = 'red')
    # ax.text(y_end,z_end,'end', color='red')
    
    ax.legend()
    ax.set_title("Performed Trajectory")
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    # ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))  # aspect ratio is 1:1:1 in data space


    NUM_STEPS = simX.shape[0]
    MEAS_EVERY_STEPS =60

    X0 = [simX[0,0], simX[0,1], simX[0,2]]
    q0 = [simX[0,3], simX[0,4], simX[0,5], simX[0,6]]
    plotDrone3D(ax,X0,q0)
    
    for step in range(NUM_STEPS):
        if step !=0 and step % MEAS_EVERY_STEPS ==0:
            X = [simX[step,0], simX[step,1], simX[step,2]]
            q = [simX[step,3], simX[step,4], simX[step,5], simX[step,6]]
            plotDrone3D(ax,X,q)

    axisEqual3D(ax)
    # ax.set_xlim3d(-5, 5)
    # ax.set_ylim3d(-5, 5)
    # ax.set_zlim3d(0, 5)

    if save == True:
        fig.savefig('figures/sim3D.png', dpi=300)


if __name__ == '__main__':
    import csv
    csv_file = 'mpc_log.csv'

    # load the data
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        data = list(reader)
    
    # extract the data
    # Row: 'time', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz', 'x_ref', 'y_ref', 'z_ref', 'qw_ref', 'qx_ref', 'qy_ref', 'qz_ref', 'vx_ref', 'vy_ref', 'vz_refthrust_ref', 'wx_ref', 'wy_ref', 'wz_ref'
    num_rows = len(data)-1
    num_cols = len(data[0])

    t = np.zeros((num_rows, 1))
    state = np.zeros((num_rows, 10))
    reference = np.zeros((num_rows, 10))
    control = np.zeros((num_rows, 4))

    for i, row in enumerate(data):
        if i == 0:
            continue
        t[i-1] = float(row[0])
        state[i-1] = [float(row[j]) for j in range(1, 11)]
        reference[i-1] = [float(row[j]) for j in range(11, 21)]
        control[i-1] = [float(row[j]) for j in range(21, 25)]

    plotSim3D(state, reference, save=False)
    plt.show()
