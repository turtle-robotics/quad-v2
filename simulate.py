from cProfile import label
from numpy import *
from numpy import array as a
from numpy.linalg import norm
from yaml import safe_load, dump
from matplotlib import pyplot as plt
import modern_robotics as mr
import argparse

lift_dur = 0.5
lift_height = 0.05
travel = 0.05
vr = 2*travel/lift_dur
duty_cycle = 20e-3


def trot(v, p0, state, h=duty_cycle):
    p = p0.copy()
    if state == 1:
        dp = a([v[0], v[1], 0])*h
        p += dp
        if p[0:2] @ p[0:2] > travel**2:
            p = p0.copy()
            state = 2
    elif state == 2:
        vhat = -v/norm(v)
        dp = a([vr*vhat[0], vr*vhat[1], 0])*h
        p += dp
        p[2] = lift_height*cos(0.5*pi*norm(p[0:2])/travel)
        if p[0:2] @ p[0:2] > travel**2:
            p = p0.copy()
            p[2] = 0.0
            state = 1
    return p, state


def gait_gen(v, total_time=2.0, h=duty_cycle):
    t = arange(0, total_time, h)
    gait = []
    for leg in range(1, 5):
        state = 1
        p = [a([0.0, 0.0, 0.0])]
        for dt in diff(t):
            p_new, state = trot(v, p[-1], state, dt)
            p.append(p_new)
        gait.append(a(p).T)
    return gait, t


def visualize(Tlist, rf):
    fig = plt.figure(figsize=(10, 10))
    ax1 = fig.add_subplot(222, projection='3d')
    ax2 = fig.add_subplot(221)
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224)
    for Tlistleg, color in zip(Tlist, 'cmyk'):
        for i, T_ in enumerate(Tlistleg):
            for ax, T in zip([ax1, ax2, ax3, ax4], [T_[0:3], T_[0:2], T_[[0, 2]], T_[1:3]]):
                ax.quiver(*T[:, 3], *T[:, 0]*rf, color='r')
                ax.quiver(*T[:, 3], *T[:, 1]*rf, color='g')
                ax.quiver(*T[:, 3], *T[:, 2]*rf, color='b')
                ax.scatter(*T[:, 3], color=color)
                ax.text(*T[:, 3], f'{i}')
    for ax, labels, title in zip([ax2, ax3, ax4], [('X', 'Y'), ('X', 'Z'), ('Y', 'Z')], ['Top', 'Right', 'Front']):
        ax.set_title(title)
        ax.set_xlabel(labels[0])
        ax.set_ylabel(labels[1])
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        if labels[0] in 'YZ':
            ax.invert_xaxis()
        if labels[1] in 'YZ':
            ax.invert_yaxis()
    ax1.set_title('3D View')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-1, 1)
    ax1.set_ylim(-1, 1)
    ax1.set_zlim(-1, 1)
    ax1.view_init(elev=20., azim=30)
    ax1.invert_yaxis()
    ax1.invert_zaxis()
    plt.show()


def simulate(config_file):
    with open(config_file, 'r') as file:
        config = safe_load(file)
    name = config['name']
    l0x = 0.5*config['chassis']['length']
    l0y = 0.5*config['chassis']['width']
    l1 = config['leg']['lengths']['shoulder']
    l2 = config['leg']['lengths']['upper']
    l3 = config['leg']['lengths']['lower']
    rf = config['leg']['lengths']['foot']
    m0 = config['chassis']['mass']
    m1 = config['leg']['mass']['shoulder']
    m2 = config['leg']['mass']['upper']
    m3 = config['leg']['mass']['lower']
    m4 = config['leg']['mass']['foot']

    legs = {}

    Tlist = [[] for _ in range(4)]
    for leg in range(1, 5):
        mx = 1 if leg in [1, 2] else -1
        my = 1 if leg in [1, 3] else -1
        # q1 = a([l0x*mx, l0y*my, 0])
        q1 = a([0, -my*l1, 0])
        s1 = a([mx, 0, 0])
        q2 = a([l2, 0, 0])
        s2 = a([0, my, 0])
        q3 = a([-l3, 0, 0])
        s3 = a([0, my, 0])
        h = a([0, 0, 0])
        S1 = mr.ScrewToAxis(q1, s1, h)
        S2 = mr.ScrewToAxis(q2, s2, h)
        S3 = mr.ScrewToAxis(q3, s3, h)
        M = a([[1, 0, 0, mx*(l3-l2+l0x)],
               [0, 1, 0, my*(l1+l0y)],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])
        Slist = a([S1, S2, S3]).T
        theta_rest = a([-mx*0, my*pi/4, -my*pi/2])

        for theta in linspace(theta_rest, a([0, 0, 0]), 5):
            T = mr.FKinBody(M, Slist, theta)
            Tlist[leg-1].append(T)
        legs[leg] = {'Slist': Slist, 'M': M}
    Tlist = array(Tlist)
    visualize(Tlist, rf)

    # gait, t = gait_gen(a([0.08, 0]))
    # fig = plt.figure()
    # ax0 = fig.add_subplot(311)
    # ax0.plot(t, gait[0, :])
    # ax0.set_ylim(-travel, travel)
    # ax1 = fig.add_subplot(312)
    # ax1.plot(t, gait[1, :])
    # ax1.set_ylim(-travel, travel)
    # ax2 = fig.add_subplot(313)
    # ax2.plot(t, gait[2, :])
    # ax2.set_ylim(-lift_height, lift_height)
    # plt.show()

    # Placeholder for simulation logic
    # results = {"status": "Simulation completed", "config_used": config}
    # return results


def play_simulation(config_file):
    results = simulate(config_file)
    # Placeholder for visualization logic
    print("Playing simulation with results:", results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='TUTRTLE QUAD Dynamics Simulation')
    parser.add_argument('--config', type=str, default='V2.yaml',
                        help='Path to the configuration file')
    args = parser.parse_args()
    results = simulate(args.config)


def __getattr__(name: str): ...  # incomplete module
