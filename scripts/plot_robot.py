from matplotlib import pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def plot_init():
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1.5, 0.5)


def draw_S(M, S):
    ax.quiver(*M[0:3, 3].T, *S[0:3], length=0.05, normalize=True, color='C0')
    return M[0:3, 3].T


def draw_M(Mp, Mc):
    print(np.c_[Mp[0:3, 3].T, Mc[0:3, 3].T])
    ax.plot(*np.c_[Mp[0:3, 3].T, Mc[0:3, 3].T], color='C1')
    return np.c_[Mp[0:3, 3].T, Mc[0:3, 3].T]


def plot_robot(Mlist, Glist, Slist):
    Mp = np.identity(4)
    Mc = Mlist[0]
    # points = []
    # print(Slist.shape[-1])
    for i in range(len(Slist.T)):
        # print(i)
        Mp = Mc.copy()
        Mc @= Mlist[i+1]
        print(draw_S(Mc, Slist[:,i].T).copy())
        print(draw_M(Mp, Mc).T)
    # points = np.array(points)
    # print(points[:, 0], points[:, 1], points[:, 2])
    # ax.set_box_aspect((np.ptp(points[:, 0]), np.ptp(
    #     points[:, 1]), np.ptp(points[:, 2])))
    # ax.quiver(1, 2, 3, 1, 1, 1, scale=0.1, normalize=True, color='C0')
    fig.show()
