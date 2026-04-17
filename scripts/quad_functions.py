import modern_robotics as mr
import numpy as np
from scipy.linalg import block_diag
from yaml import YAMLObject


def parse_yaml(config):
    l, Glist = [], []

    for leg in config["joints"].values():
        l += [leg['len']]
        Glist += [np.diag(leg['inertia']+3*[leg['mass']])]

    l, Glist = np.array(l), np.array(Glist)

    l1, l2, l3, l4 = l
    m0 = config["chassis"]["mass"]
    # lx0 = config["chassis"]["length"]
    # ly0 = config["chassis"]["width"]
    # f_offset = np.array([0, 0, l4])

    xd = [1, 1, -1, -1]
    yd = [1, -1, 1, -1]

    def leg(n):
        n -= 1
        h = 0
        s1 = np.array([xd[n], 0, 0])
        s2 = np.array([0, yd[n], 0])
        s3 = np.array([0, yd[n], 0])
        q1 = np.array([0, 0, 0])
        q2 = np.array([0, yd[n]*l1, 0])
        q3 = np.array([-l2, yd[n]*l1, 0])
        Slist = np.array([mr.ScrewToAxis(q1, s1, h), mr.ScrewToAxis(
            q2, s2, h), mr.ScrewToAxis(q3, s3, h)]).T

        p01 = np.array([0, 0, 0])
        p12 = np.array([0, yd[n]*l1, 0])
        p23 = np.array([-l2, 0, 0])
        p34 = np.array([l3, 0, 0])
        R = np.identity(3)
        Mlist = np.array([mr.RpToTrans(R, p01), mr.RpToTrans(R, p12), mr.RpToTrans(
            R, p23), mr.RpToTrans(R, p34)])
        M = mr.RpToTrans(R, p01+p12+p23+p34)
        # Mlistinv = np.array([mr.TransInv(M), Mlist[-1], Mlist[-2], Mlist[-3]])

        Blist = mr.Adjoint(mr.TransInv(M)) @ Slist

        return Slist, Blist, Mlist, M, Glist, l

    return leg, m0


def fk(theta, l):
    l1, l2, l3, rf = l
    th1 = theta[0]
    th2 = theta[1]
    th3 = -(theta[1]+theta[2])
    h = l2*np.sin(th2)+l3*np.sin(th3)
    s1 = np.sin(th1)
    c1 = np.cos(th1)
    p = np.array([
        -l2 * np.cos(th2) + l3 * np.cos(th3),
        l1 * c1 - h * s1,
        l1 * s1 + h * c1  # + rf
    ])
    return p


def ik(p, l):
    l1, l2, l3, rf = l
    x = p[0]
    y = p[1]
    z = p[2]  # -rf
    th1 = np.arctan2(z, y)-np.arccos(l1/np.sqrt(y**2+z**2))
    z = np.sqrt(z**2+y**2-l1**2)
    d2 = x**2+z**2
    th2 = np.arctan2(z, x)-np.arccos((l2**2+d2-l3**2)/(2*l2*np.sqrt(d2)))
    th3 = -np.arccos((l2**2+l3**2-d2)/(2*l2*l3))
    return np.array([th1, th2, th3])


def p(phi, xo, zo, xlim, zlim):
    return np.c_[
        xo-np.where(np.pi/2 < phi, np.where(phi < 3*np.pi/2, max(xlim),
                                            -min(xlim)), -min(xlim))*np.cos(phi),
        0*phi,
        zo+np.where(phi > np.pi, max(zlim), min(zlim))*np.sin(phi)
    ].T


def IKinSpace3D(Slist, M, p, thetalist0, ev):
    """Computes inverse kinematics in the space frame for an open chain robot

    :param Slist: The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value np.where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Slist = np.array([[0, 0,  1,  4, 0,    0],
                          [0, 0,  0,  0, 1,    0],
                          [0, 0, -1, -6, 0, -0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[1, 0, 0,     -5],
                      [0, 1, 0,      4],
                      [0, 0, 1, 1.6858],
                      [0, 0, 0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        ev = 0.001
    Output:
        (np.array([ 1.57073783,  2.99966384,  3.1415342 ]), True)
    TODO: Check ^
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    print(mr.FKinSpace(M, Slist, thetalist))
    psb = mr.FKinSpace(M, Slist, thetalist)[0:3, 3].flatten()
    vs = p-psb
    err = np.linalg.norm(vs) > ev
    while err and i < maxiterations:
        # print(np.linalg.pinv(mr.JacobianSpace(Slist, thetalist)))
        thetalist += np.linalg.pinv(mr.JacobianSpace(Slist,
                                    thetalist)) @ np.r_[np.array([0.0, 0.0, 0.0]), vs]
        i += 1
        psb = mr.FKinSpace(M, Slist, thetalist)[0:3, 3].flatten()
        print(psb, vs)
        vs = p-psb
        err = np.linalg.norm(vs) > ev
    return (thetalist, not err)


def NormalizedVirtualPower(Slist, F):
    """Computes the normalized virtual power K from a list of normalized screw coordinates Slist and a normalized destabilizing wrench F

    :param Slist: The normalized screw coordinates of S_ij along the perimeter of a robot
    :param F: The normalized destabilizing wrench
    :return K: The normalized virtual power

    Example Input:
        S_ij = np.array([-1, 0, 0, 0, 0, 0.5])
        F = np.array([0, 0, -1, 0.15, 0.1, 0])
    Output:
        K = 0.35
    """
    # Interchange Linear and Angular (ray-order)
    Delta = np.eye(6, k=3) + np.eye(6, k=-3)
    return Slist.T @ Delta @ F


def ScrewFromPoints(p1, p2, h=0, normalize=True):
    """Computes normalized screw coordinates of a line between two points

    :param p1: 3 vector of the first point
    :param p2: 3 vector of the second point
    :param h: screw pitch
    :param normalize: return normalized screw coordinates
    :return S: 6-vector screw

    Example Input:
        p1 = np.array([1, 2, 3])
        p2 = np.array([1, 3, 5])
        h = 0
        normalize = True
    Output:
        S = np.array([0, 0.4472136, 0.89442719, 0.4472136, -0.89442719, 0.4472136 ])
    """
    S = mr.ScrewToAxis(p1, p2-p1, h)
    if not normalize:
        return S
    if np.all(np.isclose(S[0:3], 0)):
        return S/np.linalg.norm(S[3:6])
    return S/np.linalg.norm(S[0:3])


if __name__ == '__main__':
    a = 0.5
    b = 0.25
    d_23 = 0.15
    d_34 = 0.65
    S_12 = np.array([-1, 0, 0, 0, 0, a])
    S_23 = np.array([0, -1, 0, 0, 0, b])
    S_34 = np.array([1, 0, 0, 0, 0, a])
    S_41 = np.array([0, 1, 0, 0, 0, b])
    F = -np.array([0, 0, -1, a - d_34, d_23 - b, 0])
    Slist = np.array([S_12, S_23, S_34, S_41]).T
    K_12 = NormalizedVirtualPower(S_12, F)
    K_23 = NormalizedVirtualPower(S_23, F)
    K_34 = NormalizedVirtualPower(S_34, F)
    K_41 = NormalizedVirtualPower(S_41, F)
    assert K_12 == 2*a-d_34
    assert K_23 == d_23
    assert K_34 == d_34
    assert K_41 == 2*b-d_23
    print(K_12)
    print(K_23)
    print(K_34)
    print(K_41)

    K = NormalizedVirtualPower(Slist, F)
    print(K, K.min())

    p1 = np.array([1, 0, 6])
    p2 = np.array([2, 4, 0])
    assert np.all(ScrewFromPoints(p1, p2, normalize=True) ==
                  mr.ScrewToAxis(p1, p2-p1, 0)/np.linalg.norm(p2-p1))

    print(mr.ScrewToAxis(p1, p2-p1, 0)/np.linalg.norm(p2-p1))
