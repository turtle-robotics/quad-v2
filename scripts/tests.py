import modern_robotics as mr
import quad_functions as quad

import numpy as np


def test_fk(Slist, Blist, Mlist, M, Glist, llist):
    thetalist = np.random.rand(3)*2*np.pi-np.pi

    p1 = quad.fk(thetalist, llist)
    p2 = mr.FKinSpace(M, Slist, thetalist)[0:3, 3]

    assert np.all(np.isclose(p1, p2)), f"QUAD: {p1}, MR: {p2}"


def test_ik(Slist, Blist, Mlist, M, Glist, llist):
    p = np.r_[np.random.normal(size=2), np.random.uniform(-1, 0)]
    p /= np.linalg.norm(p)
    p *= np.random.uniform(0.001, np.sqrt(llist[0]**2+llist[1:3].sum()**2))
    p[2] -= llist[-1]

    print(p)

    thetalist1 = quad.ik(p, llist)
    T1 = mr.FKinSpace(M, Slist, thetalist1)
    thealist2, converged = mr.IKinSpace(Slist, M, T1, [0, 0, 0], np.inf, 1e-6)
    assert converged, "MR IK did not converge"
    assert np.all(np.isclose(thetalist1, thealist2, 1e-4)
                  ), f"QUAD: {thetalist1}, MR: {thealist2}"


def test_J(Slist, Blist, Mlist, M, Glist, llist):
    thetalist = np.random.normal(loc=-np.pi, scale=2*np.pi, size=3)

    J2 = mr.JacobianSpace(Slist, thetalist)


def test_Jinv(Slist, Blist, Mlist, M, Glist, llist):
    pass
