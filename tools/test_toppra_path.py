#!/usr/bin/env python3
"""Check spline reconstruction independently of the optional TOPPRA dependency."""
import unittest
import numpy as np
from toppra_retime import SavedQuinticPath


class PathTest(unittest.TestCase):
    def test_boundary_derivatives_and_midpoint(self):
        data = dict(points=[dict(time=0,positions=[0],velocities=[0],accelerations=[0]),
                            dict(time=2,positions=[1],velocities=[0],accelerations=[0])])
        path = SavedQuinticPath(data)
        np.testing.assert_allclose(path([0,1,2]).ravel(), [0,.5,1])
        np.testing.assert_allclose(path([0,2],1), [[0],[0]], atol=1e-12)
        np.testing.assert_allclose(path([0,2],2), [[0],[0]], atol=1e-12)
        eps=1e-5
        np.testing.assert_allclose(path(1,1),(path(1+eps)-path(1-eps))/(2*eps),rtol=1e-8)
        np.testing.assert_allclose(path(.7,2),(path(.7+eps,1)-path(.7-eps,1))/(2*eps),rtol=1e-8)


if __name__=='__main__': unittest.main()
