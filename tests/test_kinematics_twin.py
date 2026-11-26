"""Verification tests for Meca500 6-DOF DH kinematics and Digital Twin geometry."""

from __future__ import annotations

import math
import numpy as np


LINK_LENGTHS = {
    "base_height": 135.0,
    "upper_arm": 135.0,
    "forearm": 175.0,
    "wrist": 72.0,
    "extruder_offset": 50.0,
}


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def fk_positions(joints_rad: list[float]) -> np.ndarray:
    dh_params = [
        (joints_rad[0], LINK_LENGTHS["base_height"], 0.0, -math.pi / 2),
        (joints_rad[1] + math.pi / 2, 0.0, LINK_LENGTHS["upper_arm"], 0.0),
        (joints_rad[2], 0.0, LINK_LENGTHS["forearm"], -math.pi / 2),
        (joints_rad[3], 0.0, 0.0, math.pi / 2),
        (joints_rad[4], 0.0, 0.0, -math.pi / 2),
        (joints_rad[5], LINK_LENGTHS["wrist"], 0.0, 0.0),
    ]
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for theta, d, a, alpha in dh_params:
        T = T @ dh_matrix(theta, d, a, alpha)
        pts.append(T[:3, 3].copy())
    return np.asarray(pts)


def test_dh_matrix_is_proper_se3():
    T = dh_matrix(0.5, 135.0, 175.0, -math.pi / 2)
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-9)


def test_zero_posture_link_lengths():
    pts = fk_positions([0.0] * 6)
    # Base to J2 is 135 mm along Z
    assert math.isclose(np.linalg.norm(pts[1] - pts[0]), 135.0, abs_tol=1e-6)
    # Upper arm J2->J3 is 135 mm
    assert math.isclose(np.linalg.norm(pts[2] - pts[1]), 135.0, abs_tol=1e-6)
    # Forearm J3->J4 is 175 mm
    assert math.isclose(np.linalg.norm(pts[3] - pts[2]), 175.0, abs_tol=1e-6)
    # Wrist offset J5->J6 is 72 mm
    assert math.isclose(np.linalg.norm(pts[6] - pts[5]), 72.0, abs_tol=1e-6)


def test_workspace_bounds_within_physical_reach():
    pts = fk_positions([0.3, -0.25, 0.6, 0.1, 0.9, -0.2])
    ee = pts[-1]
    # Max radial reach from shoulder (0, 0, 135) cannot exceed L1 + L2 + wrist
    dist_from_shoulder = np.linalg.norm(ee - np.array([0.0, 0.0, 135.0]))
    assert dist_from_shoulder <= 135.0 + 175.0 + 72.0 + 1e-6
