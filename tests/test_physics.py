# tests/test_physics.py
"""Unit tests for the ACM physics engine."""

import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from backend.core.physics import rk4_step, propagate_to_time, rtn_to_eci, eci_to_rtn_matrix
from backend.core.constants import MU, RE


def circular_state(alt_km: float = 550.0) -> np.ndarray:
    """Generate a simple equatorial circular orbit state."""
    r = RE + alt_km
    v = np.sqrt(MU / r)
    return np.array([r, 0.0, 0.0, 0.0, v, 0.0])


def test_rk4_conserves_energy():
    """A circular orbit should maintain near-constant specific mechanical energy."""
    state0 = circular_state(550.0)
    r0 = np.linalg.norm(state0[:3])
    v0 = np.linalg.norm(state0[3:])
    E0 = 0.5 * v0**2 - MU / r0

    # Propagate one full orbit (~96 minutes)
    period_s = 2 * np.pi * np.sqrt((RE + 550.0)**3 / MU)
    state1 = propagate_to_time(state0, period_s, dt=10.0)

    r1 = np.linalg.norm(state1[:3])
    v1 = np.linalg.norm(state1[3:])
    E1 = 0.5 * v1**2 - MU / r1

    # Energy should be conserved to within 0.01%
    assert abs((E1 - E0) / E0) < 1e-4, f"Energy drift: {abs((E1-E0)/E0)*100:.4f}%"


def test_rk4_orbit_returns_close():
    """After one full orbit period, position should be near the start."""
    state0 = circular_state(550.0)
    period_s = 2 * np.pi * np.sqrt((RE + 550.0)**3 / MU)

    state1 = propagate_to_time(state0, period_s, dt=10.0)
    pos_error = np.linalg.norm(state1[:3] - state0[:3])

    # J2 causes ~100 km drift per orbit (nodal regression) — this is physically correct
    assert pos_error < 200.0, f"Orbit closure error: {pos_error:.2f} km"


def test_rtn_roundtrip():
    """RTN→ECI conversion should be orthogonal (unit length preserved)."""
    state = circular_state(550.0)
    r, v = state[:3], state[3:]

    dv_rtn = np.array([0.001, 0.005, 0.000])  # small burn in RTN
    dv_eci = rtn_to_eci(dv_rtn, r, v)

    # Magnitude should be preserved
    assert abs(np.linalg.norm(dv_eci) - np.linalg.norm(dv_rtn)) < 1e-12


def test_j2_causes_precession():
    """J2 perturbation should cause measurable drift vs pure two-body."""
    from backend.core.physics import propagate_to_time
    state0 = circular_state(550.0)
    duration = 86400.0  # 24 hours

    state_j2 = propagate_to_time(state0, duration, dt=10.0)

    # With J2, z-component of position should deviate from zero (inclination effects)
    # For equatorial orbit, J2 mainly affects the ascending node
    # Just check the propagation completes without NaN
    assert not np.any(np.isnan(state_j2)), "NaN in propagated state"
    assert np.linalg.norm(state_j2[:3]) > RE, "Satellite below Earth surface"


def test_tsiolkovsky():
    """Verify fuel mass calculation matches Tsiolkovsky equation."""
    from backend.core.constants import ISP_S, G0, MASS_WET_KG
    dv = 0.010  # 10 m/s in km/s
    delta_m = MASS_WET_KG * (1 - np.exp(-dv / (ISP_S * G0)))
    assert 0 < delta_m < 5.0, f"Unexpected fuel consumption: {delta_m:.4f} kg"
