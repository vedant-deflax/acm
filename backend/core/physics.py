# backend/core/physics.py
"""
Orbital propagation engine using RK4 integration with J2 perturbation.
All units: km, km/s, seconds.
"""

import numpy as np
from .constants import MU, RE, J2


def j2_acceleration(r: np.ndarray) -> np.ndarray:
    """
    Compute the J2 perturbation acceleration vector in ECI frame.
    Args:
        r: position vector [x, y, z] in km
    Returns:
        acceleration vector [ax, ay, az] in km/s^2
    """
    x, y, z = r
    r_norm = np.linalg.norm(r)
    r2 = r_norm ** 2
    factor = (3.0 / 2.0) * J2 * MU * RE**2 / r_norm**5

    ax = factor * x * (5.0 * z**2 / r2 - 1.0)
    ay = factor * y * (5.0 * z**2 / r2 - 1.0)
    az = factor * z * (5.0 * z**2 / r2 - 3.0)

    return np.array([ax, ay, az])


def state_derivative(state: np.ndarray) -> np.ndarray:
    """
    Compute the time derivative of a 6D state vector [r, v].
    d/dt [r, v] = [v, a_gravity + a_J2]
    """
    r = state[:3]
    v = state[3:]

    r_norm = np.linalg.norm(r)
    a_grav = -(MU / r_norm**3) * r
    a_j2   = j2_acceleration(r)
    a_total = a_grav + a_j2

    return np.concatenate([v, a_total])


def rk4_step(state: np.ndarray, dt: float) -> np.ndarray:
    """
    Advance a state vector by dt seconds using classic 4th-order Runge-Kutta.
    Args:
        state: [x, y, z, vx, vy, vz] in km and km/s
        dt:    timestep in seconds
    Returns:
        new_state after dt
    """
    k1 = state_derivative(state)
    k2 = state_derivative(state + 0.5 * dt * k1)
    k3 = state_derivative(state + 0.5 * dt * k2)
    k4 = state_derivative(state + dt * k3)

    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def propagate(state: np.ndarray, duration_s: float, dt: float = 10.0) -> list[np.ndarray]:
    """
    Propagate a state vector forward for a given duration.
    Args:
        state:      initial [x, y, z, vx, vy, vz]
        duration_s: total propagation time in seconds
        dt:         RK4 timestep in seconds
    Returns:
        list of state vectors at each timestep (including initial)
    """
    states = [state.copy()]
    t = 0.0
    current = state.copy()

    while t < duration_s:
        step = min(dt, duration_s - t)
        current = rk4_step(current, step)
        states.append(current.copy())
        t += step

    return states


def propagate_to_time(state: np.ndarray, duration_s: float, dt: float = 10.0) -> np.ndarray:
    """Propagate and return only the final state vector."""
    current = state.copy()
    t = 0.0
    while t < duration_s:
        step = min(dt, duration_s - t)
        current = rk4_step(current, step)
        t += step
    return current


def eci_to_latlon(r_eci: np.ndarray, gst_rad: float = 0.0) -> tuple[float, float, float]:
    """
    Convert ECI position vector to geodetic lat/lon/altitude.
    Args:
        r_eci:   [x, y, z] in km (ECI)
        gst_rad: Greenwich Sidereal Time in radians (0 for simplified)
    Returns:
        (latitude_deg, longitude_deg, altitude_km)
    """
    x, y, z = r_eci
    r_norm = np.linalg.norm(r_eci)

    # Latitude (geocentric)
    lat_rad = np.arcsin(z / r_norm)

    # Longitude (rotate by GST to get ECEF-like)
    lon_rad = np.arctan2(y, x) - gst_rad
    lon_rad = (lon_rad + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-π, π]

    altitude_km = r_norm - RE

    return np.degrees(lat_rad), np.degrees(lon_rad), altitude_km


# --- RTN Frame Utilities ---

def eci_to_rtn_matrix(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Compute the rotation matrix from ECI to RTN (Radial-Transverse-Normal) frame.
    R: r̂ (radial),  T: along-track,  N: r × v (normal)
    """
    r_hat = r / np.linalg.norm(r)
    n_vec = np.cross(r, v)
    n_hat = n_vec / np.linalg.norm(n_vec)
    t_hat = np.cross(n_hat, r_hat)

    # Rows are the RTN axes expressed in ECI
    return np.array([r_hat, t_hat, n_hat])


def rtn_to_eci(delta_v_rtn: np.ndarray, r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Convert a ΔV vector from RTN frame to ECI frame.
    Args:
        delta_v_rtn: [dVr, dVt, dVn] in km/s
        r: satellite ECI position (km)
        v: satellite ECI velocity (km/s)
    Returns:
        delta_v_eci: [dvx, dvy, dvz] in km/s
    """
    M = eci_to_rtn_matrix(r, v)
    # M transforms ECI→RTN, so M^T (transpose) transforms RTN→ECI
    return M.T @ delta_v_rtn
