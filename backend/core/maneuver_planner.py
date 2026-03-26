# backend/core/maneuver_planner.py
"""
Autonomous Collision Avoidance (COLA) & Recovery Maneuver Planner.

Algorithm:
  1. Check all active conjunction warnings.
  2. For each CRITICAL event, compute an evasion ΔV in RTN frame.
  3. Convert to ECI and schedule the burn.
  4. After TCA passes, schedule a recovery burn to return to slot.
"""

import numpy as np
import logging
from datetime import datetime, timedelta, timezone
from typing import Optional

from .physics import rtn_to_eci, propagate_to_time
from .conjunction import ConjunctionEvent
from .constants import (
    MAX_DELTAV_KM_S, THRUSTER_COOLDOWN_S, SIGNAL_LATENCY_S,
    CONJUNCTION_WARNING_KM,
)

logger = logging.getLogger("acm.planner")

# Safety stand-off after evasion (km) — want at least this miss distance
TARGET_STANDOFF_KM = 2.0

# How far in transverse direction to push (km) — translates to ΔV
# For LEO ~7.5 km/s orbital velocity, 1 m/s ≈ 0.56 km lateral displacement per min
EVASION_DV_TRANSVERSE = 0.005   # km/s = 5 m/s prograde push
EVASION_DV_RADIAL     = 0.002   # km/s = 2 m/s radial push for additional separation

# Recovery burn magnitude (slightly less than evasion to account for mass change)
RECOVERY_DV_SCALE = 0.98


def compute_evasion_dv_rtn(
    sat_state: np.ndarray,
    deb_state: np.ndarray,
    miss_km: float,
) -> np.ndarray:
    """
    Compute evasion ΔV in RTN frame.
    
    Strategy: prograde burn (T direction) raises apogee, changing orbital period
    so the satellite is not at the conjunction point when debris arrives.
    A small radial component adds geometric separation.
    
    Returns: [dVr, dVt, dVn] in km/s
    """
    sat_r = sat_state[:3]
    sat_v = sat_state[3:]
    deb_r = deb_state[:3]

    # Relative position vector (debris w.r.t. satellite)
    rel = deb_r - sat_r

    # Determine sign: push away from debris in transverse direction
    # Project rel onto velocity direction
    v_hat = sat_v / (np.linalg.norm(sat_v) + 1e-12)
    rel_transverse = np.dot(rel, v_hat)

    # If debris is ahead (positive T), go retrograde; if behind, go prograde
    dv_t = -np.sign(rel_transverse) * EVASION_DV_TRANSVERSE

    # Radial: push away from debris in R direction
    r_hat = sat_r / (np.linalg.norm(sat_r) + 1e-12)
    rel_radial = np.dot(rel, r_hat)
    dv_r = -np.sign(rel_radial) * EVASION_DV_RADIAL

    # Scale based on how close the threat is — more aggressive for closer approaches
    scale = min(2.0, TARGET_STANDOFF_KM / max(miss_km, 0.01))
    dv_r = np.clip(dv_r * scale, -MAX_DELTAV_KM_S, MAX_DELTAV_KM_S)
    dv_t = np.clip(dv_t * scale, -MAX_DELTAV_KM_S, MAX_DELTAV_KM_S)

    return np.array([dv_r, dv_t, 0.0])


def compute_recovery_dv_rtn(evasion_dv_rtn: np.ndarray) -> np.ndarray:
    """
    Compute recovery burn in RTN frame.
    Simple reversal of evasion burn (scaled slightly) to return to nominal slot.
    """
    return -evasion_dv_rtn * RECOVERY_DV_SCALE


def plan_cola_maneuvers(
    sat_id: str,
    sat_state: np.ndarray,
    event: ConjunctionEvent,
    current_time: datetime,
    last_burn_time: Optional[datetime],
    mass_fuel_kg: float,
) -> Optional[list[dict]]:
    """
    Plan a full evasion + recovery maneuver sequence for a conjunction event.

    Returns a maneuver_sequence list ready for SimulationState.schedule_maneuver(),
    or None if the burn cannot be scheduled (cooldown, no fuel, etc.).
    """
    # --- Cooldown check ---
    earliest_burn = current_time + timedelta(seconds=SIGNAL_LATENCY_S)
    if last_burn_time is not None:
        cooldown_end = last_burn_time + timedelta(seconds=THRUSTER_COOLDOWN_S)
        if cooldown_end > earliest_burn:
            earliest_burn = cooldown_end

    # TCA time
    tca_time = current_time + timedelta(seconds=event.tca_seconds_from_now)

    # Burn must happen before TCA with at least 60s margin
    burn_margin_s = 60.0
    latest_evasion = tca_time - timedelta(seconds=burn_margin_s)

    if earliest_burn >= latest_evasion:
        logger.warning(
            f"[PLANNER] Cannot schedule evasion for {sat_id} vs {event.debris_id}: "
            f"no time window (TCA in {event.tca_seconds_from_now:.0f}s)"
        )
        return None

    if mass_fuel_kg <= 0.5:
        logger.warning(f"[PLANNER] {sat_id} has insufficient fuel for evasion")
        return None

    # Propagate satellite to burn time to get accurate state
    burn_time = earliest_burn
    dt_to_burn = (burn_time - current_time).total_seconds()
    sat_state_at_burn = propagate_to_time(sat_state, dt_to_burn) if dt_to_burn > 0 else sat_state.copy()
    deb_state_at_burn = propagate_to_time(
        np.array([event.deb_pos_at_tca[0], event.deb_pos_at_tca[1], event.deb_pos_at_tca[2],
                  0, 0, 0]),  # simplified — debris position at TCA used as proxy
        0
    )
    # Use current debris state (we don't have it here, use event data)
    deb_proxy = np.zeros(6)
    deb_proxy[:3] = event.deb_pos_at_tca

    # Compute evasion ΔV in RTN
    dv_rtn = compute_evasion_dv_rtn(sat_state_at_burn, deb_proxy, event.miss_distance_km)

    # Convert to ECI
    r = sat_state_at_burn[:3]
    v = sat_state_at_burn[3:]
    dv_eci = rtn_to_eci(dv_rtn, r, v)

    # Clamp magnitude
    mag = np.linalg.norm(dv_eci)
    if mag > MAX_DELTAV_KM_S:
        dv_eci = dv_eci * (MAX_DELTAV_KM_S / mag)

    # Recovery burn: after TCA + cooldown
    recovery_time = tca_time + timedelta(seconds=THRUSTER_COOLDOWN_S + 60.0)
    rec_dv_rtn = compute_recovery_dv_rtn(dv_rtn)
    # Propagate to recovery time for accurate RTN frame
    dt_to_recovery = (recovery_time - current_time).total_seconds()
    sat_at_recovery = propagate_to_time(sat_state, dt_to_recovery)
    rec_dv_eci = rtn_to_eci(rec_dv_rtn, sat_at_recovery[:3], sat_at_recovery[3:])

    rec_mag = np.linalg.norm(rec_dv_eci)
    if rec_mag > MAX_DELTAV_KM_S:
        rec_dv_eci = rec_dv_eci * (MAX_DELTAV_KM_S / rec_mag)

    burn_id_base = f"{sat_id}_{event.debris_id}_{int(current_time.timestamp())}"

    sequence = [
        {
            "burn_id": f"EVASION_{burn_id_base}",
            "burnTime": burn_time.isoformat().replace("+00:00", "Z"),
            "deltaV_vector": {
                "x": round(float(dv_eci[0]), 6),
                "y": round(float(dv_eci[1]), 6),
                "z": round(float(dv_eci[2]), 6),
            },
        },
        {
            "burn_id": f"RECOVERY_{burn_id_base}",
            "burnTime": recovery_time.isoformat().replace("+00:00", "Z"),
            "deltaV_vector": {
                "x": round(float(rec_dv_eci[0]), 6),
                "y": round(float(rec_dv_eci[1]), 6),
                "z": round(float(rec_dv_eci[2]), 6),
            },
        },
    ]

    logger.info(
        f"[PLANNER] Planned COLA for {sat_id} vs {event.debris_id} | "
        f"TCA in {event.tca_seconds_from_now:.0f}s | "
        f"Evasion ΔV={np.linalg.norm(dv_eci)*1000:.2f} m/s | "
        f"Burn at {burn_time.isoformat()}"
    )

    return sequence


def plan_eol_graveyard(
    sat_id: str,
    sat_state: np.ndarray,
    current_time: datetime,
    last_burn_time: Optional[datetime],
) -> Optional[list[dict]]:
    """
    Plan a graveyard deorbit burn for an EOL satellite.
    Retrograde burn to lower perigee below 200 km for eventual reentry.
    """
    earliest_burn = current_time + timedelta(seconds=SIGNAL_LATENCY_S)
    if last_burn_time is not None:
        cooldown_end = last_burn_time + timedelta(seconds=THRUSTER_COOLDOWN_S)
        if cooldown_end > earliest_burn:
            earliest_burn = cooldown_end

    # Retrograde burn (negative T in RTN) — use max ΔV to deorbit efficiently
    dv_rtn = np.array([0.0, -MAX_DELTAV_KM_S, 0.0])
    r = sat_state[:3]
    v = sat_state[3:]
    dv_eci = rtn_to_eci(dv_rtn, r, v)

    burn_id = f"EOL_GRAVEYARD_{sat_id}_{int(current_time.timestamp())}"

    logger.warning(f"[PLANNER] EOL graveyard burn scheduled for {sat_id}")

    return [{
        "burn_id": burn_id,
        "burnTime": earliest_burn.isoformat().replace("+00:00", "Z"),
        "deltaV_vector": {
            "x": round(float(dv_eci[0]), 6),
            "y": round(float(dv_eci[1]), 6),
            "z": round(float(dv_eci[2]), 6),
        },
    }]
