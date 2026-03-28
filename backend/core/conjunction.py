# backend/core/conjunction.py
"""
Conjunction Assessment (CA) engine.
Uses KD-Tree spatial indexing to avoid O(N^2) brute-force checks.
"""

import numpy as np
from scipy.spatial import KDTree
from dataclasses import dataclass, field
from typing import Optional
# backend/core/conjunction.py
from .physics import rk4_step, propagate_to_time  # <--- Add propagate_to_time here
from .constants import (
    CONJUNCTION_THRESHOLD_KM,
    CONJUNCTION_WARNING_KM,
    KDTREE_COARSE_RADIUS_KM,
    PROPAGATION_HORIZON_S,
    DEFAULT_DT_S,
)


@dataclass
class ConjunctionEvent:
    satellite_id: str
    debris_id: str
    tca_seconds_from_now: float       # Time of Closest Approach (s)
    miss_distance_km: float
    risk_level: str                    # "CRITICAL", "WARNING", "SAFE"
    sat_pos_at_tca: np.ndarray = field(default_factory=lambda: np.zeros(3))
    deb_pos_at_tca: np.ndarray = field(default_factory=lambda: np.zeros(3))


def _classify_risk(miss_km: float) -> str:
    if miss_km < CONJUNCTION_THRESHOLD_KM:
        return "CRITICAL"
    elif miss_km < 1.0:
        return "CRITICAL"
    elif miss_km < CONJUNCTION_WARNING_KM:
        return "WARNING"
    return "SAFE"


def find_tca(
    sat_state: np.ndarray,
    deb_state: np.ndarray,
    horizon_s: float = PROPAGATION_HORIZON_S,
    dt: float = DEFAULT_DT_S,
) -> tuple[float, float, np.ndarray, np.ndarray]:
    sat = sat_state.copy()
    deb = deb_state.copy()

    min_dist = np.inf
    min_t = 0.0

    # 1. Coarse Search (The "Global" Scan)
    t = 0.0
    while t <= horizon_s:
        dist = np.linalg.norm(sat[:3] - deb[:3])
        if dist < min_dist:
            min_dist = dist
            min_t = t
        
        sat = rk4_step(sat, dt)
        deb = rk4_step(deb, dt)
        t += dt

    # 2. Bisection Refinement (The "Zoom")
    # We look in the window [min_t - dt, min_t + dt]
    t_start = max(0, min_t - dt)
    t_end = min(horizon_s, min_t + dt)
    
    # Simple Golden Section Search or Bisection on the interval
    # For a hackathon, a 5-step sub-scan is usually enough to "catch" the hit
    refine_dt = dt / 10.0
    curr_t = t_start
    # Reset to the start of the "danger window"
    s_refine = propagate_to_time(sat_state, t_start)
    d_refine = propagate_to_time(deb_state, t_start)
    
    final_sat_pos = s_refine[:3].copy()
    final_deb_pos = d_refine[:3].copy()

    while curr_t <= t_end:
        dist = np.linalg.norm(s_refine[:3] - d_refine[:3])
        if dist < min_dist:
            min_dist = dist
            min_t = curr_t
            final_sat_pos = s_refine[:3].copy()
            final_deb_pos = d_refine[:3].copy()
            
        s_refine = rk4_step(s_refine, refine_dt)
        d_refine = rk4_step(d_refine, refine_dt)
        curr_t += refine_dt

    return min_t, min_dist, final_sat_pos, final_deb_pos


class ConjunctionAssessor:
    """
    High-performance conjunction assessor using KD-Tree for spatial pre-filtering.
    
    Algorithm:
      1. Build KD-Tree over all debris positions.
      2. For each satellite, query debris within KDTREE_COARSE_RADIUS_KM.
      3. Run precise TCA analysis only on candidate pairs (~O(N log N) total).
    """

    def __init__(self):
        self._debris_states: dict[str, np.ndarray] = {}   # id → [r, v]
        self._satellite_states: dict[str, np.ndarray] = {}
        self._kdtree: Optional[KDTree] = None
        self._debris_ids: list[str] = []

    def update_debris(self, debris_states: dict[str, np.ndarray]) -> None:
        """Ingest updated debris state vectors and rebuild KD-Tree."""
        self._debris_states = debris_states
        self._debris_ids    = list(debris_states.keys())

        if self._debris_ids:
            positions = np.array([debris_states[d][:3] for d in self._debris_ids])
            self._kdtree = KDTree(positions)

    def update_satellites(self, satellite_states: dict[str, np.ndarray]) -> None:
        self._satellite_states = satellite_states

    def assess_all(
        self,
        horizon_s: float = PROPAGATION_HORIZON_S,
        dt: float = DEFAULT_DT_S,
    ) -> list[ConjunctionEvent]:
        """
        Run full conjunction assessment for all satellites vs all debris.
        Returns list of ConjunctionEvents sorted by risk (critical first).
        """
        if self._kdtree is None or not self._satellite_states:
            return []

        events: list[ConjunctionEvent] = []

        for sat_id, sat_state in self._satellite_states.items():
            sat_pos = sat_state[:3]

            # --- Step 1: KD-Tree coarse filter ---
            candidate_indices = self._kdtree.query_ball_point(
                sat_pos, KDTREE_COARSE_RADIUS_KM
            )

            # --- Step 2: Precise TCA for each candidate ---
            for idx in candidate_indices:
                deb_id    = self._debris_ids[idx]
                deb_state = self._debris_states[deb_id]

                tca_s, miss_km, sat_pos_tca, deb_pos_tca = find_tca(
                    sat_state, deb_state, horizon_s, dt
                )

                risk = _classify_risk(miss_km)
                if risk in ("CRITICAL", "WARNING"):
                    events.append(ConjunctionEvent(
                        satellite_id=sat_id,
                        debris_id=deb_id,
                        tca_seconds_from_now=tca_s,
                        miss_distance_km=miss_km,
                        risk_level=risk,
                        sat_pos_at_tca=sat_pos_tca,
                        deb_pos_at_tca=deb_pos_tca,
                    ))

        # Sort: CRITICAL first, then by TCA time
        events.sort(key=lambda e: (0 if e.risk_level == "CRITICAL" else 1, e.tca_seconds_from_now))
        return events

    def quick_count(self) -> int:
        """Return number of currently tracked active warnings."""
        events = self.assess_all(horizon_s=86400.0, dt=30.0)
        return len([e for e in events if e.risk_level == "CRITICAL"])
