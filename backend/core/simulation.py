# backend/core/simulation.py
"""
Central Simulation State Manager.
Holds all satellite and debris states, advances time, executes maneuvers.
"""

import numpy as np
from datetime import datetime, timedelta, timezone
from typing import Optional
import logging
from .autonomy import run_autonomy

from .physics import rk4_step, propagate_to_time, eci_to_latlon
from .conjunction import ConjunctionAssessor, ConjunctionEvent
from .ground_stations import has_ground_contact
from .constants import (
    DEFAULT_DT_S, SIGNAL_LATENCY_S,
    FUEL_EOL_FRACTION, MASS_DRY_KG,
)
from backend.models.satellite import Satellite, BurnCommand

logger = logging.getLogger("acm.simulation")


class SimulationState:
    def __init__(self):
        self.current_time: datetime = datetime.now(timezone.utc)
        self.satellites: dict[str, Satellite] = {}
        self.debris_states: dict[str, np.ndarray] = {}  # id → [r, v]
        self.conjunction_assessor = ConjunctionAssessor()
        self.active_cdm_warnings: list[ConjunctionEvent] = []
        self.collision_count: int = 0
        self.maneuvers_executed: int = 0
        self._gst_rad: float = 0.0  # Simplified: track GST

    # ------------------------------------------------------------------ #
    #  Telemetry Ingestion
    # ------------------------------------------------------------------ #

    def ingest_telemetry(self, timestamp: datetime, objects: list[dict]) -> int:
        """Parse incoming telemetry and update internal states."""
        count = 0
        for obj in objects:
            obj_id   = obj["id"]
            obj_type = obj["type"]
            r = np.array([obj["r"]["x"], obj["r"]["y"], obj["r"]["z"]])
            v = np.array([obj["v"]["x"], obj["v"]["y"], obj["v"]["z"]])
            state = np.concatenate([r, v])

            if obj_type == "DEBRIS":
                self.debris_states[obj_id] = state

            elif obj_type == "SATELLITE":
                if obj_id not in self.satellites:
                    # Register new satellite
                    self.satellites[obj_id] = Satellite(
                        sat_id=obj_id,
                        state=state,
                        nominal_state=state.copy(),  # First seen position = nominal
                    )
                else:
                    self.satellites[obj_id].state = state

            count += 1

        # Rebuild conjunction assessor with latest data
        sat_states = {sid: s.state for sid, s in self.satellites.items()}
        self.conjunction_assessor.update_debris(self.debris_states)
        self.conjunction_assessor.update_satellites(sat_states)

        self.current_time = timestamp
        return count

    # ------------------------------------------------------------------ #
    #  Schedule Maneuver
    # ------------------------------------------------------------------ #

    def schedule_maneuver(self, satellite_id: str, maneuver_sequence: list[dict]) -> dict:
        """Validate and queue a maneuver sequence for a satellite."""
        if satellite_id not in self.satellites:
            return {"status": "ERROR", "reason": f"Unknown satellite {satellite_id}"}

        sat = self.satellites[satellite_id]
        results = []
        projected_mass = sat.mass_total_kg
        projected_fuel = sat.mass_fuel_kg

        for burn_data in maneuver_sequence:
            burn_time = datetime.fromisoformat(burn_data["burnTime"].replace("Z", "+00:00"))
            dv = burn_data["deltaV_vector"]
            dv_eci = np.array([dv["x"], dv["y"], dv["z"]])

            # Validate: minimum time = now + latency
            earliest = self.current_time + timedelta(seconds=SIGNAL_LATENCY_S)
            if burn_time < earliest:
                return {
                    "status": "REJECTED",
                    "reason": f"Burn {burn_data['burn_id']} scheduled before latency window"
                }

            # Validate: LOS at burn time (simplified - check current position)
            los_ok, gs_id = has_ground_contact(sat.state[:3], self._gst_rad)

            # Validate: fuel
            from backend.core.constants import ISP_S, G0
            dv_mag = float(np.linalg.norm(dv_eci))
            delta_m = projected_mass * (1.0 - np.exp(-dv_mag / (ISP_S * G0)))
            if delta_m > projected_fuel:
                return {
                    "status": "REJECTED",
                    "reason": f"Insufficient fuel for burn {burn_data['burn_id']}"
                }

            projected_mass -= delta_m
            projected_fuel -= delta_m

            burn = BurnCommand(
                burn_id=burn_data["burn_id"],
                burn_time=burn_time,
                delta_v_eci=dv_eci,
            )
            sat.queue_maneuver(burn)
            results.append(burn_data["burn_id"])

        return {
            "status": "SCHEDULED",
            "validation": {
                "ground_station_los": los_ok,
                "sufficient_fuel": True,
                "projected_mass_remaining_kg": round(projected_mass, 2),
            }
        }

    # ------------------------------------------------------------------ #
    #  Simulation Step (Tick)
    # ------------------------------------------------------------------ #

    def step(self, step_seconds: float) -> dict:
        dt = DEFAULT_DT_S
        end_time = self.current_time + timedelta(seconds=step_seconds)
        total_collisions_this_tick = 0
        maneuvers_this_step = 0

        t = 0.0
        while t < step_seconds:
            chunk = min(dt, step_seconds - t)

            # 1. Move the Brain: Run autonomy BEFORE the physics jump
            # This allows satellites to schedule burns for the current window
            run_autonomy(self)

            # 2. Propagate all debris
            for deb_id in list(self.debris_states.keys()):
                self.debris_states[deb_id] = rk4_step(self.debris_states[deb_id], chunk)

            # 3. Propagate satellites and execute burns
            for sat in self.satellites.values():
                sat.state = rk4_step(sat.state, chunk)
                sat.nominal_state = rk4_step(sat.nominal_state, chunk)

                # Execute burns that are due in THIS chunk
                sub_time = self.current_time + timedelta(seconds=t + chunk)
                for burn in sat.pop_due_burns(sub_time):
                    try:
                        result = sat.apply_burn(burn)
                        maneuvers_this_step += 1
                        self.maneuvers_executed += 1
                    except ValueError as e:
                        logger.warning(f"[BURN FAILED] {sat.sat_id}: {e}")

            # 4. Intra-step Collision Check
            # This catches the 1.3s hit even if you step 60s
            collisions_in_chunk = self._check_collisions()
            total_collisions_this_tick += collisions_in_chunk
            
            t += chunk

        self.collision_count += total_collisions_this_tick
        self.current_time = end_time

        return {
            "status": "STEP_COMPLETE",
            "new_timestamp": self.current_time.isoformat().replace("+00:00", "Z"),
            "collisions_detected": total_collisions_this_tick,
            "maneuvers_executed": maneuvers_this_step,
        }

    def _check_collisions(self) -> int:
        """Optimized collision check using the KD-Tree spatial index."""
        from backend.core.constants import CONJUNCTION_THRESHOLD_KM
        count = 0
        
        # 1. Update the tree with current debris positions
        self.conjunction_assessor.update_debris(self.debris_states)
        
        for sat in self.satellites.values():
            # 2. Only query debris within a small radius (e.g., 2km)
            # This reduces 10,000 checks down to ~0-5 checks per satellite
            neighbor_indices = self.conjunction_assessor._kdtree.query_ball_point(
                sat.state[:3], r=2.0 
            )
            
            for idx in neighbor_indices:
                # Retrieve the state of the nearby debris
                deb_id = list(self.debris_states.keys())[idx]
                deb_state = self.debris_states[deb_id]
                
                dist = float(np.linalg.norm(sat.state[:3] - deb_state[:3]))
                if dist < CONJUNCTION_THRESHOLD_KM:
                    count += 1
                    sat.status = "COLLISION"
                    logger.error(f"[COLLISION] {sat.sat_id} hit {deb_id}!")
        return count

    # ------------------------------------------------------------------ #
    #  Visualization Snapshot
    # ------------------------------------------------------------------ #

    def get_snapshot(self) -> dict:
        """Return optimized snapshot payload for the frontend visualizer."""
        satellites_out = []
        for sat in self.satellites.values():
            lat, lon, alt = eci_to_latlon(sat.state[:3], self._gst_rad)
            satellites_out.append({
                "id":      sat.sat_id,
                "lat":     round(lat, 4),
                "lon":     round(lon, 4),
                "alt_km":  round(alt, 2),
                "fuel_kg": round(sat.mass_fuel_kg, 2),
                "status":  sat.status,
                "in_slot": sat.is_in_station_box(),
            })

        # Compact debris cloud: [id, lat, lon, alt]
        debris_cloud = []
        for deb_id, deb_state in self.debris_states.items():
            lat, lon, alt = eci_to_latlon(deb_state[:3], self._gst_rad)
            debris_cloud.append([deb_id, round(lat, 3), round(lon, 3), round(alt, 1)])

        return {
            "timestamp":   self.current_time.isoformat().replace("+00:00", "Z"),
            "satellites":  satellites_out,
            "debris_cloud": debris_cloud,
        }


# Global singleton
_sim: Optional[SimulationState] = None

def get_simulation() -> SimulationState:
    global _sim
    if _sim is None:
        _sim = SimulationState()
    return _sim
