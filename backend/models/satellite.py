# backend/models/satellite.py
"""
Satellite data model with fuel tracking, maneuver queue, and station-keeping logic.
"""

import numpy as np
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional
from backend.core.constants import (
    MASS_DRY_KG, MASS_FUEL_INIT_KG, ISP_S, G0,
    MAX_DELTAV_KM_S, THRUSTER_COOLDOWN_S,
    FUEL_EOL_FRACTION, STATION_KEEP_BOX_KM,
)


@dataclass
class BurnCommand:
    burn_id: str
    burn_time: datetime
    delta_v_eci: np.ndarray        # [dvx, dvy, dvz] in km/s
    executed: bool = False
    fuel_consumed_kg: float = 0.0


@dataclass
class Satellite:
    sat_id: str
    state: np.ndarray              # [x, y, z, vx, vy, vz] ECI, km/km·s⁻¹
    nominal_state: np.ndarray      # Nominal slot state vector
    mass_dry_kg: float   = MASS_DRY_KG
    mass_fuel_kg: float  = MASS_FUEL_INIT_KG
    status: str          = "NOMINAL"   # NOMINAL | EVADING | RECOVERING | EOL
    maneuver_queue: list[BurnCommand] = field(default_factory=list)
    last_burn_time: Optional[datetime] = None
    total_dv_used_km_s: float = 0.0
    outage_seconds: float = 0.0

    @property
    def mass_total_kg(self) -> float:
        return self.mass_dry_kg + self.mass_fuel_kg

    @property
    def fuel_fraction(self) -> float:
        return self.mass_fuel_kg / MASS_FUEL_INIT_KG

    @property
    def is_eol(self) -> bool:
        return self.fuel_fraction <= FUEL_EOL_FRACTION

    def can_burn(self, at_time: datetime) -> tuple[bool, str]:
        """Check if thruster is ready (cooldown respected, fuel available, not EOL)."""
        if self.is_eol:
            return False, "EOL: insufficient fuel"
        if self.last_burn_time is not None:
            elapsed = (at_time - self.last_burn_time).total_seconds()
            if elapsed < THRUSTER_COOLDOWN_S:
                remaining = THRUSTER_COOLDOWN_S - elapsed
                return False, f"Thruster cooldown: {remaining:.0f}s remaining"
        return True, "OK"

    def apply_burn(self, burn: BurnCommand) -> dict:
        """
        Apply an impulsive burn: update velocity, consume fuel via Tsiolkovsky.
        Returns a result dict with fuel metrics.
        """
        dv_mag = np.linalg.norm(burn.delta_v_eci)  # km/s

        if dv_mag > MAX_DELTAV_KM_S:
            raise ValueError(f"ΔV {dv_mag*1000:.2f} m/s exceeds max {MAX_DELTAV_KM_S*1000:.1f} m/s")

        # Tsiolkovsky: Δm = m_current * (1 - exp(-|ΔV| / (Isp * g0)))
        # Note: G0 is already in km/s^2, ISP in seconds → Isp*g0 in km/s
        delta_m = self.mass_total_kg * (1.0 - np.exp(-dv_mag / (ISP_S * G0)))

        if delta_m > self.mass_fuel_kg:
            raise ValueError("Insufficient fuel for this maneuver")

        # Apply ΔV (position unchanged, velocity updated)
        self.state[3:] += burn.delta_v_eci
        self.mass_fuel_kg -= delta_m
        self.total_dv_used_km_s += dv_mag
        self.last_burn_time = burn.burn_time
        burn.executed = True
        burn.fuel_consumed_kg = delta_m

        return {
            "dv_km_s": dv_mag,
            "fuel_consumed_kg": delta_m,
            "mass_remaining_kg": self.mass_total_kg,
            "fuel_kg_remaining": self.mass_fuel_kg,
        }

    def distance_from_slot(self) -> float:
        """Distance from current position to nominal orbital slot (km)."""
        return float(np.linalg.norm(self.state[:3] - self.nominal_state[:3]))

    def is_in_station_box(self) -> bool:
        return self.distance_from_slot() <= STATION_KEEP_BOX_KM

    def queue_maneuver(self, burn: BurnCommand) -> None:
        """Add a burn to the maneuver queue, sorted by burn time."""
        self.maneuver_queue.append(burn)
        self.maneuver_queue.sort(key=lambda b: b.burn_time)

    def pop_due_burns(self, current_time: datetime) -> list[BurnCommand]:
        """Return all burns due at or before current_time (not yet executed)."""
        due = [b for b in self.maneuver_queue if b.burn_time <= current_time and not b.executed]
        return due
