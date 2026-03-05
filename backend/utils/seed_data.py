#!/usr/bin/env python3
# backend/utils/seed_data.py
"""
Generate seed data for the ACM simulation:
  - 50 active satellites in LEO (~550 km circular)
  - 10,000 debris objects spread across LEO
Outputs initial telemetry payload for POST /api/telemetry
"""

import numpy as np
import json
from datetime import datetime, timezone

# Earth constants
MU = 398600.4418
RE = 6378.137


def circular_orbit_state(altitude_km: float, inclination_deg: float,
                          raan_deg: float, true_anomaly_deg: float) -> dict:
    """Generate ECI state vector for a circular orbit."""
    a = RE + altitude_km
    v_circ = np.sqrt(MU / a)

    inc  = np.radians(inclination_deg)
    raan = np.radians(raan_deg)
    nu   = np.radians(true_anomaly_deg)

    # Position in perifocal frame
    r_peri = a * np.array([np.cos(nu), np.sin(nu), 0.0])
    v_peri = v_circ * np.array([-np.sin(nu), np.cos(nu), 0.0])

    # Rotation: perifocal → ECI (ω=0, so just RAAN + inclination)
    cos_r, sin_r = np.cos(raan), np.sin(raan)
    cos_i, sin_i = np.cos(inc),  np.sin(inc)

    R = np.array([
        [cos_r, -sin_r * cos_i,  sin_r * sin_i],
        [sin_r,  cos_r * cos_i, -cos_r * sin_i],
        [0,      sin_i,          cos_i         ],
    ])

    r_eci = R @ r_peri
    v_eci = R @ v_peri

    return {
        "r": {"x": round(r_eci[0], 3), "y": round(r_eci[1], 3), "z": round(r_eci[2], 3)},
        "v": {"x": round(v_eci[0], 6), "y": round(v_eci[1], 6), "z": round(v_eci[2], 6)},
    }


def generate_satellites(n: int = 50) -> list[dict]:
    """Generate n satellites in a Walker-like constellation at ~550 km."""
    sats = []
    planes = 5
    sats_per_plane = n // planes

    for plane in range(planes):
        raan = plane * (360.0 / planes)
        for slot in range(sats_per_plane):
            ta = slot * (360.0 / sats_per_plane)
            sat_id = f"SAT-Alpha-{plane * sats_per_plane + slot + 1:02d}"
            sv = circular_orbit_state(
                altitude_km=550.0,
                inclination_deg=53.0,
                raan_deg=raan,
                true_anomaly_deg=ta,
            )
            sats.append({"id": sat_id, "type": "SATELLITE", **sv})

    return sats


def generate_debris(n: int = 10000) -> list[dict]:
    """Generate n debris objects spread across LEO (400–800 km)."""
    rng = np.random.default_rng(42)
    debris = []

    for i in range(n):
        alt    = rng.uniform(400, 800)
        inc    = rng.uniform(0, 100)
        raan   = rng.uniform(0, 360)
        ta     = rng.uniform(0, 360)
        deb_id = f"DEB-{10000 + i}"
        sv = circular_orbit_state(alt, inc, raan, ta)
        # Add slight eccentricity perturbation to velocity
        v_noise = rng.uniform(-0.002, 0.002, 3)
        sv["v"]["x"] += round(float(v_noise[0]), 6)
        sv["v"]["y"] += round(float(v_noise[1]), 6)
        sv["v"]["z"] += round(float(v_noise[2]), 6)
        debris.append({"id": deb_id, "type": "DEBRIS", **sv})

    return debris


def build_initial_telemetry() -> dict:
    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    objects = generate_satellites(50) + generate_debris(10000)
    return {"timestamp": ts, "objects": objects}


if __name__ == "__main__":
    payload = build_initial_telemetry()
    out_path = "data/initial_telemetry.json"
    import os; os.makedirs("data", exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(payload, f)
    print(f"Generated {len(payload['objects'])} objects → {out_path}")
    sats   = sum(1 for o in payload["objects"] if o["type"] == "SATELLITE")
    debris = sum(1 for o in payload["objects"] if o["type"] == "DEBRIS")
    print(f"  Satellites: {sats}")
    print(f"  Debris:     {debris}")
