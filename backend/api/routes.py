# backend/api/routes.py
"""
FastAPI route definitions for all ACM endpoints.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Any
from datetime import datetime, timezone
import logging

from backend.core.simulation import get_simulation
from backend.core.autonomy import run_autonomy

logger = logging.getLogger("acm.api")
router = APIRouter()


# ── Request / Response Models ──────────────────────────────────────────────── #

class TelemetryRequest(BaseModel):
    timestamp: str
    objects: list[dict]

class ManeuverRequest(BaseModel):
    satelliteId: str
    maneuver_sequence: list[dict]

class StepRequest(BaseModel):
    step_seconds: float


# ── Endpoints ─────────────────────────────────────────────────────────────── #

@router.post("/api/telemetry")
async def ingest_telemetry(body: TelemetryRequest):
    """Ingest orbital state vectors and trigger autonomous conjunction assessment."""
    sim = get_simulation()
    try:
        ts = datetime.fromisoformat(body.timestamp.replace("Z", "+00:00"))
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid timestamp format")

    count = sim.ingest_telemetry(ts, body.objects)

    # Run autonomy after every telemetry update
    run_autonomy(sim)

    return {
        "status": "ACK",
        "processed_count": count,
        "active_cdm_warnings": len(sim.active_cdm_warnings),
    }

@router.get("/api/status/fleet")
async def get_fleet_status():
    """Specific endpoint requested by the frontend for the fleet dashboard."""
    sim = get_simulation()
    return [
        {
            "id": sid,
            "status": sat.status,
            "fuel_kg": round(sat.mass_fuel_kg, 2),
            "fuel_percent": round(sat.fuel_fraction * 100, 1),
            "in_box": sat.is_in_station_box(),
            "pending_burns": len(sat.maneuver_queue),
            "lat": round(sat.last_lat, 2) if hasattr(sat, 'last_lat') else 0.0,
            "lon": round(sat.last_lon, 2) if hasattr(sat, 'last_lon') else 0.0,
        }
        for sid, sat in sim.satellites.items()
    ]


@router.post("/api/maneuver/schedule")
async def schedule_maneuver(body: ManeuverRequest):
    """Validate and schedule a maneuver sequence for a satellite."""
    sim = get_simulation()
    result = sim.schedule_maneuver(body.satelliteId, body.maneuver_sequence)

    if result.get("status") == "ERROR":
        raise HTTPException(status_code=404, detail=result["reason"])
    if result.get("status") == "REJECTED":
        raise HTTPException(status_code=422, detail=result["reason"])

    return result


@router.post("/api/simulate/step")
async def simulate_step(body: StepRequest):
    """Advance simulation by step_seconds, propagating all objects and executing burns."""
    sim = get_simulation()

    if body.step_seconds <= 0:
        raise HTTPException(status_code=400, detail="step_seconds must be positive")

    result = sim.step(body.step_seconds)

    # Run autonomy after the step to catch new conjunctions
    run_autonomy(sim)

    return result


@router.get("/api/visualization/snapshot")
async def get_snapshot():
    """Return optimized snapshot payload for frontend visualizer."""
    sim = get_simulation()
    return sim.get_snapshot()


@router.get("/api/status")
async def get_status():
    """System health and summary statistics."""
    sim = get_simulation()
    return {
        "status": "OPERATIONAL",
        "current_time": sim.current_time.isoformat().replace("+00:00", "Z"),
        "satellites_tracked": len(sim.satellites),
        "debris_tracked": len(sim.debris_states),
        "active_warnings": len(sim.active_cdm_warnings),
        "collision_count": sim.collision_count,
        "maneuvers_executed": sim.maneuvers_executed,
        "satellite_statuses": {
            sid: s.status for sid, s in sim.satellites.items()
        },
    }


@router.get("/api/conjunctions")
async def get_conjunctions():
    """Return all active conjunction warnings with full detail."""
    sim = get_simulation()
    return {
        "timestamp": sim.current_time.isoformat().replace("+00:00", "Z"),
        "warnings": [
            {
                "satellite_id": e.satellite_id,
                "debris_id": e.debris_id,
                "tca_seconds": round(e.tca_seconds_from_now, 1),
                "miss_distance_km": round(e.miss_distance_km, 4),
                "risk_level": e.risk_level,
            }
            for e in sim.active_cdm_warnings
        ],
    }


@router.get("/api/satellites/{sat_id}")
async def get_satellite(sat_id: str):
    """Return detailed telemetry for a specific satellite."""
    sim = get_simulation()
    sat = sim.satellites.get(sat_id)
    if sat is None:
        raise HTTPException(status_code=404, detail=f"Satellite {sat_id} not found")

    from backend.core.physics import eci_to_latlon
    lat, lon, alt = eci_to_latlon(sat.state[:3], sim._gst_rad)

    return {
        "id": sat_id,
        "status": sat.status,
        "state_eci": {
            "r": {"x": sat.state[0], "y": sat.state[1], "z": sat.state[2]},
            "v": {"x": sat.state[3], "y": sat.state[4], "z": sat.state[5]},
        },
        "lat": round(lat, 4),
        "lon": round(lon, 4),
        "alt_km": round(alt, 2),
        "fuel_kg": round(sat.mass_fuel_kg, 3),
        "fuel_fraction": round(sat.fuel_fraction, 4),
        "total_dv_used_km_s": round(sat.total_dv_used_km_s, 6),
        "in_station_box": sat.is_in_station_box(),
        "distance_from_slot_km": round(sat.distance_from_slot(), 3),
        "outage_seconds": round(sat.outage_seconds, 1),
        "is_eol": sat.is_eol,
        "pending_burns": len(sat.maneuver_queue),
    }


@router.post("/api/reset")
async def reset_simulation():
    """Reset simulation state (for testing)."""
    import backend.core.simulation as sim_module
    from backend.core.autonomy import reset_autonomy
    sim_module._sim = None
    reset_autonomy()
    return {"status": "RESET"}
