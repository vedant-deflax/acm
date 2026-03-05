# backend/main.py
"""
Autonomous Constellation Manager (ACM) — FastAPI Application
Exposes all required REST endpoints on port 8000.
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from datetime import datetime
from typing import Optional
import logging
import os

from backend.core.simulation import get_simulation

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("acm.api")

app = FastAPI(
    title="Autonomous Constellation Manager",
    description="National Space Hackathon 2026 — ACM Backend API",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ------------------------------------------------------------------ #
#  Pydantic Request/Response Models
# ------------------------------------------------------------------ #

class Vector3(BaseModel):
    x: float
    y: float
    z: float

class SpaceObject(BaseModel):
    id: str
    type: str   # "SATELLITE" | "DEBRIS"
    r: Vector3
    v: Vector3

class TelemetryRequest(BaseModel):
    timestamp: str
    objects: list[SpaceObject]

class BurnRequest(BaseModel):
    burn_id: str
    burnTime: str
    deltaV_vector: Vector3

class ManeuverRequest(BaseModel):
    satelliteId: str
    maneuver_sequence: list[BurnRequest]

class SimStepRequest(BaseModel):
    step_seconds: float


# ------------------------------------------------------------------ #
#  Endpoints
# ------------------------------------------------------------------ #

@app.get("/health")
async def health():
    sim = get_simulation()
    return {
        "status": "OK",
        "simulation_time": sim.current_time.isoformat(),
        "satellites": len(sim.satellites),
        "debris_objects": len(sim.debris_states),
    }


@app.post("/api/telemetry")
async def ingest_telemetry(req: TelemetryRequest):
    """Ingest high-frequency telemetry state vectors."""
    sim = get_simulation()

    try:
        ts = datetime.fromisoformat(req.timestamp.replace("Z", "+00:00"))
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid timestamp format")

    objects_raw = [
        {
            "id":   obj.id,
            "type": obj.type,
            "r":    {"x": obj.r.x, "y": obj.r.y, "z": obj.r.z},
            "v":    {"x": obj.v.x, "y": obj.v.y, "z": obj.v.z},
        }
        for obj in req.objects
    ]

    count = sim.ingest_telemetry(ts, objects_raw)
    active_warnings = sim.conjunction_assessor.quick_count()

    logger.info(f"Telemetry: {count} objects ingested, {active_warnings} active CDMs")

    return {
        "status": "ACK",
        "processed_count": count,
        "active_cdm_warnings": active_warnings,
    }


@app.post("/api/maneuver/schedule", status_code=202)
async def schedule_maneuver(req: ManeuverRequest):
    """Schedule an evasion/recovery maneuver sequence for a satellite."""
    sim = get_simulation()

    sequence_raw = [
        {
            "burn_id":       b.burn_id,
            "burnTime":      b.burnTime,
            "deltaV_vector": {"x": b.deltaV_vector.x, "y": b.deltaV_vector.y, "z": b.deltaV_vector.z},
        }
        for b in req.maneuver_sequence
    ]

    result = sim.schedule_maneuver(req.satelliteId, sequence_raw)

    if result["status"] in ("ERROR", "REJECTED"):
        raise HTTPException(status_code=400, detail=result.get("reason", "Unknown error"))

    return result


@app.post("/api/simulate/step")
async def simulate_step(req: SimStepRequest):
    """Advance simulation clock by step_seconds, propagating all objects."""
    if req.step_seconds <= 0 or req.step_seconds > 86400:
        raise HTTPException(status_code=400, detail="step_seconds must be between 0 and 86400")

    sim = get_simulation()
    result = sim.step(req.step_seconds)
    logger.info(f"Tick: +{req.step_seconds}s | {result['collisions_detected']} collisions | "
                f"{result['maneuvers_executed']} maneuvers")
    return result


@app.get("/api/visualization/snapshot")
async def get_snapshot():
    """Return optimized snapshot for the Orbital Insight visualizer."""
    sim = get_simulation()
    return sim.get_snapshot()


@app.get("/api/status/fleet")
async def fleet_status():
    """Return full fleet health summary."""
    sim = get_simulation()
    fleet = []
    for sat in sim.satellites.values():
        fleet.append({
            "id":             sat.sat_id,
            "status":         sat.status,
            "fuel_kg":        round(sat.mass_fuel_kg, 3),
            "fuel_fraction":  round(sat.fuel_fraction, 4),
            "is_eol":         sat.is_eol,
            "in_slot":        sat.is_in_station_box(),
            "total_dv_m_s":   round(sat.total_dv_used_km_s * 1000, 4),
            "outage_seconds": round(sat.outage_seconds, 1),
            "queued_burns":   len([b for b in sat.maneuver_queue if not b.executed]),
        })
    return {
        "timestamp": sim.current_time.isoformat(),
        "fleet":     fleet,
        "total_collisions": sim.collision_count,
    }


@app.get("/api/conjunctions")
async def get_conjunctions():
    """Return current active conjunction warnings."""
    sim = get_simulation()
    events = sim.conjunction_assessor.assess_all(horizon_s=86400, dt=30.0)
    return {
        "timestamp": sim.current_time.isoformat(),
        "total": len(events),
        "events": [
            {
                "satellite_id":          e.satellite_id,
                "debris_id":             e.debris_id,
                "tca_seconds_from_now":  round(e.tca_seconds_from_now, 1),
                "miss_distance_km":      round(e.miss_distance_km, 4),
                "risk_level":            e.risk_level,
            }
            for e in events[:50]  # Return top 50 most critical
        ]
    }


# Serve frontend build if available
frontend_build = "frontend/dist"
if os.path.exists(frontend_build):
    app.mount("/", StaticFiles(directory=frontend_build, html=True), name="frontend")
