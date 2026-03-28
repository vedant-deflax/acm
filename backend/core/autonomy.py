# backend/core/autonomy.py
"""
Autonomy Engine — runs after every telemetry ingest and simulation step.
Detects critical conjunctions and triggers COLA maneuvers automatically.
"""

import logging
from datetime import datetime, timezone

from .maneuver_planner import plan_cola_maneuvers, plan_eol_graveyard
from .conjunction import ConjunctionEvent

logger = logging.getLogger("acm.autonomy")

# Track which (sat, debris) pairs already have a maneuver scheduled
# to avoid scheduling duplicates on every telemetry tick
_active_evasions: set[str] = set()
_eol_scheduled: set[str] = set()


def run_autonomy(sim) -> dict:
    """
    Run the autonomy loop on the current simulation state.
    Called after telemetry ingest and after each step.
    
    Returns summary of actions taken.
    """
    global _active_evasions, _eol_scheduled

    actions = {"cola_planned": 0, "eol_planned": 0, "skipped": 0}

    # --- EOL check first ---
    for sat_id, sat in sim.satellites.items():
        if sat.is_eol and sat_id not in _eol_scheduled:
            sequence = plan_eol_graveyard(
                sat_id=sat_id,
                sat_state=sat.state,
                current_time=sim.current_time,
                last_burn_time=sat.last_burn_time,
            )
            if sequence:
                result = sim.schedule_maneuver(sat_id, sequence)
                if result.get("status") == "SCHEDULED":
                    _eol_scheduled.add(sat_id)
                    sat.status = "EOL"
                    actions["eol_planned"] += 1
                    logger.warning(f"[AUTONOMY] EOL graveyard burn scheduled for {sat_id}")

    # --- Conjunction assessment ---
    events = sim.conjunction_assessor.assess_all()
    sim.active_cdm_warnings = events

    active_critical_satellites = set()

    for event in events:
        if event.risk_level != "CRITICAL":
            continue

        pair_key = f"{event.satellite_id}::{event.debris_id}"
        active_critical_satellites.add(event.satellite_id)

        if pair_key in _active_evasions:
            actions["skipped"] += 1
            continue

        sat = sim.satellites.get(event.satellite_id)
        if sat is None or sat.status in ("EOL", "COLLISION"):
            continue

        sequence = plan_cola_maneuvers(
            sat_id=event.satellite_id,
            sat_state=sat.state,
            event=event,
            current_time=sim.current_time,
            last_burn_time=sat.last_burn_time,
            mass_fuel_kg=sat.mass_fuel_kg,
        )

        if sequence:
            result = sim.schedule_maneuver(event.satellite_id, sequence)
            if result.get("status") == "SCHEDULED":
                _active_evasions.add(pair_key)
                sat.status = "EVADING"
                actions["cola_planned"] += 1
                logger.info(
                    f"[AUTONOMY] COLA scheduled: {event.satellite_id} vs {event.debris_id} "
                    f"(TCA in {event.tca_seconds_from_now:.0f}s, miss={event.miss_distance_km:.3f} km)"
                )
            else:
                logger.warning(
                    f"[AUTONOMY] Maneuver rejected for {event.satellite_id}: {result}"
                )
        else:
            actions["skipped"] += 1

    # Mark strategy statuses clearly so dashboard reflects active state
    for sat_id, sat in sim.satellites.items():
        if sat.status in ("EOL", "COLLISION"):
            continue
        if sat_id in active_critical_satellites:
            sat.status = "EVADING"
        elif any(burn for burn in sat.maneuver_queue if not burn.executed):
            sat.status = "EVADING"
        else:
            sat.status = "NOMINAL"

    # Prune stale evasion keys (conjunctions that are no longer active)
    still_active = set()
    for pair in _active_evasions:
        # If the pair is still in the current warnings, keep it
        if any(f"{e.satellite_id}::{e.debris_id}" == pair for e in events):
            still_active.add(pair)
        else:
            # If it's gone from warnings, check if it passed or was dodged
            # For now, we'll let it clear so the sat can return to NOMINAL
            pass 
            
    _active_evasions = still_active

    return actions


def reset_autonomy():
    """Reset autonomy state (call on simulation reset)."""
    global _active_evasions, _eol_scheduled
    _active_evasions = set()
    _eol_scheduled = set()
