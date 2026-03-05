# backend/core/ground_stations.py
"""
Ground station Line-of-Sight (LOS) checker.
Determines if a satellite is visible from any ground station at a given time.
"""

import numpy as np
from dataclasses import dataclass
from backend.core.constants import RE


@dataclass
class GroundStation:
    station_id: str
    name: str
    lat_deg: float
    lon_deg: float
    elevation_m: float
    min_elevation_angle_deg: float


# Pre-loaded ground station network (from problem statement)
GROUND_STATIONS = [
    GroundStation("GS-001", "ISTRAC_Bengaluru",      13.0333,   77.5167,  820, 5.0),
    GroundStation("GS-002", "Svalbard_Sat_Station",  78.2297,   15.4077,  400, 5.0),
    GroundStation("GS-003", "Goldstone_Tracking",    35.4266, -116.8900, 1000, 10.0),
    GroundStation("GS-004", "Punta_Arenas",         -53.1500,  -70.9167,   30, 5.0),
    GroundStation("GS-005", "IIT_Delhi_Ground_Node", 28.5450,   77.1926,  225, 15.0),
    GroundStation("GS-006", "McMurdo_Station",      -77.8463,  166.6682,   10, 5.0),
]


def gs_to_ecef(gs: GroundStation) -> np.ndarray:
    """Convert ground station geodetic coordinates to ECEF (km)."""
    lat = np.radians(gs.lat_deg)
    lon = np.radians(gs.lon_deg)
    alt_km = gs.elevation_m / 1000.0

    r = RE + alt_km
    x = r * np.cos(lat) * np.cos(lon)
    y = r * np.cos(lat) * np.sin(lon)
    z = r * np.sin(lat)
    return np.array([x, y, z])


def elevation_angle(sat_ecef: np.ndarray, gs_ecef: np.ndarray) -> float:
    """
    Compute elevation angle (degrees) of satellite as seen from ground station.
    Both vectors in ECEF (km).
    """
    # Vector from GS to satellite
    rho = sat_ecef - gs_ecef

    # Unit vector along local vertical at GS (= GS position normalised)
    up = gs_ecef / np.linalg.norm(gs_ecef)

    rho_norm = np.linalg.norm(rho)
    if rho_norm == 0:
        return 90.0

    # Elevation = arcsin(dot(rho_hat, up))
    sin_el = np.dot(rho, up) / rho_norm
    return float(np.degrees(np.arcsin(np.clip(sin_el, -1.0, 1.0))))


def eci_to_ecef(r_eci: np.ndarray, gst_rad: float) -> np.ndarray:
    """Rotate ECI position to ECEF using Greenwich Sidereal Time."""
    cos_gst = np.cos(gst_rad)
    sin_gst = np.sin(gst_rad)
    R = np.array([
        [ cos_gst, sin_gst, 0],
        [-sin_gst, cos_gst, 0],
        [       0,       0, 1],
    ])
    return R @ r_eci


def has_ground_contact(
    sat_r_eci: np.ndarray,
    gst_rad: float = 0.0,
    stations: list[GroundStation] = GROUND_STATIONS,
) -> tuple[bool, str]:
    """
    Check if a satellite has LOS to at least one ground station.
    Args:
        sat_r_eci: satellite ECI position (km)
        gst_rad:   Greenwich Sidereal Time (radians); 0 = simplified
        stations:  list of GroundStation objects
    Returns:
        (has_contact: bool, best_station_id: str or "NONE")
    """
    sat_ecef = eci_to_ecef(sat_r_eci, gst_rad)

    best_el   = -90.0
    best_gs   = "NONE"

    for gs in stations:
        gs_ecef = gs_to_ecef(gs)
        el = elevation_angle(sat_ecef, gs_ecef)
        if el >= gs.min_elevation_angle_deg:
            if el > best_el:
                best_el = el
                best_gs = gs.station_id

    return best_gs != "NONE", best_gs
