# backend/core/constants.py
"""
Physical and simulation constants for the ACM physics engine.
"""

# --- Earth & Gravity ---
MU = 398600.4418          # Earth gravitational parameter (km^3/s^2)
RE = 6378.137             # Earth equatorial radius (km)
J2 = 1.08263e-3           # J2 perturbation coefficient
G0 = 9.80665e-3           # Standard gravity in km/s^2 (for Tsiolkovsky)

# --- Spacecraft Bus ---
MASS_DRY_KG       = 500.0   # Dry mass (kg)
MASS_FUEL_INIT_KG = 50.0    # Initial propellant mass (kg)
MASS_WET_KG       = MASS_DRY_KG + MASS_FUEL_INIT_KG
ISP_S             = 300.0   # Specific impulse (seconds)
MAX_DELTAV_KM_S   = 0.015   # Max ΔV per burn (15 m/s → km/s)
THRUSTER_COOLDOWN_S = 600.0 # Mandatory cooldown between burns (seconds)
FUEL_EOL_FRACTION = 0.05    # End-of-life fuel threshold (5%)

# --- Conjunction & Safety ---
CONJUNCTION_THRESHOLD_KM = 0.100   # Critical miss distance (100 m → km)
CONJUNCTION_WARNING_KM   = 5.0     # Yellow warning zone (km)
STATION_KEEP_BOX_KM      = 10.0    # Station-keeping radius (km)

# --- Propagation ---
PROPAGATION_HORIZON_S = 86400.0    # 24-hour prediction window (seconds)
DEFAULT_DT_S          = 10.0       # Default RK4 timestep (seconds)
SIGNAL_LATENCY_S      = 10.0       # Ground uplink latency (seconds)

# --- Spatial Indexing ---
KDTREE_COARSE_RADIUS_KM = 50.0     # Pre-filter radius before precise TCA check
