# Line-by-Line Walkthrough: Test Collision TCA Calculation

## Test Case Setup (from `test_collision.py`)

```python
SAT-TARGET-01: r = (6928.0, 0.0, 0.0) km,  v = (0.0, 7.6, 0.0) km/s
DEB-INTERSECT-01: r = (6928.05, 20.0, 0.0) km,  v = (0.0, -7.6, 0.0) km/s
```

### Initial Geometry
- **Separation in X**: 6928.05 - 6928.0 = 0.05 km (50 meters)
- **Separation in Y**: 20.0 - 0.0 = 20.0 km
- **Initial distance**: √(0.05² + 20.0²) = **20.000062 km**

---

## The `find_tca()` Algorithm

Located in [core/conjunction.py](core/conjunction.py), the function uses a **two-phase search**:

### PHASE 1: Coarse Global Scan (10-second timesteps)

```python
# From line 53-64 of conjunction.py
min_dist = np.inf
min_t = 0.0

t = 0.0
while t <= horizon_s:                           # horizon_s = 86400 seconds (24 hours)
    dist = np.linalg.norm(sat[:3] - deb[:3])   # Calculate 3D distance
    if dist < min_dist:
        min_dist = dist
        min_t = t
    
    sat = rk4_step(sat, dt)                     # dt = 10 seconds
    deb = rk4_step(deb, dt)
    t += dt
```

#### How `rk4_step()` Works

`rk4_step()` advances the 6D state vector `[x, y, z, vx, vy, vz]` using 4th-order Runge-Kutta integration.

**At t=0:**
- State: `[6928.0, 0.0, 0.0, 0.0, 7.6, 0.0]`
- `state_derivative()` returns: `[dr/dt, dv/dt]`
  - Position rates = velocity = `[0, 7.6, 0]`
  - Velocity rates = gravity acceleration ≈ `[-0.00832, 0, 0]` (toward Earth)

**After 10-second step:**
- Satellite moves ~76 km in Y direction
- Satellite moves ~50 meters in negative X direction (orbital decay)
- New position ≈ `[6927.58, 76.00, 0]`

---

### PHASE 2: Refinement Search (1-second timesteps)

```python
# From line 66-88 of conjunction.py
t_start = max(0, min_t - dt)      # min_t found in Phase 1; dt=10
t_end = min(horizon_s, min_t + dt)
refine_dt = dt / 10.0              # 1 second timestep

# Jump to the danger window start using propagate_to_time
s_refine = propagate_to_time(sat_state, t_start)
d_refine = propagate_to_time(deb_state, t_start)

# Scan with finer resolution
while curr_t <= t_end:
    dist = np.linalg.norm(s_refine[:3] - d_refine[:3])
    if dist < min_dist:
        min_dist = dist
        min_t = curr_t
        final_sat_pos = s_refine[:3].copy()
        final_deb_pos = d_refine[:3].copy()
    
    s_refine = rk4_step(s_refine, refine_dt)  # 1-second steps
    d_refine = rk4_step(d_refine, refine_dt)
    curr_t += refine_dt
```

---

## Step-by-Step Trace for Test Case

### Phase 1 Results
Phase 1 identified that the minimum distance occurs around **t=0 to t=10 seconds**.

Result:
- `min_t = 0.0s`
- `min_dist = 20.000062 km` (initial distance)

**Refinement window**: `[0.0s, 10.0s]`

### Phase 2 Refinement (1-second steps)

| Time (s) | Satellite Y (km) | Debris Y (km) | Y Separation (km) | 3D Distance (km) | Status |
|----------|------------------|---------------|--------------------|------------------|--------|
| 0.00     | 0.0000           | 20.0000       | 20.0000            | 20.000062        | Initial |
| 1.00     | 7.6000           | 12.4000       | 4.8000             | **4.800251**     | ← **NEW MINIMUM** |
| 2.00     | 15.2000          | 4.8000        | 10.4000            | 10.400144        | Distance increases |
| 3.00     | 22.8000          | -2.8001       | 25.6000            | 25.600075        | Diverging |
| 4.00+    | *Moving apart*   | *Moving apart* | *Increasing*       | *Increasing*     | No longer threat |

**Physical Interpretation:**
1. **t=0**: Satellite at origin, debris 20 km away in Y
2. **t=1s**: Satellite moves +7.6 km in Y (velocity × time), debris moves -7.6 km in Y
   - Y separation reduces to 4.8 km
   - They pass closest to each other
   - **Minimum distance: 4.8 km (WARNING level)**
3. **t>1s**: Satellite continues +Y, debris continues -Y, so they diverge again

---

## Key Constants Used

From [core/constants.py](core/constants.py):

| Constant | Value | Purpose |
|----------|-------|---------|
| `PROPAGATION_HORIZON_S` | 86400 | 24-hour prediction window |
| `DEFAULT_DT_S` | 10 | Phase 1 coarse timestep |
| `CONJUNCTION_THRESHOLD_KM` | 0.1 | CRITICAL threshold (100m) |
| `CONJUNCTION_WARNING_KM` | 5.0 | WARNING threshold (5km) |
| `KDTREE_COARSE_RADIUS_KM` | 50 | Pre-filter radius before TCA check |

---

## Risk Classification

```python
# From line 33-40 in conjunction.py
def _classify_risk(miss_km: float) -> str:
    if miss_km < CONJUNCTION_THRESHOLD_KM:      # < 100m
        return "CRITICAL"
    elif miss_km < 1.0:                         # < 1km
        return "CRITICAL"
    elif miss_km < CONJUNCTION_WARNING_KM:      # < 5km
        return "WARNING"
    return "SAFE"
```

For this test case:
- miss_km = 4.800 km
- Classification: **WARNING** ⚠️

---

## Final Output

```
Time of Closest Approach (TCA): 1.00 seconds
Minimum Distance:              4.800251 km (4,800 meters)
Satellite position at TCA:     (6927.996, 7.600, 0) km
Debris position at TCA:        (6928.046, 12.400, 0) km
Risk Level:                    WARNING
```

---

## How It's Used in the System

1. **Upload test data** → `POST /api/telemetry`
2. **ConjunctionAssessor.assess_all()** calls `find_tca()` for each satellite-debris pair
3. **Risk-level sorted** and returned as `ConjunctionEvent` objects
4. **Dashboard displays** the WARNING conjunction to the user

See [api/routes.py](api/routes.py#L136) for the `/api/conjunctions` endpoint that returns these events.
