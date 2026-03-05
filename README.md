# 🛰️ Autonomous Constellation Manager (ACM)
### National Space Hackathon 2026 — IIT Delhi

> An autonomous orbital debris avoidance and constellation management system capable of
> managing 50+ satellites against 10,000+ debris objects in real-time.

---

## 🏗️ Project Structure

```
acm/
├── Dockerfile                    # Ubuntu 22.04 base — REQUIRED for grader
├── docker-compose.yml            # Local development
├── README.md
├── data/
│   └── initial_telemetry.json    # Generated seed data (run seed_data.py)
├── backend/
│   ├── main.py                   # FastAPI app — all API endpoints
│   ├── requirements.txt
│   ├── core/
│   │   ├── constants.py          # Physical & simulation constants
│   │   ├── physics.py            # RK4 propagator + J2 perturbation
│   │   ├── conjunction.py        # KD-Tree conjunction assessor
│   │   ├── ground_stations.py    # LOS checker (6 ground stations)
│   │   └── simulation.py        # Central state manager / tick engine
│   ├── models/
│   │   └── satellite.py          # Satellite model + fuel/burn tracking
│   └── utils/
│       └── seed_data.py          # Generates 50 sats + 10k debris
├── frontend/
│   ├── package.json
│   ├── vite.config.js
│   └── src/
│       ├── components/           # React UI components
│       ├── hooks/                # Custom hooks (useSnapshot, useFleet)
│       └── utils/                # ECI/lat-lon helpers, color coding
└── tests/
    └── test_physics.py           # Physics engine unit tests
```

---

## 🚀 Quick Start

### Option 1: Docker (Recommended / Required for Grader)
```bash
docker build -t acm .
docker run -p 8000:8000 acm
```

### Option 2: Local Dev
```bash
# Backend
cd backend
pip install -r requirements.txt
python -m uvicorn backend.main:app --reload --port 8000

# Frontend (separate terminal)
cd frontend
npm install
npm run dev   # Runs on :5173, proxies /api → :8000
```

### Seed Initial Data
```bash
python -m backend.utils.seed_data
# Generates data/initial_telemetry.json with 50 sats + 10,000 debris
```

---

## 📡 API Reference

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/telemetry` | Ingest state vectors for satellites & debris |
| `POST` | `/api/maneuver/schedule` | Schedule evasion/recovery burn sequence |
| `POST` | `/api/simulate/step` | Advance simulation clock |
| `GET`  | `/api/visualization/snapshot` | Optimized frontend snapshot |
| `GET`  | `/api/conjunctions` | Active CDM warnings (sorted by risk) |
| `GET`  | `/api/status/fleet` | Full fleet health summary |
| `GET`  | `/health` | Health check |

---

## 🔢 Physics Engine

- **Propagator**: 4th-Order Runge-Kutta (RK4), timestep = 10s
- **Perturbations**: J2 oblateness (nodal regression + apsidal precession)
- **Reference Frame**: Earth-Centered Inertial (ECI, J2000)
- **Conjunction Threshold**: 100 m (0.100 km)

## ⚡ Spatial Optimization

Conjunction assessment uses a **KD-Tree** over all debris positions:
- Coarse filter: query all debris within 50 km of each satellite → O(N log N)
- Precise TCA: run RK4 propagation only on candidate pairs
- Result: handles 50 sats × 10,000 debris without O(N²) blowup

## 🚀 Maneuver Strategy

1. **Evasion**: Prograde/retrograde ΔV in RTN frame (most fuel-efficient)
2. **Recovery**: Hohmann-like phasing burn back to station-keeping box (±10 km)
3. **EOL**: Graveyard orbit maneuver when fuel < 5%
4. **Constraints**: 15 m/s max per burn, 600s cooldown, LOS required

---

## 🧪 Running Tests
```bash
pytest tests/ -v
```

---

## 📦 Deployment Notes

- Dockerfile uses `ubuntu:22.04` base image as required
- Application binds to `0.0.0.0:8000` (not localhost)
- Port 8000 is exported for automated grading
