from backend.core.simulation import get_simulation
from backend.utils.test_collision import generate_test_collision
from datetime import datetime

sim = get_simulation()
prec = generate_test_collision()
print('ingest ts', prec['timestamp'])
sim.ingest_telemetry(datetime.fromisoformat(prec['timestamp'].replace('Z', '+00:00')), prec['objects'])
from backend.core.conjunction import ConjunctionAssessor

for i in range(10):
    warnings = sim.active_cdm_warnings
    print(f"step {i} time {sim.current_time.isoformat().replace('+00:00','Z')} warnings {len(warnings)}")
    for e in warnings:
        print('   ', e.satellite_id, e.debris_id, round(e.tca_seconds_from_now/3600, 4), 'h', round(e.miss_distance_km, 6), e.risk_level)
    sim.step(3600.0)
