import json
import requests
from datetime import datetime, timezone

def generate_test_collision():
    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    
    # 1. Start them 50km apart (so the KDTree picks them up)
    # 2. Aim them so their paths cross in the next few minutes
    objects = [
        {
            "id": "SAT-TARGET-01",
            "type": "SATELLITE",
            "r": {"x": 6928.0, "y": 0.0, "z": 0.0},
            "v": {"x": 0.0, "y": 7.6, "z": 0.0} 
        },
        {
            "id": "DEB-INTERSECT-01",
            "type": "DEBRIS",
            # Start 20km away on the Y axis, moving toward Y=0
            "r": {"x": 6928.05, "y": 50.0, "z": 0.0}, # Offset by only 50m on X
            "v": {"x": 0.0, "y": -7.6, "z": 0.0}     # Head-on collision
        }
    ]
    
    return {"timestamp": ts, "objects": objects}

if __name__ == "__main__":
    # 1. Generate the specific collision data
    payload = generate_test_collision()
    
    # 2. Upload to the backend
    print("Uploading test collision to backend...")
    try:
        response = requests.post("http://localhost:8000/api/telemetry", json=payload)
        if response.status_code == 200:
            print(f"Successfully ingested! Status: {response.json().get('status')}")
            print("Now check your dashboard and hit 'STEP' to see the alert!")
        else:
            print(f"Failed to ingest: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Could not connect to backend: {e}")