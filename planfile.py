import json
import math

def generate_grid_plan(start_lat, start_lon, rows, cols, spacing_meters, altitude=10):
    def offset_coordinates(lat, lon, dx, dy):
        # dx, dy in meters
        R = 6378137  # Radius of Earth in meters
        new_lat = lat + (dy / R) * (180 / math.pi)
        new_lon = lon + (dx / (R * math.cos(math.pi * lat / 180))) * (180 / math.pi)
        return new_lat, new_lon

    waypoints = []
    seq = 0
    for row in range(rows):
        for col in range(cols):
            dx = col * spacing_meters
            dy = row * spacing_meters
            lat, lon = offset_coordinates(start_lat, start_lon, dx, dy)
            waypoint = {
                "AMSLAltAboveTerrain": None,
                "Altitude": altitude,
                "AltitudeMode": 1,
                "command": 16,
                "doJumpId": seq + 1,
                "frame": 3,
                "params": [0, 0, 0, None, lat, lon, altitude],
                "autoContinue": True,
                "type": "SimpleItem"
            }
            waypoints.append(waypoint)
            seq += 1

    plan = {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 15,
            "firmwareType": 12,
            "hoverSpeed": 5,
            "items": waypoints,
            "plannedHomePosition": [start_lat, start_lon, altitude],
            "vehicleType": 2,
            "version": 2
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1
    }

    with open("grid_mission.plan", "w") as f:
        json.dump(plan, f, indent=4)

    print("Plan file 'grid_mission.plan' generated successfully.")


# Example usage
# Start from some known GPS location, e.g., a test field
generate_grid_plan(start_lat=12.9716, start_lon=77.5946, rows=4, cols=4, spacing_meters=25)
