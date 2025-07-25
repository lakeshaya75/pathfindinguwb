import json
import math
import time
import argparse
import geopandas as gpd
import networkx as nx
from dronekit import connect, VehicleMode, LocationGlobalRelative


def connectMyCopter():
    parser = argparse.ArgumentParser(description='Drone command line')
    parser.add_argument('--connect', default='127.0.0.1:14550', help="Vehicle connection string")
    parser.add_argument('--geojson', default='uwb.geojson', help="GeoJSON file with paths")
    args = parser.parse_args()
    connection_string = args.connect
    vehicle = connect(connection_string, baud=57600, wait_ready=True)
    return vehicle, args

def get_direction_to_target(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    angle = math.atan2(dLon, dLat) * (180 / math.pi)
    if angle < 0:
        angle += 360 

    if 337.5 <= angle < 360 or 0 <= angle < 22.5:
        direction = "N"
    elif 22.5 <= angle < 67.5:
        direction = "NE"
    elif 67.5 <= angle < 112.5:
        direction = "E"
    elif 112.5 <= angle < 157.5:
        direction = "SE"
    elif 157.5 <= angle < 202.5:
        direction = "S"
    elif 202.5 <= angle < 247.5:
        direction = "SW"
    elif 247.5 <= angle < 292.5:
        direction = "W"
    elif 292.5 <= angle < 337.5:
        direction = "NW"

    return f"{direction} ({angle:.1f}°)"

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_direction(targetLocation, vehicle):
    while True:
        currentLocation = vehicle.location.global_relative_frame
        if currentLocation and currentLocation.lat and currentLocation.lon:
            direction = get_direction_to_target(targetLocation, currentLocation)
            distance = haversine_distance(
                currentLocation.lat, currentLocation.lon, targetLocation.lat, targetLocation.lon
            )
            print(f"Current Location: ({currentLocation.lat}, {currentLocation.lon})")
            print(f"Target: ({targetLocation.lat}, {targetLocation.lon})")
            print(f"Direction to target: {direction}")
            print(f"Distance: {distance:.2f} meters\n")
            if distance < 5:
                print("✅ Reached waypoint!")
                break
            time.sleep(1)
        else:
            print("Waiting for valid location data...")
            time.sleep(1)

# Pathfinding Functions

def load_graph_from_geojson(geojson_file):
    with open(geojson_file, "r") as f:
        gj = json.load(f)
    gdf = gpd.GeoDataFrame.from_features(gj["features"])
    print("GeoDataFrame head:")
    print(gdf.head())

    G = nx.Graph()
    # Build graph using LineString features
    for idx, row in gdf.iterrows():
        geom = row['geometry']
        if geom.geom_type == 'LineString':
            coords = list(geom.coords)
            for i in range(len(coords) - 1):
                x1, y1 = coords[i]
                x2, y2 = coords[i + 1]
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                G.add_edge((y1, x1), (y2, x2), weight=distance)
    return G

def compute_path(G, start_target, end_target):
    start = min(G.nodes, key=lambda n: (n[0] - start_target[0])**2 + (n[1] - start_target[1])**2)
    end = min(G.nodes, key=lambda n: (n[0] - end_target[0])**2 + (n[1] - end_target[1])**2)
    print(f"Start node: {start}")
    print(f"End node: {end}")
    return nx.shortest_path(G, source=start, target=end, weight='weight')

# Main Executable Section

def main():
    vehicle, args = connectMyCopter()
    G = load_graph_from_geojson(args.geojson)
    print(f"Graph has {len(G.nodes)} nodes and {len(G.edges)} edges")
    
    start_target = (47.7589, -122.1913)
    end_target   = (47.7592, -122.1899)
    path_nodes = compute_path(G, start_target, end_target)
    
    altitude = 2 
    waypoints = [LocationGlobalRelative(lat, lon, altitude) for lat, lon in path_nodes]
    
    print("\nComputed Waypoints:")
    for i, wp in enumerate(waypoints, 1):
        print(f"{i}: ({wp.lat}, {wp.lon}, {wp.alt})")
    
    try:
        for i, waypoint in enumerate(waypoints, 1):
            print(f"\n--- Navigating to waypoint {i} ---")
            calculate_direction(waypoint, vehicle)
        print("\n All waypoints reached successfully!")
    finally:
        vehicle.close()

if __name__ == "__main__":
    main()
