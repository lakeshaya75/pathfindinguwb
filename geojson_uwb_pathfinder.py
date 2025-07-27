import json
import networkx as nx
import geopandas as gpd
import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative

def haversine_distance(lat1, lon1, lat2, lon2):
    from math import radians, cos, sin, sqrt, atan2
    R = 6371e3
    φ1, φ2 = radians(lat1), radians(lat2)
    Δφ = radians(lat2 - lat1)
    Δλ = radians(lon2 - lon1)

    a = sin(Δφ / 2)**2 + cos(φ1) * cos(φ2) * sin(Δλ / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def load_graph_from_geojson(geojson_file):
    with open(geojson_file, "r") as f:
        gj = json.load(f)
    gdf = gpd.GeoDataFrame.from_features(gj["features"])
    print("GeoDataFrame head:")
    print(gdf.head())

    G = nx.Graph()
    for idx, row in gdf.iterrows():
        geom = row['geometry']
        if geom.geom_type == 'LineString':
            coords = list(geom.coords)
            for i in range(len(coords) - 1):
                x1, y1, *z1 = coords[i]
                x2, y2, *z2 = coords[i + 1]
                ele1 = z1[0] if z1 else 0
                ele2 = z2[0] if z2 else 0
                distance = haversine_distance(y1, x1, y2, x2)
                G.add_edge((y1, x1, ele1), (y2, x2, ele2), weight=distance)
    return G

def compute_path(G, start_target, end_target):
    def dist2D(n, target):
        return (n[0] - target[0])**2 + (n[1] - target[1])**2

    start = min(G.nodes, key=lambda n: dist2D(n, start_target))
    end = min(G.nodes, key=lambda n: dist2D(n, end_target))
    print(f"Start node: {start}")
    print(f"End node: {end}")
    return nx.shortest_path(G, source=start, target=end, weight='weight')

def arm_and_drive(vehicle):
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Vehicle armed and in GUIDED mode")

def goto_location(vehicle, location):
    print(f"Going to: {location.lat}, {location.lon}")
    vehicle.simple_goto(location)
    time.sleep(5)  

def main():
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    print("Connected to vehicle")

    graph = load_graph_from_geojson("uwb.geojson")

    start_coord = (47.758, -122.191)  # example
    end_coord = (47.760, -122.188)

    path_nodes = compute_path(graph, start_coord, end_coord)
    print("Computed path:", path_nodes)

    waypoints = [
        LocationGlobalRelative(lat, lon, 0) 
        for lat, lon, _ in path_nodes
    ]

    arm_and_drive(vehicle)
    for wp in waypoints:
        goto_location(vehicle, wp)

    print("Path completed.")
    vehicle.mode = VehicleMode("HOLD")
    vehicle.close()

if __name__ == "__main__":
    main()
