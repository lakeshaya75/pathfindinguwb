import json
import geopandas as gpd
import networkx as nx

geojson_file = "uwb.geojson" # file of UWB

with open(geojson_file, "r") as f:
    gj = json.load(f)

gdf = gpd.GeoDataFrame.from_features(gj["features"])

print(gdf.head())

G = nx.Graph()

for idx, row in gdf.iterrows():
    geom = row['geometry']
    if geom.geom_type == 'LineString':
        coords = list(geom.coords)
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]
            distance = ((x2-x1)**2 + (y2-y1)**2) ** 0.5
            G.add_edge((y1, x1), (y2, x2), weight=distance)

print(f"Graph has {len(G.nodes)} nodes and {len(G.edges)} edges") # testing

start = min(G.nodes, key=lambda n: (n[0]-47.7589)**2 + (n[1]+122.1913)**2)
end = min(G.nodes, key=lambda n: (n[0]-47.7592)**2 + (n[1]+122.1899)**2)

print(f"Start node: {start}")
print(f"End node: {end}")

path = nx.shortest_path(G, source=start, target=end, weight='weight')

print("\nWaypoints:")
for idx, (lat, lon) in enumerate(path, 1):
    print(f"{idx}: ({lat}, {lon})")
