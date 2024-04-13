import xml.etree.ElementTree as ET
import math
from typing import List, Tuple, Dict
import time
import cProfile

start_time = time.time()


class Node:
    def __init__(self, latitude: float, longitude: float, node_id: str):
        self.latitude = latitude
        self.longitude = longitude
        self.node_id = node_id

class NodeData:
    def __init__(self, lat: float, lon: float):
        self.lat = lat
        self.lon = lon

class Edge:
    def __init__(self, source: int, target: int):
        self.source = source
        self.target = target

def process_nodes(node, nodes, id_mapping, id_map, node_map):
    node_id = int(node.attrib["id"])
    latitude = float(node.attrib["lat"])
    longitude = float(node.attrib["lon"])
    new_node = Node(latitude, longitude, str(node_id))
    data_node = NodeData(latitude, longitude)
    node_map[node_id] = data_node
    nodes.append(new_node)
    id_mapping[node_id] = len(nodes) - 1

def process_ways(way, edges, id_mapping):
    prev_node_id = -1
    for nd in way.findall("nd"):
        node_id = int(nd.attrib["ref"])
        if prev_node_id != -1:
            edges.append(Edge(id_mapping[prev_node_id], id_mapping[node_id]))
        prev_node_id = node_id

def get_lat_lon_from_node_id(node_id, node_map):
    if node_id in node_map:
        node_data = node_map[node_id]
        return node_data.lat, node_data.lon
    else:
        return 0.0, 0.0  # Default values if node_id not found

def haversine_distance(source, target):
    PI = 3.14159265
    earth_radius = 6371.0  # Earth radius in kilometers

    lat1_rad = source.latitude * PI / 180.0
    lon1_rad = source.longitude * PI / 180.0
    lat2_rad = target.latitude * PI / 180.0
    lon2_rad = target.longitude * PI / 180.0

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) * math.sin(dlon / 2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return earth_radius * c  # Distance in kilometers

def dijkstra(graph, source, destination, num_nodes, nodes):
    if source < 0 or source >= num_nodes:
        print("Error: Invalid source index")
        return

    if destination < 0 or destination >= num_nodes:
        print("Error: Invalid destination index")
        return

    dist = [float('inf')] * num_nodes
    visited = [False] * num_nodes
    previous = [-1] * num_nodes

    dist[source] = 0

    for _ in range(num_nodes):
        min_index = -1
        min_dist = float('inf')

        for j in range(num_nodes):
            if not visited[j] and dist[j] < min_dist:
                min_dist = dist[j]
                min_index = j

        if min_index == destination:
            print(f"Shortest Distance: {dist[min_index]}")
            break

        if min_index == -1:
            break

        visited[min_index] = True

        for k in range(num_nodes):
            if not visited[k] and graph[min_index][k] != 0 and dist[min_index] != float('inf') and dist[min_index] + graph[min_index][k] < dist[k]:
                dist[k] = dist[min_index] + graph[min_index][k]
                previous[k] = min_index

    # Reconstruct the path
    current = destination
    path = []
    while current != source:
        path.append(nodes[current].node_id)
        current = previous[current]

    path.append(nodes[source].node_id)
    path.reverse()

    return path

# def find_closest_node(nodes, latitude, longitude):
#     print("Getting the closest Nodes .. ")
#     closest_node_id = -1
#     min_distance = float('inf')

#     for i, node in enumerate(nodes):
#         distance = haversine_distance(node, Node(latitude, longitude, ""))
#         if distance < min_distance:
#             min_distance = distance
#             closest_node_id = i

#     return closest_node_id

def find_closest_node(nodes, edges, latitude, longitude):
    print("Getting the closest Nodes .. ")
    closest_node_id = -1
    min_distance = float('inf')

    for edge in edges:
        source_node = nodes[edge.source]
        # Calculate the distance from the point to the source node of the edge
        distance = haversine_distance(source_node, Node(latitude, longitude, ""))
        if distance < min_distance:
            min_distance = distance
            closest_node_id = edge.source

    return closest_node_id

def main():
    # Load OSM file using ElementTree
    tree = ET.parse("ina_2.osm")
    root = tree.getroot()

    nodes = []
    edges = []
    id_mapping = {}
    node_map = {}

    # Process nodes and ways
    for node in root.findall(".//node"):
        process_nodes(node, nodes, id_mapping, 0, node_map)

    print("Number of Nodes:", len(nodes))

    for way in root.findall(".//way"):
        process_ways(way, edges, id_mapping)

    print("Number of Edges:", len(edges))

    # Mapping of ways and creating an Adjacency Matrix with weighted edges
    num_nodes = len(nodes)
    adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]

    for edge in edges:
        dist = haversine_distance(nodes[edge.source], nodes[edge.target])
        adjacency_matrix[edge.source][edge.target] = dist
        adjacency_matrix[edge.target][edge.source] = dist

    # Example: Find the closest nodes to the provided latitude and longitude for source and destination
    source_latitude = 28.5745268
    source_longitude = 77.2149880
    destination_latitude = 28.5741560
    destination_longitude = 77.2120026
    # double sourceLatitude = 28.5745268;
    # double sourceLongitude = 77.2149880;
    # double destinationLatitude = 28.5741560;
    # double destinationLongitude = 77.2120026;

    closest_source_node_id = find_closest_node(nodes,edges, source_latitude, source_longitude)
    closest_destination_node_id = find_closest_node(nodes,edges, destination_latitude, destination_longitude)
    print("Closest Source Node:", closest_source_node_id)
    # print("Closest Node Id ", id_mapping[closest_source_node_id] )
    print("Closest Destination Node:", closest_destination_node_id)
    # print("Closest Node Id ", id_mapping[closest_destination_node_id] )


    # Run Dijkstra's algorithm
    path = dijkstra(adjacency_matrix, closest_source_node_id, closest_destination_node_id, num_nodes, nodes)

    # Display the shortest path
    print(f"Shortest Path from Node {closest_source_node_id} to Node {closest_destination_node_id}:")
    for node_id in path:
        print(node_id, "->")
    
    cnt = 0
    new_cnt = 0
    for node in id_mapping:
        if cnt == 17146:
            print(node , id_mapping[node])
        if new_cnt == 16759:
            print(node, id_mapping[node])
        cnt = cnt + 1
        new_cnt = new_cnt + 1

        


    print("\n")
    for node_id in path:
        path_lat_lon = []
        node = int(node_id)
        lat_lon = get_lat_lon_from_node_id(node, node_map)
        latitude = lat_lon[0]
        longitude = lat_lon[1]
        path_lat_lon.append((latitude, longitude))
        # print(f"Node ID: {node_id} Latitude: {latitude}, Longitude: {longitude}")

        # for i in range(len(path_lat_lon)):
            
            # print(path_lat_lon[i][0], "----", path_lat_lon[i][1])
            
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Total execution time: {elapsed_time} seconds")

if __name__ == "__main__":
    main()
    
cProfile.run('main()')

