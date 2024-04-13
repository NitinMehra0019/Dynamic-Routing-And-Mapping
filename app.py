
import os
from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
import geopandas as gpd
import json
import math
import heapq
from time import time

# Global Variable
execution_time = 0

# Classes and Functions
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
    print("Inside process nodes function")
    node_id = int(node["osmid"])
    latitude = node.geometry.y
    longitude = node.geometry.x
    new_node = Node(latitude, longitude, str(node_id))
    data_node = NodeData(latitude, longitude)
    node_map[node_id] = data_node

    nodes.append(new_node)
    id_mapping[node_id] = len(nodes) - 1

def process_ways(way, edges, id_mapping):
    print("Inside Process ways function")
    prev_node_id = -1
    for node_id in way["nodes"]:
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

# Dijkstra Algo for Dictionary
def shortest_path(graph, source, destination, num_nodes, nodes):
    if source < 0 or source >= num_nodes or destination < 0 or destination >= num_nodes:
        print("Error: Invalid source or destination index")
        return

    dist = {node_id: float('inf') for node_id in graph}
    visited = {node_id: False for node_id in graph}
    previous = {node_id: -1 for node_id in graph}

    dist[source] = 0
    priority_queue = [(0, source)]

    while priority_queue:
        current_distance, min_index = heapq.heappop(priority_queue)

        if min_index == destination:
            print(f"Shortest Distance: {current_distance}")
            break

        if visited[min_index]:
            continue

        visited[min_index] = True

        for neighbor, weight in graph[min_index].items():
            if not visited[neighbor]:
                new_distance = current_distance + weight
                if new_distance < dist[neighbor]:
                    dist[neighbor] = new_distance
                    previous[neighbor] = min_index
                    heapq.heappush(priority_queue, (new_distance, neighbor))

    else:
        print("Error: No path found from source to destination")
        return jsonify({'error': 'No path found'})

    # Reconstruct the path
    current = destination
    path = []
    while current != source:
        path.append(nodes[current].node_id)
        current = previous[current]

    path.append(nodes[source].node_id)
    path.reverse()

    return path, dist[destination]

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

def create_adjacency_matrix(nodes, edges):
    print("Matrix is ready")
    adjacency_matrix = {}
    
    for edge in edges:
        source_node = nodes[edge.source]
        target_node = nodes[edge.target]
        dist = haversine_distance(source_node, target_node)

        if edge.source not in adjacency_matrix:
            adjacency_matrix[edge.source] = {}

        if edge.target not in adjacency_matrix:
            adjacency_matrix[edge.target] = {}

        adjacency_matrix[edge.source][edge.target] = dist
        adjacency_matrix[edge.target][edge.source] = dist

    return adjacency_matrix

app = Flask(__name__)
CORS(app)



def load_or_parse_data():
    print("Load function")
    graph_data_file = 'graph_data.json'

    if os.path.exists(graph_data_file):
        # Load data from the file
        with open(graph_data_file, 'r') as json_file:
            data = json.load(json_file)
        nodes = [Node(node['latitude'], node['longitude'], node['node_id']) for node in data['nodes']]
        edges = [Edge(edge['source'], edge['target']) for edge in data['edges']]
        id_mapping = {int(node_id): i for i, node_id in enumerate(data['id_mapping'])}
        node_map = {int(node_id): NodeData(node['lat'], node['lon']) for node_id, node in data['node_map'].items()}
    else:
        # Read shapefile using Geopandas
        gdf = gpd.read_file("delhi_highway/delhi_highway.shp")

        nodes = []
        edges = []
        id_mapping = {}
        node_map = {}
        coord_to_node_id = {}  # New mapping to store coordinates to node IDs

        for index, row in gdf.iterrows():
            # Assuming the shapefile has a 'geometry' column representing the LineString
            geometry = row['geometry']
            if geometry.geom_type == 'LineString':
                coords = list(geometry.coords)
                for i, (lon, lat) in enumerate(coords):
                    coord = (lat, lon)
                    if coord not in coord_to_node_id:
                        node_id = f"{index}_{i}"
                        coord_to_node_id[coord] = node_id
                        new_node = Node(lat, lon, node_id)
                        data_node = NodeData(lat, lon)
                        nodes.append(new_node)
                        id_mapping[node_id] = len(nodes) - 1
                        node_map[node_id] = data_node

                for i in range(len(coords) - 1):
                    source_node_id = coord_to_node_id[(coords[i][1], coords[i][0])]
                    target_node_id = coord_to_node_id[(coords[i + 1][1], coords[i + 1][0])]
                    edges.append(Edge(id_mapping[source_node_id], id_mapping[target_node_id]))

        # Write nodes and edges data to a JSON file
        data = {
            'nodes': [{'latitude': node.latitude, 'longitude': node.longitude, 'node_id': node.node_id} for node in nodes],
            'edges': [{'source': edge.source, 'target': edge.target} for edge in edges],
            'id_mapping': [str(node_id) for node_id in id_mapping],
            'node_map': {str(node_id): {'lat': node.lat, 'lon': node.lon} for node_id, node in node_map.items()},
        }

        # Convert keys to integers
        data['id_mapping'] = [int(node_id) for node_id in data['id_mapping']]
        data['node_map'] = {int(node_id): {'lat': node_data['lat'], 'lon': node_data['lon']} for node_id, node_data in data['node_map'].items()}

        with open(graph_data_file, 'w') as json_file:
            json.dump(data, json_file)

    return nodes, edges, id_mapping, node_map

nodes, edges, id_mapping, node_map = load_or_parse_data()

@app.route('/')
def hello_world():
    print("render")
    return render_template('index.html')

@app.route('/get_execution_time', methods=['GET'])
def get_execution_time():
    print("Execution time")
    global execution_time
    return jsonify({'execution_time': execution_time})

@app.route('/process_coordinates', methods=['POST'])
def process_coordinates():
    print("Process Coordinate")
    global execution_time
    
    init = time()
    start_lat = float(request.json.get('sourceLat'))
    start_lon = float(request.json.get('sourceLon'))
    end_lat = float(request.json.get('destLat'))
    end_lon = float(request.json.get('destLon'))

    closest_source_node_id = find_closest_node(nodes, edges, start_lat, start_lon)
    closest_destination_node_id = find_closest_node(nodes, edges, end_lat, end_lon)
    print("Source node : ", closest_source_node_id)
    print("Destination node : ", closest_destination_node_id)

    num_nodes = len(nodes)

    # Creating Adjacency Dictionary
    adjacency_matrix = create_adjacency_matrix(nodes, edges)

    i = 0
    for node, neighbors in adjacency_matrix.items():
        if i == 20:
            break
        print(f"{node} -> {neighbors}")
        print("-------------------------------------------------------------------------------------------------")
        i = i+1
    print("Matrix is ready")

    path, shortest_distance = shortest_path(adjacency_matrix, closest_source_node_id, closest_destination_node_id, num_nodes, nodes)
    execution_time = time() - init
    print("Execution time : ", execution_time)

    path_lat_lon = []

    for node_id in path:
        node = int(node_id)
        lat_lon = get_lat_lon_from_node_id(node, node_map)
        latitude = lat_lon[0]
        longitude = lat_lon[1]
        path_lat_lon.append((latitude, longitude))
        print(f"Node Id : {node_id}")
        print(latitude)
        print(longitude)

    return jsonify({'path': path_lat_lon, 'execution_time': execution_time, 'shortest_distance': shortest_distance})

if __name__ == "__main__":
    app.run(debug=True, port=5000)















"""
import os
from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
import xml.etree.ElementTree as ET
import json
import math
import heapq
from time import time
import cProfile
import numpy as np

# Global Variable 
execution_time = 0

# Classes and Functions
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

# Dijkstra Algo for Dictionary
def shortest_path(graph, source, destination, num_nodes, nodes):
    if source < 0 or source >= num_nodes or destination < 0 or destination >= num_nodes:
        print("Error: Invalid source or destination index")
        return

    dist = {node_id: float('inf') for node_id in graph}
    visited = {node_id: False for node_id in graph}
    previous = {node_id: -1 for node_id in graph}

    dist[source] = 0
    priority_queue = [(0, source)]

    while priority_queue:
        current_distance, min_index = heapq.heappop(priority_queue)

        if min_index == destination:
            print(f"Shortest Distance: {current_distance}")
            break

        if visited[min_index]:
            continue

        visited[min_index] = True

        for neighbor, weight in graph[min_index].items():
            if not visited[neighbor]:
                new_distance = current_distance + weight
                if new_distance < dist[neighbor]:
                    dist[neighbor] = new_distance
                    previous[neighbor] = min_index
                    heapq.heappush(priority_queue, (new_distance, neighbor))

    else:
        print("Error: No path found from source to destination")
        return jsonify({'error': 'No path found'})

    # Reconstruct the path
    current = destination
    path = []
    while current != source:
        path.append(nodes[current].node_id)
        current = previous[current]

    path.append(nodes[source].node_id)
    path.reverse()

    return path





# Dijkstra WOrking on 2d Matrix
# def dijkstra(graph, source, destination, num_nodes, nodes):
#     if source < 0 or source >= num_nodes or destination < 0 or destination >= num_nodes:
#         print("Error: Invalid source or destination index")
#         return

#     dist = [float('inf')] * num_nodes
#     visited = [False] * num_nodes
#     previous = [-1] * num_nodes

#     dist[source] = 0
#     priority_queue = [(0, source)]

#     while priority_queue:
#         current_distance, min_index = heapq.heappop(priority_queue)

#         if min_index == destination:
#             print(f"Shortest Distance: {current_distance}")
#             break

#         if visited[min_index]:
#             continue

#         visited[min_index] = True

#         for k in range(num_nodes):
#             weight = graph[min_index][k]
#             if not visited[k] and weight > 0:
#                 new_distance = current_distance + weight
#                 if new_distance < dist[k]:
#                     dist[k] = new_distance
#                     previous[k] = min_index
#                     heapq.heappush(priority_queue, (new_distance, k))

#     else:
#         print("Error: No path found from source to destination")
#         return jsonify({'error': 'No path found'})

#     # Reconstruct the path
#     current = destination
#     path = []
#     while current != source:
#         path.append(nodes[current].node_id)
#         current = previous[current]

#     path.append(nodes[source].node_id)
#     path.reverse()

#     return path


# def dijkstra(graph, source, destination, num_nodes, nodes):
#     if source < 0 or source >= num_nodes:
#         print("Error: Invalid source index")
#         return

#     if destination < 0 or destination >= num_nodes:
#         print("Error: Invalid destination index")
#         return

#     dist = [float('inf')] * num_nodes
#     visited = [False] * num_nodes
#     previous = [-1] * num_nodes

#     dist[source] = 0

#     for _ in range(num_nodes):
#         min_index = -1
#         min_dist = float('inf')

#         for j in range(num_nodes):
#             if not visited[j] and dist[j] < min_dist:
#                 min_dist = dist[j]
#                 min_index = j

#         if min_index == destination:
#             print(f"Shortest Distance: {dist[min_index]}")
#             break
#         if min_index == -1 or dist[min_index] == float('inf'):
#             print("Error: No path found from source to destination")
#             return jsonify({'error': 'No path found'})

#         visited[min_index] = True

#         for k in range(num_nodes):
#             if not visited[k] and graph[min_index][k] != 0 and dist[min_index] != float('inf') and dist[min_index] + graph[min_index][k] < dist[k]:
#                 dist[k] = dist[min_index] + graph[min_index][k]
#                 previous[k] = min_index

#     # Reconstruct the path
#     current = destination
#     path = []
#     while current != source:
#         path.append(nodes[current].node_id)
#         current = previous[current]

#     path.append(nodes[source].node_id)
#     path.reverse()

#     return path

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

# def save_adjacency_matrix(adjacency_matrix, filename='adjacency_matrix.npy'):
#     np.save(filename, adjacency_matrix)

# def load_adjacency_matrix(filename='adjacency_matrix.npy'):
#      return np.load(filename)



# Optimizing 2D matrix to dictionary 
def create_adjacency_matrix(nodes, edges):
    print("Matrix is ready")
    adjacency_matrix = {}
    
    for edge in edges:
        source_node = nodes[edge.source]
        target_node = nodes[edge.target]
        dist = haversine_distance(source_node, target_node)

        if edge.source not in adjacency_matrix:
            adjacency_matrix[edge.source] = {}

        if edge.target not in adjacency_matrix:
            adjacency_matrix[edge.target] = {}

        adjacency_matrix[edge.source][edge.target] = dist
        adjacency_matrix[edge.target][edge.source] = dist

    return adjacency_matrix

app = Flask(__name__)
CORS(app)



def load_or_parse_data():
    print("Load function")

    graph_data_file = 'graph_data.json'

    # adjacency_matrix_file = 'adjacency_matrix.npy'


    if os.path.exists(graph_data_file):
    # and os.path.exists(adjacency_matrix_file):
        # Load data from the file

        with open(graph_data_file, 'r') as json_file:
            data = json.load(json_file)
        nodes = [Node(node['latitude'], node['longitude'], node['node_id']) for node in data['nodes']]
        edges = [Edge(edge['source'], edge['target']) for edge in data['edges']]
        id_mapping = {int(node_id): i for i, node_id in enumerate(data['id_mapping'])}
        node_map = {int(node_id): NodeData(node['lat'], node['lon']) for node_id, node in data['node_map'].items()}
        # adjacency_matrix = load_adjacency_matrix()

    else:

        # Parse OSM file and process data
        tree = ET.parse("ina_2.osm")
        root = tree.getroot()

        nodes = []
        edges = []
        id_mapping = {}
        node_map = {}

        # Process nodes and ways
        for node in root.findall(".//node"):
            process_nodes(node, nodes, id_mapping, 0, node_map)

        for way in root.findall(".//way"):
            process_ways(way, edges, id_mapping)

        # Mapping of ways and creating an Adjacency Matrix with weighted edges
        # num_nodes = len(nodes)
        # adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]

        # for edge in edges:
        #     dist = haversine_distance(nodes[edge.source], nodes[edge.target])
        #     adjacency_matrix[edge.source][edge.target] = dist
        #     adjacency_matrix[edge.target][edge.source] = dist
        

        # Write nodes and edges data to a JSON file
        # Write nodes and edges data to a JSON file
        data = {
            'nodes': [{'latitude': node.latitude, 'longitude': node.longitude, 'node_id': node.node_id} for node in nodes],
            'edges': [{'source': edge.source, 'target': edge.target} for edge in edges],
            'id_mapping': [str(node_id) for node_id in id_mapping],
            'node_map': {str(node_id): {'lat': node.lat, 'lon': node.lon} for node_id, node in node_map.items()},
            }

            # Convert keys to integers
        data['id_mapping'] = [int(node_id) for node_id in data['id_mapping']]
        data['node_map'] = {int(node_id): {'lat': node_data['lat'], 'lon': node_data['lon']} for node_id, node_data in data['node_map'].items()}

        with open(graph_data_file, 'w') as json_file:
            json.dump(data, json_file)
         # Save adjacency matrix
        # save_adjacency_matrix(adjacency_matrix)

    return nodes, edges, id_mapping, node_map
# , adjacency_matrix



nodes, edges, id_mapping, node_map = load_or_parse_data()

@app.route('/')
def hello_world():
    print("render")
    return render_template('index.html')


@app.route('/get_execution_time', methods=['GET'])
def get_execution_time():
    print("Execution time")
    global execution_time
    return jsonify({'execution_time': execution_time})

@app.route('/process_coordinates', methods=['POST'])
def process_coordinates():
    print("Process Coordinate")
    global execution_time
    
    init = time()
    start_lat = float(request.json.get('sourceLat'))
    start_lon = float(request.json.get('sourceLon'))
    end_lat = float(request.json.get('destLat'))
    end_lon = float(request.json.get('destLon'))

    closest_source_node_id = find_closest_node(nodes, edges, start_lat, start_lon)
    closest_destination_node_id = find_closest_node(nodes, edges, end_lat, end_lon)
    print("Source node : ", closest_source_node_id)
    print("Destination node : ", closest_destination_node_id)

    num_nodes = len(nodes)
    # adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]
    # # num_nodes = len(nodes)
    # # adjacency_matrix = load_adjacency_matrix()

    # for edge in edges:
    #     dist = haversine_distance(nodes[edge.source], nodes[edge.target])
    #     adjacency_matrix[edge.source][edge.target] = dist
    #     adjacency_matrix[edge.target][edge.source] = dist
    
    
    
    #Creating Adjacency Dictionary 
    adjacency_matrix = create_adjacency_matrix(nodes, edges)

    print("Matrix is ready")

    path = shortest_path(adjacency_matrix, closest_source_node_id, closest_destination_node_id, num_nodes, nodes)
    execution_time = time() - init
    print("Execution time : ", execution_time)
    # print("Execution time : ", time()-init)

    path_lat_lon = []

    for node_id in path:
        node = int(node_id)
        lat_lon = get_lat_lon_from_node_id(node, node_map)
        latitude = lat_lon[0]
        longitude = lat_lon[1]
        path_lat_lon.append((latitude, longitude))
        print(f"Node Id : {node_id}")

   

    return jsonify({'path': path_lat_lon, 'execution_time' : execution_time})

if __name__ == "__main__":
    app.run(debug=True, port=8000)
    
"""