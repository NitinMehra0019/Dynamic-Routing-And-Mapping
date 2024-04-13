
from http.client import REQUEST_URI_TOO_LONG
import json
import os
import time
from urllib import response
from flask import Flask, render_template, jsonify, request
from flask_caching import Cache
from flask_socketio import SocketIO, emit
import requests
import memcache
import polyline
import socket
import struct
import threading
from threading import Lock
import ctypes
import asyncio
import websockets
import timeit
import matplotlib
from memory_profiler import profile

# Define the BasePacket structure
# class BasePacket(ctypes.Structure):
#     _fields_ = [
#         ("crc32", ctypes.c_uint32),
#         ("count", ctypes.c_uint16),
#         ("version", ctypes.c_uint16),
#         ("timestamp", ctypes.c_uint64),
#         ("payload", ctypes.c_char * 64000)
#     ]

# # Define the PDUMessage structure
# class PDUMessage(ctypes.Structure):
#     _fields_ = [
#         ("messageID", ctypes.c_uint),
#         ("messageLength", ctypes.c_uint),
#         ("basepacket", BasePacket)
#     ]




thread = None
thread_lock = Lock()

app = Flask(__name__)
app.config['SECRET_KEY'] = 'Valhalla!'

socketio = SocketIO(app, cors_allowed_origins="*")
# socketio = SocketIO(app)



# Global variables for UDP receiver
RECEIVER_PORT = 17781

# RECEIVER_PORT = 17782

BUFFER_SIZE = 1024

latitude = None
longitude = None

def clean_gps_data_file():
    with open("./static/gps_data.txt", "w") as file:
        file.write("")  # Clearing the file contents
        print("Cleared gps data")
        
# Function to receive data via UDP socket
def receive_data_via_udp():
    # Create a UDP socket
    print("Inside reciver function ")
    clean_gps_data_file()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind the socket to the port
    server_address = ('', RECEIVER_PORT)
    sock.bind(server_address)
    print("Receiver is listening on port", RECEIVER_PORT)
    while True:
        print("\nWaiting to receive message...")
        data, address = sock.recvfrom(80)
        
        
        print("Received message from:", address)
        # Unpack the received data according to the C++ struct
        # message_id, message_length, crc32, count, version, timestamp = struct.unpack('IIIHHL', data[:24])
        payload = data
        # print("Header Values : \n")
        # print("Message Id : ", message_id)
        # print("Message Length : ",message_length)
        # print("Count : ", count)
        # print("crc32 : ", crc32)
        # print("timestamp : ", timestamp)
        # print("Version : ", version)


        
        
        # print("Payload : ", payload)
        print("-------------------------------------------------------------------------------------------------")
        # Extract GPS packet from payload
        # gps_packet = struct.unpack('dbdbddd', payload[:56])  # Assuming gps_packet_t is 56 bytes
        gps_packet = struct.unpack('dbdbddd', payload[:56])  # Assuming gps_packet_t is 56 bytes
        # gps_packet = struct.unpack('ddbddbddd', payload[:80])  # Assuming gps_packet_t is 56 bytes


        latitude = str(gps_packet[0])
        latitude_decimal = float(latitude[:2]) + float(latitude[2:]) / 60
        
        
        latitude_indicator = chr(gps_packet[1])
        
        longitude = str(gps_packet[2])
        longitude_decimal = float(longitude[:2]) + float(longitude[2:]) / 60

        
        longitude_indicator = chr(gps_packet[3])

        # dilution_of_precision = gps_packet[4]
        # horizontal_dilution = gps_packet[5]
        # vertical_dilution = gps_packet[6]
        
         
        # Print received GPS data
        print("\nReceived GPS Data:")
        print("Latitude:", latitude, latitude_indicator)
        print("Longitude:", longitude, longitude_indicator)
        # print("Dilution of Precision:", gps_packet[4])
        # print("Horizontal Dilution of Precision:", gps_packet[5])
        # print("Vertical Dilution of Precision:", gps_packet[6])
        
        with open("./static/gps_data.txt", "a") as file:
            file.write(f"[{latitude}, {longitude}],\n")
            print("write complete")
        # return jsonify(data)
        # socketio.emit('gps_data', {'latitude': latitude,'latitude_indicator':latitude_indicator, 'longitude': longitude, 'longitude_indicator': longitude_indicator})
        socketio.emit('gps_data', {'latitude': latitude_decimal,'latitude_indicator':latitude_indicator, 'longitude': longitude_decimal, 'longitude_indicator': longitude_indicator})

        socketio.sleep(1)
        # print("Data sent to webpage")


# Start UDP receiver thread
# udp_receiver_thread = threading.Thread(target=receive_data_via_udp)
# udp_receiver_thread.daemon = True
# udp_receiver_thread.start()


# Function to send data via UDP socket
def send_data_via_udp(data):
    
    # Set the destination address and port
    # dest_address = '127.0.0.1'  # Destination IP address (localhost)
    # dest_port = 12345  # Destination port
    
    
    
    # UDP server address and port
    # udp_server_address = ('192.168.68.133', 17782)  # Change this to your destination UDP server address and port
    udp_server_address = ('127.0.0.1', 17780)  # Change this to your destination UDP server address and port
    
    # udp_server_address = ('192.168.98.16', 17780)  # Change this to your destination UDP server address and port

    

    # Create a UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Convert data to JSON string
        # json_data = json.dumps(data)
        
        new_json_data = data
           
        # Send data via UDP socket
        udp_socket.sendto(new_json_data.encode(), udp_server_address)
        # udp_socket.sendto(json_data, udp_server_address)
        
        
        print("Data sent via UDP successfully.")
    except Exception as e:
        print("Error while sending data via UDP:", e)
    finally:
        # Close the UDP socket
        udp_socket.close()

# Start UDP receiver thread
# udp_sender_thread = threading.Thread(target=send_data_via_udp)
# udp_sender_thread.daemon = True
# udp_sender_thread.start()



# Memchache function to check Memcache data
def memcache_data():
    print("Inside Memecached Data")
    client = memcache.Client(['localhost:11211'])

    # Get general statistics
    stats = client.get_stats()

    # Check if stats is not empty
    if stats:
        # Iterate through all the items in stats
        for stat_item in stats:
            # Extract the 'total_items' field from each item
            total_items = stat_item[1].get('total_items', 0)

            # Retrieve and print all data for each key
            for i in range(total_items):
                key = f'key-{i+1}'
                data = client.get(key)
                print(f'Data for key {key}: {data}')
    else:
        print("No stats available")


# Configure Flask-Caching to use Memcached
app.config['CACHE_TYPE'] = 'memcached'
app.config['CACHE_MEMCACHED_SERVERS'] = ['127.0.0.1:11211']  # Defualt Memcached port
app.config['CACHE_KEY_PREFIX'] = 'my_cache_'

# Initialize the cache
cache = Cache(app)



# End point to render the webpage (.html file)
@app.route('/')
def index():
    # return render_template('newindex.html')
    return render_template('mapbox.html')



# End point where the coordinate are recieved from the user

# @profile
@app.route('/process_coordinates', methods=['POST'])
def process_coordinates():
    start = time.time()
    
    print("Inside Process Coordinates")
    # Deserializing the input coordinates from string to Float
    start_lat = float(request.json.get('sourceLat'))
    start_lon = float(request.json.get('sourceLon'))
    end_lat = float(request.json.get('destLat'))
    end_lon = float(request.json.get('destLon'))

    
    # Check if the result is already in the cache
    cache_key = f"{start_lat}_{start_lon}_{end_lat}_{end_lon}"
    cached_result = cache.get(cache_key)
    # send_data_via_udp(cached_result)

    print("Cache Key", cache_key)
    
    # If the result is in cache result is directly given without server call
    if cached_result:
        print("Result retrieved from cache")
        # data_from_cache = json.dumps(cached_result)
        
        # print("Type of cache result", type(cached_result))
        cached_data = json.dumps(cached_result)
        
        # print("Type of Cached data ", type(cached_data))
        
        #this is not working as of now
        send_data_via_udp(cached_data)
        cache.delete(cache_key)
        return jsonify(cached_result)
    
    locations = "locations"
    
    request_url = f"http://localhost:8002/route?json={{\"{locations}\": [{{\"lat\": {start_lat}, \"lon\": {start_lon}}}, {{\"lat\": {end_lat}, \"lon\": {end_lon}}}], \"costing\": \"auto\"}}"
    # request_url = f"http://localhost:8002/route?json={{\"{locations}\": [{{\"lat\": {start_lat}, \"lon\": {start_lon}}}, {{\"lat\": {end_lat}, \"lon\": {end_lon}}}], \"costing\": \"pedestrian\"}}"

    
    print(request_url)
   
    
    # Make the request
    response = requests.get(request_url)
    

    
    # Check if the request was successful (status code 200)
 
    if response.status_code == 200:
        # Accessing the JSON content of the response
        data = response.json()
        # print("Data Length : ", len(data))
        # print(data)
        
        
        

    
        # Accessing the shape string from the JSON
        shape_string = data['trip']['legs'][0]['shape']

        distance = data['trip']['summary']['length']
        time_sec = data['trip']['summary']['time']
        instruction = data['trip']['legs'][0]['maneuvers']
        
        print(distance)

        #decode in the polyline to get [Laitude , Longitude]
                                      #(string , number of decimal point needed after whole number)
        coordinates = polyline.decode(shape_string, 6) 
        path_size = len(coordinates)
        print("PAth size : ", path_size)
        instruction_size = len(instruction)
        print("Instrcution size : ", instruction_size)

   
        # print("Coordinate length : ", (coordinates))
        # print(coordinates)
        
        # Send data via UDP
        
        
         # Store the result in the cache
        cache.set(cache_key, {'path_size': path_size, 'instruction_size': instruction_size, 'path': coordinates, 'distance': distance, 'instruction': instruction, 'time': time_sec}, timeout = 5)
        
        data = {'path_size': path_size, 'instruction_size': instruction_size, 'path': coordinates, 'distance': distance, 'instruction': instruction, 'time': time_sec}  
        json_data = json.dumps(data)
        data_length = len(json_data)      
    
        data_to_send = {'data_length': data_length, 'path_size': path_size, 'instruction_size': instruction_size, 'path': coordinates, 'distance': distance, 'instruction': instruction, 'time': time_sec}
        # Populate BasePacket fields with the sample data
        # Serialize data_to_send dictionary to JSON string
        json_payload = json.dumps(data_to_send)
        
        
        # Convert JSON string to bytes
        payload_bytes = json_payload.encode()
        # print("Payload Bytes : ", payload_bytes)
        
        # base_packet = BasePacket(
        #     crc32=0,  # Calculate CRC32 if needed
        #     count=0,
        #     version=0,
        #     timestamp=0,
        #     payload=payload_bytes  # Assuming coordinates are bytes
        # )
        
        # Initialize PDUMessage with the sample data
        # pdu_message = PDUMessage(
        #     messageID=2164260865,  # You can set an appropriate message ID
        #     messageLength=ctypes.sizeof(BasePacket),  # Assuming fixed length
        #     basepacket=base_packet
        # )
        
        # pdu_message_bytes = ctypes.string_at(ctypes.addressof(pdu_message), ctypes.sizeof(pdu_message))
        
        # print("Pdu Message : ", pdu_message_bytes)
        # print("Length : ", len(pdu_message_bytes))

        # create_pduMessage(pdu_message_bytes)
        # data_to_send = data
        # send_data_via_udp(json_data)
        


        # return jsonify({'path': coordinates, 'distance': distance, 'instruction': instruction, 'time': time})
        end = time.time()
        exe = end - start
        print("Execution time : ", exe, " seconds " )
        return jsonify(data_to_send)

    return jsonify({'error': 'Failed to retrieve route data'}), 500



# End point to get the traffic information (Not using right now)    
@app.route('/processTraffic', methods=['POST'])   
def processTraffic():
    mapbox_api_key = 'sk.eyJ1IjoibWVocmEwMDE5IiwiYSI6ImNscnFjNm8yYjAxbWQycW5jNXJ6MHdzYmsifQ.uF1ogfIE83KheurGEzj1Dw'
    # Extract start_lat and start_lon from the request data
    start_lat = float(request.json.get('sourceLat'))
    start_lon = float(request.json.get('sourceLon'))
    end_lat = float(request.json.get('destLat'))
    end_lon = float(request.json.get('destLon'))
    
    # Mapbox Traffic API endpoint

    endpoint = "https://api.mapbox.com/directions/v5/mapbox/driving-traffic"
    # Convert coordinates to a string format for the API
    # coordinates_str = ";".join([f"{lng},{lat}" for lat, lng in route_coordinates])
    
     # Build the API request URL
    url = f"{endpoint}/{start_lon},{start_lat};{end_lon},{end_lat}"
    
    params = {
        "alternatives": "true",  # Enable alternative routes
        "geometries": "geojson",  # Get the route geometry in GeoJSON format
        "steps": "true",  # Include turn-by-turn instructions
        "access_token": mapbox_api_key,
    }
    
    print(url)
    
    # Make the API request
    response = requests.get(url, params=params)
    
    if response.status_code == 200:
        traffic = response.json()
        return jsonify({'traffic': traffic})
    else:
        print(f"Error: {response.status_code}")
        print(response.text)
        return None 
      
 
@socketio.on('connect')
def connect():
    global thread
    print('Client connected')

    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(receive_data_via_udp) 
      
@socketio.on('disconnect')
def disconnect():
    print('Client disconnected',  request.sid)
    

# Defining port
if __name__ == '__main__':
    clean_gps_data_file()


    # app.run(debug=True, port=8000)
    socketio.run(app, debug=True, port=8000)
        # app.run(debug=True, host= '192.168.1.23')
