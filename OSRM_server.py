
import os
from flask import Flask, render_template, jsonify, request
from flask_caching import Cache
import requests
import memcache
app = Flask(__name__)

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
            jsonify({})
            # Retrieve and print all data for each key
            for i in range(total_items):
                key = f'key-{i+1}'
                data = client.get(key)
                print(f'Data for key {key}: {data}')
    else:
        print("No stats available")


# Configure Flask-Caching to use Memcached
app.config['CACHE_TYPE'] = 'memcached'
app.config['CACHE_MEMCACHED_SERVERS'] = ['127.0.0.1:11211']  # Adjust this based on your Memcached server configuration
app.config['CACHE_KEY_PREFIX'] = 'my_cache_'

# Initialize the cache
cache = Cache(app)


@app.route('/')
def index():
    # return render_template('newindex.html')
    return render_template('geocode.html')



@app.route('/process_coordinates', methods=['POST'])
def process_coordinates():
    print("Inside Process Coordinates")
    start_lat = float(request.json.get('sourceLat'))
    start_lon = float(request.json.get('sourceLon'))
    end_lat = float(request.json.get('destLat'))
    end_lon = float(request.json.get('destLon'))
    
    
    # Check if the result is already in the cache
    cache_key = f"{start_lat}_{start_lon}_{end_lat}_{end_lon}"
    cached_result = cache.get(cache_key)
    print("Cache Key", cache_key)
    

    if cached_result:
        print("Result retrieved from cache")
        return jsonify(cached_result)
    
    
    
    
    # # Specify the OSRM API endpoint
    osrm_endpoint = "https://router.project-osrm.org/route/v1/driving/"

    # # Construct the request URL
    request_url = f"{osrm_endpoint}{start_lon},{start_lat};{end_lon},{end_lat}?steps=true&geometries=geojson"
    
    # # Make the request
    response = requests.get(request_url)

     # # Check if the request was successful (status code 200)
 
    if response.status_code == 200:
    # Parse the JSON response
        route_data = response.json()

    # Extract relevant information, e.g., duration and distance
        duration = route_data['routes'][0]['duration'] 
        distance = route_data['routes'][0]['distance']
        
        new_distance = distance / 1000
        print(f"Duration: {duration} seconds")
        print(f"Distance: {distance} meters")
        
        path_coordinates = [] 
        # path = path_coordinates.append

        # Iterate through all coordinates in the response
        for leg in route_data['routes'][0]['legs']:
            for step in leg['steps']:
                path_coordinates.extend(step['geometry']['coordinates'])

        # Print or use the path_coordinates as needed
        print("Path Coordinates:")
        for coordinate in path_coordinates:
            print(coordinate)
            
            
         # Store the result in the cache
        cache.set(cache_key, {'duration': duration, 'distance': new_distance, 'path': path_coordinates})

        # memcache_data()
     
        
        return jsonify({'duration': duration, 'distance': new_distance, 'path': path_coordinates})
    return jsonify({'error': 'Failed to retrieve route data'}), 500




if __name__ == '__main__':
    app.run(debug=True, port=5000)
    

































# import requests

# # Replace these coordinates with your own start and end points
# start_point = "77.2101438046,28.5743888140"  # Latitude, Longitude
# end_point = "77.2194242477,28.6328598007"    # Latitude, LongitudeMumbai, Maharashtra

# # Specify the OSRM API endpoint
# osrm_endpoint = "https://router.project-osrm.org/route/v1/driving/"

# # Construct the request URL
# request_url = f"{osrm_endpoint}{start_point};{end_point}?steps=true&geometries=geojson"

# # Make the request
# response = requests.get(request_url)

# # Check if the request was successful (status code 200)
# if response.status_code == 200:
#     # Parse the JSON response
#     route_data = response.json()

#     # Extract relevant information, e.g., duration and distance
#     duration = route_data['routes'][0]['duration']
#     distance = route_data['routes'][0]['distance']

#     print(f"Duration: {duration} seconds")
#     print(f"Distance: {distance} meters")
    
#     path_coordinates = route_data['routes'][0]['geometry']['coordinates']
#     print("Path Coordinates:")
#     for coordinate in path_coordinates:
#         print(coordinate)

#     # You can also extract and use other information like the detailed geometry or steps
# else:
#     # Print an error message if the request was not successful
#     print(f"Error: {response.status_code}")
#     print(response.text)

# server.py