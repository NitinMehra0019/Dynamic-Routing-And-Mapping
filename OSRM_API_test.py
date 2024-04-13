import requests

# Replace these coordinates with your own start and end points
start_point = "77.1556091309,28.7514065765"  # Latitude, Longitude
end_point = "77.2119140625,28.6267198751"    # Latitude, LongitudeMumbai, Maharashtra

# Specify the OSRM API endpoint
osrm_endpoint = "https://router.project-osrm.org/route/v1/driving/"

# Construct the request URL
request_url = f"{osrm_endpoint}{start_point};{end_point}?steps=true&geometries=geojson"

# Make the request
response = requests.get(request_url)

# Check if the request was successful (status code 200)
if response.status_code == 200:
    # Parse the JSON response
    route_data = response.json()

    # Extract relevant information, e.g., duration and distance
    duration = route_data['routes'][0]['duration']
    distance = route_data['routes'][0]['distance']

    print(f"Duration: {duration} seconds")
    print(f"Distance: {distance} meters")
    
    path_coordinates = route_data['routes'][0]['geometry']['coordinates']
    print("Path Coordinates:")
    for coordinate in path_coordinates:
        print(coordinate)

    # You can also extract and use other information like the detailed geometry or steps
else:
    # Print an error message if the request was not successful
    print(f"Error: {response.status_code}")
    print(response.text)