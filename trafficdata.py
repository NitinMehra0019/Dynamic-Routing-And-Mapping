
import requests

# Replace with your actual TomTom API key
api_key = "JLwGzAAA8EAxA5r6TBbOZknwJkbvusYy"

# Replace with your start and end location coordinates
start_location = "28.598283, 77.256178"
end_location = "28.603434, 77.268467"

# TomTom Traffic API endpoint URL
api_url = f"https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json?key={api_key}&point=28.598283,77.256178"

# Parameters for the request
# params = {
#     "key": api_key,
#     "computeBestOrder": "true",  # Optional parameter for optimizing the route order
# }

# Make the API request
response = requests.get(api_url)

# Check if the request was successful (status code 200)
if response.status_code == 200:
    # Parse the JSON response
    data = response.json()
    
    # Access the relevant information from the response
    # route_summary = data["routes"][0]["summary"]
    # travel_time_seconds = route_summary["totalTimeSeconds"]
    # travel_distance_km = route_summary["totalLength"]

    # print(f"Estimated travel time: {travel_time_seconds} seconds")
    # print(f"Estimated travel distance: {travel_distance_km} kilometers")
    
    # traffic = data['flowSegmentData']['coordinates']['coordinate']

    
    
    print("Data : ", data)
else:
    print(f"Error {response.status_code}: {response.text}")
    
    
    
    # MApbox api = sk.eyJ1IjoibWVocmEwMDE5IiwiYSI6ImNscnFjNm8yYjAxbWQycW5jNXJ6MHdzYmsifQ.uF1ogfIE83KheurGEzj1Dw
    
"""
import requests

def get_mapbox_traffic(route_coordinates, api_key):
    # Mapbox Traffic API endpoint
    endpoint = "https://api.mapbox.com/directions/v5/mapbox/driving-traffic"

    # Convert coordinates to a string format for the API
    coordinates_str = ";".join([f"{lng},{lat}" for lat, lng in route_coordinates])

    # Build the API request URL
    url = f"{endpoint}/{coordinates_str}"

    # Set up query parameters
    params = {
        "alternatives": "true",  # Enable alternative routes
        "geometries": "geojson",  # Get the route geometry in GeoJSON format
        "steps": "true",  # Include turn-by-turn instructions
        "access_token": api_key,
    }
    print(url)

    # Make the API request
    response = requests.get(url, params=params)

    # Check if the request was successful (status code 200)
    if response.status_code == 200:
        return response.json()
    else:
        # Print an error message if the request was not successful
        print(f"Error: {response.status_code}")
        print(response.text)
        return None

if __name__ == "__main__":
    # Replace 'YOUR_MAPBOX_API_KEY' with your actual Mapbox API key
    mapbox_api_key = 'sk.eyJ1IjoibWVocmEwMDE5IiwiYSI6ImNscnFjNm8yYjAxbWQycW5jNXJ6MHdzYmsifQ.uF1ogfIE83KheurGEzj1Dw'

    # Example route coordinates (replace with your actual coordinates)
    route_coordinates = [
        (28.7041, 77.1025),  # San Francisco, CA
        (28.4595, 77.0266),  # Los Angeles, CA
    ]

    # Call the Mapbox Traffic API
    result = get_mapbox_traffic(route_coordinates, mapbox_api_key)

    # Display the result (modify as needed based on your requirements)
    if result:
        print(result)
"""