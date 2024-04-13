
import requests
import polyline


lat_lon_1 = {"lat":28.7514,"lon":77.1556}
lat_lon_2 = {"lat":28.6267,"lon":77.2119}
locations = "locations"
costing = "costing"
auto = "auto"
# request_url = f"http://localhost:8002/route?json={{\"{locations}\":[{lat_lon_1},{lat_lon_2}],\"{costing}\":\"{auto}\"}}"

request_url = f"http://localhost:8002/route?json={{\"{locations}\": [{{\"lat\": {lat_lon_1['lat']}, \"lon\": {lat_lon_1['lon']}}}, {{\"lat\": {lat_lon_2['lat']}, \"lon\": {lat_lon_2['lon']}}}], \"costing\": \"auto\"}}"
print(request_url)
response = requests.get(request_url)


if response.status_code == 200:
    # Accessing the JSON content of the response
    data = response.json()
    
    # Accessing the shape string from the JSON
    shape_string = data['trip']['legs'][0]['shape']

    coordinates = polyline.decode(shape_string, 6)
    
    print(coordinates)


