# Dynamic Routing Web Application
This web application is designed to provide route planning functionality using the OpenStreetMap data and Valhalla routing engine. It allows users to input their source and destination addresses, and it returns the route information including the path visualisation, distance, time, and turn-by-turn instructions.

# Technologies Used
**Python:** The backend of the application is developed using Python.\
**Flask:** Flask is used as the web framework for handling HTTP requests and responses.\
**Vahalla Rouitng engine:** Valhalla is an open source routing engine and accompanying libraries for use with OpenStreetMap data.\
**Flask-Caching:** Flask-Caching extension is used to cache route information to improve performance.\
**Memcached:** Memcached is used as the caching system to store route information temporarily.

# Setup
**Clone the Repository:** Clone this repository to your local machine.

```bash
https://github.com/NitinMehra0019/Dynamic-Routing-And-Mapping.git
```
**Build virtual environment**
```bash
pip install virtualenv
virtualenv env
cd env/bin
source activate
```
**Install Dependencies:** Navigate to the project directory and install the required dependencies using pip.
```bash
pip install flask
pip install CORS
pip install flask_cors
pip install numpy
pip install flask_caching
```
**Run the Application:** Start the Flask application.

```bash
python ./valhalla.py
```

**Access the Web Application:** Open your web browser and navigate to http://localhost:8000 to access the route planner.



# Valhalla Routing Engine Installation
Valhalla is an open-source routing engine that provides high-performance routing services with customizable routing profiles. This guide will walk you through the steps to install and set up Valhalla on your system.

## Prerequisites
Before installing Valhalla, ensure that you have the following prerequisites installed on your system:

Operating System: Valhalla supports Linux-based operating systems such as Ubuntu, CentOS, or Debian.\
CMake: Make sure you have CMake installed on your system. You can install it using your package manager (e.g., apt, yum).\
GCC/G++: Ensure that you have a C++ compiler installed on your system (e.g., gcc, g++).\
Boost Libraries: Valhalla requires Boost libraries. You can install them using your package manager.

## Installation Steps
Check out [Valhalla installation part 1](https://wiki.jangoo.us:3443/en/jangoo/Dynamic_Routing/Valhalla_Install/Valhalla_install_part_1) and [Valhalla installation part 2](https://wiki.jangoo.us:3443/en/jangoo/Dynamic_Routing/Valhalla_Install/Valhalla_install_part_2).

## Usage
Open terminal and run below command
```bash
python ./valhalla.py
```

## Architecture Diagram
![map](https://github.com/NitinMehra0019/Dynamic-Routing-And-Mapping/assets/84515402/37eb7ee8-b066-4b1d-b79a-711acfa72e14)

Link to tech stack doc : [Tech statck Doc](https://drive.google.com/file/d/1u7xB7V71RALhngYwVXjdgTsBiRLa55fu/view?usp=sharing)

Link to video in which GPS data is integrated : [Video](https://drive.google.com/file/d/1XTwU49nhMbhl85bosQ66gIvOMrr1ilj7/view?usp=sharing)

**Input Address:** On the homepage of the web application, enter the source and destination coordinates in the provided input fields.

**Submit Request:** Click on the "Direction" button to submit the request.

**View Route Information:** The application will display the route information including the path coordinates, distance, time, and turn-by-turn instructions.

<a href="https://ibb.co/6Rsvm6P"><img src="https://i.ibb.co/99YwNRT/Screenshot-from-2024-02-20-15-53-11.png" alt="Screenshot-from-2024-02-20-15-53-11" border="0"></a>

<a href="https://ibb.co/jwzzxMj"><img src="https://i.ibb.co/09KKWMz/Screenshot-from-2024-02-20-16-02-14.png" alt="Screenshot-from-2024-02-20-16-02-14" border="0"></a>












