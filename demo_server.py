import os
from flask import Flask, render_template
from flask_cors import CORS




app = Flask(__name__)
# CORS(app)



# This end Point is rending the Home page with file index.html
@app.route('/')
def hello_world():
    print("render")
    return render_template('mobile_location.html')




if __name__ == "__main__":
# app.run(debug=True, port=8000)
    app.run(debug=True, host= '192.168.1.23')