from flask import Flask, render_template, request, jsonify
import sqlite3
import socket
hostname = socket.gethostname()
ip_address = socket.gethostbyname(hostname)
print(f"Flask Server IP: {ip_address}")

app = Flask(__name__)

# Function to insert the sensor data into the database
def insert_robot_data(robot_id, speed, object_left, object_right, object_middle=None):
    conn = sqlite3.connect('robots.db')
    cursor = conn.cursor()

    if robot_id == 1: # King Julien
        cursor.execute("INSERT INTO king_julien (robot_id, speed, object_distance, line) VALUES (?, ?, ?, ?)",(robot_id, speed, object_left, object_right))
    elif robot_id == 2:  # Moto-Moto
        cursor.execute("INSERT INTO moto_moto (robot_id, speed, object_distance, line) VALUES (?, ?, ?, ?)", 
                       (robot_id, speed, object_left, object_right))
    elif robot_id == 3:  # Mort
        cursor.execute("INSERT INTO mort (robot_id, speed, object_left, object_right, object_middle, line) VALUES (?, ?, ?, ?, ?, ?)",
                       (robot_id, speed, object_left, object_right, object_middle, 'line_solution_here'))     

    conn.commit()
    conn.close()

# Route to handle incoming data from Master
@app.route('/update_robot_data', methods=['GET'])
def update_robot_data():
    data = request.args.get('data').split(',')

    robot_id = int(data[0])
    speed = float(data[1])
    object_left = float(data[2])
    object_right = float(data[3])
    object_middle = float(data[4]) if len(data) > 4 else None

    # Insert the data into the database
    insert_robot_data(robot_id, speed, object_left, object_right, object_middle)
    
    return jsonify({"status": "success", "message": "Data inserted successfully"}), 200

# Route for the homepage
@app.route('/')
def home():
    return render_template('home-page.html')

@app.route('/specification')
def specification():
    return render_template('specification.html')

@app.route('/contact')
def contact():
    return render_template('contact.html')

@app.route('/reviews')
def reviews():
    return render_template('reviews.html')

@app.route('/ourteam')
def ourteam():
    return render_template('ourteam.html')

# Route for the dashboard
@app.route('/dashboard')
def dashboard():
    # Connect to the database
    conn = sqlite3.connect('robots.db')
    cursor = conn.cursor()

    # Query for data from each robot
    cursor.execute("SELECT * FROM mort")
    mort_data = cursor.fetchall()

    cursor.execute("SELECT * FROM king_julien")
    king_julien_data = cursor.fetchall()

    cursor.execute("SELECT * FROM moto_moto")
    moto_moto_data = cursor.fetchall()

    conn.close()

    # Pass data to the dashboard template
    return render_template(
        'dashboard.html', 
        king_julien_data=king_julien_data,
        moto_moto_data=moto_moto_data,
        mort_data=mort_data
    )

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)