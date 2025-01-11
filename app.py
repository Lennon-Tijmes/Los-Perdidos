from flask import Flask, render_template, request, jsonify
import sqlite3
import socket
hostname = socket.gethostname()
ip_address = socket.gethostbyname(hostname)
print(f"Flask Server IP: {ip_address}")

app = Flask(__name__)

# Function to insert the sensor data into the database
def insert_robot_data(robot_id, speed, object_left, object_right, object_middle=None, line="Unknown"):
    conn = sqlite3.connect('robots.db')
    cursor = conn.cursor()

    if robot_id == 1: # King Julien
        cursor.execute("INSERT INTO king_julien (robot_id, speed, object_distance, line) VALUES (?, ?, ?, ?)",
                        (robot_id, speed, object_middle, line))
    elif robot_id == 2:  # Moto-Moto
        cursor.execute("INSERT INTO moto_moto (robot_id, speed, object_distance, line) VALUES (?, ?, ?, ?)", 
                        (robot_id, speed, object_middle, line))
    elif robot_id == 3:  # Mort
        cursor.execute("INSERT INTO mort (robot_id, speed, object_left, object_right, object_middle, line) VALUES (?, ?, ?, ?, ?, ?)",
                        (robot_id, speed, object_left, object_right, object_middle, line))     

    conn.commit()
    conn.close()

# Route to handle incoming data from Master
@app.route('/update_robot_data', methods=['GET'])
def update_robot_data():
    # Extract parameters from the GET request
    robot_id = request.args.get('robot_id')
    speed = request.args.get('speed')
    object_left = request.args.get('object_left')
    object_right = request.args.get('object_right')
    object_middle = request.args.get('object_middle', None)  # Optional parameter
    line = request.args.get('line', "Unknown")  # Default line state if not provided

    # Validate the parameters
    if not all([robot_id, speed, object_left, object_right, line]):
        return "<p style='color: red;'>Missing required parameters. Please include 'robot_id', 'speed', 'object_left', 'object_right', and 'line' in the URL.</p>", 400
    
    # Validate 'line' to ensure it's either "On Line" or "Not On Line"
    if line not in ["On Line", "Not On Line"]:
        return "<p style='color: red;'>Invalid 'line' state. It must be either 'On Line' or 'Not On Line'.</p>", 400

    try:
        # Convert parameters to their appropriate types
        robot_id = int(robot_id)
        speed = float(speed)
        object_left = float(object_left)
        object_right = float(object_right)
        object_middle = float(object_middle) if object_middle else None
        
        # Save data to the database (assuming insert_robot_data function exists)
        insert_robot_data(robot_id, speed, object_left, object_right, object_middle, line)
        print("test")
        
        # Return a success message
        return f"""
        <h1>Robot Data Updated</h1>
        <p>Robot ID: {robot_id}</p>
        <p>Speed: {speed}</p>
        <p>Object Distance (Left): {object_left}</p>
        <p>Object Distance (Right): {object_right}</p>
        {"<p>Object Distance (Middle): " + str(object_middle) + "</p>" if object_middle else ""}
        <p>Line Status: {line}</p>
        <p>Data successfully recorded in the database.</p>
        <p><a href='/'>Go back to the main page</a></p>
        """, 200

    except ValueError:
        return "<p style='color: red;'>Invalid data types. Ensure 'robot_id' is an integer and other values are numbers.</p>", 400

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