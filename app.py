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
    # Extract and validate GET parameters
    try:
        robot_id = int(request.args.get('robot_id'))
        speed = float(request.args.get('speed'))
        object_left = float(request.args.get('object_left'))
        object_right = float(request.args.get('object_right'))
        object_middle = request.args.get('object_middle')
        line = request.args.get('line', "Unknown")

        # Convert optional object_middle if provided
        object_middle = float(object_middle) if object_middle else None

        # Validate 'line' status
        if line not in ["On Line", "Not On Line"]:
            raise ValueError("Invalid 'line' status")

        # Insert data into the database
        insert_robot_data(robot_id, speed, object_left, object_right, object_middle, line)

        # Success response
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

    except (ValueError, TypeError):
        return "<p style='color: red;'>Invalid or missing parameters. Ensure all required fields are correctly formatted.</p>", 400

# Gets the latest data for Mort from the database and returns it as JSON
@app.route('/api/mort_data', methods=['GET'])
def get_mort_data():
    try:
        # Establish a connection to the SQLite database
        conn = sqlite3.connect('robots.db')
        cursor = conn.cursor()

        # Fetch the speed for Mort (robot_id = 3) from the latest entry
        cursor.execute("SELECT speed FROM mort WHERE robot_id = 3 ORDER BY id DESC LIMIT 1")
        result = cursor.fetchone()

        # Close the connection after the query
        conn.close()

        if result:
            return jsonify({'speed': result[0]}), 200
        return jsonify({'speed': 0}), 200  # Default to 0 if no data found

    except Exception as e:
        return jsonify({"error": str(e)}), 500


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
    conn = sqlite3.connect('robots.db')
    cursor = conn.cursor()

    # Query data for each robot without including the 'robot_id' and 'id' columns in the result
    cursor.execute("SELECT speed, object_left, object_right, object_middle, line FROM mort ORDER BY id DESC LIMIT 10")
    mort_data = cursor.fetchall()

    cursor.execute("SELECT speed, object_distance, line FROM king_julien ORDER BY id DESC LIMIT 10")
    king_julien_data = cursor.fetchall()

    cursor.execute("SELECT speed, object_distance, line FROM moto_moto ORDER BY id DESC LIMIT 10")
    moto_moto_data = cursor.fetchall()

    conn.close()

    # Pass data to the dashboard template
    return render_template(
        'dashboard.html',
        mort_data=mort_data,
        king_julien_data=king_julien_data,
        moto_moto_data=moto_moto_data
    )

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)