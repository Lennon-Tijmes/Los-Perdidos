from flask import Flask, render_template
import sqlite3

app = Flask(__name__)

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
        mort_data=mort_data,
        king_julien_data=king_julien_data,
        moto_moto_data=moto_moto_data
    )

if __name__ == '__main__':
    app.run(debug=True)