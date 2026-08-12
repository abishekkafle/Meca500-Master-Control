from flask import Flask, render_template, request, jsonify, send_file
from inverse_kinematics import get_angles, forward_kin
import matplotlib.pyplot as plt
import os

app = Flask(__name__)

arm_state = {'x': 0.4, 'y': 1.3, 'z': 3.0, 'roll': 1.0, 'pitch': -1.0, 'yaw': -1.0}
limits = {'x': (0, 2), 'y': (0, 3), 'z': (0, 5), 'roll': (-3.14, 3.14), 'pitch': (-3.14, 3.14), 'yaw': (-3.14, 3.14)}

@app.route('/')
def index():
    return render_template('index.html', arm_state=arm_state)

@app.route('/update', methods=['POST'])
def update():
    data = request.json
    axis, increment = data['axis'], data.get('increment', 0)
    min_val, max_val = limits[axis]
    arm_state[axis] = round(max(min(arm_state[axis] + increment, max_val), min_val), 2)
    return jsonify(success=True, new_value=arm_state[axis])

@app.route('/send', methods=['POST'])
def send():
    q1, q2, q3, q4, q5, q6 = get_angles(**arm_state)
    plot_arm(q1, q2, q3, q4, q5, q6)
    data_string = f"{q1:.2f},{q2:.2f},{q3:.2f},{q4:.2f},{q5:.2f},{q6:.2f}"
    return jsonify(success=True, data=data_string)

def plot_arm(q1, q2, q3, q4, q5, q6):
    px, py, pz = forward_kin(q1, q2, q3, q4, q5, q6)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([0, px], [0, py], [0, pz], marker='o', color='r')
    ax.set_xlim(0, 2); ax.set_ylim(0, 3); ax.set_zlim(0, 5)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    plt.savefig('static/plot.png')
    plt.close(fig)

@app.route('/plot')
def plot():
    plot_path = 'static/plot.png'
    return send_file(plot_path, mimetype='image/png') if os.path.exists(plot_path) else ('', 404)

if __name__ == '__main__':
    # Prevent SystemExit in VSCode debugging
    app.run(debug=True, use_reloader=False, port=5000)