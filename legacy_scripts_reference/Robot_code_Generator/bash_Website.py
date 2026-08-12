import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import socket
import time
import json
from threading import Lock

# Initialize the Dash app
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
server = app.server

# Robot connection variables
robot_connection = None
connection_lock = Lock()
connection_status = "Disconnected"

# Default parameters
default_params = {
    'x': 212.563,
    'y': -61.575,
    'z': 70.599,
    'a': -94.998,
    'b': 79.029,
    'c': 33.056,
    'size': 40.0,
    'z_step': 2.0,
    'layers': 20
}

# App layout
app.layout = dbc.Container([
    html.H1("MECA500 Robot Control", className="mb-4 text-center"),

    dbc.Card([
        dbc.CardHeader("Robot Connection", className="bg-primary text-white"),
        dbc.CardBody([
            dbc.Row([
                dbc.Col(dbc.Input(id="ip-input", placeholder="Robot IP", value="192.168.0.100"), width=4),
                dbc.Col(dbc.Input(id="port-input", placeholder="Port", value="10000"), width=2),
                dbc.Col(dbc.Button("Connect", id="connect-btn", color="success"), width=2),
                dbc.Col(dbc.Badge("Disconnected", id="connection-status", color="danger", className="p-2"), width=4),
            ]),
        ]),
    ], className="mb-4"),

    dbc.Card([
        dbc.CardHeader("Path Generation", className="bg-primary text-white"),
        dbc.CardBody([
            dbc.Row([
                dbc.Col([
                    html.H5("Initial Position"),
                    dbc.InputGroup([dbc.InputGroupText("X"), dbc.Input(id="x-input", type="number", value=default_params['x'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Y"), dbc.Input(id="y-input", type="number", value=default_params['y'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Z"), dbc.Input(id="z-input", type="number", value=default_params['z'])], className="mb-2"),
                ], width=4),

                dbc.Col([
                    html.H5("Orientation"),
                    dbc.InputGroup([dbc.InputGroupText("A"), dbc.Input(id="a-input", type="number", value=default_params['a'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("B"), dbc.Input(id="b-input", type="number", value=default_params['b'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("C"), dbc.Input(id="c-input", type="number", value=default_params['c'])], className="mb-2"),
                ], width=4),

                dbc.Col([
                    html.H5("Path Parameters"),
                    dbc.InputGroup([dbc.InputGroupText("Size (mm)"), dbc.Input(id="size-input", type="number", value=default_params['size'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Z Step (mm)"), dbc.Input(id="zstep-input", type="number", value=default_params['z_step'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Layers"), dbc.Input(id="layers-input", type="number", value=default_params['layers'])], className="mb-2"),
                ], width=4),
            ]),

            dbc.Row([
                dbc.Col(dbc.Button("Generate Commands", id="generate-btn", color="primary"), width=3),
                dbc.Col(dbc.Button("Send to Robot", id="send-btn", color="success", disabled=True), width=3),
                dbc.Col(dbc.Button("Stop Robot", id="stop-btn", color="danger"), width=3),
            ], className="mt-3"),

            dbc.Row([
                dbc.Col(dbc.Button("Activate Robot", id="activate-btn", color="warning"), width=3),
                dbc.Col(dbc.Button("Home Robot", id="home-btn", color="info"), width=3),
                dbc.Col(dbc.Button("Deactivate Robot", id="deactivate-btn", color="dark"), width=3),
            ], className="mt-3"),
        ]),
    ], className="mb-4"),

    dbc.Card([
        dbc.CardHeader("Generated Commands", className="bg-primary text-white"),
        dbc.CardBody([
            dcc.Textarea(id="command-output", style={'width': '100%', 'height': 300}, readOnly=True),
            dbc.Row([
                dbc.Col(dbc.Button("Save to File", id="save-btn", color="info"), width=2),
                dbc.Col(html.Div(id="save-status"), width=10),
            ], className="mt-3"),
            html.Hr(),
            html.H5("Live Robot Feedback"),
            html.Div(id="robot-feedback", style={"height": "150px", "overflowY": "auto", "whiteSpace": "pre-wrap"}, className="border p-2")
        ]),
    ]),

    html.Div(id="dummy-div", style={"display": "none"}),
    dcc.Store(id="command-store"),
], fluid=True)

# Helper functions
def connect_to_robot(ip, port):
    global robot_connection, connection_status
    try:
        with connection_lock:
            robot_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            robot_connection.connect((ip, int(port)))
            robot_connection.settimeout(2.0)
            connection_status = "Connected"
            return True
    except Exception as e:
        connection_status = f"Connection failed: {str(e)}"
        return False

def disconnect_from_robot():
    global robot_connection, connection_status
    with connection_lock:
        if robot_connection:
            try:
                robot_connection.close()
            except:
                pass
            robot_connection = None
        connection_status = "Disconnected"

def send_command_to_robot(command):
    global robot_connection
    if not robot_connection:
        return "Error: Not connected to robot"
    try:
        with connection_lock:
            robot_connection.sendall((command + "\n").encode())
            response = robot_connection.recv(1024).decode().strip()
            return f"Sent: {command}\nResponse: {response}"
    except Exception as e:
        return f"Error sending command: {str(e)}"

def generate_commands(params):
    commands = []
    x, y, z = params['x'], params['y'], params['z']
    a, b, c = params['a'], params['b'], params['c']
    size, z_step, layers = params['size'], params['z_step'], params['layers']

    for layer in range(layers):
        current_z = z + (layer * z_step)
        commands.append(f"MovePose({x:.3f}, {y:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        commands.append(f"MovePose({x + size:.3f}, {y:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        commands.append(f"MovePose({x + size:.3f}, {y + size:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        commands.append(f"MovePose({x:.3f}, {y + size:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        commands.append(f"MovePose({x:.3f}, {y:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")

    return commands

# Callbacks
@app.callback(
    [Output("connection-status", "children"),
     Output("connection-status", "color"),
     Output("connect-btn", "children"),
     Output("send-btn", "disabled")],
    Input("connect-btn", "n_clicks"),
    State("ip-input", "value"),
    State("port-input", "value"),
    prevent_initial_call=True
)
def handle_connection(n_clicks, ip, port):
    global connection_status
    if not n_clicks:
        return dash.no_update

    if connection_status == "Connected":
        disconnect_from_robot()
        return "Disconnected", "danger", "Connect", True

    if connect_to_robot(ip, port):
        return "Connected", "success", "Disconnect", False
    else:
        return connection_status, "danger", "Connect", True

@app.callback(
    [Output("command-output", "value"),
     Output("command-store", "data")],
    Input("generate-btn", "n_clicks"),
    State("x-input", "value"), State("y-input", "value"), State("z-input", "value"),
    State("a-input", "value"), State("b-input", "value"), State("c-input", "value"),
    State("size-input", "value"), State("zstep-input", "value"), State("layers-input", "value"),
    prevent_initial_call=True
)
def generate_command_output(n_clicks, x, y, z, a, b, c, size, z_step, layers):
    if not n_clicks:
        return dash.no_update

    params = {
        'x': float(x), 'y': float(y), 'z': float(z),
        'a': float(a), 'b': float(b), 'c': float(c),
        'size': float(size), 'z_step': float(z_step), 'layers': int(layers)
    }

    commands = generate_commands(params)
    return "\n".join(commands), {'commands': commands}

@app.callback(
    Output("save-status", "children"),
    Input("save-btn", "n_clicks"),
    State("command-store", "data"),
    prevent_initial_call=True
)
def save_commands(n_clicks, data):
    if not n_clicks or not data:
        return dash.no_update
    try:
        with open("meca500_commands.txt", "w") as f:
            f.write("\n".join(data['commands']))
        return "Commands saved to meca500_commands.txt"
    except Exception as e:
        return f"Error saving file: {str(e)}"

@app.callback(
    [Output("command-output", "value", allow_duplicate=True),
     Output("robot-feedback", "children", allow_duplicate=True)],
    Input("send-btn", "n_clicks"),
    State("command-store", "data"),
    prevent_initial_call=True
)
def send_commands(n_clicks, data):
    if not n_clicks or not data or connection_status != "Connected":
        return dash.no_update, dash.no_update

    feedback = []
    for cmd in data['commands']:
        response = send_command_to_robot(cmd)
        feedback.append(response)
        time.sleep(0.1)

    return "\n".join(feedback), "\n".join(feedback)

@app.callback(
    Output("robot-feedback", "children", allow_duplicate=True),
    Input("stop-btn", "n_clicks"),
    prevent_initial_call=True
)
def stop_robot(n_clicks):
    if not n_clicks:
        return dash.no_update
    return send_command_to_robot("Stop")

@app.callback(
    Output("robot-feedback", "children", allow_duplicate=True),
    Input("activate-btn", "n_clicks"),
    prevent_initial_call=True
)
def activate_robot(n_clicks):
    if not n_clicks:
        return dash.no_update
    return send_command_to_robot("ActivateRobot()")

@app.callback(
    Output("robot-feedback", "children", allow_duplicate=True),
    Input("home-btn", "n_clicks"),
    prevent_initial_call=True
)
def home_robot(n_clicks):
    if not n_clicks:
        return dash.no_update
    return send_command_to_robot("Home()")

@app.callback(
    Output("robot-feedback", "children", allow_duplicate=True),
    Input("deactivate-btn", "n_clicks"),
    prevent_initial_call=True
)
def deactivate_robot(n_clicks):
    if not n_clicks:
        return dash.no_update
    return send_command_to_robot("DeactivateRobot()")

if __name__ == "__main__":
    app.run_server(debug=True)
