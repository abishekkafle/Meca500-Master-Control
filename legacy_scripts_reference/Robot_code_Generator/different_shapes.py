import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import socket
import time
import json
import math
import numpy as np
from math import cos, sin, pi
from threading import Lock
import plotly.graph_objects as go
import plotly.express as px

# Initialize the Dash app
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
server = app.server

# Robot connection variables
robot_connection = None
connection_lock = Lock()
connection_status = "Disconnected"

# Default parameters
default_params = {
    'shape': 'rectangle',
    'x': 212.563, 'y': -61.575, 'z': 70.599,
    'a': -94.998, 'b': 79.029, 'c': 33.056,
    'size': 40.0, 'size2': 40.0,
    'z_step': 2.0, 'layers': 20,
    'pattern': 'rectilinear', 'density': 50
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
                    html.H5("Shape Parameters"),
                    dbc.InputGroup([
                        dbc.InputGroupText("Shape"),
                        dbc.Select(
                            id="shape-select",
                            options=[
                                {"label": "Rectangle", "value": "rectangle"},
                                {"label": "Cylinder", "value": "cylinder"},
                                {"label": "Mesh Grid", "value": "meshgrid"},
                                {"label": "Spiral", "value": "spiral"},
                            ],
                            value=default_params['shape'],
                        ),
                    ], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Size 1 (mm)"), dbc.Input(id="size-input", type="number", value=default_params['size'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Size 2 (mm)"), dbc.Input(id="size2-input", type="number", value=default_params['size2'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Z Step (mm)"), dbc.Input(id="zstep-input", type="number", value=default_params['z_step'])], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Layers"), dbc.Input(id="layers-input", type="number", value=default_params['layers'])], className="mb-2"),
                    dbc.InputGroup([
                        dbc.InputGroupText("Infill Pattern"),
                        dbc.Select(
                            id="infill-pattern",
                            options=[
                                {"label": "Rectilinear", "value": "rectilinear"},
                                {"label": "Concentric", "value": "concentric"},
                                {"label": "Grid", "value": "grid"},
                                {"label": "Triangles", "value": "triangles"},
                                {"label": "Honeycomb", "value": "honeycomb"},
                            ],
                            value=default_params['pattern'],
                        ),
                    ], className="mb-2"),
                    dbc.InputGroup([dbc.InputGroupText("Infill Density (%)"), dbc.Input(id="infill-density", type="number", value=default_params['density'], min=0, max=100)], className="mb-2"),
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
        dbc.CardHeader("Visualization", className="bg-primary text-white"),
        dbc.CardBody([
            dcc.Graph(id="path-visualization", style={'height': '500px'}),
            dbc.Row([
                dbc.Col(dbc.RadioItems(
                    id="view-selector",
                    options=[
                        {"label": "3D View", "value": "3d"},
                        {"label": "Top View", "value": "top"},
                        {"label": "Side View", "value": "side"},
                    ],
                    value="3d",
                    inline=True
                ), width=12),
            ]),
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
    dcc.Store(id="path-data-store"),
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
    path_data = {'x': [], 'y': [], 'z': [], 'layer': [], 'segment_type': []}
    
    shape = params.get('shape', 'rectangle')
    x, y, z = params['x'], params['y'], params['z']
    a, b, c = params['a'], params['b'], params['c']
    size1, size2 = params['size'], params['size2']
    z_step, layers = params['z_step'], params['layers']
    pattern = params.get('pattern', 'rectilinear')
    density = max(0.05, min(1.0, params.get('density', 50) / 100.0))

    for layer in range(layers):
        current_z = z + (layer * z_step)
        
        if shape == 'rectangle':
            # Rectangle perimeter
            perimeter_points = [
                (x, y, current_z),
                (x + size1, y, current_z),
                (x + size1, y + size2, current_z),
                (x, y + size2, current_z),
                (x, y, current_z)
            ]
            
        elif shape == 'cylinder':
            # Cylinder perimeter (circle)
            radius = size1 / 2
            perimeter_points = []
            for angle in np.linspace(0, 2*pi, 36):  # 36 points for smooth circle
                px = x + radius * cos(angle)
                py = y + radius * sin(angle)
                perimeter_points.append((px, py, current_z))
            perimeter_points.append(perimeter_points[0])  # Close the circle
            
        elif shape == 'meshgrid':
            # Mesh grid perimeter (rectangle with grid pattern)
            perimeter_points = [
                (x, y, current_z),
                (x + size1, y, current_z),
                (x + size1, y + size2, current_z),
                (x, y + size2, current_z),
                (x, y, current_z)
            ]
            
        elif shape == 'spiral':
            # Spiral pattern
            spiral_points = []
            radius = size1 / 2
            turns = 3
            for t in np.linspace(0, turns*2*pi, 100):
                r = radius * (1 - t/(turns*2*pi))  # Decreasing radius
                px = x + r * cos(t)
                py = y + r * sin(t)
                spiral_points.append((px, py, current_z))
            
            for point in spiral_points:
                commands.append(f"MovePose({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                path_data['x'].append(point[0])
                path_data['y'].append(point[1])
                path_data['z'].append(point[2])
                path_data['layer'].append(layer)
                path_data['segment_type'].append('perimeter')
            
            # Skip infill for spiral
            continue
        
        # Add perimeter points for all shapes except spiral
        for point in perimeter_points:
            commands.append(f"MovePose({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            path_data['x'].append(point[0])
            path_data['y'].append(point[1])
            path_data['z'].append(point[2])
            path_data['layer'].append(layer)
            path_data['segment_type'].append('perimeter')
        
        # Generate infill
        if density > 0.05 and shape != 'spiral':
            if shape == 'meshgrid':
                infill_commands, infill_data = generate_meshgrid_infill(
                    x, y, current_z, a, b, c,
                    size1, size2, density, layer
                )
            elif shape == 'cylinder':
                infill_commands, infill_data = generate_cylinder_infill(
                    x, y, current_z, a, b, c,
                    size1, density, layer
                )
            else:
                infill_commands, infill_data = generate_infill(
                    x, y, current_z, a, b, c,
                    size1, size2, pattern, density, layer, shape
                )
            
            commands.extend(infill_commands)
            for key in infill_data:
                path_data[key].extend(infill_data[key])

    return commands, path_data

def generate_infill(x, y, z, a, b, c, width, height, pattern, density, layer, shape):
    commands = []
    path_data = {'x': [], 'y': [], 'z': [], 'layer': [], 'segment_type': []}
    
    min_spacing = 5  # Minimum spacing in mm
    max_spacing = min(width, height) / 2
    spacing = max(min_spacing, max_spacing * (1 - density))
    
    if pattern == "rectilinear":
        # Horizontal lines
        current_y = y + spacing
        while current_y < y + height - spacing/2:
            if (x >= x and x + width <= x + width and 
                current_y >= y and current_y <= y + height):
                commands.append(f"MovePose({x:.3f}, {current_y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                commands.append(f"MovePose({x + width:.3f}, {current_y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                
                path_data['x'].extend([x, x + width])
                path_data['y'].extend([current_y, current_y])
                path_data['z'].extend([z, z])
                path_data['layer'].extend([layer, layer])
                path_data['segment_type'].extend(['infill', 'infill'])
            
            current_y += spacing
        
        if density > 0.3:
            current_x = x + spacing
            while current_x < x + width - spacing/2:
                if (current_x >= x and current_x <= x + width and 
                    y >= y and y + height <= y + height):
                    commands.append(f"MovePose({current_x:.3f}, {y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                    commands.append(f"MovePose({current_x:.3f}, {y + height:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                    
                    path_data['x'].extend([current_x, current_x])
                    path_data['y'].extend([y, y + height])
                    path_data['z'].extend([z, z])
                    path_data['layer'].extend([layer, layer])
                    path_data['segment_type'].extend(['infill', 'infill'])
                
                current_x += spacing
    
    elif pattern == "concentric":
        offset = spacing
        max_offset = min(width, height)/2
        while offset < max_offset:
            points = [
                (x + offset, y + offset),
                (x + width - offset, y + offset),
                (x + width - offset, y + height - offset),
                (x + offset, y + height - offset),
                (x + offset, y + offset)
            ]
            
            if all(x <= px <= x + width and y <= py <= y + height for px, py in points):
                for px, py in points:
                    commands.append(f"MovePose({px:.3f}, {py:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                    path_data['x'].append(px)
                    path_data['y'].append(py)
                    path_data['z'].append(z)
                    path_data['layer'].append(layer)
                    path_data['segment_type'].append('infill')
            
            offset += spacing
    
    elif pattern == "grid":
        current_y = y + spacing
        while current_y < y + height - spacing/2:
            commands.append(f"MovePose({x:.3f}, {current_y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            commands.append(f"MovePose({x + width:.3f}, {current_y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            
            path_data['x'].extend([x, x + width])
            path_data['y'].extend([current_y, current_y])
            path_data['z'].extend([z, z])
            path_data['layer'].extend([layer, layer])
            path_data['segment_type'].extend(['infill', 'infill'])
            
            current_y += spacing
        
        current_x = x + spacing
        while current_x < x + width - spacing/2:
            commands.append(f"MovePose({current_x:.3f}, {y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            commands.append(f"MovePose({current_x:.3f}, {y + height:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            
            path_data['x'].extend([current_x, current_x])
            path_data['y'].extend([y, y + height])
            path_data['z'].extend([z, z])
            path_data['layer'].extend([layer, layer])
            path_data['segment_type'].extend(['infill', 'infill'])
            
            current_x += spacing
    
    elif pattern == "triangles":
        triangle_size = spacing * 2
        rows = int(height / (triangle_size * math.sqrt(3)/2)) + 1
        cols = int(width / triangle_size) + 1
        
        for row in range(rows):
            y_offset = row * triangle_size * math.sqrt(3)/2
            x_start = x
            if row % 2 == 1:
                x_start += triangle_size/2
            
            for col in range(cols):
                x_pos = x_start + col * triangle_size
                if x_pos > x + width:
                    continue
                
                points = [
                    (x_pos, y + y_offset),
                    (x_pos + triangle_size/2, y + y_offset + triangle_size * math.sqrt(3)/2),
                    (x_pos + triangle_size, y + y_offset),
                    (x_pos, y + y_offset)
                ]
                
                for px, py in points:
                    commands.append(f"MovePose({px:.3f}, {py:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                    path_data['x'].append(px)
                    path_data['y'].append(py)
                    path_data['z'].append(z)
                    path_data['layer'].append(layer)
                    path_data['segment_type'].append('infill')
    
    elif pattern == "honeycomb":
        hex_size = spacing * 2 / math.sqrt(3)
        rows = int((height) / (hex_size * 1.5)) + 1
        cols = int((width) / (spacing * 2)) + 1
        
        for row in range(rows):
            y_offset = row * hex_size * 1.5
            x_start = x + spacing
            if row % 2 == 1:
                x_start += spacing
            
            for col in range(cols):
                x_pos = x_start + col * spacing * 2
                if x_pos > x + width:
                    continue
                
                points = []
                for i in range(6):
                    angle = math.pi/3 * i
                    px = x_pos + spacing * math.cos(angle)
                    py = y + y_offset + spacing * math.sin(angle)
                    points.append((px, py))
                
                for px, py in points + [points[0]]:
                    commands.append(f"MovePose({px:.3f}, {py:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
                    path_data['x'].append(px)
                    path_data['y'].append(py)
                    path_data['z'].append(z)
                    path_data['layer'].append(layer)
                    path_data['segment_type'].append('infill')
    
    return commands, path_data

def generate_meshgrid_infill(x, y, z, a, b, c, width, height, density, layer):
    commands = []
    path_data = {'x': [], 'y': [], 'z': [], 'layer': [], 'segment_type': []}
    
    spacing = max(5, min(width, height) * (1 - density))
    
    # Horizontal lines
    current_y = y + spacing
    while current_y < y + height - spacing/2:
        commands.append(f"MovePose({x:.3f}, {current_y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        commands.append(f"MovePose({x + width:.3f}, {current_y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        
        path_data['x'].extend([x, x + width])
        path_data['y'].extend([current_y, current_y])
        path_data['z'].extend([z, z])
        path_data['layer'].extend([layer, layer])
        path_data['segment_type'].extend(['infill', 'infill'])
        
        current_y += spacing
    
    # Vertical lines
    current_x = x + spacing
    while current_x < x + width - spacing/2:
        commands.append(f"MovePose({current_x:.3f}, {y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        commands.append(f"MovePose({current_x:.3f}, {y + height:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        
        path_data['x'].extend([current_x, current_x])
        path_data['y'].extend([y, y + height])
        path_data['z'].extend([z, z])
        path_data['layer'].extend([layer, layer])
        path_data['segment_type'].extend(['infill', 'infill'])
        
        current_x += spacing
    
    return commands, path_data

def generate_cylinder_infill(x, y, z, a, b, c, diameter, density, layer):
    commands = []
    path_data = {'x': [], 'y': [], 'z': [], 'layer': [], 'segment_type': []}
    
    radius = diameter / 2
    spacing = max(5, radius * (1 - density))
    
    # Radial infill lines
    current_r = spacing
    while current_r < radius - spacing/2:
        points = []
        for angle in np.linspace(0, 2*pi, 36):
            px = x + current_r * cos(angle)
            py = y + current_r * sin(angle)
            points.append((px, py))
        
        for px, py in points + [points[0]]:
            commands.append(f"MovePose({px:.3f}, {py:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            path_data['x'].append(px)
            path_data['y'].append(py)
            path_data['z'].append(z)
            path_data['layer'].append(layer)
            path_data['segment_type'].append('infill')
        
        current_r += spacing
    
    return commands, path_data

def create_visualization(path_data, view_type='3d'):
    if not path_data or not path_data['x']:
        return go.Figure()
    
    # Create a color map for layers
    unique_layers = sorted(list(set(path_data['layer'])))
    color_cycle = px.colors.qualitative.Plotly
    colors = [color_cycle[i % len(color_cycle)] for i in range(len(unique_layers))]
    
    fig = go.Figure()
    
    # Group by segment type and layer
    perimeter_data = {k: [v[i] for i in range(len(v)) if path_data['segment_type'][i] == 'perimeter'] for k, v in path_data.items()}
    infill_data = {k: [v[i] for i in range(len(v)) if path_data['segment_type'][i] == 'infill'] for k, v in path_data.items()}
    
    # Add perimeter traces
    for i, layer in enumerate(unique_layers):
        layer_idx = [idx for idx, l in enumerate(perimeter_data['layer']) if l == layer]
        if not layer_idx:
            continue
            
        x = [perimeter_data['x'][idx] for idx in layer_idx]
        y = [perimeter_data['y'][idx] for idx in layer_idx]
        z = [perimeter_data['z'][idx] for idx in layer_idx]
        
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color=colors[i], width=4),
            name=f'Perimeter L{layer}',
            legendgroup=f'layer_{layer}',
            showlegend=True
        ))
    
    # Add infill traces
    for i, layer in enumerate(unique_layers):
        layer_idx = [idx for idx, l in enumerate(infill_data['layer']) if l == layer]
        if not layer_idx:
            continue
            
        x = [infill_data['x'][idx] for idx in layer_idx]
        y = [infill_data['y'][idx] for idx in layer_idx]
        z = [infill_data['z'][idx] for idx in layer_idx]
        
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color=colors[i], width=2),
            name=f'Infill L{layer}',
            legendgroup=f'layer_{layer}',
            showlegend=True,
            opacity=0.7
        ))
    
    # Set camera view based on selection
    if view_type == 'top':
        camera = dict(up=dict(x=0, y=0, z=1), center=dict(x=0, y=0, z=0), eye=dict(x=0, y=0, z=2))
    elif view_type == 'side':
        camera = dict(up=dict(x=0, y=1, z=0), center=dict(x=0, y=0, z=0), eye=dict(x=2, y=0, z=0))
    else:  # 3D
        camera = dict(up=dict(x=0, y=0, z=1), center=dict(x=0, y=0, z=0), eye=dict(x=1.5, y=1.5, z=1))
    
    fig.update_layout(
        scene=dict(
            xaxis_title='X (mm)',
            yaxis_title='Y (mm)',
            zaxis_title='Z (mm)',
            aspectmode='data',
            camera=camera
        ),
        margin=dict(l=0, r=0, b=0, t=30),
        height=500
    )
    
    if view_type != '3d':
        fig.update_layout(scene=dict(zaxis=dict(showticklabels=True, visible=True)))
    
    return fig

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
     Output("command-store", "data"),
     Output("path-data-store", "data")],
    Input("generate-btn", "n_clicks"),
    [State("shape-select", "value"),
     State("x-input", "value"), State("y-input", "value"), State("z-input", "value"),
     State("a-input", "value"), State("b-input", "value"), State("c-input", "value"),
     State("size-input", "value"), State("size2-input", "value"),
     State("zstep-input", "value"), State("layers-input", "value"),
     State("infill-pattern", "value"), State("infill-density", "value")],
    prevent_initial_call=True
)
def generate_command_output(n_clicks, shape, x, y, z, a, b, c, size, size2, z_step, layers, pattern, density):
    if not n_clicks:
        return dash.no_update, dash.no_update, dash.no_update

    params = {
        'shape': shape,
        'x': float(x), 'y': float(y), 'z': float(z),
        'a': float(a), 'b': float(b), 'c': float(c),
        'size': float(size), 'size2': float(size2),
        'z_step': float(z_step), 'layers': int(layers),
        'pattern': pattern, 'density': float(density)
    }

    commands, path_data = generate_commands(params)
    return "\n".join(commands), {'commands': commands}, path_data

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

@app.callback(
    Output("path-visualization", "figure"),
    [Input("path-data-store", "data"),
     Input("view-selector", "value")],
    prevent_initial_call=True
)
def update_visualization(path_data, view_type):
    if not path_data:
        return go.Figure()
    return create_visualization(path_data, view_type)

if __name__ == "__main__":
    app.run_server(debug=True)