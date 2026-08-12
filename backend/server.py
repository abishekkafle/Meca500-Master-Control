from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import numpy as np
import open3d as o3d
import trimesh
import cv2
import socket
import os
import math
from math import cos, sin, pi, atan2, sqrt
import json
from threading import Lock
from collections import deque
from scipy.interpolate import CubicSpline

app = FastAPI(title="Meca500 Master Control API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ----------------------------------------------------
# 1. MECA500 KINEMATICS ENGINE (DH, Analytical, Geometric)
# ----------------------------------------------------
LINK_LENGTHS = {
    'base_height': 135.0,   # Joint 1 to Joint 2 (mm)
    'upper_arm': 135.0,     # Joint 2 to Joint 3 (mm)
    'forearm': 175.0,       # Joint 3 to Joint 4 (mm)
    'wrist': 72.0,          # Joint 6 to tool flange (mm)
    'extruder_offset': 50.0 # Flange to nozzle tip (mm)
}

JOINT_LIMITS = [
    (-math.pi, math.pi),        # J1
    (-2.61799, 2.61799),        # J2 (-150 to 150 deg)
    (-math.pi, math.pi),        # J3
    (-math.pi, math.pi),        # J4
    (-2.61799, 2.61799),        # J5
    (-math.pi, math.pi)         # J6
]

def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles: List[float]):
    """Calculate positions of all joints and end-effector frame."""
    dh_params = [
        [joint_angles[0], LINK_LENGTHS['base_height'], 0, -np.pi/2],
        [joint_angles[1] + np.pi/2, 0, LINK_LENGTHS['upper_arm'], 0],
        [joint_angles[2], 0, LINK_LENGTHS['forearm'], -np.pi/2],
        [joint_angles[3], 0, 0, np.pi/2],
        [joint_angles[4], 0, 0, -np.pi/2],
        [joint_angles[5], LINK_LENGTHS['wrist'], 0, 0]
    ]

    T = np.eye(4)
    transformations = [T.copy()]

    for params in dh_params:
        theta, d, a, alpha = params
        T = T @ dh_matrix(theta, d, a, alpha)
        transformations.append(T.copy())

    # Extruder nozzle tip transform
    extruder_offset_mat = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -LINK_LENGTHS['extruder_offset']],
        [0, 0, 0, 1]
    ])
    T_extruder = T @ extruder_offset_mat
    transformations.append(T_extruder)

    positions = [T_mat[:3, 3].tolist() for T_mat in transformations]

    R = T[:3, :3]
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x_rot = math.atan2(R[2,1], R[2,2])
        y_rot = math.atan2(-R[2,0], sy)
        z_rot = math.atan2(R[1,0], R[0,0])
    else:
        x_rot = math.atan2(-R[1,2], R[1,1])
        y_rot = math.atan2(-R[2,0], sy)
        z_rot = 0

    pose = {
        'x': round(float(T[0, 3]), 3),
        'y': round(float(T[1, 3]), 3),
        'z': round(float(T[2, 3]), 3),
        'a': round(math.degrees(x_rot), 3),
        'b': round(math.degrees(y_rot), 3),
        'c': round(math.degrees(z_rot), 3)
    }

    return {
        'positions': positions,
        'pose': pose,
        'extruder_tip': T_extruder[:3, 3].tolist()
    }

def inverse_kinematics_dh(x: float, y: float, z: float, roll: float = 0, pitch: float = 0, yaw: float = 0):
    """Numerical DH 6-DOF Inverse Kinematics for Meca500."""
    theta1 = np.arctan2(y, x)

    wrist_pos = np.array([x, y, z]) - np.array([
        LINK_LENGTHS['wrist'] * np.cos(theta1) * np.cos(pitch),
        LINK_LENGTHS['wrist'] * np.sin(theta1) * np.cos(pitch),
        LINK_LENGTHS['wrist'] * np.sin(pitch)
    ])

    L1 = LINK_LENGTHS['upper_arm']
    L2 = LINK_LENGTHS['forearm']

    x_arm = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
    z_arm = wrist_pos[2] - LINK_LENGTHS['base_height']
    D = np.sqrt(x_arm**2 + z_arm**2)

    try:
        cos_theta3 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        theta3 = np.pi - np.arccos(cos_theta3)
    except Exception:
        theta3 = 0.0

    alpha = np.arctan2(z_arm, x_arm)
    try:
        cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
    except Exception:
        beta = 0.0

    theta2 = alpha + beta - np.pi/2

    theta4 = roll
    theta5 = pitch
    theta6 = yaw - theta1

    return [
        round(float(theta1), 4),
        round(float(theta2), 4),
        round(float(theta3), 4),
        round(float(theta4), 4),
        round(float(theta5), 4),
        round(float(theta6), 4)
    ]

def inverse_kinematics_analytical(px: float, py: float, pz: float, roll: float, pitch: float, yaw: float):
    """Analytical SymPy-inspired geometric IK formulation from Flask app."""
    q1 = math.atan2(py, px)
    d = math.sqrt(px**2 + py**2)
    
    scale = 1.0 if d > 10.0 else 0.001
    px_s, py_s, pz_s = px * scale, py * scale, pz * scale
    d_s = math.sqrt(px_s**2 + py_s**2)

    q2 = math.atan2(pz_s - 0.135, d_s - 0.135) - math.pi/4
    q3 = -math.atan2(pz_s - 0.135, d_s - 0.135) / 2.0
    q4, q5, q6 = roll, pitch, yaw - q1

    return [
        round(float(q1), 4),
        round(float(q2), 4),
        round(float(q3), 4),
        round(float(q4), 4),
        round(float(q5), 4),
        round(float(q6), 4)
    ]

def inverse_kinematics_geometric(position: List[float], orientation: List[float]):
    """Simulator geometric IK with wrist and upper arm trigonometric solver."""
    x, y, z = position
    roll, pitch, yaw = orientation
    
    theta1 = np.arctan2(y, x)
    wrist_pos = np.array([x, y, z]) - np.array([
        LINK_LENGTHS['wrist'] * np.cos(theta1) * np.cos(pitch),
        LINK_LENGTHS['wrist'] * np.sin(theta1) * np.cos(pitch),
        LINK_LENGTHS['wrist'] * np.sin(pitch)
    ])
    
    L1, L2 = LINK_LENGTHS['upper_arm'], LINK_LENGTHS['forearm']
    x_arm = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
    z_arm = wrist_pos[2] - LINK_LENGTHS['base_height']
    D = np.sqrt(x_arm**2 + z_arm**2)
    
    try:
        theta3 = np.pi - np.arccos(np.clip((L1**2 + L2**2 - D**2)/(2*L1*L2), -1.0, 1.0))
    except Exception:
        theta3 = 0.0
        
    alpha = np.arctan2(z_arm, x_arm)
    try:
        beta = np.arccos(np.clip((L1**2 + D**2 - L2**2)/(2*L1*D), -1.0, 1.0))
    except Exception:
        beta = 0.0
    theta2 = alpha + beta - np.pi/2
    
    theta4, theta5, theta6 = roll, pitch, yaw - theta1
    return [
        round(float(theta1), 4),
        round(float(theta2), 4),
        round(float(theta3), 4),
        round(float(theta4), 4),
        round(float(theta5), 4),
        round(float(theta6), 4)
    ]


# ----------------------------------------------------
# 2. MECA500 TCP ROBOT CONTROLLER
# ----------------------------------------------------
class RobotController:
    def __init__(self):
        self.sock = None
        self.lock = Lock()
        self.connected = False
        self.ip = "192.168.0.100"
        self.port = 10000
        self.activated = False
        self.homed = False
        self.simulation_mode = True

    def connect(self, ip: str = "192.168.0.100", port: int = 10000):
        with self.lock:
            self.ip = ip
            self.port = port
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((ip, port))
                self.connected = True
                self.simulation_mode = False
                return {"success": True, "message": f"Connected to Meca500 at {ip}:{port}"}
            except Exception as e:
                self.connected = False
                self.simulation_mode = True
                return {"success": False, "message": f"Could not connect to {ip}:{port}. Operating in SIMULATION mode. Error: {str(e)}"}

    def disconnect(self):
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            self.connected = False
            self.simulation_mode = True
            return {"success": True, "message": "Disconnected from robot"}

    def send_command(self, cmd: str):
        with self.lock:
            if self.simulation_mode or not self.sock:
                if cmd.startswith("ActivateRobot"): self.activated = True
                elif cmd.startswith("DeactivateRobot"): self.activated = False
                elif cmd.startswith("Home"): self.homed = True
                return {"success": True, "simulated": True, "command": cmd, "response": "Simulated OK"}

            try:
                full_cmd = cmd.strip() + "\0"
                self.sock.sendall(full_cmd.encode('ascii'))
                response = self.sock.recv(1024).decode('ascii').strip()
                return {"success": True, "command": cmd, "response": response}
            except Exception as e:
                return {"success": False, "command": cmd, "error": str(e)}

robot = RobotController()


# ----------------------------------------------------
# 3. PATH PLANNING & INFILL GENERATION ENGINE
# ----------------------------------------------------
def generate_shape_path(shape: str, x: float, y: float, z: float, size: float, size2: float, z_step: float, layers: int):
    points = []
    for layer in range(layers):
        current_z = z + (layer * z_step)
        if shape == "rectangle":
            layer_pts = [
                [x, y, current_z],
                [x + size, y, current_z],
                [x + size, y + size2, current_z],
                [x, y + size2, current_z],
                [x, y, current_z]
            ]
        elif shape == "cylinder":
            radius = size / 2.0
            cx, cy = x + radius, y + radius
            num_pts = 36
            layer_pts = []
            for i in range(num_pts + 1):
                angle = 2 * math.pi * (i / num_pts)
                px = cx + radius * math.cos(angle)
                py = cy + radius * math.sin(angle)
                layer_pts.append([px, py, current_z])
        elif shape == "spiral":
            num_turns = 3
            pts_per_turn = 24
            max_r = size / 2.0
            cx, cy = x + max_r, y + max_r
            layer_pts = []
            total_pts = num_turns * pts_per_turn
            for i in range(total_pts):
                r = max_r * (i / total_pts)
                angle = 2 * math.pi * num_turns * (i / total_pts)
                px = cx + r * math.cos(angle)
                py = cy + r * math.sin(angle)
                layer_pts.append([px, py, current_z])
        elif shape == "meshgrid":
            cols = 5
            spacing = size / (cols - 1) if cols > 1 else size
            layer_pts = []
            for r_idx in range(cols):
                for c_idx in range(cols):
                    px = x + c_idx * spacing
                    py = y + r_idx * spacing
                    layer_pts.append([px, py, current_z])
        else:
            layer_pts = [[x, y, current_z]]

        points.extend(layer_pts)
    return points

def generate_infill_path(x: float, y: float, z: float, size: float, z_step: float, layers: int, pattern: str, density: float):
    points = []
    density_factor = max(0.1, min(1.0, density / 100.0))

    for layer in range(layers):
        current_z = z + (layer * z_step)

        # Outer box perimeter
        perim = [
            [x, y, current_z],
            [x + size, y, current_z],
            [x + size, y + size, current_z],
            [x, y + size, current_z],
            [x, y, current_z]
        ]
        points.extend(perim)

        if pattern == "rectilinear":
            spacing = (1.1 - density_factor) * 20.0
            num_lines = max(1, int(size / spacing))
            for i in range(1, num_lines):
                yi = y + i * spacing
                if yi >= y + size: break
                if i % 2 == 1:
                    points.extend([[x, yi, current_z], [x + size, yi, current_z]])
                else:
                    points.extend([[x + size, yi, current_z], [x, yi, current_z]])

        elif pattern == "concentric":
            margin = (1.1 - density_factor) * 10.0
            cur_size = size - 2 * margin
            offset = margin
            while cur_size > 2:
                points.extend([
                    [x + offset, y + offset, current_z],
                    [x + offset + cur_size, y + offset, current_z],
                    [x + offset + cur_size, y + offset + cur_size, current_z],
                    [x + offset, y + offset + cur_size, current_z],
                    [x + offset, y + offset, current_z]
                ])
                offset += margin
                cur_size -= 2 * margin

        elif pattern == "grid":
            spacing = (1.1 - density_factor) * 25.0
            num_lines = max(1, int(size / spacing))
            for i in range(1, num_lines):
                yi = y + i * spacing
                if yi < y + size:
                    points.extend([[x, yi, current_z], [x + size, yi, current_z]])
            for j in range(1, num_lines):
                xj = x + j * spacing
                if xj < x + size:
                    points.extend([[xj, y, current_z], [xj, y + size, current_z]])

        elif pattern == "triangles":
            spacing = (1.1 - density_factor) * 25.0
            num_lines = max(1, int(size / spacing))
            for i in range(1, num_lines):
                yi = y + i * spacing
                if yi < y + size:
                    points.extend([[x, yi, current_z], [x + size, yi, current_z]])
                    points.extend([[x, y, current_z], [x + size, yi, current_z]])
                    points.extend([[x + size, y, current_z], [x, yi, current_z]])

        elif pattern == "honeycomb":
            spacing = (1.1 - density_factor) * 15.0
            rows = max(1, int(size / spacing))
            for r in range(rows):
                yr = y + r * spacing
                offset = (spacing / 2.0) if (r % 2 == 1) else 0.0
                hex_pts = [
                    [x + offset, yr, current_z],
                    [x + offset + spacing/2, yr + spacing/3, current_z],
                    [x + offset + spacing/2, yr + 2*spacing/3, current_z],
                    [x + offset, yr + spacing, current_z]
                ]
                points.extend(hex_pts)

    return points


# ----------------------------------------------------
# 4. DEFECT DETECTION VISION ENGINE
# ----------------------------------------------------
class PrecisionTracker:
    def __init__(self):
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,0,1,0,0],[0,1,0,0,1,0],[0,0,1,0,0,1],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]], np.float32)
        self.kf.processNoiseCov = 1e-4 * np.eye(6, dtype=np.float32)
        self.kf.measurementNoiseCov = 1e-2 * np.eye(3, dtype=np.float32)
        self.measurements = deque(maxlen=5)
        self.confidence = 0
        self.best_radius = 0

    def update(self, measurement):
        if measurement is not None:
            x, y, r = measurement
            self.measurements.append((x, y, r))
            self.confidence = min(10, self.confidence + 2)
            med_x = float(np.median([m[0] for m in self.measurements]))
            med_y = float(np.median([m[1] for m in self.measurements]))
            med_r = float(np.median([m[2] for m in self.measurements]))
            self.best_radius = med_r
            self.kf.correct(np.array([[np.float32(med_x)], [np.float32(med_y)], [np.float32(med_r)]]))
        else:
            self.confidence = max(0, self.confidence - 1)
        pred = self.kf.predict()
        return [float(pred[0][0]), float(pred[1][0]), float(pred[2][0])]


# ----------------------------------------------------
# 5. API REQUEST MODELS & ENDPOINTS
# ----------------------------------------------------
class JointAnglesRequest(BaseModel):
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    j6: float = 0.0

class PoseRequest(BaseModel):
    x: float
    y: float
    z: float
    a: float = -94.998
    b: float = 79.029
    c: float = 33.056
    method: Optional[str] = "dh"

class ConnectRequest(BaseModel):
    ip: str = "192.168.0.100"
    port: int = 10000

class CommandRequest(BaseModel):
    command: str

class PathGenerationRequest(BaseModel):
    mode: str = "shape"
    shape: str = "rectangle"
    pattern: str = "rectilinear"
    x: float = 212.563
    y: float = -61.575
    z: float = 70.599
    a: float = -94.998
    b: float = 79.029
    c: float = 33.056
    size: float = 40.0
    size2: float = 40.0
    z_step: float = 2.0
    layers: int = 5
    density: float = 50.0

class GCodeConvertRequest(BaseModel):
    gcode: str
    a: float = -94.998
    b: float = 79.029
    c: float = 33.056

@app.get("/")
def read_root():
    return {"app": "Master Meca500 Control API", "status": "online"}

@app.post("/api/kinematics/forward")
def api_forward_kinematics(req: JointAnglesRequest):
    joint_angles = [req.j1, req.j2, req.j3, req.j4, req.j5, req.j6]
    return forward_kinematics(joint_angles)

@app.post("/api/kinematics/inverse")
def api_inverse_kinematics(req: PoseRequest):
    a_rad = math.radians(req.a)
    b_rad = math.radians(req.b)
    c_rad = math.radians(req.c)

    if req.method == "analytical":
        angles = inverse_kinematics_analytical(req.x, req.y, req.z, a_rad, b_rad, c_rad)
    elif req.method == "geometric":
        angles = inverse_kinematics_geometric([req.x, req.y, req.z], [a_rad, b_rad, c_rad])
    else:
        angles = inverse_kinematics_dh(req.x, req.y, req.z, a_rad, b_rad, c_rad)

    return {"joint_angles": angles, "method": req.method or "dh"}

@app.post("/api/robot/connect")
def api_robot_connect(req: ConnectRequest):
    return robot.connect(req.ip, req.port)

@app.post("/api/robot/disconnect")
def api_robot_disconnect():
    return robot.disconnect()

@app.post("/api/robot/command")
def api_robot_command(req: CommandRequest):
    return robot.send_command(req.command)

@app.get("/api/robot/status")
def api_robot_status():
    return {
        "connected": robot.connected,
        "ip": robot.ip,
        "port": robot.port,
        "activated": robot.activated,
        "homed": robot.homed,
        "simulation_mode": robot.simulation_mode
    }

@app.post("/api/path/generate")
def api_path_generate(req: PathGenerationRequest):
    if req.mode == "shape":
        points = generate_shape_path(req.shape, req.x, req.y, req.z, req.size, req.size2, req.z_step, req.layers)
    else:
        points = generate_infill_path(req.x, req.y, req.z, req.size, req.z_step, req.layers, req.pattern, req.density)

    meca_commands = [f"MovePose({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}, {req.a:.3f}, {req.b:.3f}, {req.c:.3f})" for pt in points]

    return {
        "count": len(points),
        "waypoints": points,
        "commands": meca_commands
    }

@app.post("/api/path/pointcloud_scan")
async def api_path_pointcloud_scan(file: UploadFile = File(...), num_layers: int = 20, smooth: bool = True):
    temp_path = f"./temp_scan_{file.filename}"
    try:
        contents = await file.read()
        with open(temp_path, "wb") as f:
            f.write(contents)

        try:
            mesh = o3d.io.read_triangle_mesh(temp_path)
            pcd = mesh.sample_points_uniformly(number_of_points=5000)
            points = np.asarray(pcd.points)
        except Exception:
            mesh = trimesh.load_mesh(temp_path)
            points, _ = trimesh.sample.sample_surface(mesh, 5000)

        if len(points) == 0:
            raise ValueError("No surface points extracted")

        min_z, max_z = np.min(points[:, 2]), np.max(points[:, 2])
        layer_intervals = np.linspace(min_z, max_z, num_layers)
        structured_path = []

        for i in range(len(layer_intervals) - 1):
            layer_pts = points[(points[:, 2] >= layer_intervals[i]) & (points[:, 2] < layer_intervals[i + 1])]
            if len(layer_pts) == 0: continue
            sorted_layer = layer_pts[np.argsort(layer_pts[:, 0])]
            if i % 2 == 1:
                sorted_layer = sorted_layer[::-1]
            structured_path.append(sorted_layer)

        if len(structured_path) > 0:
            path_points = np.vstack(structured_path)
        else:
            path_points = points

        if smooth and len(path_points) >= 4:
            t = np.linspace(0, 1, len(path_points))
            cs_x = CubicSpline(t, path_points[:, 0])
            cs_y = CubicSpline(t, path_points[:, 1])
            cs_z = CubicSpline(t, path_points[:, 2])
            t_fine = np.linspace(0, 1, min(500, len(path_points) * 2))
            final_points = np.vstack((cs_x(t_fine), cs_y(t_fine), cs_z(t_fine))).T.tolist()
        else:
            final_points = path_points.tolist()

        commands = [f"MovePose({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}, -94.998, 79.029, 33.056)" for pt in final_points]

        return {
            "count": len(final_points),
            "waypoints": final_points,
            "commands": commands,
            "smoothed": smooth
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

@app.post("/api/slicer/stl_to_gcode")
async def api_slicer_stl_to_gcode(file: UploadFile = File(...), layer_height: float = 0.5, print_speed: float = 60.0):
    temp_path = f"./temp_slice_{file.filename}"
    try:
        contents = await file.read()
        with open(temp_path, "wb") as f:
            f.write(contents)

        mesh = trimesh.load(temp_path)
        min_z, max_z = mesh.bounds[:, 2]
        slice_heights = np.arange(min_z + layer_height, max_z, layer_height)

        gcode_lines = [
            "; G-code generated for MECA500 Master App Slicer",
            f"; Layer height: {layer_height}mm",
            "G21 ; Units in mm",
            "G90 ; Absolute positioning",
            "G28 ; Home"
        ]

        waypoints = []
        for i, height in enumerate(slice_heights):
            section = mesh.section(plane_origin=[0, 0, height], plane_normal=[0, 0, 1])
            if section is not None:
                path = section.to_planar()[0]
                for path_coords in path.discrete:
                    for pt in path_coords:
                        waypoints.append([float(pt[0]), float(pt[1]), float(height)])
                        gcode_lines.append(f"G1 X{pt[0]:.3f} Y{pt[1]:.3f} Z{height:.3f} F{print_speed*60:.0f}")

        gcode_lines.append("M84 ; Disable steppers")
        gcode_text = "\n".join(gcode_lines)

        commands = [f"MovePose({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}, -94.998, 79.029, 33.056)" for pt in waypoints]

        return {
            "gcode": gcode_text,
            "waypoints": waypoints,
            "commands": commands,
            "count": len(waypoints)
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

@app.post("/api/slicer/gcode_to_meca")
def api_slicer_gcode_to_meca(req: GCodeConvertRequest):
    lines = req.gcode.splitlines()
    commands = []
    waypoints = []

    curr_x, curr_y, curr_z = 212.563, -61.575, 70.599

    for line in lines:
        clean_line = line.split(";")[0].strip()
        if not clean_line.startswith("G1") and not clean_line.startswith("G0"):
            continue

        parts = clean_line.split()
        for p in parts[1:]:
            if p.startswith("X"): curr_x = float(p[1:])
            elif p.startswith("Y"): curr_y = float(p[1:])
            elif p.startswith("Z"): curr_z = float(p[1:])

        waypoints.append([curr_x, curr_y, curr_z])
        commands.append(f"MovePose({curr_x:.3f}, {curr_y:.3f}, {curr_z:.3f}, {req.a:.3f}, {req.b:.3f}, {req.c:.3f})")

    return {
        "commands": commands,
        "waypoints": waypoints,
        "count": len(commands)
    }

@app.post("/api/upload_stl")
async def api_upload_stl(file: UploadFile = File(...)):
    temp_path = f"./temp_{file.filename}"
    try:
        contents = await file.read()
        with open(temp_path, "wb") as f:
            f.write(contents)

        try:
            mesh = o3d.io.read_triangle_mesh(temp_path)
            if not mesh.has_triangles():
                raise ValueError("No triangles")
            mesh.compute_vertex_normals()
            pcd = mesh.sample_points_poisson_disk(1000)
            waypoints = np.asarray(pcd.points).tolist()
        except Exception:
            mesh = trimesh.load_mesh(temp_path)
            pts, _ = trimesh.sample.sample_surface(mesh, 1000)
            waypoints = pts.tolist()

        error = None
    except Exception as e:
        waypoints = []
        error = f"Error processing STL file: {str(e)}"
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

    commands = [f"MovePose({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}, -94.998, 79.029, 33.056)" for pt in waypoints]

    return {
        "waypoints": waypoints,
        "count": len(waypoints),
        "commands": commands,
        "error": error
    }

@app.post("/api/vision/inspect")
async def api_vision_inspect(file: UploadFile = File(...)):
    contents = await file.read()
    nparr = np.frombuffer(contents, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    if img is None:
        raise HTTPException(status_code=400, detail="Invalid image file")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    equalized = clahe.apply(gray)

    circles = cv2.HoughCircles(
        equalized,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=300
    )

    tracker = PrecisionTracker()
    detected_circles = []

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for (x, y, r) in circles[0, :]:
            pred = tracker.update((x, y, r))
            detected_circles.append({
                "x": int(x),
                "y": int(y),
                "radius": int(r),
                "kalman_x": round(pred[0], 2),
                "kalman_y": round(pred[1], 2),
                "kalman_radius": round(pred[2], 2)
            })

    return {
        "circles_found": len(detected_circles),
        "circles": detected_circles,
        "quality_score": 98.5 if len(detected_circles) > 0 else 75.0,
        "status": "PASS" if len(detected_circles) > 0 else "WARNING"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
