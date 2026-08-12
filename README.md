# MECA 500 Master Control Platform

An integrated robotics software platform for the **Mecademic Meca500 (Six-Axis Industrial Robot)**, combining 6-DOF Kinematic Solvers, Parametric & Infill Toolpath Planning, 3D Mesh Surface Point-Cloud Scanning, G-code Conversion, Real-Time TCP Execution/Simulation, and Vision Defect Inspection into a unified full-stack application.

---

## 📁 Repository Structure

```text
MECA500_Master_App_Submission/
├── README.md                      # Primary Project Documentation
├── requirements.txt                # Python Backend Package Dependencies
├── run_app.bat                     # One-Click Double-Click Windows Batch Launcher
├── backend/
│   ├── server.py                   # FastAPI REST Server & Kinematics Engine
│   └── requirements.txt            # Backend Python Dependencies
├── frontend/
│   ├── package.json                # React Frontend Package Dependencies
│   ├── public/                     # Static HTML & Web Assets
│   └── src/
│       ├── App.js                  # Master React App UI Component
│       ├── App.css                 # Glassmorphic Dark Mode Design System
│       └── index.js                # React Entry Point
└── sample_models/
    └── sample_cube.stl             # Sample 3D Model for Slicing & Surface Scanning
```

---

## 🛠️ Prerequisites

Ensure the following tools are installed on your system before running:

- **Python**: `v3.9` or higher ([python.org](https://www.python.org/))
- **Node.js**: `v16.0` or higher ([nodejs.org](https://nodejs.org/))
- **npm**: Included with Node.js (`v8.0+`)

---

## 🚀 Quick Start Instructions

### Option 1: One-Click Launch (Windows)

Simply double-click **`run_app.bat`**. This script will:
1. Install Python backend dependencies.
2. Launch the FastAPI server at `http://localhost:8000`.
3. Install Node.js packages and launch the React dashboard at `http://localhost:3000`.

---

### Option 2: Manual Terminal Setup

#### Step 1: Start Python Backend Server
```bash
cd backend
pip install -r requirements.txt
python server.py
```
*The backend API will be live at `http://127.0.0.1:8000` with interactive Swagger API docs at `http://127.0.0.1:8000/docs`.*

#### Step 2: Start React Frontend Dashboard
Open a new terminal window:
```bash
cd frontend
npm install
npm start
```
*The web application will automatically open in your browser at `http://localhost:3000`.*

---

## ⚡ Key Architecture & Features

### 1. Multi-Algorithm Kinematics Engine
- **Numerical Denavit-Hartenberg (DH)**: Standard forward & inverse kinematics transformation matrices.
- **SymPy Analytical Solver**: Closed-form geometric inverse kinematics derivation.
- **Geometric Arm Solver**: Direct trigonometric upper arm and forearm link solver.

### 2. Path Planning & Infill Slicing Suite
- **Parametric Shapes**: Rectangle, Cylinder, Mesh Grid, and Spiral toolpaths.
- **Advanced Infill Patterns**: Rectilinear, Concentric, Grid, Triangles, and Honeycomb slicing with configurable density percentage (0–100%).
- **3D Surface Point-Cloud Scanner**: Surface sampling via Open3D / Trimesh with optional high-resolution **Cubic Spline Path Smoothing**.
- **G-Code Parser & MECA 500 Converter**: Direct conversion of standard G-code `G1` commands into Meca500 native `MovePose(x, y, z, a, b, c)` commands.

### 3. Real Robot TCP & Simulation Controller
- Socket connection over TCP (`Port 10000`) for direct execution on physical Meca500 hardware.
- Safe simulation fallback mode when disconnected.
- Step-by-step progress tracking, speed adjustment slider, and MECA script `.txt` exporter.

### 4. Vision Defect Inspection
- Computer vision pipeline using **Contrast Limited Adaptive Histogram Equalization (CLAHE)**, Hough Circle Transforms, and a 6-state **Kalman Filter (`PrecisionTracker`)** for real-time feature tracking and layer quality grading.

---

## 🔌 API Endpoint Documentation

| Endpoint | Method | Description |
| :--- | :--- | :--- |
| `GET /` | `GET` | API Health Check |
| `/api/kinematics/forward` | `POST` | Calculate end-effector position & posture from joint angles (`J1`..`J6`) |
| `/api/kinematics/inverse` | `POST` | Calculate joint angles using chosen solver (`dh`, `analytical`, `geometric`) |
| `/api/path/generate` | `POST` | Generate parametric shape or infill toolpath waypoints and MECA commands |
| `/api/path/pointcloud_scan` | `POST` | Extract surface point cloud from STL/OBJ with optional CubicSpline smoothing |
| `/api/slicer/stl_to_gcode` | `POST` | Slice 3D mesh into standard G-code lines |
| `/api/slicer/gcode_to_meca` | `POST` | Convert raw G-code text to MECA 500 `MovePose` commands |
| `/api/vision/inspect` | `POST` | Analyze layer snapshot image for defects & return Kalman tracker metrics |
| `/api/robot/connect` | `POST` | Connect to physical Meca500 IP address via TCP socket |
| `/api/robot/command` | `POST` | Send single raw ASCII command (`ActivateRobot`, `Home`, `ResetError`, etc.) |

---

## 🎓 Academic Reference & Future Scope

This project consolidates research code developed for MECA 500 robotic 3D printing and trajectory control. Future expansions include:
- Multi-robot synchronized collision detection.
- Real-time closed-loop visual servoing during active printing.
