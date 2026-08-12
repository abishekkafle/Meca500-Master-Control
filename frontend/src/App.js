import React, { useState, useEffect, useRef } from "react";
import axios from "axios";
import Plot from "react-plotly.js";
import "./App.css";
import {
  FaRobot,
  FaSlidersH,
  FaLayerGroup,
  FaPlay,
  FaPause,
  FaStop,
  FaEye,
  FaFileUpload,
  FaDownload,
  FaPlug,
  FaHome,
  FaRedo,
  FaCheckCircle,
  FaExclamationTriangle,
  FaBolt,
  FaCode,
  FaCube,
  FaWaveSquare
} from "react-icons/fa";

const API_BASE = "http://127.0.0.1:8000";

const App = () => {
  // Navigation
  const [activeTab, setActiveTab] = useState("kinematics"); // kinematics | path | execution | vision

  // Robot Connection State
  const [robotIp, setRobotIp] = useState("192.168.0.100");
  const [robotPort, setRobotPort] = useState(10000);
  const [robotStatus, setRobotStatus] = useState({
    connected: false,
    simulation_mode: true,
    activated: false,
    homed: false
  });
  const [consoleLogs, setConsoleLogs] = useState(["[SYSTEM] Meca500 Master App Initialized."]);

  // Joint Kinematics State
  const [ikMethod, setIkMethod] = useState("dh"); // dh | analytical | geometric
  const [joints, setJoints] = useState({ j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 });
  const [poseTarget, setPoseTarget] = useState({ x: 212.563, y: -61.575, z: 70.599, a: -94.998, b: 79.029, c: 33.056 });
  const [armPositions, setArmPositions] = useState([]);
  const [fkPose, setFkPose] = useState({ x: 0, y: 0, z: 0, a: 0, b: 0, c: 0 });

  // Path & Infill Slicing State
  const [pathMode, setPathMode] = useState("shape"); // shape | infill | pointcloud | gcode
  const [shape, setShape] = useState("rectangle");
  const [infillPattern, setInfillPattern] = useState("rectilinear");
  const [pathParams, setPathParams] = useState({
    x: 212.563,
    y: -61.575,
    z: 70.599,
    a: -94.998,
    b: 79.029,
    c: 33.056,
    size: 40.0,
    size2: 40.0,
    z_step: 2.0,
    layers: 5,
    density: 50
  });
  const [waypoints, setWaypoints] = useState([]);
  const [mecaCommands, setMecaCommands] = useState([]);
  const [stlFile, setStlFile] = useState(null);
  const [scanSmooth, setScanSmooth] = useState(true);
  const [gcodeText, setGcodeText] = useState("");
  const [isGenerating, setIsGenerating] = useState(false);

  // Playback / Simulation State
  const [isPlaying, setIsPlaying] = useState(false);
  const [currentStep, setCurrentStep] = useState(0);
  const [playbackSpeed, setPlaybackSpeed] = useState(100); // ms per step
  const playbackRef = useRef(null);

  // Vision Defect Detection State
  const [visionFile, setVisionFile] = useState(null);
  const [visionResult, setVisionResult] = useState(null);
  const [isInspecting, setIsInspecting] = useState(false);

  // Helper logging
  const addLog = (msg) => {
    setConsoleLogs((prev) => [`[${new Date().toLocaleTimeString()}] ${msg}`, ...prev.slice(0, 49)]);
  };

  // Fetch Kinematics when joints update
  const fetchForwardKinematics = async (currentJoints) => {
    try {
      const res = await axios.post(`${API_BASE}/api/kinematics/forward`, currentJoints);
      setArmPositions(res.data.positions);
      setFkPose(res.data.pose);
    } catch (err) {
      console.error("FK Error", err);
    }
  };

  useEffect(() => {
    fetchForwardKinematics(joints);
  }, [joints]);

  // Handle Joint Slider
  const handleJointChange = (jKey, val) => {
    const updated = { ...joints, [jKey]: parseFloat(val) };
    setJoints(updated);
  };

  // Calculate IK from Pose inputs using selected solver
  const handleCalculateIK = async () => {
    try {
      const res = await axios.post(`${API_BASE}/api/kinematics/inverse`, {
        ...poseTarget,
        method: ikMethod
      });
      const [j1, j2, j3, j4, j5, j6] = res.data.joint_angles;
      const newJoints = { j1, j2, j3, j4, j5, j6 };
      setJoints(newJoints);
      addLog(`Inverse Kinematics (${ikMethod.toUpperCase()}) calculated. J: [${j1}, ${j2}, ${j3}, ${j4}, ${j5}, ${j6}]`);
    } catch (err) {
      addLog(`IK Error (${ikMethod}): ${err.message}`);
    }
  };

  // Connect to Robot
  const handleConnect = async () => {
    try {
      const res = await axios.post(`${API_BASE}/api/robot/connect`, { ip: robotIp, port: parseInt(robotPort) });
      addLog(res.data.message);
      checkRobotStatus();
    } catch (err) {
      addLog(`Connection failed: ${err.message}`);
    }
  };

  const handleDisconnect = async () => {
    try {
      const res = await axios.post(`${API_BASE}/api/robot/disconnect`);
      addLog(res.data.message);
      checkRobotStatus();
    } catch (err) {
      addLog(`Disconnect failed: ${err.message}`);
    }
  };

  const checkRobotStatus = async () => {
    try {
      const res = await axios.get(`${API_BASE}/api/robot/status`);
      setRobotStatus(res.data);
    } catch (err) {
      console.error("Status check failed", err);
    }
  };

  const sendRobotCommand = async (cmd) => {
    try {
      const res = await axios.post(`${API_BASE}/api/robot/command`, { command: cmd });
      addLog(`Command: ${cmd} -> ${res.data.response || "OK"}`);
      checkRobotStatus();
    } catch (err) {
      addLog(`Command error: ${err.message}`);
    }
  };

  // Path & Slicing Generation
  const handleGeneratePath = async () => {
    setIsGenerating(true);
    try {
      if (pathMode === "pointcloud") {
        if (!stlFile) {
          alert("Please upload an STL/OBJ file first.");
          setIsGenerating(false);
          return;
        }
        const formData = new FormData();
        formData.append("file", stlFile);
        const res = await axios.post(
          `${API_BASE}/api/path/pointcloud_scan?num_layers=${pathParams.layers}&smooth=${scanSmooth}`,
          formData
        );
        setWaypoints(res.data.waypoints);
        setMecaCommands(res.data.commands);
        addLog(`Extracted ${res.data.count} points with ${scanSmooth ? "CubicSpline Smooth" : "Raw"} scanning.`);
      } else if (pathMode === "gcode") {
        if (stlFile) {
          const formData = new FormData();
          formData.append("file", stlFile);
          const res = await axios.post(`${API_BASE}/api/slicer/stl_to_gcode?layer_height=${pathParams.z_step}`, formData);
          setGcodeText(res.data.gcode);
          setWaypoints(res.data.waypoints);
          setMecaCommands(res.data.commands);
          addLog(`Sliced STL to G-code. Generated ${res.data.count} move commands.`);
        } else if (gcodeText.trim()) {
          const res = await axios.post(`${API_BASE}/api/slicer/gcode_to_meca`, {
            gcode: gcodeText,
            a: pathParams.a,
            b: pathParams.b,
            c: pathParams.c
          });
          setWaypoints(res.data.waypoints);
          setMecaCommands(res.data.commands);
          addLog(`Converted G-code to MECA500 MovePose commands. Total: ${res.data.count}`);
        } else {
          alert("Upload an STL file or paste G-code text.");
        }
      } else {
        const payload = {
          mode: pathMode,
          shape: shape,
          pattern: infillPattern,
          ...pathParams
        };
        const res = await axios.post(`${API_BASE}/api/path/generate`, payload);
        setWaypoints(res.data.waypoints);
        setMecaCommands(res.data.commands);
        addLog(`Generated ${res.data.count} waypoints for ${pathMode === "shape" ? shape : infillPattern}.`);
      }
    } catch (err) {
      addLog(`Path generation error: ${err.message}`);
    } finally {
      setIsGenerating(false);
    }
  };

  // Download Commands File
  const handleDownloadCommands = () => {
    if (mecaCommands.length === 0) {
      alert("No generated commands to download.");
      return;
    }
    const element = document.createElement("a");
    const file = new Blob([mecaCommands.join("\n")], { type: "text/plain" });
    element.href = URL.createObjectURL(file);
    element.download = "meca500_commands.txt";
    document.body.appendChild(element);
    element.click();
    document.body.removeChild(element);
  };

  // Playback Simulation Engine
  useEffect(() => {
    if (isPlaying && waypoints.length > 0) {
      playbackRef.current = setInterval(() => {
        setCurrentStep((prevStep) => {
          const nextStep = prevStep + 1;
          if (nextStep >= waypoints.length) {
            setIsPlaying(false);
            addLog("Simulation execution finished.");
            return 0;
          }
          const pt = waypoints[nextStep];
          axios
            .post(`${API_BASE}/api/kinematics/inverse`, {
              x: pt[0],
              y: pt[1],
              z: pt[2],
              a: pathParams.a,
              b: pathParams.b,
              c: pathParams.c,
              method: ikMethod
            })
            .then((res) => {
              const [j1, j2, j3, j4, j5, j6] = res.data.joint_angles;
              setJoints({ j1, j2, j3, j4, j5, j6 });
            })
            .catch(() => {});

          if (robotStatus.connected && mecaCommands[nextStep]) {
            sendRobotCommand(mecaCommands[nextStep]);
          }

          return nextStep;
        });
      }, playbackSpeed);
    } else {
      clearInterval(playbackRef.current);
    }
    return () => clearInterval(playbackRef.current);
  }, [isPlaying, waypoints, playbackSpeed, robotStatus.connected, ikMethod]);

  // Vision Defect Detection
  const handleInspectImage = async () => {
    if (!visionFile) {
      alert("Upload a layer snapshot image first.");
      return;
    }
    setIsInspecting(true);
    try {
      const formData = new FormData();
      formData.append("file", visionFile);
      const res = await axios.post(`${API_BASE}/api/vision/inspect`, formData);
      setVisionResult(res.data);
      addLog(`Defect inspection complete. Found ${res.data.circles_found} features. Status: ${res.data.status}`);
    } catch (err) {
      addLog(`Inspection error: ${err.message}`);
    } finally {
      setIsInspecting(false);
    }
  };

  return (
    <div className="dashboard-container">
      {/* Top Header & Connection Bar */}
      <header className="top-header">
        <div className="brand">
          <FaRobot className="brand-icon" />
          <div>
            <h1>Meca500 Master Control</h1>
            <p style={{ fontSize: "11px", color: "var(--text-secondary)" }}>Consolidated Kinematics, Path & Vision Suite</p>
          </div>
        </div>

        <div className="connection-bar">
          <div className="input-group">
            <label>IP:</label>
            <input type="text" className="input-field" value={robotIp} onChange={(e) => setRobotIp(e.target.value)} />
          </div>
          <div className="input-group">
            <label>Port:</label>
            <input type="number" className="input-field" style={{ width: "60px" }} value={robotPort} onChange={(e) => setRobotPort(e.target.value)} />
          </div>

          {!robotStatus.connected ? (
            <button className="btn btn-primary" onClick={handleConnect}>
              <FaPlug /> Connect
            </button>
          ) : (
            <button className="btn btn-danger" onClick={handleDisconnect}>
              Disconnect
            </button>
          )}

          <span className={`badge ${robotStatus.connected ? "badge-connected" : robotStatus.simulation_mode ? "badge-simulation" : "badge-disconnected"}`}>
            {robotStatus.connected ? "REAL ROBOT" : "SIMULATION MODE"}
          </span>

          <div style={{ display: "flex", gap: "6px", marginLeft: "12px" }}>
            <button className="btn btn-secondary" title="Activate" onClick={() => sendRobotCommand("ActivateRobot")}>
              <FaBolt /> Activate
            </button>
            <button className="btn btn-secondary" title="Home" onClick={() => sendRobotCommand("Home")}>
              <FaHome /> Home
            </button>
            <button className="btn btn-secondary" title="Reset Error" onClick={() => sendRobotCommand("ResetError")}>
              <FaRedo /> Reset
            </button>
          </div>
        </div>
      </header>

      {/* Main Tab Navigation */}
      <nav className="tab-nav">
        <button className={`tab-btn ${activeTab === "kinematics" ? "active" : ""}`} onClick={() => setActiveTab("kinematics")}>
          <FaSlidersH /> Kinematics & 3D Simulator
        </button>
        <button className={`tab-btn ${activeTab === "path" ? "active" : ""}`} onClick={() => setActiveTab("path")}>
          <FaLayerGroup /> Path & Slicing Suite
        </button>
        <button className={`tab-btn ${activeTab === "execution" ? "active" : ""}`} onClick={() => setActiveTab("execution")}>
          <FaPlay /> Live Print Simulation
        </button>
        <button className={`tab-btn ${activeTab === "vision" ? "active" : ""}`} onClick={() => setActiveTab("vision")}>
          <FaEye /> Defect Inspection Vision
        </button>
      </nav>

      {/* Main Content Area */}
      <main className="main-content">
        {/* TAB 1: KINEMATICS & 3D SIMULATOR */}
        {activeTab === "kinematics" && (
          <div className="grid-2col">
            <div className="panel">
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                <h2><FaSlidersH /> Joint Controls & IK Solvers</h2>
                <div style={{ display: "flex", alignItems: "center", gap: "6px" }}>
                  <label style={{ fontSize: "12px", color: "var(--text-secondary)" }}>Solver:</label>
                  <select className="select-field" value={ikMethod} onChange={(e) => setIkMethod(e.target.value)}>
                    <option value="dh">Numerical DH</option>
                    <option value="analytical">SymPy Analytical</option>
                    <option value="geometric">Geometric Arm</option>
                  </select>
                </div>
              </div>

              <div style={{ margin: "14px 0" }}>
                {Object.keys(joints).map((jKey, idx) => (
                  <div className="slider-group" key={jKey}>
                    <label>
                      Joint {idx + 1} ({jKey.toUpperCase()}):
                    </label>
                    <input
                      type="range"
                      min={idx === 1 || idx === 4 ? -2.61 : -3.14}
                      max={idx === 1 || idx === 4 ? 2.61 : 3.14}
                      step="0.01"
                      value={joints[jKey]}
                      onChange={(e) => handleJointChange(jKey, e.target.value)}
                    />
                    <span className="slider-val">
                      {joints[jKey].toFixed(3)} rad ({(joints[jKey] * (180 / Math.PI)).toFixed(1)}°)
                    </span>
                  </div>
                ))}
              </div>

              <button className="btn btn-secondary" style={{ width: "100%", marginBottom: "20px" }} onClick={() => setJoints({ j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 })}>
                Reset Joints to Zero
              </button>

              <hr style={{ borderColor: "rgba(255,255,255,0.1)", margin: "20px 0" }} />

              <h2><FaRobot /> Pose Target IK Solver</h2>
              <div className="form-grid-3">
                <div className="input-group">
                  <label>X (mm)</label>
                  <input type="number" className="input-field" value={poseTarget.x} onChange={(e) => setPoseTarget({ ...poseTarget, x: parseFloat(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>Y (mm)</label>
                  <input type="number" className="input-field" value={poseTarget.y} onChange={(e) => setPoseTarget({ ...poseTarget, y: parseFloat(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>Z (mm)</label>
                  <input type="number" className="input-field" value={poseTarget.z} onChange={(e) => setPoseTarget({ ...poseTarget, z: parseFloat(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>A (deg)</label>
                  <input type="number" className="input-field" value={poseTarget.a} onChange={(e) => setPoseTarget({ ...poseTarget, a: parseFloat(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>B (deg)</label>
                  <input type="number" className="input-field" value={poseTarget.b} onChange={(e) => setPoseTarget({ ...poseTarget, b: parseFloat(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>C (deg)</label>
                  <input type="number" className="input-field" value={poseTarget.c} onChange={(e) => setPoseTarget({ ...poseTarget, c: parseFloat(e.target.value) })} />
                </div>
              </div>
              <button className="btn btn-primary" style={{ width: "100%", marginTop: "12px" }} onClick={handleCalculateIK}>
                Calculate & Set IK ({ikMethod.toUpperCase()})
              </button>

              <div style={{ marginTop: "16px", display: "flex", justifyContent: "space-around", fontSize: "12px", background: "rgba(0,0,0,0.3)", padding: "10px", borderRadius: "8px" }}>
                <span>FK X: <strong>{fkPose.x}</strong></span>
                <span>FK Y: <strong>{fkPose.y}</strong></span>
                <span>FK Z: <strong>{fkPose.z}</strong></span>
              </div>
            </div>

            <div className="panel">
              <h2>Interactive 3D Arm Visualization</h2>
              <div className="plot-container">
                <Plot
                  data={[
                    // Arm Skeleton Links
                    {
                      x: armPositions.slice(0, -1).map((p) => p[0]),
                      y: armPositions.slice(0, -1).map((p) => p[1]),
                      z: armPositions.slice(0, -1).map((p) => p[2]),
                      type: "scatter3d",
                      mode: "lines+markers",
                      marker: { size: 6, color: "#00f2fe" },
                      line: { width: 8, color: "#00f2fe" },
                      name: "Meca500 Arm"
                    },
                    // Extruder Nozzle Tip
                    {
                      x: armPositions.length > 0 ? [armPositions[armPositions.length - 2][0], armPositions[armPositions.length - 1][0]] : [],
                      y: armPositions.length > 0 ? [armPositions[armPositions.length - 2][1], armPositions[armPositions.length - 1][1]] : [],
                      z: armPositions.length > 0 ? [armPositions[armPositions.length - 2][2], armPositions[armPositions.length - 1][2]] : [],
                      type: "scatter3d",
                      mode: "lines+markers",
                      marker: { size: 5, color: "#ff4b2b" },
                      line: { width: 6, color: "#ff4b2b" },
                      name: "Extruder Nozzle"
                    }
                  ]}
                  layout={{
                    autosize: true,
                    paper_bgcolor: "transparent",
                    plot_bgcolor: "transparent",
                    margin: { l: 0, r: 0, b: 0, t: 0 },
                    scene: {
                      xaxis: { range: [-350, 350], title: "X" },
                      yaxis: { range: [-350, 350], title: "Y" },
                      zaxis: { range: [0, 450], title: "Z" },
                      camera: { eye: { x: 1.5, y: 1.5, z: 1.2 } }
                    }
                  }}
                  style={{ width: "100%", height: "100%" }}
                  useResizeHandler={true}
                />
              </div>
            </div>
          </div>
        )}

        {/* TAB 2: PATH & SLICING SUITE */}
        {activeTab === "path" && (
          <div className="grid-2col">
            <div className="panel">
              <h2><FaLayerGroup /> Path Generator & Slicer Config</h2>
              <div style={{ display: "flex", gap: "8px", margin: "14px 0" }}>
                <button className={`btn ${pathMode === "shape" ? "btn-primary" : "btn-secondary"}`} onClick={() => setPathMode("shape")}>
                  <FaCube /> Shapes
                </button>
                <button className={`btn ${pathMode === "infill" ? "btn-primary" : "btn-secondary"}`} onClick={() => setPathMode("infill")}>
                  <FaWaveSquare /> Infills
                </button>
                <button className={`btn ${pathMode === "pointcloud" ? "btn-primary" : "btn-secondary"}`} onClick={() => setPathMode("pointcloud")}>
                  <FaRobot /> 3D Surface Scanner
                </button>
                <button className={`btn ${pathMode === "gcode" ? "btn-primary" : "btn-secondary"}`} onClick={() => setPathMode("gcode")}>
                  <FaCode /> G-Code / Slicer
                </button>
              </div>

              {pathMode === "shape" && (
                <div className="form-grid-2">
                  <div className="input-group">
                    <label>Shape</label>
                    <select className="select-field" value={shape} onChange={(e) => setShape(e.target.value)}>
                      <option value="rectangle">Rectangle</option>
                      <option value="cylinder">Cylinder</option>
                      <option value="meshgrid">Mesh Grid</option>
                      <option value="spiral">Spiral</option>
                    </select>
                  </div>
                  <div className="input-group">
                    <label>Size 1 (mm)</label>
                    <input type="number" className="input-field" value={pathParams.size} onChange={(e) => setPathParams({ ...pathParams, size: parseFloat(e.target.value) })} />
                  </div>
                </div>
              )}

              {pathMode === "infill" && (
                <div className="form-grid-2">
                  <div className="input-group">
                    <label>Infill Pattern</label>
                    <select className="select-field" value={infillPattern} onChange={(e) => setInfillPattern(e.target.value)}>
                      <option value="rectilinear">Rectilinear</option>
                      <option value="concentric">Concentric</option>
                      <option value="grid">Grid</option>
                      <option value="triangles">Triangles</option>
                      <option value="honeycomb">Honeycomb</option>
                    </select>
                  </div>
                  <div className="input-group">
                    <label>Density (%)</label>
                    <input type="number" className="input-field" value={pathParams.density} onChange={(e) => setPathParams({ ...pathParams, density: parseFloat(e.target.value) })} />
                  </div>
                </div>
              )}

              {pathMode === "pointcloud" && (
                <div>
                  <div className="input-group" style={{ marginBottom: "12px" }}>
                    <label>Upload STL / OBJ Model:</label>
                    <input type="file" accept=".stl,.obj" className="input-field" onChange={(e) => setStlFile(e.target.files[0])} />
                  </div>
                  <div style={{ display: "flex", alignItems: "center", gap: "10px" }}>
                    <input type="checkbox" checked={scanSmooth} onChange={(e) => setScanSmooth(e.target.checked)} />
                    <label style={{ fontSize: "13px" }}>Enable Cubic Spline Path Smoothing</label>
                  </div>
                </div>
              )}

              {pathMode === "gcode" && (
                <div>
                  <div className="input-group" style={{ marginBottom: "12px" }}>
                    <label>Upload STL to Slice or Paste G-code:</label>
                    <input type="file" accept=".stl,.gcode,.txt" className="input-field" onChange={(e) => setStlFile(e.target.files[0])} />
                  </div>
                  <textarea
                    className="input-field"
                    style={{ height: "100px", fontFamily: "monospace", fontSize: "11px" }}
                    placeholder="G1 X100 Y50 Z10 F3000..."
                    value={gcodeText}
                    onChange={(e) => setGcodeText(e.target.value)}
                  />
                </div>
              )}

              <hr style={{ borderColor: "rgba(255,255,255,0.1)", margin: "16px 0" }} />

              <div className="form-grid-3">
                <div className="input-group">
                  <label>Z Step (mm)</label>
                  <input type="number" className="input-field" value={pathParams.z_step} onChange={(e) => setPathParams({ ...pathParams, z_step: parseFloat(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>Layers</label>
                  <input type="number" className="input-field" value={pathParams.layers} onChange={(e) => setPathParams({ ...pathParams, layers: parseInt(e.target.value) })} />
                </div>
                <div className="input-group">
                  <label>Count</label>
                  <input type="number" className="input-field" value={waypoints.length} disabled />
                </div>
              </div>

              <div style={{ display: "flex", gap: "10px", marginTop: "16px" }}>
                <button className="btn btn-primary" style={{ flex: 1 }} onClick={handleGeneratePath} disabled={isGenerating}>
                  {isGenerating ? "Generating..." : "Generate Path & Commands"}
                </button>
                <button className="btn btn-secondary" onClick={handleDownloadCommands}>
                  <FaDownload /> Download Script
                </button>
              </div>
            </div>

            <div className="panel">
              <h2>Generated Waypoints Visualization</h2>
              <div className="plot-container">
                <Plot
                  data={[
                    {
                      x: waypoints.map((p) => p[0]),
                      y: waypoints.map((p) => p[1]),
                      z: waypoints.map((p) => p[2]),
                      type: "scatter3d",
                      mode: "lines+markers",
                      marker: { size: 3, color: waypoints.map((p) => p[2]), colorscale: "Viridis" },
                      line: { width: 4, color: "#4facfe" },
                      name: "Print Toolpath"
                    }
                  ]}
                  layout={{
                    autosize: true,
                    paper_bgcolor: "transparent",
                    plot_bgcolor: "transparent",
                    margin: { l: 0, r: 0, b: 0, t: 0 },
                    scene: { aspectmode: "data" }
                  }}
                  style={{ width: "100%", height: "100%" }}
                  useResizeHandler={true}
                />
              </div>
            </div>
          </div>
        )}

        {/* TAB 3: LIVE PRINT SIMULATION */}
        {activeTab === "execution" && (
          <div className="grid-2col">
            <div className="panel">
              <h2><FaPlay /> Live Print Simulation Engine</h2>

              <div className="controls-bar">
                {!isPlaying ? (
                  <button className="btn btn-primary" onClick={() => setIsPlaying(true)} disabled={waypoints.length === 0}>
                    <FaPlay /> Start Simulation
                  </button>
                ) : (
                  <button className="btn btn-danger" onClick={() => setIsPlaying(false)}>
                    <FaPause /> Pause
                  </button>
                )}
                <button className="btn btn-secondary" onClick={() => setCurrentStep(0)}>
                  <FaStop /> Reset Progress
                </button>
              </div>

              <div style={{ margin: "20px 0" }}>
                <label style={{ fontSize: "12px", color: "var(--text-secondary)" }}>Speed: {playbackSpeed} ms/step</label>
                <input
                  type="range"
                  min="20"
                  max="500"
                  value={playbackSpeed}
                  onChange={(e) => setPlaybackSpeed(parseInt(e.target.value))}
                />
              </div>

              <div style={{ background: "rgba(0,0,0,0.3)", padding: "14px", borderRadius: "8px" }}>
                <p style={{ fontSize: "13px", color: "var(--text-secondary)" }}>
                  Step: <strong>{currentStep} / {waypoints.length}</strong>
                </p>
                <p style={{ fontSize: "12px", color: "#00f2fe", wordBreak: "break-all" }}>
                  Current Command: <code>{mecaCommands[currentStep] || "N/A"}</code>
                </p>
              </div>
            </div>

            <div className="panel">
              <h2>Layer Build Visualizer</h2>
              <div className="plot-container">
                <Plot
                  data={[
                    {
                      x: waypoints.slice(0, currentStep).map((p) => p[0]),
                      y: waypoints.slice(0, currentStep).map((p) => p[1]),
                      z: waypoints.slice(0, currentStep).map((p) => p[2]),
                      type: "scatter3d",
                      mode: "lines",
                      line: { width: 6, color: "#ff0844" },
                      name: "Printed Material"
                    },
                    {
                      x: armPositions.slice(0, -1).map((p) => p[0]),
                      y: armPositions.slice(0, -1).map((p) => p[1]),
                      z: armPositions.slice(0, -1).map((p) => p[2]),
                      type: "scatter3d",
                      mode: "lines+markers",
                      marker: { size: 5, color: "#00f2fe" },
                      line: { width: 6, color: "#00f2fe" },
                      name: "Robot Arm"
                    }
                  ]}
                  layout={{
                    autosize: true,
                    paper_bgcolor: "transparent",
                    plot_bgcolor: "transparent",
                    margin: { l: 0, r: 0, b: 0, t: 0 },
                    scene: { aspectmode: "data" }
                  }}
                  style={{ width: "100%", height: "100%" }}
                  useResizeHandler={true}
                />
              </div>
            </div>
          </div>
        )}

        {/* TAB 4: DEFECT INSPECTION VISION */}
        {activeTab === "vision" && (
          <div className="grid-2col">
            <div className="panel">
              <h2><FaEye /> Vision Inspection & Defect Detection</h2>
              <div className="input-group" style={{ margin: "16px 0" }}>
                <label>Upload Layer Snapshot Image:</label>
                <input type="file" accept="image/*" className="input-field" onChange={(e) => setVisionFile(e.target.files[0])} />
              </div>

              <button className="btn btn-primary" style={{ width: "100%" }} onClick={handleInspectImage} disabled={isInspecting}>
                {isInspecting ? "Inspecting..." : "Run CLAHE & Kalman Circle Inspection"}
              </button>

              {visionResult && (
                <div style={{ marginTop: "20px", background: "rgba(0,0,0,0.3)", padding: "16px", borderRadius: "8px" }}>
                  <h3>Inspection Results</h3>
                  <p>Features / Circles Found: <strong>{visionResult.circles_found}</strong></p>
                  <p>Layer Quality Score: <strong>{visionResult.quality_score}%</strong></p>
                  <p>Status: <span className={`badge ${visionResult.status === "PASS" ? "badge-connected" : "badge-disconnected"}`}>{visionResult.status}</span></p>
                </div>
              )}
            </div>

            <div className="panel">
              <h2>Inspection Image Preview</h2>
              {visionFile ? (
                <img
                  src={URL.createObjectURL(visionFile)}
                  alt="Snapshot Preview"
                  style={{ maxWidth: "100%", maxHeight: "350px", borderRadius: "8px", objectFit: "contain", border: "1px solid rgba(255,255,255,0.1)" }}
                />
              ) : (
                <div style={{ height: "300px", display: "flex", justifyContent: "center", alignItems: "center", color: "var(--text-secondary)" }}>
                  Upload a layer image to analyze.
                </div>
              )}
            </div>
          </div>
        )}
      </main>

      {/* Footer Console Log */}
      <footer className="console-log">
        <p style={{ margin: 0, fontWeight: "bold", fontSize: "11px", color: "var(--text-secondary)" }}>Console Event Log:</p>
        <div style={{ maxHeight: "60px", overflowY: "auto", fontSize: "11px", fontFamily: "monospace" }}>
          {consoleLogs.map((log, index) => (
            <div key={index}>{log}</div>
          ))}
        </div>
      </footer>
    </div>
  );
};

export default App;
