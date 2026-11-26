/**
 * Interactive UI controller for the Meca500 6-DOF Automation Digital Twin.
 */

const M = window.MecaTwin;
const el = (id) => document.getElementById(id);

const state = {
  jointsDeg: [0, -15, 35, 0, 55, 0],
  pose: { x: 185, y: -35, z: 75, a: -95, b: 79, c: 33 },
  pathMode: "shape",
  shape: "spiral",
  pattern: "rectilinear",
  pathX: 165,
  pathY: -35,
  pathZ: 65,
  size: 55,
  zStep: 3,
  layers: 4,
  density: 55,
  feedRate: 45,
  waypoints: [],
  commands: [],
  audit: null,
  isPlaying: false,
  step: 0,
  timer: null,
  obstacle: { enabled: true, xMin: 180, xMax: 215, yMin: 5, yMax: 40, zMin: 0, zMax: 85 },
  logs: ["[TWIN] Meca500 6-DOF Automation Digital Twin initialized (DH Kinematics + Jacobian + Safety Interlocks)."],
};

function log(msg) {
  const ts = new Date().toLocaleTimeString();
  state.logs.unshift(`[${ts}] ${msg}`);
  if (state.logs.length > 40) state.logs.pop();
  el("consoleBox").innerHTML = state.logs.map((l) => `<div>${l}</div>`).join("");
}

function jointsRad() {
  return state.jointsDeg.map((d) => (d * Math.PI) / 180);
}

function buildObstacleTraces() {
  const o = state.obstacle;
  if (!o.enabled) return [];
  const x = [o.xMin, o.xMax, o.xMax, o.xMin, o.xMin, o.xMax, o.xMax, o.xMin];
  const y = [o.yMin, o.yMin, o.yMax, o.yMax, o.yMin, o.yMin, o.yMax, o.yMax];
  const z = [o.zMin, o.zMin, o.zMin, o.zMin, o.zMax, o.zMax, o.zMax, o.zMax];
  return [
    {
      type: "mesh3d",
      x, y, z,
      i: [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
      j: [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
      k: [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
      opacity: 0.28,
      color: "#ef4444",
      name: "Keep-Out Fixture",
    },
  ];
}

function buildPlateTrace() {
  return {
    type: "mesh3d",
    x: [110, 260, 260, 110],
    y: [-80, -80, 80, 80],
    z: [0, 0, 0, 0],
    i: [0, 0], j: [1, 2], k: [2, 3],
    opacity: 0.18,
    color: "#38bdf8",
    name: "Build Plate (Z=0)",
  };
}

function renderKPIs() {
  const fk = M.forwardKinematics(jointsRad());
  const manip = M.computeManipulability(jointsRad());
  const audit = state.audit;

  const manipCls = manip.singularityRisk === "LOW" ? "ok" : manip.singularityRisk === "MODERATE" ? "warn" : "danger";
  const safetyCls = audit && audit.collisionCount === 0 && audit.reachErrors === 0 ? "ok" : "danger";

  el("kpiGrid").innerHTML = `
    <div class="kpi">
      <div class="kpi-label">TCP Position (mm)</div>
      <div class="kpi-val" style="font-size:1.05rem;">${fk.pose.x}, ${fk.pose.y}, ${fk.pose.z}</div>
      <div class="kpi-sub">Euler A/B/C: ${fk.pose.a}° / ${fk.pose.b}° / ${fk.pose.c}°</div>
    </div>
    <div class="kpi ${manipCls}">
      <div class="kpi-label">Manipulability Index</div>
      <div class="kpi-val">${manip.index.toFixed(3)}</div>
      <div class="kpi-sub">Singularity Risk: ${manip.singularityRisk}</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Toolpath Length</div>
      <div class="kpi-val">${audit ? audit.totalDistMm : 0} mm</div>
      <div class="kpi-sub">${state.waypoints.length} MovePose waypoints</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Est. Cycle Time</div>
      <div class="kpi-val">${audit ? audit.cycleTimeS : 0} s</div>
      <div class="kpi-sub">Feed: ${state.feedRate} mm/s</div>
    </div>
    <div class="kpi ${audit && audit.maxJointSpeedPct > 100 ? "danger" : "ok"}">
      <div class="kpi-label">Peak Joint Speed</div>
      <div class="kpi-val">${audit ? audit.maxJointSpeedPct : 0}%</div>
      <div class="kpi-sub">Of Meca500 rated limit</div>
    </div>
    <div class="kpi ${safetyCls}">
      <div class="kpi-label">Safety Pre-Flight</div>
      <div class="kpi-val">${audit ? (audit.collisionCount === 0 && audit.reachErrors === 0 ? "CLEAR" : `${audit.collisionCount + audit.reachErrors} FAULT`) : "—"}</div>
      <div class="kpi-sub">${audit ? audit.issues.length : 0} total advisories</div>
    </div>
  `;
}

function renderArmPlot(targetDivId, showDeposited = false) {
  const fk = M.forwardKinematics(jointsRad());
  const armPts = fk.positions.slice(0, -1);
  const tipPts = fk.positions.slice(-2);

  const traces = [
    buildPlateTrace(),
    ...buildObstacleTraces(),
    {
      x: armPts.map((p) => p[0]),
      y: armPts.map((p) => p[1]),
      z: armPts.map((p) => p[2]),
      type: "scatter3d",
      mode: "lines+markers",
      line: { width: 9, color: "#0ea5e9" },
      marker: { size: 5, color: "#38bdf8" },
      name: "Meca500 Links",
    },
    {
      x: tipPts.map((p) => p[0]),
      y: tipPts.map((p) => p[1]),
      z: tipPts.map((p) => p[2]),
      type: "scatter3d",
      mode: "lines+markers",
      line: { width: 6, color: "#f97316" },
      marker: { size: 4, color: "#fb923c" },
      name: "Extruder Nozzle",
    },
  ];

  if (state.waypoints.length) {
    traces.push({
      x: state.waypoints.map((p) => p[0]),
      y: state.waypoints.map((p) => p[1]),
      z: state.waypoints.map((p) => p[2]),
      type: "scatter3d",
      mode: "lines",
      line: { width: 3, color: showDeposited ? "rgba(148,163,184,0.35)" : "#14b8a6" },
      name: "Planned Path",
    });
  }

  if (showDeposited && state.step > 0) {
    const dep = state.waypoints.slice(0, state.step + 1);
    traces.push({
      x: dep.map((p) => p[0]),
      y: dep.map((p) => p[1]),
      z: dep.map((p) => p[2]),
      type: "scatter3d",
      mode: "lines",
      line: { width: 6, color: "#22c55e" },
      name: "Deposited Material",
    });
  }

  Plotly.react(targetDivId, traces, {
    margin: { l: 0, r: 0, t: 0, b: 0 },
    paper_bgcolor: "transparent",
    legend: { orientation: "h", y: 1.02, font: { color: "#cbd5e1", size: 11 } },
    scene: {
      bgcolor: "#0b1220",
      xaxis: { range: [-150, 320], title: "X (mm)", color: "#94a3b8", gridcolor: "#1e293b" },
      yaxis: { range: [-200, 200], title: "Y (mm)", color: "#94a3b8", gridcolor: "#1e293b" },
      zaxis: { range: [0, 360], title: "Z (mm)", color: "#94a3b8", gridcolor: "#1e293b" },
      camera: { eye: { x: 1.45, y: 1.35, z: 0.95 } },
    },
  }, { responsive: true });
}

function renderTelemetryCharts() {
  const a = state.audit;
  if (!a || !a.times.length) return;

  const colors = ["#0ea5e9", "#14b8a6", "#f59e0b", "#a855f7", "#ec4899", "#22c55e"];
  const jTraces = a.jointHistory.map((arr, idx) => ({
    x: a.times,
    y: arr,
    mode: "lines",
    name: `J${idx + 1}`,
    line: { width: 2, color: colors[idx] },
  }));

  Plotly.react("jointPlot", jTraces, {
    margin: { t: 18, r: 16, b: 40, l: 50 },
    paper_bgcolor: "transparent",
    plot_bgcolor: "#0b1220",
    font: { color: "#cbd5e1" },
    xaxis: { title: "Time (s)", gridcolor: "#1e293b" },
    yaxis: { title: "Joint Angle (deg)", gridcolor: "#1e293b" },
    legend: { orientation: "h", y: 1.15 },
  }, { responsive: true });

  Plotly.react("manipPlot", [{
    x: a.times,
    y: a.manipHistory,
    mode: "lines",
    fill: "tozeroy",
    line: { color: "#14b8a6", width: 2 },
    fillcolor: "rgba(20,184,166,0.16)",
    name: "Manipulability",
  }], {
    margin: { t: 18, r: 16, b: 40, l: 50 },
    paper_bgcolor: "transparent",
    plot_bgcolor: "#0b1220",
    font: { color: "#cbd5e1" },
    xaxis: { title: "Time (s)", gridcolor: "#1e293b" },
    yaxis: { title: "Yoshikawa Index", gridcolor: "#1e293b" },
    shapes: [{ type: "line", xref: "paper", x0: 0, x1: 1, y0: 0.12, y1: 0.12, line: { color: "#ef4444", dash: "dash" } }],
  }, { responsive: true });
}

function renderAuditTable() {
  const a = state.audit;
  if (!a) return;
  el("safetyBanner").innerHTML =
    a.collisionCount === 0 && a.reachErrors === 0
      ? `<div class="alert alert-ok"><strong>Pre-Flight Safety Clear:</strong> Zero fixture collisions and zero reach violations across all ${state.waypoints.length} waypoints (Min Manipulability = ${a.minManip}).</div>`
      : `<div class="alert alert-danger"><strong>Safety Interlock Triggered:</strong> ${a.collisionCount} keep-out collision(s) and ${a.reachErrors} reach error(s) detected. Adjust toolpath origin or obstacle clearance before hardware execution.</div>`;

  if (!a.issues.length) {
    el("auditTable").innerHTML = `<p style="padding:12px;margin:0;color:#86efac;">All waypoints passed reachability, joint-limit, singularity, and collision checks.</p>`;
  } else {
    el("auditTable").innerHTML = `<table><thead><tr><th>Step</th><th>Category</th><th>Diagnostic Detail</th></tr></thead><tbody>${
      a.issues.slice(0, 60).map((it) => `<tr><td>#${it.step}</td><td><span class="pill ${it.type === "COLLISION" || it.type === "REACH" ? "pill-danger" : "pill-warn"}">${it.type}</span></td><td>${it.detail}</td></tr>`).join("")
    }</tbody></table>`;
  }
}

function regeneratePath() {
  let out;
  if (state.pathMode === "gcode") {
    const txt = el("gcodeBox").value.trim();
    out = M.parseGCodeToMeca(txt, state.pose.a, state.pose.b, state.pose.c);
  } else {
    out = M.generateToolpath({
      mode: state.pathMode,
      shape: state.shape,
      pattern: state.pattern,
      x: state.pathX,
      y: state.pathY,
      z: state.pathZ,
      size: state.size,
      zStep: state.zStep,
      layers: state.layers,
      density: state.density,
      a: state.pose.a,
      b: state.pose.b,
      c: state.pose.c,
    });
  }
  state.waypoints = out.waypoints;
  state.commands = out.commands;
  state.step = 0;
  state.audit = M.auditTrajectory(state.waypoints, state.pose, state.feedRate, state.obstacle);

  el("scriptPreview").value = state.commands.slice(0, 40).join("\n") + (state.commands.length > 40 ? `\n... (${state.commands.length} total commands)` : "");
  log(`Generated ${state.waypoints.length} waypoints (${state.audit.totalDistMm} mm, est. ${state.audit.cycleTimeS}s, ${state.audit.collisionCount} collisions).`);
  refreshAllVisuals();
}

function refreshAllVisuals() {
  renderKPIs();
  renderArmPlot("armPlot3d", false);
  renderArmPlot("execPlot3d", true);
  renderTelemetryCharts();
  renderAuditTable();
  el("stepReadout").textContent = `${state.step} / ${state.waypoints.length}`;
  el("cmdReadout").textContent = state.commands[state.step] || "Ready";
}

function syncJointSliders() {
  state.jointsDeg.forEach((d, i) => {
    const s = el(`j${i + 1}Slider`);
    if (s) s.value = d;
    const lbl = el(`j${i + 1}Val`);
    if (lbl) lbl.textContent = `${d.toFixed(1)}°`;
  });
}

function runSimulatedVisionQA() {
  const defectInjected = el("visionFaultToggle").checked;
  const score = defectInjected ? 68.4 : 98.7;
  const circles = defectInjected ? 1 : 4;
  const errMm = defectInjected ? 0.42 : 0.06;
  const status = defectInjected ? "INTERLOCK — PAUSE & RE-INSPECT" : "PASS — NOMINAL LAYER";
  const cls = defectInjected ? "alert-danger" : "alert-ok";
  el("visionReadout").innerHTML = `
    <div class="alert ${cls}">
      <strong>${status}</strong><br/>
      Kalman-Tracked Features: <strong>${circles}</strong> &nbsp;|&nbsp;
      Bead Centroid Drift: <strong>${errMm} mm</strong> &nbsp;|&nbsp;
      Layer Quality Score: <strong>${score}%</strong>
    </div>
  `;
  log(`Vision QA (${defectInjected ? "Defect Injected" : "Nominal"}): ${status} (Drift ${errMm} mm, Score ${score}%).`);
}

function init() {
  // Tabs
  document.querySelectorAll(".tab-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach((p) => p.classList.remove("active"));
      btn.classList.add("active");
      el(btn.dataset.tab).classList.add("active");
      window.dispatchEvent(new Event("resize"));
    });
  });

  // Joint sliders
  for (let i = 0; i < 6; i++) {
    el(`j${i + 1}Slider`).addEventListener("input", (e) => {
      state.jointsDeg[i] = Number(e.target.value);
      el(`j${i + 1}Val`).textContent = `${state.jointsDeg[i].toFixed(1)}°`;
      refreshAllVisuals();
    });
  }

  el("resetJointsBtn").addEventListener("click", () => {
    state.jointsDeg = [0, -15, 35, 0, 55, 0];
    syncJointSliders();
    log("Joints reset to home posture.");
    refreshAllVisuals();
  });

  el("solveIkBtn").addEventListener("click", () => {
    const x = Number(el("ikX").value), y = Number(el("ikY").value), z = Number(el("ikZ").value);
    const ik = M.inverseKinematics(x, y, z, state.pose.a, state.pose.b, state.pose.c);
    state.jointsDeg = ik.jointsDeg;
    syncJointSliders();
    log(`IK Solved for (${x}, ${y}, ${z}) -> [${ik.jointsDeg.join(", ")}]° (${ik.reachable ? "Reachable" : "OUT OF REACH"}).`);
    refreshAllVisuals();
  });

  // Toolpath controls
  el("pathModeSelect").addEventListener("change", (e) => {
    state.pathMode = e.target.value;
    el("shapeGroup").style.display = state.pathMode === "shape" ? "block" : "none";
    el("infillGroup").style.display = state.pathMode === "infill" ? "block" : "none";
    el("gcodeGroup").style.display = state.pathMode === "gcode" ? "block" : "none";
    regeneratePath();
  });
  el("shapeSelect").addEventListener("change", (e) => { state.shape = e.target.value; regeneratePath(); });
  el("patternSelect").addEventListener("change", (e) => { state.pattern = e.target.value; regeneratePath(); });
  el("generatePathBtn").addEventListener("click", () => {
    state.pathX = Number(el("pathX").value);
    state.pathY = Number(el("pathY").value);
    state.pathZ = Number(el("pathZ").value);
    state.size = Number(el("pathSize").value);
    state.layers = Number(el("pathLayers").value);
    state.feedRate = Number(el("feedRate").value);
    regeneratePath();
  });

  el("downloadScriptBtn").addEventListener("click", () => {
    const blob = new Blob([state.commands.join("\n")], { type: "text/plain" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "meca500_digital_twin_script.txt";
    a.click();
  });

  // Playback
  el("playBtn").addEventListener("click", () => {
    if (state.isPlaying || !state.waypoints.length) return;
    state.isPlaying = true;
    log("Started live 3D Digital Twin trajectory playback.");
    state.timer = setInterval(() => {
      if (state.step >= state.waypoints.length - 1) {
        clearInterval(state.timer);
        state.isPlaying = false;
        log("Trajectory playback complete.");
        return;
      }
      state.step++;
      const pt = state.waypoints[state.step];
      const ik = M.inverseKinematics(pt[0], pt[1], pt[2], state.pose.a, state.pose.b, state.pose.c);
      state.jointsDeg = ik.jointsDeg;
      syncJointSliders();
      refreshAllVisuals();
    }, 90);
  });

  el("pauseBtn").addEventListener("click", () => {
    clearInterval(state.timer);
    state.isPlaying = false;
    log(`Paused playback at step ${state.step}.`);
  });

  el("resetExecBtn").addEventListener("click", () => {
    clearInterval(state.timer);
    state.isPlaying = false;
    state.step = 0;
    refreshAllVisuals();
  });

  // Obstacle & Vision controls
  const updateObstacle = () => {
    state.obstacle.enabled = el("obsToggle").checked;
    state.obstacle.xMin = Number(el("obsXMin").value);
    state.obstacle.xMax = Number(el("obsXMax").value);
    state.obstacle.yMin = Number(el("obsYMin").value);
    state.obstacle.yMax = Number(el("obsYMax").value);
    state.obstacle.zMax = Number(el("obsZMax").value);
    state.audit = M.auditTrajectory(state.waypoints, state.pose, state.feedRate, state.obstacle);
    log(`Updated keep-out fixture (${state.obstacle.enabled ? "ACTIVE" : "DISABLED"}) — ${state.audit.collisionCount} collisions.`);
    refreshAllVisuals();
  };
  ["obsToggle", "obsXMin", "obsXMax", "obsYMin", "obsYMax", "obsZMax"].forEach((id) => el(id).addEventListener("change", updateObstacle));
  el("runVisionBtn").addEventListener("click", runSimulatedVisionQA);

  syncJointSliders();
  regeneratePath();
  runSimulatedVisionQA();

  const initialTab = new URLSearchParams(window.location.search).get("tab");
  if (initialTab) {
    const btn = document.querySelector(`.tab-btn[data-tab="${initialTab}"]`);
    if (btn) btn.click();
  }
}

document.addEventListener("DOMContentLoaded", init);
