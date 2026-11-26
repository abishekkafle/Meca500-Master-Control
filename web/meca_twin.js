/**
 * Meca500 6-DOF Digital Twin Kinematics, Jacobian/Manipulability,
 * Toolpath Slicing, Collision Checking & Telemetry Engine.
 * Matches exact link geometry from backend/server.py.
 */

const LINK_LENGTHS = {
  base_height: 135.0,
  upper_arm: 135.0,
  forearm: 175.0,
  wrist: 72.0,
  extruder_offset: 50.0,
};

// Meca500 official joint limits (degrees) & max angular speeds (deg/s)
const JOINT_LIMITS_DEG = [
  [-175, 175], // J1
  [-70, 90],   // J2
  [-135, 70],  // J3
  [-170, 170], // J4
  [-115, 115], // J5
  [-180, 180], // J6
];
const MAX_JOINT_SPEED_DEG_S = [150, 150, 180, 300, 300, 500];

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

function matMul4(A, B) {
  const C = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
  ];
  for (let i = 0; i < 4; i++) {
    for (let k = 0; k < 4; k++) {
      const aik = A[i][k];
      for (let j = 0; j < 4; j++) C[i][j] += aik * B[k][j];
    }
  }
  return C;
}

function dhMatrix(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

function forwardKinematics(jointsRad) {
  const dhParams = [
    [jointsRad[0], LINK_LENGTHS.base_height, 0, -Math.PI / 2],
    [jointsRad[1] + Math.PI / 2, 0, LINK_LENGTHS.upper_arm, 0],
    [jointsRad[2], 0, LINK_LENGTHS.forearm, -Math.PI / 2],
    [jointsRad[3], 0, 0, Math.PI / 2],
    [jointsRad[4], 0, 0, -Math.PI / 2],
    [jointsRad[5], LINK_LENGTHS.wrist, 0, 0],
  ];

  let T = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
  const positions = [[0, 0, 0]];

  for (const [theta, d, a, alpha] of dhParams) {
    T = matMul4(T, dhMatrix(theta, d, a, alpha));
    positions.push([T[0][3], T[1][3], T[2][3]]);
  }

  const extMat = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -LINK_LENGTHS.extruder_offset],
    [0, 0, 0, 1],
  ];
  const T_ext = matMul4(T, extMat);
  const extruderTip = [T_ext[0][3], T_ext[1][3], T_ext[2][3]];
  positions.push(extruderTip);

  const sy = Math.sqrt(T[0][0] * T[0][0] + T[1][0] * T[1][0]);
  let aRot, bRot, cRot;
  if (sy >= 1e-6) {
    aRot = Math.atan2(T[2][1], T[2][2]);
    bRot = Math.atan2(-T[2][0], sy);
    cRot = Math.atan2(T[1][0], T[0][0]);
  } else {
    aRot = Math.atan2(-T[1][2], T[1][1]);
    bRot = Math.atan2(-T[2][0], sy);
    cRot = 0;
  }

  const pose = {
    x: +T[0][3].toFixed(2),
    y: +T[1][3].toFixed(2),
    z: +T[2][3].toFixed(2),
    a: +(aRot * RAD2DEG).toFixed(2),
    b: +(bRot * RAD2DEG).toFixed(2),
    c: +(cRot * RAD2DEG).toFixed(2),
  };

  return { positions, pose, extruderTip };
}

function inverseKinematics(x, y, z, aDeg = -95, bDeg = 79, cDeg = 33) {
  const roll = aDeg * DEG2RAD;
  const pitch = bDeg * DEG2RAD;
  const yaw = cDeg * DEG2RAD;

  const theta1 = Math.atan2(y, x);
  const wx = x - LINK_LENGTHS.wrist * Math.cos(theta1) * Math.cos(pitch);
  const wy = y - LINK_LENGTHS.wrist * Math.sin(theta1) * Math.cos(pitch);
  const wz = z - LINK_LENGTHS.wrist * Math.sin(pitch);

  const L1 = LINK_LENGTHS.upper_arm;
  const L2 = LINK_LENGTHS.forearm;
  const xArm = Math.sqrt(wx * wx + wy * wy);
  const zArm = wz - LINK_LENGTHS.base_height;
  const D = Math.sqrt(xArm * xArm + zArm * zArm);

  const reachable = D <= L1 + L2 - 1e-3 && D >= Math.abs(L1 - L2) + 1e-3;
  const cosT3 = Math.max(-1, Math.min(1, (L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2)));
  const theta3 = Math.PI - Math.acos(cosT3);

  const alpha = Math.atan2(zArm, xArm);
  const cosBeta = Math.max(-1, Math.min(1, (L1 * L1 + D * D - L2 * L2) / Math.max(2 * L1 * D, 1e-9)));
  const beta = Math.acos(cosBeta);
  const theta2 = alpha + beta - Math.PI / 2;

  const jointsRad = [theta1, theta2, theta3, roll, pitch, yaw - theta1];
  const jointsDeg = jointsRad.map((r) => +(r * RAD2DEG).toFixed(2));
  const limitViolations = [];
  jointsDeg.forEach((deg, i) => {
    const [lo, hi] = JOINT_LIMITS_DEG[i];
    if (deg < lo || deg > hi) limitViolations.push(`J${i + 1} (${deg}° outside [${lo}°, ${hi}°])`);
  });

  return { jointsRad, jointsDeg, reachable, reachDistance: +D.toFixed(1), limitViolations };
}

/**
 * Numerical 3x6 positional Jacobian & Yoshikawa manipulability index:
 * w = sqrt(det(J * J^T)) normalized so typical workspace ~ 0.4 - 1.0.
 */
function computeManipulability(jointsRad) {
  const eps = 1e-4;
  const base = forwardKinematics(jointsRad).positions[6];
  const J = [[], [], []];
  for (let j = 0; j < 6; j++) {
    const q = [...jointsRad];
    q[j] += eps;
    const p = forwardKinematics(q).positions[6];
    J[0].push((p[0] - base[0]) / eps);
    J[1].push((p[1] - base[1]) / eps);
    J[2].push((p[2] - base[2]) / eps);
  }
  // 3x3 M = J * J^T
  const M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      let s = 0;
      for (let k = 0; k < 6; k++) s += J[r][k] * J[c][k];
      M[r][c] = s;
    }
  }
  const det =
    M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
    M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
    M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

  const raw = Math.sqrt(Math.max(det, 0));
  // Normalize by characteristic link volume (175^3)
  const norm = Math.min(1.5, raw / (175 * 175 * 175) * 4.5);
  const singularityRisk = norm < 0.12 ? "HIGH" : norm < 0.28 ? "MODERATE" : "LOW";
  return { index: +norm.toFixed(3), singularityRisk };
}

// ------------------------------------------------------------------ Collision & Keep-Out Checker
function checkPointCollision(pt, obstacle, floorZ = 0) {
  const [x, y, z] = pt;
  if (z < floorZ) return "Floor Z-collision";
  if (
    obstacle &&
    obstacle.enabled &&
    x >= obstacle.xMin && x <= obstacle.xMax &&
    y >= obstacle.yMin && y <= obstacle.yMax &&
    z >= obstacle.zMin && z <= obstacle.zMax
  ) {
    return "Keep-out fixture collision";
  }
  return null;
}

// ------------------------------------------------------------------ Toolpath & Infill Generators
function generateToolpath(opts) {
  const { mode, shape, pattern, x, y, z, size, zStep, layers, density, a, b, c } = opts;
  const pts = [];
  const dFactor = Math.max(0.1, Math.min(1.0, density / 100));

  for (let layer = 0; layer < layers; layer++) {
    const cz = +(z + layer * zStep).toFixed(2);
    if (mode === "shape") {
      if (shape === "rectangle") {
        pts.push([x, y, cz], [x + size, y, cz], [x + size, y + size, cz], [x, y + size, cz], [x, y, cz]);
      } else if (shape === "cylinder") {
        const r = size / 2, cx = x + r, cy = y + r, n = 28;
        for (let i = 0; i <= n; i++) {
          const ang = (2 * Math.PI * i) / n;
          pts.push([+(cx + r * Math.cos(ang)).toFixed(2), +(cy + r * Math.sin(ang)).toFixed(2), cz]);
        }
      } else if (shape === "spiral") {
        const turns = 3, perTurn = 18, maxR = size / 2, cx = x + maxR, cy = y + maxR, tot = turns * perTurn;
        for (let i = 0; i < tot; i++) {
          const r = maxR * (i / tot), ang = 2 * Math.PI * turns * (i / tot);
          pts.push([+(cx + r * Math.cos(ang)).toFixed(2), +(cy + r * Math.sin(ang)).toFixed(2), cz]);
        }
      } else {
        const cols = 5, sp = size / (cols - 1);
        for (let r = 0; r < cols; r++) {
          for (let col = 0; col < cols; col++) {
            const cIdx = r % 2 === 0 ? col : cols - 1 - col;
            pts.push([+(x + cIdx * sp).toFixed(2), +(y + r * sp).toFixed(2), cz]);
          }
        }
      }
    } else {
      // Infill modes: outer perimeter + pattern
      pts.push([x, y, cz], [x + size, y, cz], [x + size, y + size, cz], [x, y + size, cz], [x, y, cz]);
      const spacing = Math.max(4, (1.15 - dFactor) * 18);
      if (pattern === "rectilinear") {
        const lines = Math.max(1, Math.floor(size / spacing));
        for (let i = 1; i < lines; i++) {
          const yi = +(y + i * spacing).toFixed(2);
          if (i % 2 === 1) pts.push([x, yi, cz], [x + size, yi, cz]);
          else pts.push([x + size, yi, cz], [x, yi, cz]);
        }
      } else if (pattern === "concentric") {
        let cur = size - 2 * spacing * 0.6, off = spacing * 0.6;
        while (cur > 3) {
          pts.push([x + off, y + off, cz], [x + off + cur, y + off, cz], [x + off + cur, y + off + cur, cz], [x + off, y + off + cur, cz], [x + off, y + off, cz]);
          off += spacing * 0.6;
          cur -= 2 * spacing * 0.6;
        }
      } else if (pattern === "grid") {
        const lines = Math.max(1, Math.floor(size / spacing));
        for (let i = 1; i < lines; i++) pts.push([x, y + i * spacing, cz], [x + size, y + i * spacing, cz]);
        for (let j = 1; j < lines; j++) pts.push([x + j * spacing, y, cz], [x + j * spacing, y + size, cz]);
      } else {
        // Honeycomb
        const rows = Math.max(1, Math.floor(size / spacing));
        for (let r = 0; r < rows; r++) {
          const yr = y + r * spacing;
          const off = r % 2 === 1 ? spacing / 2 : 0;
          pts.push([x + off, yr, cz], [x + off + spacing / 2, yr + spacing / 3, cz], [x + off + spacing / 2, yr + (2 * spacing) / 3, cz], [x + off, yr + spacing, cz]);
        }
      }
    }
  }

  const commands = pts.map((p) => `MovePose(${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)}, ${a.toFixed(2)}, ${b.toFixed(2)}, ${c.toFixed(2)})`);
  return { waypoints: pts, commands };
}

function parseGCodeToMeca(gcode, a = -95, b = 79, c = 33) {
  let cx = 180, cy = -30, cz = 60;
  const waypoints = [], commands = [];
  for (const raw of gcode.split(/\r?\n/)) {
    const line = raw.split(";")[0].trim();
    if (!line.startsWith("G1") && !line.startsWith("G0")) continue;
    for (const tok of line.split(/\s+/).slice(1)) {
      if (tok.startsWith("X")) cx = parseFloat(tok.slice(1));
      else if (tok.startsWith("Y")) cy = parseFloat(tok.slice(1));
      else if (tok.startsWith("Z")) cz = parseFloat(tok.slice(1));
    }
    if (!Number.isNaN(cx) && !Number.isNaN(cy) && !Number.isNaN(cz)) {
      waypoints.push([cx, cy, cz]);
      commands.push(`MovePose(${cx.toFixed(3)}, ${cy.toFixed(3)}, ${cz.toFixed(3)}, ${a.toFixed(2)}, ${b.toFixed(2)}, ${c.toFixed(2)})`);
    }
  }
  return { waypoints, commands };
}

// ------------------------------------------------------------------ Full Trajectory Pre-Flight Audit & Telemetry
function auditTrajectory(waypoints, poseAngles, feedRateMmS = 45, obstacle = null) {
  let totalDist = 0;
  let totalTime = 0;
  const times = [];
  const jointHistory = [[], [], [], [], [], []];
  const manipHistory = [];
  const issues = [];
  let maxJointSpeedPct = 0;

  let prevPt = null;
  let prevDeg = null;

  waypoints.forEach((pt, idx) => {
    let dt = 0.08;
    if (prevPt) {
      const dist = Math.hypot(pt[0] - prevPt[0], pt[1] - prevPt[1], pt[2] - prevPt[2]);
      totalDist += dist;
      dt = Math.max(dist / Math.max(feedRateMmS, 5), 0.04);
      totalTime += dt;
    }
    times.push(+totalTime.toFixed(2));

    const ik = inverseKinematics(pt[0], pt[1], pt[2], poseAngles.a, poseAngles.b, poseAngles.c);
    ik.jointsDeg.forEach((d, j) => jointHistory[j].push(d));
    const manip = computeManipulability(ik.jointsRad);
    manipHistory.push(manip.index);

    if (prevDeg) {
      ik.jointsDeg.forEach((d, j) => {
        const vel = Math.abs(d - prevDeg[j]) / dt;
        const pct = (100 * vel) / MAX_JOINT_SPEED_DEG_S[j];
        if (pct > maxJointSpeedPct) maxJointSpeedPct = pct;
      });
    }

    const col = checkPointCollision(pt, obstacle, 0);
    if (col) issues.push({ step: idx + 1, type: "COLLISION", detail: `${col} at (${pt[0]}, ${pt[1]}, ${pt[2]})` });
    if (!ik.reachable) issues.push({ step: idx + 1, type: "REACH", detail: `Target distance ${ik.reachDistance} mm exceeds arm reach` });
    if (ik.limitViolations.length) issues.push({ step: idx + 1, type: "JOINT_LIMIT", detail: ik.limitViolations[0] });
    if (manip.singularityRisk === "HIGH") issues.push({ step: idx + 1, type: "SINGULARITY", detail: `Manipulability ${manip.index} (near singularity)` });

    prevPt = pt;
    prevDeg = ik.jointsDeg;
  });

  const minManip = manipHistory.length ? Math.min(...manipHistory) : 0;
  return {
    totalDistMm: +totalDist.toFixed(1),
    cycleTimeS: +totalTime.toFixed(2),
    times,
    jointHistory,
    manipHistory,
    minManip: +minManip.toFixed(3),
    maxJointSpeedPct: +Math.min(maxJointSpeedPct, 999).toFixed(1),
    issues,
    collisionCount: issues.filter((i) => i.type === "COLLISION").length,
    reachErrors: issues.filter((i) => i.type === "REACH").length,
  };
}

window.MecaTwin = {
  LINK_LENGTHS,
  JOINT_LIMITS_DEG,
  forwardKinematics,
  inverseKinematics,
  computeManipulability,
  checkPointCollision,
  generateToolpath,
  parseGCodeToMeca,
  auditTrajectory,
};
