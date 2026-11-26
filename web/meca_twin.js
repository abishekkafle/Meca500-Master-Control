/**
 * Meca500 R3/R4 6-DOF Digital Twin Kinematics, Jacobian/Manipulability,
 * Toolpath Slicing, Collision Checking & Telemetry Engine.
 *
 * Official Mecademic Meca500 R3/R4 geometry (Z-up, right-handed, mm):
 * - Base height to J2 shoulder axis:   135 mm
 * - Upper arm (J2 -> J3):              135 mm (vertical +Z at J2 = 0)
 * - Elbow Z-offset (J3 -> J4 axis):     38 mm
 * - Forearm (J3 -> J5 wrist center):   120 mm (forward +X at J2=J3=0)
 * - Wrist flange (J5 -> J6 flange):     70 mm (forward +X at J5 = 0)
 */

const LINK_LENGTHS = {
  base_height: 135.0,
  upper_arm: 135.0,
  elbow_z: 38.0,
  forearm: 120.0,
  wrist: 70.0,
  nozzle_dx: 68.0,   // DIW syringe nozzle forward offset from J6 flange
  nozzle_dz: -58.0,  // DIW syringe nozzle downward offset from J6 flange
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

function trans(x, y, z) {
  return [
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1],
  ];
}

function rotX(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [
    [1, 0, 0, 0],
    [0, c, -s, 0],
    [0, s, c, 0],
    [0, 0, 0, 1],
  ];
}

function rotY(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [
    [c, 0, s, 0],
    [0, 1, 0, 0],
    [-s, 0, c, 0],
    [0, 0, 0, 1],
  ];
}

function rotZ(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [
    [c, -s, 0, 0],
    [s, c, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

/**
 * Exact Meca500 R3/R4 Forward Kinematics (matches Three.js scene graph 1:1).
 * At [0,0,0,0,0,0]: J2=(0,0,135), J3=(0,0,270), J5=(120,0,308), Flange=(190,0,308).
 */
function forwardKinematics(jointsRad) {
  const [q1, q2, q3, q4, q5, q6] = jointsRad;
  let T = trans(0, 0, 0);
  const positions = [[0, 0, 0]];

  // J1: rotate around Z, up to shoulder J2
  T = matMul4(T, rotZ(q1));
  T = matMul4(T, trans(0, 0, LINK_LENGTHS.base_height));
  positions.push([T[0][3], T[1][3], T[2][3]]); // J2 shoulder

  // J2: pitch around Y, up along upper arm to J3
  T = matMul4(T, rotY(q2));
  T = matMul4(T, trans(0, 0, LINK_LENGTHS.upper_arm));
  positions.push([T[0][3], T[1][3], T[2][3]]); // J3 elbow

  // J3: pitch around Y, elbow offset + forearm to J5 wrist center
  T = matMul4(T, rotY(q3));
  T = matMul4(T, trans(0, 0, LINK_LENGTHS.elbow_z));
  positions.push([T[0][3], T[1][3], T[2][3]]); // J4 housing
  T = matMul4(T, rotX(q4));
  T = matMul4(T, trans(LINK_LENGTHS.forearm, 0, 0));
  positions.push([T[0][3], T[1][3], T[2][3]]); // J5 wrist center

  // J5: pitch around Y, wrist to J6 flange
  T = matMul4(T, rotY(q5));
  T = matMul4(T, trans(LINK_LENGTHS.wrist, 0, 0));
  T = matMul4(T, rotX(q6));
  const flangePos = [T[0][3], T[1][3], T[2][3]];
  positions.push(flangePos); // J6 flange (index 5)

  // DIW Syringe Nozzle Tip transform in J6 frame (dx = +68, dz = -58)
  const T_tip = matMul4(T, trans(LINK_LENGTHS.nozzle_dx, 0, LINK_LENGTHS.nozzle_dz));
  const extruderTip = [T_tip[0][3], T_tip[1][3], T_tip[2][3]];
  positions.push(extruderTip); // Nozzle tip (index 6)

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
    x: +extruderTip[0].toFixed(2),
    y: +extruderTip[1].toFixed(2),
    z: +extruderTip[2].toFixed(2),
    a: +(aRot * RAD2DEG).toFixed(2),
    b: +(bRot * RAD2DEG).toFixed(2),
    c: +(cRot * RAD2DEG).toFixed(2),
  };

  return { positions, pose, flangePos, extruderTip };
}

/**
 * Closed-form geometric Inverse Kinematics for upright Meca500 R3/R4 with vertical DIW syringe head.
 * Default pitch bDeg = 0° keeps the J6 flange horizontal so the vertical syringe nozzle points straight down (-Z).
 */
function inverseKinematics(x, y, z, aDeg = 0, bDeg = 0, cDeg = 0) {
  const q1 = Math.atan2(y, x);
  const pitchRad = bDeg * DEG2RAD; // 0 deg = flange horizontal -> vertical syringe nozzle pointing straight down

  // Back off from nozzle tip (dx=68, dz=-58) + wrist (70) to J5 wrist center
  const totalForward = LINK_LENGTHS.wrist + LINK_LENGTHS.nozzle_dx;
  const rTarget = Math.hypot(x, y);
  const wr = rTarget - (totalForward * Math.cos(pitchRad) + LINK_LENGTHS.nozzle_dz * Math.sin(pitchRad));
  const wz = z - (-totalForward * Math.sin(pitchRad) + LINK_LENGTHS.nozzle_dz * Math.cos(pitchRad));

  const L1 = LINK_LENGTHS.upper_arm;
  const L2eff = Math.hypot(LINK_LENGTHS.forearm, LINK_LENGTHS.elbow_z);
  const elbowBeta = Math.atan2(LINK_LENGTHS.elbow_z, LINK_LENGTHS.forearm);

  const dx = wr;
  const dz = wz - LINK_LENGTHS.base_height;
  const D = Math.hypot(dx, dz);

  const reachable = D <= L1 + L2eff - 0.5 && D >= Math.abs(L1 - L2eff) + 0.5;
  const cosGamma = Math.max(-1, Math.min(1, (L1 * L1 + L2eff * L2eff - D * D) / (2 * L1 * L2eff)));
  const gamma = Math.acos(cosGamma); // interior elbow angle

  const cosPhi = Math.max(-1, Math.min(1, (L1 * L1 + D * D - L2eff * L2eff) / Math.max(2 * L1 * D, 1e-9)));
  const phi = Math.acos(cosPhi);
  const elev = Math.atan2(dz, dx); // angle from +r toward +z

  // Elbow-up configuration (like reference photo)
  const shoulderFromZ = Math.PI / 2 - (elev + phi);
  const q2 = shoulderFromZ;
  const q3 = Math.PI / 2 - gamma + elbowBeta;

  // Keep tool pitch at pitchRad (q2 + q3 + q5 = pitchRad)
  const q4 = aDeg * DEG2RAD;
  const q5 = pitchRad - q2 - q3;
  const q6 = cDeg * DEG2RAD;

  const jointsRad = [q1, q2, q3, q4, q5, q6];
  const jointsDeg = jointsRad.map((r) => +(r * RAD2DEG).toFixed(2));
  const limitViolations = [];
  jointsDeg.forEach((deg, i) => {
    const [lo, hi] = JOINT_LIMITS_DEG[i];
    if (deg < lo || deg > hi) limitViolations.push(`J${i + 1} (${deg}° outside [${lo}°, ${hi}°])`);
  });

  return { jointsRad, jointsDeg, reachable, reachDistance: +D.toFixed(1), limitViolations };
}

/**
 * Numerical 3x6 positional Jacobian & Yoshikawa manipulability index using unrounded flangePos.
 */
function computeManipulability(jointsRad) {
  const eps = 1e-4;
  const base = forwardKinematics(jointsRad).flangePos;
  const J = [[], [], []];
  for (let j = 0; j < 6; j++) {
    const q = [...jointsRad];
    q[j] += eps;
    const p = forwardKinematics(q).flangePos;
    J[0].push((p[0] - base[0]) / eps);
    J[1].push((p[1] - base[1]) / eps);
    J[2].push((p[2] - base[2]) / eps);
  }
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
  const norm = Math.min(1.5, (raw / (140 * 140 * 140)) * 2.5);
  const singularityRisk = norm < 0.14 ? "HIGH" : norm < 0.30 ? "MODERATE" : "LOW";
  return { index: +norm.toFixed(3), singularityRisk };
}

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

function parseGCodeToMeca(gcode, a = 0, b = 90, c = 0) {
  let cx = 160, cy = -25, cz = 75;
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

function auditTrajectory(waypoints, poseAngles, feedRateMmS = 45, obstacle = null) {
  let totalDist = 0, totalTime = 0, maxJointSpeedPct = 0;
  const times = [], jointHistory = [[], [], [], [], [], []], manipHistory = [], issues = [];
  let prevPt = null, prevDeg = null;

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
