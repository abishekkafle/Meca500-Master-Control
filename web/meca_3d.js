/**
 * Realistic Procedural CAD-Style Three.js 3D Renderer for the Mecademic Meca500.
 *
 * Models the iconic Meca500 appearance:
 * - Matte-black powder-coated base pedestal + mounting flange + rear M12 connector block
 * - CNC-machined brushed aluminum links (J1 turret, J2 stadium upper arm, J3 elbow housing,
 *   J4 forearm barrel, J5 rounded wrist clevis, J6 dark ISO 6-bolt tool flange)
 * - Side access cover plates with socket-head cap screws
 * - Interactive OrbitControls, studio lighting, build plate, keep-out collision box,
 *   planned toolpath, and live deposited trajectory.
 */

(function () {
  const DEG2RAD = Math.PI / 180;

  // Helper: create a rounded stadium (capsule-like extruded) link in XZ or XY
  function createStadiumGeometry(length, radius, depth) {
    const shape = new THREE.Shape();
    shape.absarc(0, 0, radius, Math.PI / 2, (3 * Math.PI) / 2, false);
    shape.lineTo(length, -radius);
    shape.absarc(length, 0, radius, -Math.PI / 2, Math.PI / 2, false);
    shape.lineTo(0, radius);

    const geom = new THREE.ExtrudeGeometry(shape, {
      depth: depth,
      bevelEnabled: true,
      bevelThickness: 2.2,
      bevelSize: 2.0,
      bevelSegments: 3,
      curveSegments: 24,
    });
    geom.translate(0, 0, -depth / 2);
    return geom;
  }

  function createRoundedBox(w, h, d, r = 4) {
    const hw = w / 2 - r, hh = h / 2 - r;
    const s = new THREE.Shape();
    s.moveTo(-hw, -h / 2);
    s.lineTo(hw, -h / 2);
    s.quadraticCurveTo(w / 2, -h / 2, w / 2, -hh);
    s.lineTo(w / 2, hh);
    s.quadraticCurveTo(w / 2, h / 2, hw, h / 2);
    s.lineTo(-hw, h / 2);
    s.quadraticCurveTo(-w / 2, h / 2, -w / 2, hh);
    s.lineTo(-w / 2, -hh);
    s.quadraticCurveTo(-w / 2, -h / 2, -hw, -h / 2);
    const geom = new THREE.ExtrudeGeometry(s, {
      depth: d,
      bevelEnabled: true,
      bevelThickness: 1.8,
      bevelSize: 1.8,
      bevelSegments: 2,
    });
    geom.translate(0, 0, -d / 2);
    return geom;
  }

  function addBolts(parent, positions, boltMat, radius = 1.6, axis = "z") {
    const geo = new THREE.CylinderGeometry(radius, radius, 2.2, 10);
    if (axis === "z") geo.rotateX(Math.PI / 2);
    if (axis === "x") geo.rotateZ(Math.PI / 2);
    for (const [bx, by, bz] of positions) {
      const m = new THREE.Mesh(geo, boltMat);
      m.position.set(bx, by, bz);
      parent.add(m);
    }
  }

  class Meca500Viewer3D {
    constructor(containerId) {
      this.container = document.getElementById(containerId);
      if (!this.container || typeof THREE === "undefined") return;

      this.container.innerHTML = "";
      const w = Math.max(this.container.clientWidth, 420);
      const h = Math.max(this.container.clientHeight, 360);

      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x0d1526);

      // Z-up robotics coordinate convention!
      this.camera = new THREE.PerspectiveCamera(36, w / h, 1, 3000);
      this.camera.up.set(0, 0, 1);
      this.camera.position.set(420, -410, 325);

      this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
      this.renderer.setSize(w, h);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 1.18;
      this.container.appendChild(this.renderer.domElement);

      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.target.set(65, 0, 168);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.update();

      this._setupLights();
      this._setupEnvironment();
      this._buildRobot();
      this._setupDynamicOverlays();

      this._onResize = () => this.resize();
      window.addEventListener("resize", this._onResize);

      const animate = () => {
        requestAnimationFrame(animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
      };
      animate();
    }

    _setupLights() {
      // Bright studio lighting so brushed aluminum gleams like the Mecademic product photo
      const hemi = new THREE.HemisphereLight(0xeef5ff, 0x1e293b, 1.05);
      hemi.position.set(0, 0, 500);
      this.scene.add(hemi);

      const key = new THREE.DirectionalLight(0xffffff, 1.55);
      key.position.set(280, -240, 420);
      key.castShadow = true;
      key.shadow.mapSize.width = 1024;
      key.shadow.mapSize.height = 1024;
      key.shadow.camera.near = 50;
      key.shadow.camera.far = 1000;
      const d = 280;
      key.shadow.camera.left = -d;
      key.shadow.camera.right = d;
      key.shadow.camera.top = d;
      key.shadow.camera.bottom = -d;
      this.scene.add(key);

      const fill = new THREE.DirectionalLight(0xbcd8ff, 0.85);
      fill.position.set(-260, 260, 240);
      this.scene.add(fill);

      const rim = new THREE.DirectionalLight(0x7dd3fc, 0.55);
      rim.position.set(-180, -260, 180);
      this.scene.add(rim);
    }

    _setupEnvironment() {
      // Floor grid (Z = 0)
      const grid = new THREE.GridHelper(520, 26, 0x334155, 0x1e293b);
      grid.rotation.x = Math.PI / 2; // lay flat on XY plane
      grid.position.set(80, 0, 0);
      this.scene.add(grid);

      // Subtle shadow-receiving floor plane
      const floor = new THREE.Mesh(
        new THREE.PlaneGeometry(600, 600),
        new THREE.ShadowMaterial({ opacity: 0.35 })
      );
      floor.position.set(80, 0, -0.2);
      floor.receiveShadow = true;
      this.scene.add(floor);

      // Translucent cyan build plate
      const plate = new THREE.Mesh(
        new THREE.BoxGeometry(150, 150, 3),
        new THREE.MeshStandardMaterial({
          color: 0x0ea5e9,
          transparent: true,
          opacity: 0.22,
          roughness: 0.3,
          metalness: 0.2,
        })
      );
      plate.position.set(175, 0, 1.5);
      this.scene.add(plate);

      // World origin axes helper
      const axes = new THREE.AxesHelper(45);
      axes.position.set(0, 0, 1);
      this.scene.add(axes);
    }

    _buildRobot() {
      // Authentic Meca500 PBR Materials
      const matBaseBlack = new THREE.MeshStandardMaterial({
        color: 0x1b2028,
        metalness: 0.25,
        roughness: 0.72,
      });
      const matAlum = new THREE.MeshStandardMaterial({
        color: 0xd0d7e2,
        metalness: 0.78,
        roughness: 0.26,
      });
      const matAlumDark = new THREE.MeshStandardMaterial({
        color: 0xa6b2c2,
        metalness: 0.80,
        roughness: 0.32,
      });
      const matCover = new THREE.MeshStandardMaterial({
        color: 0xdce2eb,
        metalness: 0.72,
        roughness: 0.22,
      });
      const matBolt = new THREE.MeshStandardMaterial({
        color: 0x111827,
        metalness: 0.85,
        roughness: 0.35,
      });
      const matFlange = new THREE.MeshStandardMaterial({
        color: 0x14181f,
        metalness: 0.65,
        roughness: 0.35,
      });

      this.robotRoot = new THREE.Group();
      this.scene.add(this.robotRoot);

      // ---------------- BASE PEDESTAL (Static, Z = 0..102 mm)
      const baseFoot = new THREE.Mesh(new THREE.BoxGeometry(118, 92, 10), matBaseBlack);
      baseFoot.position.set(-14, 0, 5);
      baseFoot.castShadow = true;
      baseFoot.receiveShadow = true;
      this.robotRoot.add(baseFoot);

      // Rear electronics/connector housing
      const rearBox = new THREE.Mesh(new THREE.BoxGeometry(58, 76, 52), matBaseBlack);
      rearBox.position.set(-42, 0, 34);
      rearBox.castShadow = true;
      this.robotRoot.add(rearBox);

      // Twin brushed-metal M12 rear connectors
      const connGeo = new THREE.CylinderGeometry(6.5, 6.5, 14, 18);
      connGeo.rotateZ(Math.PI / 2);
      [-14, 14].forEach((cy) => {
        const c1 = new THREE.Mesh(connGeo, matAlum);
        c1.position.set(-74, cy, 38);
        this.robotRoot.add(c1);
        const c2 = new THREE.Mesh(connGeo, matAlum);
        c2.position.set(-74, cy, 20);
        this.robotRoot.add(c2);
      });

      // Cylindrical matte-black pedestal column
      const colGeo = new THREE.CylinderGeometry(40, 42, 92, 32);
      colGeo.rotateX(Math.PI / 2);
      const col = new THREE.Mesh(colGeo, matBaseBlack);
      col.position.set(0, 0, 52);
      col.castShadow = true;
      this.robotRoot.add(col);

      // Brushed aluminum rotary collar ring at J1
      const collarGeo = new THREE.CylinderGeometry(40, 40, 12, 32);
      collarGeo.rotateX(Math.PI / 2);
      const collar = new THREE.Mesh(collarGeo, matAlum);
      collar.position.set(0, 0, 104);
      this.robotRoot.add(collar);

      // ---------------- J1: BASE ROTATION (about Z)
      this.j1Pivot = new THREE.Group();
      this.j1Pivot.position.set(0, 0, 110);
      this.robotRoot.add(this.j1Pivot);

      // Shoulder turret drum & U-clevis yoke (centered at local z=25 -> world z=135)
      const j2DrumGeo = new THREE.CylinderGeometry(31, 31, 78, 32);
      const j2Drum = new THREE.Mesh(j2DrumGeo, matAlum);
      j2Drum.position.set(0, 0, 25);
      j2Drum.castShadow = true;
      this.j1Pivot.add(j2Drum);

      // Twin vertical cheek plates on J1 yoke + side cover plates
      [-36, 36].forEach((sideY) => {
        const cheek = new THREE.Mesh(new THREE.BoxGeometry(58, 12, 46), matAlumDark);
        cheek.position.set(0, sideY, 16);
        cheek.castShadow = true;
        this.j1Pivot.add(cheek);

        const capGeo = new THREE.CylinderGeometry(28, 28, 5, 24);
        const cap = new THREE.Mesh(capGeo, matCover);
        cap.position.set(0, sideY + Math.sign(sideY) * 6, 25);
        this.j1Pivot.add(cap);
      });

      // ---------------- J2: SHOULDER PITCH (about Y, at world Z = 135)
      this.j2Pivot = new THREE.Group();
      this.j2Pivot.position.set(0, 0, 25); // 110 + 25 = 135 mm
      this.j1Pivot.add(this.j2Pivot);

      // Iconic machined-aluminum stadium upper arm (length 135 mm along +Z)
      const arm2Geo = createStadiumGeometry(135, 27, 52);
      // Stadium is along +X in XY plane -> rotate so it points along +Z with thickness along Y
      arm2Geo.rotateY(-Math.PI / 2);
      arm2Geo.rotateZ(Math.PI / 2);
      const arm2 = new THREE.Mesh(arm2Geo, matAlum);
      arm2.castShadow = true;
      this.j2Pivot.add(arm2);

      // Side cover plates + bolts on upper arm (signature Meca500 look!)
      [-28.5, 28.5].forEach((sideY) => {
        const covGeo = createStadiumGeometry(118, 19, 3.2);
        covGeo.rotateY(-Math.PI / 2);
        covGeo.rotateZ(Math.PI / 2);
        const cov = new THREE.Mesh(covGeo, matCover);
        cov.position.set(0, sideY, 8);
        this.j2Pivot.add(cov);

        addBolts(
          this.j2Pivot,
          [
            [-14, sideY + Math.sign(sideY) * 2.2, 22],
            [14, sideY + Math.sign(sideY) * 2.2, 22],
            [-14, sideY + Math.sign(sideY) * 2.2, 68],
            [14, sideY + Math.sign(sideY) * 2.2, 68],
            [-14, sideY + Math.sign(sideY) * 2.2, 112],
            [14, sideY + Math.sign(sideY) * 2.2, 112],
          ],
          matBolt,
          1.7,
          "y"
        );
      });

      // ---------------- J3: ELBOW PITCH (about Y, at local Z = 135)
      this.j3Pivot = new THREE.Group();
      this.j3Pivot.position.set(0, 0, 135);
      this.j2Pivot.add(this.j3Pivot);

      // Elbow joint drum
      const j3Drum = new THREE.Mesh(new THREE.CylinderGeometry(25, 25, 62, 28), matAlumDark);
      j3Drum.castShadow = true;
      this.j3Pivot.add(j3Drum);

      // Signature chamfered elbow housing block rising +38 mm in Z
      const elbowGeo = createRoundedBox(68, 52, 58, 7);
      elbowGeo.rotateX(Math.PI / 2);
      const elbowBox = new THREE.Mesh(elbowGeo, matAlum);
      elbowBox.position.set(8, 0, 32);
      elbowBox.castShadow = true;
      this.j3Pivot.add(elbowBox);

      // Elbow housing bolts
      [-28, 28].forEach((sideY) => {
        addBolts(
          this.j3Pivot,
          [
            [-18, sideY, 52],
            [32, sideY, 52],
            [32, sideY, 14],
          ],
          matBolt,
          1.6,
          "y"
        );
      });

      // ---------------- J4: FOREARM ROLL (about X, at local Z = 38)
      this.j4Pivot = new THREE.Group();
      this.j4Pivot.position.set(0, 0, 38);
      this.j3Pivot.add(this.j4Pivot);

      // Rotary seam ring at J4
      const j4RingGeo = new THREE.CylinderGeometry(23, 23, 8, 28);
      j4RingGeo.rotateZ(Math.PI / 2);
      const j4Ring = new THREE.Mesh(j4RingGeo, matAlumDark);
      j4Ring.position.set(44, 0, 0);
      this.j4Pivot.add(j4Ring);

      // Machined forearm barrel (from X=48 to X=118)
      const forearmGeo = createRoundedBox(70, 44, 44, 8);
      const forearm = new THREE.Mesh(forearmGeo, matAlum);
      forearm.position.set(83, 0, 0);
      forearm.castShadow = true;
      this.j4Pivot.add(forearm);

      // ---------------- J5: WRIST PITCH (about Y, at local X = 120)
      this.j5Pivot = new THREE.Group();
      this.j5Pivot.position.set(120, 0, 0);
      this.j4Pivot.add(this.j5Pivot);

      // Iconic rounded stadium wrist housing (from X=0 to X=56)
      const wristGeo = createStadiumGeometry(52, 20, 42);
      wristGeo.rotateX(Math.PI / 2);
      const wrist = new THREE.Mesh(wristGeo, matAlum);
      wrist.castShadow = true;
      this.j5Pivot.add(wrist);

      // Top/bottom wrist cover plates
      [-22.5, 22.5].forEach((sideZ) => {
        const wCovGeo = createStadiumGeometry(38, 13, 3);
        const wCov = new THREE.Mesh(wCovGeo, matCover);
        wCov.position.set(6, 0, sideZ);
        this.j5Pivot.add(wCov);
      });

      // ---------------- J6: TOOL FLANGE ROLL (about X, at local X = 70)
      this.j6Pivot = new THREE.Group();
      this.j6Pivot.position.set(70, 0, 0);
      this.j5Pivot.add(this.j6Pivot);

      // Silver flange collar
      const fCollarGeo = new THREE.CylinderGeometry(16, 17, 10, 28);
      fCollarGeo.rotateZ(Math.PI / 2);
      const fCollar = new THREE.Mesh(fCollarGeo, matAlumDark);
      fCollar.position.set(-5, 0, 0);
      this.j6Pivot.add(fCollar);

      // Signature black ISO 6-bolt tool flange face
      const fFaceGeo = new THREE.CylinderGeometry(13.5, 13.5, 5, 28);
      fFaceGeo.rotateZ(Math.PI / 2);
      const fFace = new THREE.Mesh(fFaceGeo, matFlange);
      fFace.position.set(1.5, 0, 0);
      this.j6Pivot.add(fFace);

      // Silver inner bore ring + 6 radial bolt holes
      const boreGeo = new THREE.CylinderGeometry(9.5, 9.5, 5.4, 24);
      boreGeo.rotateZ(Math.PI / 2);
      const bore = new THREE.Mesh(boreGeo, matAlum);
      bore.position.set(1.7, 0, 0);
      this.j6Pivot.add(bore);

      const holePos = [];
      for (let i = 0; i < 6; i++) {
        const a = (i * Math.PI) / 3;
        holePos.push([4.3, 6.2 * Math.cos(a), 6.2 * Math.sin(a)]);
      }
      addBolts(this.j6Pivot, holePos, matBolt, 1.35, "x");

      // Subtle TCP coordinate triad on flange
      const tcpAxes = new THREE.AxesHelper(28);
      tcpAxes.position.set(4, 0, 0);
      this.j6Pivot.add(tcpAxes);
    }

    _setupDynamicOverlays() {
      // Keep-out fixture box
      this.obstacleMesh = new THREE.Mesh(
        new THREE.BoxGeometry(1, 1, 1),
        new THREE.MeshStandardMaterial({
          color: 0xef4444,
          transparent: true,
          opacity: 0.30,
          roughness: 0.4,
        })
      );
      this.obstacleWire = new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)),
        new THREE.LineBasicMaterial({ color: 0xf87171 })
      );
      this.obstacleMesh.add(this.obstacleWire);
      this.scene.add(this.obstacleMesh);

      // Planned toolpath line
      this.plannedLine = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0x14b8a6, linewidth: 2 })
      );
      this.scene.add(this.plannedLine);

      // Deposited toolpath line
      this.depositedLine = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0x22c55e, linewidth: 3 })
      );
      this.scene.add(this.depositedLine);
    }

    setJointsDeg(jointsDeg) {
      if (!this.j1Pivot) return;
      this.j1Pivot.rotation.z = jointsDeg[0] * DEG2RAD;
      this.j2Pivot.rotation.y = jointsDeg[1] * DEG2RAD;
      this.j3Pivot.rotation.y = jointsDeg[2] * DEG2RAD;
      this.j4Pivot.rotation.x = jointsDeg[3] * DEG2RAD;
      this.j5Pivot.rotation.y = jointsDeg[4] * DEG2RAD;
      this.j6Pivot.rotation.x = jointsDeg[5] * DEG2RAD;
    }

    setObstacle(obs) {
      if (!this.obstacleMesh) return;
      this.obstacleMesh.visible = Boolean(obs && obs.enabled);
      if (!obs || !obs.enabled) return;
      const sx = Math.max(1, obs.xMax - obs.xMin);
      const sy = Math.max(1, obs.yMax - obs.yMin);
      const sz = Math.max(1, obs.zMax - obs.zMin);
      this.obstacleMesh.scale.set(sx, sy, sz);
      this.obstacleMesh.position.set(
        0.5 * (obs.xMin + obs.xMax),
        0.5 * (obs.yMin + obs.yMax),
        0.5 * (obs.zMin + obs.zMax)
      );
    }

    setToolpath(waypoints, depositedCount = 0) {
      if (!this.plannedLine) return;
      const pts = waypoints.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
      this.plannedLine.geometry.dispose();
      this.plannedLine.geometry = new THREE.BufferGeometry().setFromPoints(pts);

      const depPts = depositedCount > 0 ? pts.slice(0, depositedCount + 1) : [];
      this.depositedLine.geometry.dispose();
      this.depositedLine.geometry = new THREE.BufferGeometry().setFromPoints(depPts);
    }

    resize() {
      if (!this.container || !this.renderer) return;
      const w = this.container.clientWidth;
      const h = this.container.clientHeight;
      if (w < 10 || h < 10) return;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h);
    }
  }

  window.Meca500Viewer3D = Meca500Viewer3D;
})();
