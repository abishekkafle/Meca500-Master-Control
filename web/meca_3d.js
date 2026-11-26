/**
 * Realistic Procedural CAD-Style Three.js 3D Renderer for the Mecademic Meca500
 * with Custom Direct Ink Writing (DIW) Syringe Printhead & 80/20 Bench Setup.
 *
 * Matches the real laboratory hardware:
 * - 80/20 T-slot aluminum extrusion rail frame + royal-blue build substrate
 * - Matte-black Meca500 pedestal + rear M12 connector block
 * - Brushed CNC-machined aluminum 6-axis links + side cover plates + bolts
 * - 3D-printed royal-blue DIW syringe printhead mounted on J6:
 *     circular flange hub, cantilevered arm, miniature silver/black linear actuator,
 *     vertical syringe cradle, translucent ink syringe barrel, animated plunger/ink,
 *     and tapered vertical dispensing nozzle tip.
 */

(function () {
  const DEG2RAD = Math.PI / 180;

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

      this.camera = new THREE.PerspectiveCamera(35, w / h, 1, 3000);
      this.camera.up.set(0, 0, 1);
      this.camera.position.set(440, -390, 310);

      this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
      this.renderer.setSize(w, h);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 1.18;
      this.container.appendChild(this.renderer.domElement);

      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.target.set(95, 0, 155);
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
      const hemi = new THREE.HemisphereLight(0xf0f6ff, 0x1e293b, 1.05);
      hemi.position.set(0, 0, 500);
      this.scene.add(hemi);

      const key = new THREE.DirectionalLight(0xffffff, 1.55);
      key.position.set(290, -240, 420);
      key.castShadow = true;
      key.shadow.mapSize.width = 1024;
      key.shadow.mapSize.height = 1024;
      key.shadow.camera.near = 50;
      key.shadow.camera.far = 1100;
      const d = 320;
      key.shadow.camera.left = -d;
      key.shadow.camera.right = d;
      key.shadow.camera.top = d;
      key.shadow.camera.bottom = -d;
      this.scene.add(key);

      const fill = new THREE.DirectionalLight(0xbcd8ff, 0.85);
      fill.position.set(-260, 260, 240);
      this.scene.add(fill);

      const rim = new THREE.DirectionalLight(0x7dd3fc, 0.5);
      rim.position.set(-180, -260, 180);
      this.scene.add(rim);
    }

    _setupEnvironment() {
      // Lab bench grid
      const grid = new THREE.GridHelper(560, 28, 0x334155, 0x1e293b);
      grid.rotation.x = Math.PI / 2;
      grid.position.set(100, 0, 0);
      this.scene.add(grid);

      const floor = new THREE.Mesh(
        new THREE.PlaneGeometry(680, 680),
        new THREE.ShadowMaterial({ opacity: 0.35 })
      );
      floor.position.set(100, 0, -0.2);
      floor.receiveShadow = true;
      this.scene.add(floor);

      // 80/20 T-slot Aluminum Extrusion Rails (matching real lab bench setup)
      const matRail = new THREE.MeshStandardMaterial({ color: 0xbac4d0, metalness: 0.78, roughness: 0.32 });
      const rail1 = new THREE.Mesh(new THREE.BoxGeometry(370, 20, 18), matRail);
      rail1.position.set(110, 0, 9);
      rail1.castShadow = true;
      rail1.receiveShadow = true;
      this.scene.add(rail1);

      const crossRail = new THREE.Mesh(new THREE.BoxGeometry(20, 200, 18), matRail);
      crossRail.position.set(220, 0, 9);
      crossRail.castShadow = true;
      this.scene.add(crossRail);

      // Royal-blue acrylic / anodized build substrate plate (matches user's lab photo!)
      const plate = new THREE.Mesh(
        new THREE.BoxGeometry(175, 175, 5),
        new THREE.MeshStandardMaterial({
          color: 0x1a75d2,
          roughness: 0.28,
          metalness: 0.15,
        })
      );
      plate.position.set(215, 0, 20.5);
      plate.receiveShadow = true;
      this.scene.add(plate);

      const axes = new THREE.AxesHelper(40);
      axes.position.set(0, 0, 1);
      this.scene.add(axes);
    }

    _buildRobot() {
      const matBaseBlack = new THREE.MeshStandardMaterial({ color: 0x1b2028, metalness: 0.25, roughness: 0.72 });
      const matAlum = new THREE.MeshStandardMaterial({ color: 0xd0d7e2, metalness: 0.78, roughness: 0.26 });
      const matAlumDark = new THREE.MeshStandardMaterial({ color: 0xa6b2c2, metalness: 0.80, roughness: 0.32 });
      const matCover = new THREE.MeshStandardMaterial({ color: 0xdce2eb, metalness: 0.72, roughness: 0.22 });
      const matBolt = new THREE.MeshStandardMaterial({ color: 0x111827, metalness: 0.85, roughness: 0.35 });
      const matFlange = new THREE.MeshStandardMaterial({ color: 0x14181f, metalness: 0.65, roughness: 0.35 });

      // 3D-printed royal-blue PLA/PETG printhead materials (matching user's lab photo!)
      const matBluePrint = new THREE.MeshStandardMaterial({ color: 0x1976d2, metalness: 0.08, roughness: 0.42 });
      const matActBody = new THREE.MeshStandardMaterial({ color: 0x1a1e24, metalness: 0.3, roughness: 0.65 });
      const matSyringeGlass = new THREE.MeshPhysicalMaterial({
        color: 0xe2eef8,
        transparent: true,
        opacity: 0.42,
        roughness: 0.15,
        metalness: 0.05,
      });
      const matInk = new THREE.MeshStandardMaterial({ color: 0x14b8a6, roughness: 0.3, metalness: 0.1 });
      const matPlunger = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.45, metalness: 0.05 });

      this.robotRoot = new THREE.Group();
      this.scene.add(this.robotRoot);

      // ---------------- BASE PEDESTAL
      const baseFoot = new THREE.Mesh(new THREE.BoxGeometry(118, 92, 10), matBaseBlack);
      baseFoot.position.set(-14, 0, 5);
      baseFoot.castShadow = true;
      baseFoot.receiveShadow = true;
      this.robotRoot.add(baseFoot);

      const rearBox = new THREE.Mesh(new THREE.BoxGeometry(58, 76, 52), matBaseBlack);
      rearBox.position.set(-42, 0, 34);
      rearBox.castShadow = true;
      this.robotRoot.add(rearBox);

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

      const colGeo = new THREE.CylinderGeometry(40, 42, 92, 32);
      colGeo.rotateX(Math.PI / 2);
      const col = new THREE.Mesh(colGeo, matBaseBlack);
      col.position.set(0, 0, 52);
      col.castShadow = true;
      this.robotRoot.add(col);

      const collarGeo = new THREE.CylinderGeometry(40, 40, 12, 32);
      collarGeo.rotateX(Math.PI / 2);
      const collar = new THREE.Mesh(collarGeo, matAlum);
      collar.position.set(0, 0, 104);
      this.robotRoot.add(collar);

      // ---------------- J1: BASE ROTATION
      this.j1Pivot = new THREE.Group();
      this.j1Pivot.position.set(0, 0, 110);
      this.robotRoot.add(this.j1Pivot);

      const j2Drum = new THREE.Mesh(new THREE.CylinderGeometry(31, 31, 78, 32), matAlum);
      j2Drum.position.set(0, 0, 25);
      j2Drum.castShadow = true;
      this.j1Pivot.add(j2Drum);

      [-36, 36].forEach((sideY) => {
        const cheek = new THREE.Mesh(new THREE.BoxGeometry(58, 12, 46), matAlumDark);
        cheek.position.set(0, sideY, 16);
        cheek.castShadow = true;
        this.j1Pivot.add(cheek);

        const cap = new THREE.Mesh(new THREE.CylinderGeometry(28, 28, 5, 24), matCover);
        cap.position.set(0, sideY + Math.sign(sideY) * 6, 25);
        this.j1Pivot.add(cap);
      });

      // ---------------- J2: SHOULDER PITCH
      this.j2Pivot = new THREE.Group();
      this.j2Pivot.position.set(0, 0, 25);
      this.j1Pivot.add(this.j2Pivot);

      const arm2Geo = createStadiumGeometry(135, 27, 52);
      arm2Geo.rotateY(-Math.PI / 2);
      arm2Geo.rotateZ(Math.PI / 2);
      const arm2 = new THREE.Mesh(arm2Geo, matAlum);
      arm2.castShadow = true;
      this.j2Pivot.add(arm2);

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

      // ---------------- J3: ELBOW PITCH
      this.j3Pivot = new THREE.Group();
      this.j3Pivot.position.set(0, 0, 135);
      this.j2Pivot.add(this.j3Pivot);

      const j3Drum = new THREE.Mesh(new THREE.CylinderGeometry(25, 25, 62, 28), matAlumDark);
      j3Drum.castShadow = true;
      this.j3Pivot.add(j3Drum);

      const elbowGeo = createRoundedBox(68, 52, 58, 7);
      elbowGeo.rotateX(Math.PI / 2);
      const elbowBox = new THREE.Mesh(elbowGeo, matAlum);
      elbowBox.position.set(8, 0, 32);
      elbowBox.castShadow = true;
      this.j3Pivot.add(elbowBox);

      [-28, 28].forEach((sideY) => {
        addBolts(
          this.j3Pivot,
          [[-18, sideY, 52], [32, sideY, 52], [32, sideY, 14]],
          matBolt,
          1.6,
          "y"
        );
      });

      // ---------------- J4: FOREARM ROLL
      this.j4Pivot = new THREE.Group();
      this.j4Pivot.position.set(0, 0, 38);
      this.j3Pivot.add(this.j4Pivot);

      const j4RingGeo = new THREE.CylinderGeometry(23, 23, 8, 28);
      j4RingGeo.rotateZ(Math.PI / 2);
      const j4Ring = new THREE.Mesh(j4RingGeo, matAlumDark);
      j4Ring.position.set(44, 0, 0);
      this.j4Pivot.add(j4Ring);

      const forearmGeo = createRoundedBox(70, 44, 44, 8);
      const forearm = new THREE.Mesh(forearmGeo, matAlum);
      forearm.position.set(83, 0, 0);
      forearm.castShadow = true;
      this.j4Pivot.add(forearm);

      // ---------------- J5: WRIST PITCH
      this.j5Pivot = new THREE.Group();
      this.j5Pivot.position.set(120, 0, 0);
      this.j4Pivot.add(this.j5Pivot);

      const wristGeo = createStadiumGeometry(52, 20, 42);
      wristGeo.rotateX(Math.PI / 2);
      const wrist = new THREE.Mesh(wristGeo, matAlum);
      wrist.castShadow = true;
      this.j5Pivot.add(wrist);

      [-22.5, 22.5].forEach((sideZ) => {
        const wCovGeo = createStadiumGeometry(38, 13, 3);
        const wCov = new THREE.Mesh(wCovGeo, matCover);
        wCov.position.set(6, 0, sideZ);
        this.j5Pivot.add(wCov);
      });

      // ---------------- J6: TOOL FLANGE + CUSTOM BLUE DIW SYRINGE PRINTHEAD
      this.j6Pivot = new THREE.Group();
      this.j6Pivot.position.set(70, 0, 0);
      this.j5Pivot.add(this.j6Pivot);

      // J6 collar & flange
      const fCollarGeo = new THREE.CylinderGeometry(16, 17, 10, 28);
      fCollarGeo.rotateZ(Math.PI / 2);
      const fCollar = new THREE.Mesh(fCollarGeo, matAlumDark);
      fCollar.position.set(-5, 0, 0);
      this.j6Pivot.add(fCollar);

      const fFaceGeo = new THREE.CylinderGeometry(13.5, 13.5, 5, 28);
      fFaceGeo.rotateZ(Math.PI / 2);
      const fFace = new THREE.Mesh(fFaceGeo, matFlange);
      fFace.position.set(1.5, 0, 0);
      this.j6Pivot.add(fFace);

      // 1) Blue circular mounting hub bolted to J6
      const hubGeo = new THREE.CylinderGeometry(16, 16, 10, 28);
      hubGeo.rotateZ(Math.PI / 2);
      const hub = new THREE.Mesh(hubGeo, matBluePrint);
      hub.position.set(9, 0, 0);
      hub.castShadow = true;
      this.j6Pivot.add(hub);

      // Mounting bolts on hub face
      addBolts(
        this.j6Pivot,
        [[14.5, 9, 0], [14.5, -9, 0], [14.5, 0, 9], [14.5, 0, -9]],
        matBolt,
        1.5,
        "x"
      );

      // 2) Cantilevered blue L-arm extending forward (+X) to hold actuator & syringe
      const armBeam = new THREE.Mesh(new THREE.BoxGeometry(62, 22, 18), matBluePrint);
      armBeam.position.set(40, 0, -4);
      armBeam.castShadow = true;
      this.j6Pivot.add(armBeam);

      // 3) Miniature linear actuator (black motor base + silver anodized barrel + top cap)
      const actBase = new THREE.Mesh(new THREE.BoxGeometry(16, 16, 26), matActBody);
      actBase.position.set(42, 0, 17);
      actBase.castShadow = true;
      this.j6Pivot.add(actBase);

      const actBarrelGeo = new THREE.CylinderGeometry(6.5, 6.5, 42, 20);
      actBarrelGeo.rotateX(Math.PI / 2);
      const actBarrel = new THREE.Mesh(actBarrelGeo, matAlum);
      actBarrel.position.set(42, 0, 50);
      actBarrel.castShadow = true;
      this.j6Pivot.add(actBarrel);

      const actCap = new THREE.Mesh(new THREE.BoxGeometry(10, 10, 8), matActBody);
      actCap.position.set(42, 0, 74);
      this.j6Pivot.add(actCap);

      // 4) Tall blue vertical syringe cradle & upper collar (at X = 68 mm)
      const cradleSpine = new THREE.Mesh(new THREE.BoxGeometry(10, 26, 94), matBluePrint);
      cradleSpine.position.set(57, 0, 34);
      cradleSpine.castShadow = true;
      this.j6Pivot.add(cradleSpine);

      const upperCollar = new THREE.Mesh(new THREE.BoxGeometry(30, 30, 22), matBluePrint);
      upperCollar.position.set(68, 0, 70);
      upperCollar.castShadow = true;
      this.j6Pivot.add(upperCollar);

      const lowerCollar = new THREE.Mesh(new THREE.BoxGeometry(26, 26, 12), matBluePrint);
      lowerCollar.position.set(68, 0, -6);
      lowerCollar.castShadow = true;
      this.j6Pivot.add(lowerCollar);

      // 5) Translucent Syringe Barrel (centered at X = 68, Z = -2..76)
      const barrelGeo = new THREE.CylinderGeometry(9.5, 9.5, 76, 24);
      barrelGeo.rotateX(Math.PI / 2);
      const barrel = new THREE.Mesh(barrelGeo, matSyringeGlass);
      barrel.position.set(68, 0, 36);
      this.j6Pivot.add(barrel);

      // Animated Ink Column inside syringe barrel
      const inkGeo = new THREE.CylinderGeometry(8.0, 8.0, 56, 20);
      inkGeo.rotateX(Math.PI / 2);
      this.inkMesh = new THREE.Mesh(inkGeo, matInk);
      this.inkMesh.position.set(68, 0, 27);
      this.j6Pivot.add(this.inkMesh);

      // Animated Plunger rod + top thumb flange
      this.plungerGroup = new THREE.Group();
      this.plungerGroup.position.set(68, 0, 56);
      const sealGeo = new THREE.CylinderGeometry(8.3, 8.3, 6, 20);
      sealGeo.rotateX(Math.PI / 2);
      this.plungerGroup.add(new THREE.Mesh(sealGeo, matActBody));
      const rodGeo = new THREE.CylinderGeometry(3.2, 3.2, 34, 14);
      rodGeo.rotateX(Math.PI / 2);
      const rod = new THREE.Mesh(rodGeo, matPlunger);
      rod.position.set(0, 0, 18);
      this.plungerGroup.add(rod);
      const thumbGeo = new THREE.CylinderGeometry(11.5, 11.5, 2.5, 22);
      thumbGeo.rotateX(Math.PI / 2);
      const thumb = new THREE.Mesh(thumbGeo, matPlunger);
      thumb.position.set(0, 0, 36);
      this.plungerGroup.add(thumb);
      this.j6Pivot.add(this.plungerGroup);

      // 6) Tapered Dispensing Nozzle Tip pointing straight down (-Z) to local (68, 0, -58)
      const luerGeo = new THREE.CylinderGeometry(4.5, 3.0, 12, 16);
      luerGeo.rotateX(Math.PI / 2);
      const luer = new THREE.Mesh(luerGeo, matPlunger);
      luer.position.set(68, 0, -18);
      this.j6Pivot.add(luer);

      const tipGeo = new THREE.CylinderGeometry(2.6, 0.7, 34, 16);
      tipGeo.rotateX(Math.PI / 2);
      const tip = new THREE.Mesh(tipGeo, matSyringeGlass);
      tip.position.set(68, 0, -41);
      this.j6Pivot.add(tip);

      // Glowing droplet indicator at exact nozzle TCP (68, 0, -58)
      this.nozzleDrop = new THREE.Mesh(
        new THREE.SphereGeometry(2.2, 12, 12),
        new THREE.MeshStandardMaterial({ color: 0x2dd4bf, emissive: 0x0d9488, emissiveIntensity: 0.7 })
      );
      this.nozzleDrop.position.set(68, 0, -58);
      this.j6Pivot.add(this.nozzleDrop);
    }

    _setupDynamicOverlays() {
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

      this.plannedLine = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0x7dd3fc, transparent: true, opacity: 0.65 })
      );
      this.scene.add(this.plannedLine);

      this.depositedLine = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0x2dd4bf, linewidth: 3 })
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

    setExtrusionProgress(fraction) {
      if (!this.inkMesh || !this.plungerGroup) return;
      const f = Math.max(0, Math.min(0.85, fraction));
      const remaining = 1.0 - f;
      this.inkMesh.scale.set(1, 1, remaining);
      this.inkMesh.position.z = -1 + 28 * remaining;
      this.plungerGroup.position.z = -1 + 56 * remaining;
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

      const frac = waypoints.length > 1 ? depositedCount / (waypoints.length - 1) : 0;
      this.setExtrusionProgress(frac);
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
