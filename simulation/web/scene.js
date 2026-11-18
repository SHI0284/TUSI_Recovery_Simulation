import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";

// Geometry communicates calculated attitude and line displacement, not cloth CFD.
export class FlightScene {
  constructor(host) {
    this.host = host;
    this.mode = "follow";
    this.data = null;
    this.lastPosition = new THREE.Vector3();
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setClearColor(0x101e30);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(0x101e30, 0.00015);
    this.camera = new THREE.PerspectiveCamera(44, 1, 0.05, 12000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = 2;
    this.controls.maxDistance = 4000;
    this.controls.maxPolarAngle = Math.PI * 0.96;
    this.scene.add(new THREE.HemisphereLight(0xc3e9ff, 0x384b40, 2.7));
    const light = new THREE.DirectionalLight(0xffffff, 3);
    light.position.set(200, 500, 200);
    this.scene.add(light);
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(16000, 16000),
      new THREE.MeshStandardMaterial({ color: 0x192d32, roughness: 1 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.3;
    this.scene.add(ground);
    this.grid = new THREE.GridHelper(4000, 80, 0x497276, 0x29464f);
    this.scene.add(this.grid);
    this.vehicle = new THREE.Group();
    this.scene.add(this.vehicle);
    this.makeVehicle();
    this.pad = new THREE.Group();
    this.scene.add(this.pad);
    this.mesh(
      new THREE.CylinderGeometry(0.5, 0.65, 0.14, 24),
      0x495762,
      this.pad,
    ).position.y = 0.07;
    this.mesh(
      new THREE.BoxGeometry(0.035, 2.8, 0.035),
      0x889ca4,
      this.pad,
    ).position.set(0.2, 1.4, 0);
    const flameMaterial = new THREE.MeshBasicMaterial({
      color: 0xffa64c,
      transparent: true,
      opacity: 0.8,
      depthWrite: false,
    });
    this.flame = new THREE.Mesh(
      new THREE.ConeGeometry(0.065, 0.7, 18),
      flameMaterial,
    );
    this.flame.rotation.z = Math.PI;
    this.vehicle.add(this.flame);
    this.flame.visible = false;
    this.paths = new THREE.Group();
    this.scene.add(this.paths);
    this.launchTrace = new THREE.Group();
    this.scene.add(this.launchTrace);
    this.target = new THREE.Group();
    this.scene.add(this.target);
    this.windArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(),
      2,
      0xffcf72,
      0.35,
      0.18,
    );
    this.scene.add(this.windArrow);
    this.headingArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, -1),
      new THREE.Vector3(),
      2,
      0xf5f8ff,
      0.3,
      0.14,
    );
    this.scene.add(this.headingArrow);
    this.gpsMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.6, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xbba5ff }),
    );
    this.scene.add(this.gpsMarker);
    this.tether = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(),
        new THREE.Vector3(),
      ]),
      new THREE.LineDashedMaterial({
        color: 0xf2c971,
        dashSize: 6,
        gapSize: 5,
        transparent: true,
        opacity: 0.6,
      }),
    );
    this.scene.add(this.tether);
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
  }
  mesh(geometry, color, parent = this.vehicle) {
    const m = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color,
        roughness: 0.6,
        metalness: 0.12,
        side: THREE.DoubleSide,
      }),
    );
    parent.add(m);
    return m;
  }
  makeVehicle() {
    this.rocket = new THREE.Group();
    this.vehicle.add(this.rocket);
    // Servo locations are a schematic overlay; the ORK contains a mass placeholder only.
    this.servoCase = this.mesh(
      new THREE.BoxGeometry(0.1, 0.09, 0.08),
      0x1b2734,
    );
    this.servoCase.position.y = 0.28;
    this.drums = [];
    for (const side of [-1, 1]) {
      const group = new THREE.Group();
      group.position.set(side * 0.065, 0.28, 0);
      this.vehicle.add(group);
      const drum = this.mesh(
        new THREE.CylinderGeometry(0.018, 0.018, 0.012, 24),
        side < 0 ? 0xb19cff : 0x67dfc3,
        group,
      );
      drum.rotation.x = Math.PI / 2;
      this.mesh(
        new THREE.BoxGeometry(0.027, 0.004, 0.004),
        0xf9f5e9,
        group,
      ).position.z = 0.009;
      this.drums.push(group);
    }
    this.canopyGeometry = new THREE.BufferGeometry();
    const indices = [];
    this.nx = 48;
    this.nz = 20;
    const vertices = new Float32Array((this.nx + 1) * (this.nz + 1) * 3);
    const colors = new Float32Array(vertices.length);
    for (let i = 0; i <= this.nx; i++)
      for (let j = 0; j <= this.nz; j++) {
        const id = i * (this.nz + 1) + j;
        const v = j / this.nz,
          edge = 0.45 + 0.035 * Math.cos((i / this.nx) * Math.PI * 6);
        const c = new THREE.Color(
          v < edge ? 0xd91925 : v < edge + 0.1 ? 0x171a20 : 0xe6dfca,
        );
        colors.set([c.r, c.g, c.b], id * 3);
        if (i < this.nx && j < this.nz) {
          const a = id,
            b = id + this.nz + 1;
          indices.push(a, b, a + 1, b, b + 1, a + 1);
        }
      }
    this.canopyGeometry.setAttribute(
      "position",
      new THREE.BufferAttribute(vertices, 3),
    );
    this.canopyGeometry.setAttribute(
      "color",
      new THREE.BufferAttribute(colors, 3),
    );
    this.canopyGeometry.setIndex(indices);
    this.canopy = new THREE.Mesh(
      this.canopyGeometry,
      new THREE.MeshStandardMaterial({
        vertexColors: true,
        side: THREE.DoubleSide,
        roughness: 0.85,
      }),
    );
    this.vehicle.add(this.canopy);
    this.lowerGeometry = this.canopyGeometry.clone();
    this.lowerCanopy = new THREE.Mesh(
      this.lowerGeometry,
      new THREE.MeshStandardMaterial({
        vertexColors: true,
        side: THREE.DoubleSide,
        roughness: 0.9,
      }),
    );
    this.vehicle.add(this.lowerCanopy);
    this.ribs = [];
    for (let i = 0; i <= 12; i++) {
      const g = new THREE.BufferGeometry().setFromPoints(
        Array.from({ length: this.nz + 1 }, () => new THREE.Vector3()),
      );
      const line = new THREE.Line(
        g,
        new THREE.LineBasicMaterial({
          color: 0x50282d,
          transparent: true,
          opacity: 0.65,
        }),
      );
      this.vehicle.add(line);
      this.ribs.push(line);
    }
    this.suspension = [];
    for (let side of [-1, 1])
      for (let z of [-0.35, 0.36])
        for (let u of [0.25, 0.6, 0.94]) {
          const g = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(),
            new THREE.Vector3(),
          ]);
          const line = new THREE.Line(
            g,
            new THREE.LineBasicMaterial({
              color: 0xbcd3cd,
              transparent: true,
              opacity: 0.7,
            }),
          );
          this.vehicle.add(line);
          this.suspension.push({ line, side, z, u });
        }
    this.brakes = [-1, 1].map((side) => {
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(),
          new THREE.Vector3(),
        ]),
        new THREE.LineBasicMaterial({ color: side < 0 ? 0xb19cff : 0x67dfc3 }),
      );
      this.vehicle.add(line);
      return { line, side };
    });
  }
  makeRocket(geometry) {
    this.disposeGroup(this.rocket);
    const top = 0.8;
    for (const p of geometry.parts) {
      let m;
      if (p.kind === "nosecone") {
        if (p.shape !== "ellipsoid")
          throw Error("Unsupported ORK nose shape: " + p.shape);
        const points = Array.from({ length: 49 }, (_, i) => {
          const t = i / 48;
          return new THREE.Vector2(
            p.aft_radius_m * Math.sqrt(Math.max(0, 1 - t * t)),
            top - p.start_m - p.length_m + p.length_m * t,
          );
        });
        m = this.mesh(
          new THREE.LatheGeometry(points, 48),
          0xe6e9e5,
          this.rocket,
        );
      } else if (p.kind === "bodytube") {
        m = this.mesh(
          new THREE.CylinderGeometry(p.radius_m, p.radius_m, p.length_m, 40),
          0xdfe6e3,
          this.rocket,
        );
        m.position.y = top - p.start_m - p.length_m / 2;
      } else if (p.kind === "transition") {
        m = this.mesh(
          new THREE.CylinderGeometry(
            p.fore_radius_m,
            p.aft_radius_m,
            p.length_m,
            40,
          ),
          0xb4bdc4,
          this.rocket,
        );
        m.position.y = top - p.start_m - p.length_m / 2;
      } else if (p.kind === "trapezoidfinset") {
        const x = p.body_radius_m,
          y = top - p.start_m,
          shape = new THREE.Shape();
        shape.moveTo(x, y);
        shape.lineTo(x + p.span_m, y - p.sweep_m);
        shape.lineTo(x + p.span_m, y - p.sweep_m - p.tip_chord_m);
        shape.lineTo(x, y - p.root_chord_m);
        shape.closePath();
        for (let i = 0; i < p.count; i++) {
          const g = new THREE.ExtrudeGeometry(shape, {
            depth: p.thickness_m,
            bevelEnabled: false,
          });
          g.translate(0, 0, -p.thickness_m / 2);
          m = this.mesh(g, 0x64787e, this.rocket);
          m.rotation.y =
            ((p.rotation_deg + (i * 360) / p.count) * Math.PI) / 180;
        }
      } else if (p.kind === "launchlug") {
        m = this.mesh(
          new THREE.CylinderGeometry(
            p.radius_m,
            p.radius_m,
            p.length_m,
            12,
            true,
          ),
          0x9daeb4,
          this.rocket,
        );
        const angle = (p.angle_deg * Math.PI) / 180;
        const r = p.body_radius_m + p.radius_m;
        m.position.set(
          r * Math.cos(angle),
          top - p.start_m - p.length_m / 2,
          r * Math.sin(angle),
        );
      }
      if (m) m.userData.orkPart = p.name;
    }
  }
  point(u, z, left, right) {
    const half = this.halfSpan || 1.25,
      chord = this.chord || 0.85,
      brake = u < 0 ? left : right;
    const drop =
      brake * 6 * Math.pow(Math.abs(u), 1.4) * Math.max(0, (z + 0.15) / 0.65);
    const cellBulge = 0.025 * Math.sin((u + 1) * 6 * Math.PI) ** 2;
    const f = this.inflation ?? 1;
    return new THREE.Vector3(
      u * half * f,
      0.28 +
        (2.6 -
          0.6 * u * u +
          0.13 * Math.sin((z + 0.5) * Math.PI) +
          cellBulge -
          drop -
          0.28) *
          f,
      (z * chord * (1 - 0.3 * Math.abs(u) ** 4) + 0.08 * u * u) * f,
    );
  }
  endpoints(line, a, b) {
    const v = line.geometry.attributes.position;
    v.setXYZ(0, a.x, a.y, a.z);
    v.setXYZ(1, b.x, b.y, b.z);
    v.needsUpdate = true;
    line.geometry.computeBoundingSphere();
  }
  deform(row) {
    this.inflation = row.canopy_fraction ?? 1;
    const f = this.inflation;
    for (const obj of [
      this.canopy,
      this.lowerCanopy,
      ...this.ribs,
      ...this.suspension.map((x) => x.line),
      ...this.brakes.map((x) => x.line),
    ])
      obj.visible = f > 0.01;
    this.servoCase.visible = f > 0.1;
    for (const drum of this.drums) drum.visible = f > 0.1;
    if (f <= 0.01) return;
    const left = row.left_brake_m,
      right = row.right_brake_m,
      positions = this.canopyGeometry.attributes.position;
    for (let i = 0; i <= this.nx; i++)
      for (let j = 0; j <= this.nz; j++) {
        const p = this.point(
          (i / this.nx) * 2 - 1,
          j / this.nz - 0.5,
          left,
          right,
        );
        positions.setXYZ(i * (this.nz + 1) + j, p.x, p.y, p.z);
      }
    const lower = this.lowerGeometry.attributes.position;
    for (let i = 0; i <= this.nx; i++)
      for (let j = 0; j <= this.nz; j++) {
        const id = i * (this.nz + 1) + j;
        lower.setXYZ(
          id,
          positions.getX(id),
          positions.getY(id) -
            (0.1 + 0.06 * Math.sin((j / this.nz) * Math.PI)) * this.chord * f,
          positions.getZ(id),
        );
      }
    lower.needsUpdate = true;
    this.lowerGeometry.computeVertexNormals();
    this.lowerGeometry.computeBoundingSphere();
    positions.needsUpdate = true;
    this.canopyGeometry.computeVertexNormals();
    this.canopyGeometry.computeBoundingSphere();
    for (let i = 0; i < this.ribs.length; i++) {
      const attr = this.ribs[i].geometry.attributes.position;
      for (let j = 0; j <= this.nz; j++) {
        const p = this.point((i / 12) * 2 - 1, j / this.nz - 0.5, left, right);
        attr.setXYZ(j, p.x, p.y + 0.002, p.z);
      }
      attr.needsUpdate = true;
      this.ribs[i].geometry.computeBoundingSphere();
    }
    for (const { line, side, z, u } of this.suspension)
      this.endpoints(
        line,
        new THREE.Vector3(
          side * 0.43 * f,
          0.28 + (0.92 - (side < 0 ? left : right) * 6) * f,
          0.08 * f,
        ),
        this.point(side * u, z, left, right),
      );
    for (const { line, side } of this.brakes)
      this.endpoints(
        line,
        new THREE.Vector3(side * 0.065, 0.28, 0),
        new THREE.Vector3(
          side * 0.43 * f,
          0.28 + (0.92 - (side < 0 ? left : right) * 6) * f,
          0.08 * f,
        ),
      );
    this.drums[0].rotation.z = (-row.left_angle_deg * Math.PI) / 180;
    this.drums[1].rotation.z = (row.right_angle_deg * Math.PI) / 180;
  }
  disposeGroup(group) {
    for (const child of [...group.children]) {
      child.traverse((o) => {
        o.geometry?.dispose();
        if (o.material)
          for (const m of Array.isArray(o.material)
            ? o.material
            : [o.material]) {
            m.map?.dispose();
            m.dispose();
          }
      });
      group.remove(child);
    }
  }
  label(text, color = 0xe5edeb) {
    const canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#101e30cc";
    ctx.fillRect(0, 0, 256, 64);
    ctx.fillStyle = "#" + color.toString(16).padStart(6, "0");
    ctx.font = "bold 26px system-ui";
    ctx.textAlign = "center";
    ctx.fillText(text, 128, 42);
    const texture = new THREE.CanvasTexture(canvas);
    const sprite = new THREE.Sprite(
      new THREE.SpriteMaterial({ map: texture, depthTest: false }),
    );
    sprite.scale.set(65, 16, 1);
    return sprite;
  }
  setData(data) {
    this.data = data;
    this.halfSpan = (this.canopyReference?.visual_span_m || 2.5) / 2;
    this.chord = this.canopyReference?.visual_chord_m || 0.85;
    this.makeRocket(data.rocket_geometry);
    this.visualHeightOffset = data.rocket_geometry.length_m - 0.8 + 0.16;
    this.disposeGroup(this.paths);
    this.disposeGroup(this.launchTrace);
    this.disposeGroup(this.target);
    const ascent = data.trajectory.filter((r) =>
      ["BOOST", "COAST"].includes(r.phase),
    );
    if (ascent.length) {
      const points = ascent.map(
        (r) => new THREE.Vector3(r.east_m, r.altitude_m, -r.north_m),
      );
      this.launchTrace.add(
        new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(points),
          new THREE.LineBasicMaterial({ color: 0xffba77 }),
        ),
      );
      const vertical = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(),
          new THREE.Vector3(0, Math.max(...ascent.map((r) => r.altitude_m)), 0),
        ]),
        new THREE.LineDashedMaterial({
          color: 0x8d9aa5,
          dashSize: 5,
          gapSize: 4,
        }),
      );
      vertical.computeLineDistances();
      this.launchTrace.add(vertical);
    }
    for (const [rows, color] of [
      [data.neutral.trajectory, 0x718699],
      [data.trajectory, 0x61ddc1],
    ]) {
      const points = rows
        .filter((_, i) => i % 5 === 0 || i === rows.length - 1)
        .map((r) => new THREE.Vector3(r.east_m, r.altitude_m, -r.north_m));
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({
          color,
          transparent: true,
          opacity: 0.95,
        }),
      );
      this.paths.add(line);
      const shadow = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(
          points.map((p) => new THREE.Vector3(p.x, 0.05, p.z)),
        ),
        new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.2 }),
      );
      this.paths.add(shadow);
    }
    const c = data.config;
    this.target.position.set(c.target_east_m, 0.12, -c.target_north_m);
    const ring = this.mesh(
      new THREE.RingGeometry(c.target_radius_m * 0.94, c.target_radius_m, 80),
      0xffce76,
      this.target,
    );
    ring.rotation.x = -Math.PI / 2;
    const pole = this.mesh(
      new THREE.CylinderGeometry(0.5, 0.5, 14, 8),
      0xffce76,
      this.target,
    );
    pole.position.y = 7;
    const flag = this.mesh(
      new THREE.PlaneGeometry(12, 7),
      0xffce76,
      this.target,
    );
    flag.position.set(6, 12, 0);
    const label = this.label("목표 · " + c.target_radius_m + " m", 0xffce76);
    label.position.set(0, 25, 0);
    this.target.add(label);
    for (const [name, x, z] of [
      ["발사점", 0, 0],
      ["N · 북", 0, -180],
      ["E · 동", 180, 0],
    ]) {
      const l = this.label(name);
      l.position.set(x, 5, z);
      this.paths.add(l);
    }
    this.lastPosition.set(
      data.trajectory[0].east_m,
      data.trajectory[0].altitude_m + this.visualHeightOffset,
      -data.trajectory[0].north_m,
    );
    this.lastRow = data.trajectory[0];
    this.setMode(this.mode, true);
    this.update(data.trajectory[0]);
  }
  setMode(mode, reset = false) {
    this.mode = mode;
    if (!this.data) return;
    const p = this.lastPosition;
    if (!["overview", "launch"].includes(mode)) {
      const close = mode === "rocket";
      this.camera.position
        .copy(p)
        .add(
          close
            ? new THREE.Vector3(1.2, 0.3, 1.8)
            : new THREE.Vector3(5, 3.4, 7),
        );
      this.controls.target.copy(p).add(new THREE.Vector3(0, close ? 0 : 1, 0));
      this.vehicle.scale.setScalar(1);
      this.controls.minDistance = close ? 0.3 : 2;
    } else if (mode === "launch") {
      const height = Math.max(30, this.data.summary.apogee_m);
      this.controls.target.set(0, height * 0.48, 0);
      this.camera.position.set(height * 0.85, height * 0.7, height * 1.15);
      this.vehicle.scale.setScalar(8);
      this.controls.minDistance = 10;
    } else {
      const rows = [...this.data.trajectory, ...this.data.neutral.trajectory];
      const box = new THREE.Box3().setFromPoints(
        rows.map((r) => new THREE.Vector3(r.east_m, r.altitude_m, -r.north_m)),
      );
      box.expandByPoint(this.target.position);
      const center = box.getCenter(new THREE.Vector3()),
        size = box.getSize(new THREE.Vector3());
      const extent = Math.max(size.x, size.y, size.z, 300);
      this.controls.target.copy(center);
      this.camera.position
        .copy(center)
        .add(new THREE.Vector3(extent * 0.85, extent * 0.85, extent * 0.9));
      this.vehicle.scale.setScalar(8);
      this.controls.minDistance = 10;
    }
    this.paths.visible = mode === "overview";
    if (this.lastRow) this.update(this.lastRow);
    this.controls.update();
  }
  update(row) {
    if (!this.data) return;
    this.lastRow = row;
    const p = new THREE.Vector3(
      row.east_m,
      row.altitude_m + this.visualHeightOffset,
      -row.north_m,
    );
    if (!["overview", "launch"].includes(this.mode)) {
      const delta = p.clone().sub(this.lastPosition);
      this.camera.position.add(delta);
      this.controls.target.add(delta);
    }
    this.vehicle.position.copy(p);
    this.vehicle.rotation.order = "YXZ";
    if (row.attitude_source && row.attitude_source !== "RECOVERY_BANK_PROXY") {
      const axis = new THREE.Vector3(
        row.body_axis_east,
        row.body_axis_up,
        -row.body_axis_north,
      ).normalize();
      this.vehicle.quaternion.setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        axis,
      );
    } else
      this.vehicle.rotation.set(
        0,
        (-row.heading_deg * Math.PI) / 180,
        (-row.bank_deg * Math.PI) / 180,
      );
    this.deform(row);
    this.pad.visible = this.data.trajectory[0].phase !== undefined;
    this.flame.visible = row.phase === "BOOST" && row.thrust_n > 1;
    if (this.flame.visible) {
      const f = Math.min(1.6, Math.max(0.15, Math.sqrt(row.thrust_n / 250)));
      this.flame.scale.set(1, f, 1);
      this.flame.position.y =
        0.8 - this.data.rocket_geometry.length_m - 0.35 * f;
    }
    const scale = ["overview", "launch"].includes(this.mode) ? 8 : 1;
    this.headingArrow.position
      .copy(p)
      .add(new THREE.Vector3(0, 2.8 * scale, 0));
    const hdg = (row.desired_heading_deg * Math.PI) / 180;
    this.headingArrow.setDirection(
      new THREE.Vector3(Math.sin(hdg), 0, -Math.cos(hdg)),
    );
    this.headingArrow.setLength(1.8 * scale, 0.3 * scale, 0.15 * scale);
    this.headingArrow.visible = row.guidance_valid && row.altitude_m > 0;
    const w = new THREE.Vector3(row.wind_east_mps, 0, -row.wind_north_mps);
    this.windArrow.position
      .copy(p)
      .add(new THREE.Vector3(-2 * scale, 3.3 * scale, 0));
    this.windArrow.visible = w.length() > 0.01;
    if (this.windArrow.visible) {
      this.windArrow.setDirection(w.clone().normalize());
      this.windArrow.setLength(
        Math.max(0.3, w.length() * 0.5) * scale,
        0.3 * scale,
        0.15 * scale,
      );
    }
    this.gpsMarker.visible = row.gps_valid;
    this.gpsMarker.position.set(row.gps_east_m, 0.8, -row.gps_north_m);
    this.gpsMarker.scale.setScalar(this.mode === "overview" ? 3 : 0.15);
    this.endpoints(
      this.tether,
      p,
      new THREE.Vector3(
        this.data.config.target_east_m,
        0.2,
        -this.data.config.target_north_m,
      ),
    );
    this.tether.computeLineDistances();
    this.tether.visible = this.mode === "overview";
    this.paths.visible = this.mode === "overview";
    this.launchTrace.visible = this.mode === "launch";
    this.lastPosition.copy(p);
  }
  resize() {
    const w = this.host.clientWidth,
      h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
