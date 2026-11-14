import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { ActorPose, SnapshotMsg } from "../api";

/** Min drag distance (m) before yaw follows the cursor; shorter = keep prior yaw. */
const PLACE_EGO_YAW_DRAG_M = 0.35;
/** Road / reference control-point pick radius for move + delete (m). */
export const EDIT_POINT_HIT_M = 1.8;
/** Tighter radius for context-menu "on point" (m). */
const EDIT_POINT_CONTEXT_HIT_M = 1.2;

/** NPC / obstacle pick radius from actor footprint (m). */
export function editActorPickRadius(length_m?: number, width_m?: number, fallback = 2): number {
  const L = length_m ?? fallback;
  const W = width_m ?? fallback;
  return Math.max(L, W) * 0.85 + 2.5;
}

export type CameraMode = "orbit" | "top" | "chase" | "overview";

export type EditTool = "move" | "add" | "delete" | "place_ego";
/** Step 1 = road/lanes; Step 2 = driving reference offset */
export type EditPhase = "lanes" | "reference";

export type EgoIcPose = {
  x_m: number;
  y_m: number;
  yaw_rad: number;
};

/** What was under the cursor when opening the edit context menu. */
export type ContextHit =
  | { kind: "empty" }
  | { kind: "point"; index: number; phase: EditPhase }
  | { kind: "npc"; id: string }
  | { kind: "obstacle"; id: string };

export type ContextMenuRequest = {
  clientX: number;
  clientY: number;
  worldX: number;
  worldY: number;
  hit: ContextHit;
};

type Props = {
  trackPoints: number[][];
  trackWidth: number;
  numLanes?: number;
  laneWidth?: number;
  /** Independent driving reference polyline (platform XY) */
  referencePoints?: number[][];
  /** Show yellow reference line (live always; edit only in reference step) */
  showReference?: boolean;
  /** Show road spine (脊线); typically true in lane edit step */
  showSpine?: boolean;
  /** When false, ribbon / curves do not join first↔last. */
  closedLoop?: boolean;
  /** Low-μ patches (platform XY polygons) */
  frictionZones?: Array<{ id: string; mu: number; points_xy: number[][] }>;
  snapshot: SnapshotMsg | null;
  cameraMode: CameraMode;
  editMode?: boolean;
  editPhase?: EditPhase;
  editTool?: EditTool;
  draftNpcs?: ActorPose[];
  draftObstacles?: ActorPose[];
  onCenterlineChange?: (pts: number[][]) => void;
  onReferenceChange?: (pts: number[][]) => void;
  onNpcMove?: (id: string, x: number, y: number) => void;
  onObstacleMove?: (id: string, x: number, y: number) => void;
  onWorldClick?: (x: number, y: number, tool: EditTool) => void;
  /** Right-click opens a context menu (hit resolved in Viewport). */
  onContextMenuRequest?: (req: ContextMenuRequest) => void;
  /** Fired once when a move-tool drag starts (for undo snapshot). */
  onEditDragBegin?: () => void;
  /** Draft ego IC marker (Edit mode). */
  egoIcPose?: EgoIcPose | null;
  /** Place-ego tool: free-plane pose (press = XY, drag = yaw, release = commit). */
  onEgoIcPlace?: (x: number, y: number, yaw_rad: number) => void;
  /** After one place-ego commit, parent should leave the tool (usually back to move). */
  onEditToolChange?: (tool: EditTool) => void;
};

type DragState =
  | { kind: "centerline"; index: number }
  | { kind: "reference"; index: number }
  | { kind: "npc"; id: string }
  | { kind: "obstacle"; id: string }
  | { kind: "place_ego"; x0: number; y0: number; yaw0: number };

function toWorld(x: number, y: number, z = 0.02): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y);
}

function platformToCurvePts(points: number[][]): THREE.Vector3[] {
  let pts = points
    .filter((p) => Array.isArray(p) && p.length >= 2)
    .map((p) => toWorld(Number(p[0]) || 0, Number(p[1]) || 0, 0));
  if (pts.length > 2 && pts[0].distanceToSquared(pts[pts.length - 1]) < 1e-8) {
    pts = pts.slice(0, -1);
  }
  return pts;
}

function smoothSpaced(
  pts: THREE.Vector3[],
  samples: number,
  closed = true
): THREE.Vector3[] {
  if (pts.length < 2) return pts;
  const curve = new THREE.CatmullRomCurve3(pts, closed, "catmullrom", 0.15);
  return curve.getSpacedPoints(Math.max(64, samples));
}

/** Flat horizontal ribbon (not a tube/cylinder). */
function buildFlatTrack(
  points: number[][],
  width: number,
  numLanes = 1,
  laneWidth?: number,
  referencePoints?: number[][],
  showReference = true,
  showSpine = true,
  closedLoop = true
): THREE.Group | null {
  if (!Array.isArray(points) || points.length < 2) return null;
  const pts = platformToCurvePts(points);
  if (pts.length < 2) return null;

  const lanes = Math.max(1, Math.min(8, Math.round(numLanes || 1)));
  const lw = laneWidth && laneWidth > 0 ? laneWidth : width / lanes;
  const totalW = lanes * lw;
  const half = Math.max(totalW, 6) * 0.5;
  const samples = Math.max(128, pts.length * 4);
  const spine = smoothSpaced(pts, samples, closedLoop);

  const left: THREE.Vector3[] = [];
  const right: THREE.Vector3[] = [];
  const normals: THREE.Vector3[] = [];
  for (let i = 0; i < spine.length; i++) {
    const p = spine[i];
    const q = closedLoop
      ? spine[(i + 1) % spine.length]
      : spine[Math.min(i + 1, spine.length - 1)];
    const dir = new THREE.Vector3().subVectors(q, p);
    dir.y = 0;
    if (dir.lengthSq() < 1e-10) {
      if (i > 0) {
        dir.copy(normals[i - 1]).set(-normals[i - 1].z, 0, normals[i - 1].x);
      } else {
        dir.set(1, 0, 0);
      }
    } else {
      dir.normalize();
    }
    const nUnit = new THREE.Vector3(-dir.z, 0, dir.x);
    normals.push(nUnit.clone());
    const n = nUnit.clone().multiplyScalar(half);
    left.push(new THREE.Vector3(p.x + n.x, 0.04, p.z + n.z));
    right.push(new THREE.Vector3(p.x - n.x, 0.04, p.z - n.z));
  }

  const positions: number[] = [];
  const indices: number[] = [];
  const n = left.length;
  for (let i = 0; i < n; i++) {
    const L = left[i];
    const R = right[i];
    positions.push(L.x, L.y, L.z, R.x, R.y, R.z);
  }
  const segCount = closedLoop ? n : Math.max(0, n - 1);
  for (let i = 0; i < segCount; i++) {
    const i0 = i * 2;
    const i1 = i0 + 1;
    const j = ((i + 1) % n) * 2;
    const j0 = j;
    const j1 = j + 1;
    indices.push(i0, i1, j0, i1, j1, j0);
  }

  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geom.setIndex(indices);
  geom.computeVertexNormals();

  const group = new THREE.Group();
  const asphalt = new THREE.Mesh(
    geom,
    new THREE.MeshStandardMaterial({
      color: 0x3a404a,
      roughness: 0.95,
      metalness: 0.02,
      side: THREE.DoubleSide,
    })
  );
  group.add(asphalt);

  // Road spine (脊线) — solid cyan along geometric road center
  if (showSpine) {
    const spinePts = spine.map((p) => new THREE.Vector3(p.x, 0.075, p.z));
    group.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(spinePts),
        new THREE.LineBasicMaterial({ color: 0x5eead4, linewidth: 2 })
      )
    );
  }

  // Lane dividers (dashed)
  if (lanes > 1) {
    const divMat = new THREE.LineDashedMaterial({
      color: 0xd1d5db,
      dashSize: 1.8,
      gapSize: 1.2,
      transparent: true,
      opacity: 0.9,
    });
    for (let k = 1; k < lanes; k++) {
      const lat = -half + k * lw;
      const divPts = spine.map((p, i) => {
        const nn = normals[i];
        return new THREE.Vector3(p.x + nn.x * lat, 0.07, p.z + nn.z * lat);
      });
      const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(divPts), divMat.clone());
      line.computeLineDistances();
      group.add(line);
    }
  }

  // Road shoulders / outer edges (路肩线) — bright solid white
  const shoulderMat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: false });
  group.add(
    new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(left.map((p) => p.clone().setY(0.09))),
      shoulderMat
    )
  );
  group.add(
    new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(right.map((p) => p.clone().setY(0.09))),
      shoulderMat.clone()
    )
  );

  // Independent driving reference (yellow dashed)
  if (showReference) {
    const refSrc =
      referencePoints && referencePoints.length >= 2 ? referencePoints : points;
    const refCurvePts = platformToCurvePts(refSrc);
    if (refCurvePts.length >= 2) {
      const refSmooth = smoothSpaced(
        refCurvePts,
        Math.max(128, refCurvePts.length * 4),
        closedLoop
      );
      const refPts = refSmooth.map((p) => new THREE.Vector3(p.x, 0.1, p.z));
      const ref = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(refPts),
        new THREE.LineDashedMaterial({ color: 0xffc44d, dashSize: 2.5, gapSize: 1.4 })
      );
      ref.computeLineDistances();
      group.add(ref);
    }
  }

  return group;
}

function syncActorGroup(
  group: THREE.Group,
  actors: ActorPose[] | undefined,
  color: number,
  userDataKind?: "npc" | "obstacle"
) {
  while (group.children.length) {
    const c = group.children.pop()!;
    group.remove(c);
    const m = c as THREE.Mesh;
    m.geometry?.dispose();
    (m.material as THREE.Material | undefined)?.dispose();
  }
  for (const a of actors || []) {
    const L = a.length_m ?? 4.0;
    const W = a.width_m ?? 2.0;
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(L, 0.6, W),
      new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.55 })
    );
    mesh.position.set(a.x_m, 0.35, -a.y_m);
    mesh.rotation.y = a.yaw_rad ?? 0;
    mesh.userData = { kind: userDataKind, id: a.id, pickR: editActorPickRadius(L, W) };
    group.add(mesh);
  }
}

export function Viewport({
  trackPoints,
  trackWidth,
  numLanes = 1,
  laneWidth,
  referencePoints,
  showReference = true,
  showSpine = true,
  closedLoop = true,
  frictionZones,
  snapshot,
  cameraMode,
  editMode = false,
  editPhase = "lanes",
  editTool = "move",
  draftNpcs,
  draftObstacles,
  onCenterlineChange,
  onReferenceChange,
  onNpcMove,
  onObstacleMove,
  onWorldClick,
  onContextMenuRequest,
  onEditDragBegin,
  egoIcPose = null,
  onEgoIcPlace,
  onEditToolChange,
}: Props) {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const carRef = useRef<THREE.Object3D | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const trackRef = useRef<THREE.Object3D | null>(null);
  const frictionRef = useRef<THREE.Group | null>(null);
  const trailRef = useRef<THREE.Line | null>(null);
  const predGroupRef = useRef<THREE.Group | null>(null);
  const npcGroupRef = useRef<THREE.Group | null>(null);
  const obsGroupRef = useRef<THREE.Group | null>(null);
  const handlesRef = useRef<THREE.Group | null>(null);
  const egoIcRef = useRef<THREE.Group | null>(null);
  const egoIcPoseRef = useRef(egoIcPose);
  const placeAimLineRef = useRef<THREE.Line | null>(null);
  const modeRef = useRef(cameraMode);
  const trackPointsRef = useRef(trackPoints);
  const trackWidthRef = useRef(trackWidth);
  const numLanesRef = useRef(numLanes);
  const laneWidthRef = useRef(laneWidth);
  const refPtsRef = useRef(referencePoints);
  const showRefRef = useRef(showReference);
  const showSpineRef = useRef(showSpine);
  const closedLoopRef = useRef(closedLoop);
  const editRef = useRef(editMode);
  const editPhaseRef = useRef(editPhase);
  const editToolRef = useRef(editTool);
  const npcRef = useRef(draftNpcs);
  const obsRef = useRef(draftObstacles);
  const onClChangeRef = useRef(onCenterlineChange);
  const onRefChangeRef = useRef(onReferenceChange);
  const onNpcMoveRef = useRef(onNpcMove);
  const onObstacleMoveRef = useRef(onObstacleMove);
  const onClickRef = useRef(onWorldClick);
  const onCtxMenuRef = useRef(onContextMenuRequest);
  const onEditDragBeginRef = useRef(onEditDragBegin);
  const onEgoIcPlaceRef = useRef(onEgoIcPlace);
  const onEditToolChangeRef = useRef(onEditToolChange);
  modeRef.current = cameraMode;
  trackPointsRef.current = trackPoints;
  trackWidthRef.current = trackWidth;
  numLanesRef.current = numLanes;
  laneWidthRef.current = laneWidth;
  refPtsRef.current = referencePoints;
  showRefRef.current = showReference;
  showSpineRef.current = showSpine;
  closedLoopRef.current = closedLoop;
  editRef.current = editMode;
  editPhaseRef.current = editPhase;
  editToolRef.current = editTool;
  npcRef.current = draftNpcs;
  obsRef.current = draftObstacles;
  onClChangeRef.current = onCenterlineChange;
  onRefChangeRef.current = onReferenceChange;
  onNpcMoveRef.current = onNpcMove;
  onObstacleMoveRef.current = onObstacleMove;
  onClickRef.current = onWorldClick;
  onCtxMenuRef.current = onContextMenuRequest;
  onEditDragBeginRef.current = onEditDragBegin;
  onEgoIcPlaceRef.current = onEgoIcPlace;
  onEditToolChangeRef.current = onEditToolChange;
  egoIcPoseRef.current = egoIcPose;

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x151820);
    scene.fog = new THREE.Fog(0x151820, 220, 520);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1200);
    camera.position.set(70, 90, 110);
    camera.lookAt(40, 0, -28);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x151820, 1);
    mount.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.maxPolarAngle = Math.PI * 0.49;
    controls.minDistance = 15;
    controls.maxDistance = 350;
    controls.target.set(40, 0, -28);
    // Left = orbit; middle = zoom; right = pan (hold-drag). Edit menu opens on short right-click release.
    controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
    controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
    controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
    controlsRef.current = controls;

    scene.add(new THREE.HemisphereLight(0xdde4f0, 0x1a1d24, 1.05));
    const sun = new THREE.DirectionalLight(0xfff1d6, 1.05);
    sun.position.set(60, 120, 40);
    scene.add(sun);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(500, 500),
      new THREE.MeshStandardMaterial({ color: 0x1a1d23, roughness: 1 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.05;
    scene.add(ground);

    const grid = new THREE.GridHelper(360, 45, 0x30363f, 0x232830);
    grid.position.y = 0.0;
    scene.add(grid);

    // Build car in local frame: +X = forward (matches platform yaw=0 → world +X).
    const car = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(4.4, 0.55, 2.1),
      new THREE.MeshStandardMaterial({ color: 0xff5a3d, metalness: 0.35, roughness: 0.35 })
    );
    body.position.y = 0.45;
    car.add(body);
    const cabin = new THREE.Mesh(
      new THREE.BoxGeometry(1.7, 0.4, 1.5),
      new THREE.MeshStandardMaterial({ color: 0x11141a, metalness: 0.2, roughness: 0.3 })
    );
    cabin.position.set(-0.35, 0.85, 0); // toward rear (-X)
    car.add(cabin);
    // nose marker so heading is obvious
    const nose = new THREE.Mesh(
      new THREE.BoxGeometry(0.5, 0.25, 1.4),
      new THREE.MeshStandardMaterial({ color: 0xffe08a, metalness: 0.2, roughness: 0.4 })
    );
    nose.position.set(2.0, 0.45, 0);
    car.add(nose);
    scene.add(car);
    carRef.current = car;

    const trail = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x5eead4, transparent: true, opacity: 0.9 })
    );
    trail.position.y = 0.08;
    scene.add(trail);
    trailRef.current = trail;

    const predGroup = new THREE.Group();
    scene.add(predGroup);
    predGroupRef.current = predGroup;

    const npcGroup = new THREE.Group();
    scene.add(npcGroup);
    npcGroupRef.current = npcGroup;
    const obsGroup = new THREE.Group();
    scene.add(obsGroup);
    obsGroupRef.current = obsGroup;
    const handles = new THREE.Group();
    scene.add(handles);
    handlesRef.current = handles;

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();
    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    let drag: DragState | null = null;
    /** Right-button press: distinguish pan-drag vs click-for-menu */
    let rmb: { x: number; y: number; clientX: number; clientY: number } | null = null;
    const RMB_CLICK_PX = 6; // max screen move to count as click (open menu)
    const worldFromEvent = (ev: PointerEvent): { x: number; y: number } | null => {
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const hit = new THREE.Vector3();
      if (!raycaster.ray.intersectPlane(groundPlane, hit)) return null;
      return { x: hit.x, y: -hit.z };
    };

    const pickMoveTarget = (w: { x: number; y: number }): DragState | null => {
      type Cand = { d: number; target: DragState };
      const cands: Cand[] = [];
      const phase = editPhaseRef.current;
      if (phase === "reference") {
        const refPts = refPtsRef.current?.length ? refPtsRef.current : trackPointsRef.current;
        for (let i = 0; i < refPts.length; i++) {
          const d = Math.hypot(refPts[i][0] - w.x, refPts[i][1] - w.y);
          if (d < EDIT_POINT_HIT_M) cands.push({ d, target: { kind: "reference", index: i } });
        }
      } else {
        const pts = trackPointsRef.current;
        for (let i = 0; i < pts.length; i++) {
          const d = Math.hypot(pts[i][0] - w.x, pts[i][1] - w.y);
          if (d < EDIT_POINT_HIT_M) cands.push({ d, target: { kind: "centerline", index: i } });
        }
      }
      for (const a of npcRef.current || []) {
        const r = editActorPickRadius(a.length_m, a.width_m, 4);
        const d = Math.hypot(a.x_m - w.x, a.y_m - w.y);
        if (d < r) cands.push({ d, target: { kind: "npc", id: a.id } });
      }
      for (const a of obsRef.current || []) {
        const r = editActorPickRadius(a.length_m, a.width_m, 1.5);
        const d = Math.hypot(a.x_m - w.x, a.y_m - w.y);
        if (d < r) cands.push({ d, target: { kind: "obstacle", id: a.id } });
      }
      if (!cands.length) return null;
      // Prefer NPC/obstacle when close to control points (distance bias).
      const score = (c: Cand) => {
        const kind = c.target.kind;
        const actor = kind === "npc" || kind === "obstacle";
        return actor ? c.d - EDIT_POINT_HIT_M : c.d;
      };
      cands.sort((a, b) => score(a) - score(b));
      return cands[0].target;
    };

    const pickContextHit = (w: { x: number; y: number }): ContextHit => {
      type Cand = { d: number; hit: ContextHit };
      const cands: Cand[] = [];
      const phase = editPhaseRef.current;
      const polyPts =
        phase === "reference"
          ? refPtsRef.current?.length
            ? refPtsRef.current
            : trackPointsRef.current
          : trackPointsRef.current;
      for (let i = 0; i < polyPts.length; i++) {
        const d = Math.hypot(polyPts[i][0] - w.x, polyPts[i][1] - w.y);
        if (d < EDIT_POINT_CONTEXT_HIT_M) cands.push({ d, hit: { kind: "point", index: i, phase } });
      }
      for (const a of npcRef.current || []) {
        const r = editActorPickRadius(a.length_m, a.width_m, 4);
        const d = Math.hypot(a.x_m - w.x, a.y_m - w.y);
        if (d < r) cands.push({ d, hit: { kind: "npc", id: a.id } });
      }
      for (const a of obsRef.current || []) {
        const r = editActorPickRadius(a.length_m, a.width_m, 1.5);
        const d = Math.hypot(a.x_m - w.x, a.y_m - w.y);
        if (d < r) cands.push({ d, hit: { kind: "obstacle", id: a.id } });
      }
      if (!cands.length) return { kind: "empty" };
      const score = (c: Cand) => {
        const kind = c.hit.kind;
        const actor = kind === "npc" || kind === "obstacle";
        return actor ? c.d - EDIT_POINT_HIT_M : c.d;
      };
      cands.sort((a, b) => score(a) - score(b));
      return cands[0].hit;
    };

    const onContextMenu = (ev: Event) => {
      if (!editRef.current) return;
      ev.preventDefault();
    };

    const onPointerDown = (ev: PointerEvent) => {
      if (!editRef.current) return;

      // Right: let OrbitControls pan while held; menu only if release is a short click
      if (ev.button === 2) {
        const w = worldFromEvent(ev);
        if (!w) return;
        rmb = { x: w.x, y: w.y, clientX: ev.clientX, clientY: ev.clientY };
        return;
      }

      const w = worldFromEvent(ev);
      if (!w) return;
      if (ev.button !== 0) return;
      const tool = editToolRef.current;
      if (tool === "place_ego") {
        const yaw0 = egoIcPoseRef.current?.yaw_rad ?? 0;
        onEditDragBeginRef.current?.();
        drag = { kind: "place_ego", x0: w.x, y0: w.y, yaw0 };
        controls.enabled = false;
        ev.preventDefault();
        ev.stopPropagation();
        const g = egoIcRef.current;
        if (g) {
          g.position.set(w.x, 0, -w.y);
          // Same yaw→Three convention as live cars / NPCs (not negated).
          g.rotation.y = yaw0;
        }
        onEgoIcPlaceRef.current?.(w.x, w.y, yaw0);
        return;
      }
      if (tool === "move") {
        const target = pickMoveTarget(w);
        if (target) {
          onEditDragBeginRef.current?.();
          drag = target;
          controls.enabled = false;
          ev.preventDefault();
          ev.stopPropagation();
        }
        return;
      }
      // Left-click add/delete tools for spine (lanes) or reference polyline
      controls.enabled = false;
      ev.preventDefault();
      ev.stopPropagation();
      onClickRef.current?.(w.x, w.y, tool);
    };
    const clearPlaceAimLine = () => {
      const line = placeAimLineRef.current;
      if (!line) return;
      scene.remove(line);
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
      placeAimLineRef.current = null;
    };
    const updatePlaceAimLine = (x0: number, y0: number, x1: number, y1: number) => {
      const a = toWorld(x0, y0, 0.08);
      const b = toWorld(x1, y1, 0.08);
      let line = placeAimLineRef.current;
      if (!line) {
        const geom = new THREE.BufferGeometry().setFromPoints([a, b]);
        line = new THREE.Line(
          geom,
          new THREE.LineBasicMaterial({ color: 0xfbbf24, linewidth: 2 })
        );
        scene.add(line);
        placeAimLineRef.current = line;
      } else {
        const pos = line.geometry.attributes.position as THREE.BufferAttribute;
        pos.setXYZ(0, a.x, a.y, a.z);
        pos.setXYZ(1, b.x, b.y, b.z);
        pos.needsUpdate = true;
      }
    };
    const yawFromPlaceDrag = (
      dragEgo: { x0: number; y0: number; yaw0: number },
      wx: number,
      wy: number
    ) => {
      const dist = Math.hypot(wx - dragEgo.x0, wy - dragEgo.y0);
      if (dist < PLACE_EGO_YAW_DRAG_M) return dragEgo.yaw0;
      return Math.atan2(wy - dragEgo.y0, wx - dragEgo.x0);
    };
    const onPointerMove = (ev: PointerEvent) => {
      if (rmb && (ev.buttons & 2) === 2) {
        const dx = ev.clientX - rmb.clientX;
        const dy = ev.clientY - rmb.clientY;
        if (Math.hypot(dx, dy) > RMB_CLICK_PX) {
          // User is panning — cancel pending menu
          rmb = null;
        }
      }
      if (drag == null || !editRef.current) return;
      const w = worldFromEvent(ev);
      if (!w) return;
      if (drag.kind === "place_ego") {
        // Live yaw on the marker only; commit React state on pointerup.
        const yaw = yawFromPlaceDrag(drag, w.x, w.y);
        const g = egoIcRef.current;
        if (g) {
          g.position.set(drag.x0, 0, -drag.y0);
          g.rotation.y = yaw;
        }
        updatePlaceAimLine(drag.x0, drag.y0, w.x, w.y);
        return;
      }
      if (drag.kind === "centerline") {
        const idx = drag.index;
        const next = trackPointsRef.current.map((p, i) =>
          i === idx ? [w.x, w.y, p[2] ?? 0] : [...p]
        );
        onClChangeRef.current?.(next);
      } else if (drag.kind === "reference") {
        const pts = refPtsRef.current?.length ? refPtsRef.current : trackPointsRef.current;
        const idx = drag.index;
        const next = pts.map((p, i) => (i === idx ? [w.x, w.y, p[2] ?? 0] : [...p]));
        onRefChangeRef.current?.(next);
      } else if (drag.kind === "npc") {
        onNpcMoveRef.current?.(drag.id, w.x, w.y);
      } else if (drag.kind === "obstacle") {
        onObstacleMoveRef.current?.(drag.id, w.x, w.y);
      }
    };
    const onPointerUp = (ev: PointerEvent) => {
      if (ev.button === 2 && rmb && editRef.current) {
        const dx = ev.clientX - rmb.clientX;
        const dy = ev.clientY - rmb.clientY;
        if (Math.hypot(dx, dy) <= RMB_CLICK_PX) {
          const w = worldFromEvent(ev) || { x: rmb.x, y: rmb.y };
          onCtxMenuRef.current?.({
            clientX: ev.clientX,
            clientY: ev.clientY,
            worldX: w.x,
            worldY: w.y,
            hit: pickContextHit(w),
          });
        }
        rmb = null;
      }
      if (drag?.kind === "place_ego" && editRef.current) {
        const w = worldFromEvent(ev);
        const yaw = w
          ? yawFromPlaceDrag(drag, w.x, w.y)
          : drag.yaw0;
        clearPlaceAimLine();
        onEgoIcPlaceRef.current?.(drag.x0, drag.y0, yaw);
        // One-shot tool: leave place_ego so NPC / spine / obstacle edits work again.
        onEditToolChangeRef.current?.("move");
      } else if (drag?.kind === "place_ego") {
        clearPlaceAimLine();
        onEditToolChangeRef.current?.("move");
      }
      drag = null;
      if (controlsRef.current) {
        controlsRef.current.enabled = true;
      }
    };
    renderer.domElement.addEventListener("pointerdown", onPointerDown);
    renderer.domElement.addEventListener("contextmenu", onContextMenu);
    window.addEventListener("pointermove", onPointerMove);
    window.addEventListener("pointerup", onPointerUp);

    const resize = () => {
      const w = Math.max(mount.clientWidth, 1);
      const h = Math.max(mount.clientHeight, 1);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h, false);
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(mount);

    let alive = true;
    let raf = 0;
    const tick = () => {
      if (!alive) return;
      raf = requestAnimationFrame(tick);
      controls.update();
      renderer.render(scene, camera);
    };
    tick();

    const track = buildFlatTrack(
      trackPointsRef.current,
      trackWidthRef.current || 12,
      numLanesRef.current || 1,
      laneWidthRef.current,
      refPtsRef.current,
      showRefRef.current,
      showSpineRef.current,
      closedLoopRef.current
    );
    if (track) {
      scene.add(track);
      trackRef.current = track;
    }

    return () => {
      alive = false;
      cancelAnimationFrame(raf);
      ro.disconnect();
      clearPlaceAimLine();
      renderer.domElement.removeEventListener("pointerdown", onPointerDown);
      renderer.domElement.removeEventListener("contextmenu", onContextMenu);
      window.removeEventListener("pointermove", onPointerMove);
      window.removeEventListener("pointerup", onPointerUp);
      controls.dispose();
      renderer.dispose();
      sceneRef.current = null;
      carRef.current = null;
      cameraRef.current = null;
      controlsRef.current = null;
      trackRef.current = null;
      trailRef.current = null;
      predGroupRef.current = null;
      npcGroupRef.current = null;
      obsGroupRef.current = null;
      handlesRef.current = null;
      if (renderer.domElement.parentElement === mount) mount.removeChild(renderer.domElement);
    };
  }, []);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;
    if (trackRef.current) {
      scene.remove(trackRef.current);
      trackRef.current.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (mesh.geometry) mesh.geometry.dispose();
      });
      trackRef.current = null;
    }
    const track = buildFlatTrack(
      trackPoints,
      trackWidth || 12,
      numLanes || 1,
      laneWidth,
      referencePoints,
      showReference,
      showSpine,
      closedLoop
    );
    if (track) {
      scene.add(track);
      trackRef.current = track;
    }
  }, [
    trackPoints,
    trackWidth,
    numLanes,
    laneWidth,
    referencePoints,
    showReference,
    showSpine,
    closedLoop,
  ]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;
    if (frictionRef.current) {
      scene.remove(frictionRef.current);
      frictionRef.current.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        mesh.geometry?.dispose();
        const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
        if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
        else mat?.dispose();
      });
      frictionRef.current = null;
    }
    const zones = frictionZones || [];
    if (!zones.length) return;
    const g = new THREE.Group();
    for (const z of zones) {
      if (!z.points_xy || z.points_xy.length < 3) continue;
      const shape = new THREE.Shape();
      // Shape in XY → rotateX(-π/2) → XZ with z = -y (platform convention)
      shape.moveTo(z.points_xy[0][0], z.points_xy[0][1]);
      for (let i = 1; i < z.points_xy.length; i++) {
        shape.lineTo(z.points_xy[i][0], z.points_xy[i][1]);
      }
      shape.closePath();
      const geo = new THREE.ShapeGeometry(shape);
      const mu = z.mu ?? 1;
      // colder (more cyan) = lower μ
      const t = Math.max(0, Math.min(1, mu));
      const mat = new THREE.MeshBasicMaterial({
        color: new THREE.Color().setHSL(0.55, 0.7, 0.25 + 0.35 * t),
        transparent: true,
        opacity: 0.35,
        depthWrite: false,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.rotation.x = -Math.PI / 2;
      mesh.position.y = 0.05;
      g.add(mesh);
    }
    scene.add(g);
    frictionRef.current = g;
  }, [frictionZones]);

  // Ego IC marker (cyan arrow) in Edit mode
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;
    if (egoIcRef.current) {
      scene.remove(egoIcRef.current);
      egoIcRef.current.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        mesh.geometry?.dispose();
        const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
        if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
        else mat?.dispose();
      });
      egoIcRef.current = null;
    }
    if (!editMode || !egoIcPose) return;
    const g = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(3.2, 0.6, 1.6),
      new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.2, roughness: 0.5 })
    );
    body.position.y = 0.35;
    g.add(body);
    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.45, 1.4, 10),
      new THREE.MeshStandardMaterial({ color: 0xfbbf24 })
    );
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(2.2, 0.35, 0);
    g.add(nose);
    g.position.set(egoIcPose.x_m, 0, -egoIcPose.y_m);
    g.rotation.y = egoIcPose.yaw_rad;
    scene.add(g);
    egoIcRef.current = g;
  }, [editMode, egoIcPose]);

  // Apply camera mode presets when mode changes
  useEffect(() => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) return;

    const ego = snapshot?.vehicles?.[0];
    const cx = ego?.x_m ?? 40;
    const cy = ego?.y_m ?? 28;

    if (cameraMode === "orbit") {
      controls.enabled = true;
      return;
    }

    controls.enabled = false;
    if (cameraMode === "top") {
      camera.position.set(cx, 160, -cy);
      controls.target.set(cx, 0, -cy);
      camera.up.set(0, 0, -1); // north-up-ish for top view
      camera.lookAt(cx, 0, -cy);
      camera.up.set(0, 1, 0);
    } else if (cameraMode === "overview") {
      camera.position.set(70, 95, 110);
      controls.target.set(40, 0, -28);
      camera.lookAt(40, 0, -28);
    } else if (cameraMode === "chase" && ego) {
      const back = 22;
      camera.position.set(
        ego.x_m - Math.cos(ego.yaw_rad) * back,
        12,
        -ego.y_m + Math.sin(ego.yaw_rad) * back
      );
      controls.target.set(ego.x_m, 0.5, -ego.y_m);
      camera.lookAt(ego.x_m, 0.5, -ego.y_m);
    }
    controls.update();
  }, [cameraMode]);

  useEffect(() => {
    const car = carRef.current;
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    const trail = trailRef.current;
    if (!car || !snapshot?.vehicles?.length) return;
    const v = snapshot.vehicles[0];
    // Platform: yaw=0 → +X, yaw CCW toward +Y. Three: (x,-y) on XZ, +Y up.
    // Local car +X is forward. Right-hand Y rotation: +yaw sends +X → -Z (= platform +Y).
    car.position.set(v.x_m, 0, -v.y_m);
    car.rotation.y = v.yaw_rad;

    const mode = modeRef.current;
    if (camera && controls && mode !== "orbit") {
      if (mode === "chase") {
        const back = 22;
        camera.position.set(
          v.x_m - Math.cos(v.yaw_rad) * back,
          12,
          -v.y_m + Math.sin(v.yaw_rad) * back
        );
        controls.target.set(v.x_m, 0.5, -v.y_m);
        camera.lookAt(v.x_m, 0.5, -v.y_m);
      } else if (mode === "top") {
        camera.position.set(v.x_m, 160, -v.y_m);
        controls.target.set(v.x_m, 0, -v.y_m);
        camera.lookAt(v.x_m, 0, -v.y_m);
      } else if (mode === "overview") {
        controls.target.lerp(new THREE.Vector3(v.x_m, 0, -v.y_m), 0.04);
      }
    }

    const pts = snapshot.trail_xy;
    if (trail && Array.isArray(pts) && pts.length > 1) {
      const vectors = pts.map((p) => toWorld(p[0], p[1], 0.12));
      trail.geometry.dispose();
      trail.geometry = new THREE.BufferGeometry().setFromPoints(vectors);
    }

    // MPPI predicted trajectories
    const pred = predGroupRef.current;
    if (pred) {
      while (pred.children.length) {
        const c = pred.children.pop()!;
        pred.remove(c);
        const line = c as THREE.Line;
        line.geometry?.dispose();
        (line.material as THREE.Material | undefined)?.dispose();
      }
      const diag = snapshot.metrics?.ctrl_diag as
        | {
            best_traj_xy?: number[][];
            sample_trajs_xy?: number[][][];
          }
        | undefined;
      if (!editMode && diag?.sample_trajs_xy) {
        for (const sample of diag.sample_trajs_xy.slice(0, 4)) {
          if (!sample || sample.length < 2) continue;
          const g = new THREE.BufferGeometry().setFromPoints(
            sample.map((p) => toWorld(p[0], p[1], 0.15))
          );
          pred.add(
            new THREE.Line(
              g,
              new THREE.LineBasicMaterial({ color: 0x64748b, transparent: true, opacity: 0.45 })
            )
          );
        }
      }
      if (!editMode && diag?.best_traj_xy && diag.best_traj_xy.length > 1) {
        const g = new THREE.BufferGeometry().setFromPoints(
          diag.best_traj_xy.map((p) => toWorld(p[0], p[1], 0.18))
        );
        pred.add(
          new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x22d3ee, linewidth: 2 }))
        );
      }
    }

    if (!editMode) {
      if (npcGroupRef.current) {
        syncActorGroup(
          npcGroupRef.current,
          snapshot.npcs || (snapshot.metrics?.npcs as ActorPose[]) || [],
          0x5b8cff
        );
      }
      if (obsGroupRef.current) {
        syncActorGroup(
          obsGroupRef.current,
          snapshot.obstacles || (snapshot.metrics?.obstacles as ActorPose[]) || [],
          0xfbbf24
        );
      }
    }
  }, [snapshot, editMode]);

  useEffect(() => {
    // Only draft actors overwrite the scene in Edit; Live/Replay use snapshot.npcs.
    if (editMode) {
      if (npcGroupRef.current) syncActorGroup(npcGroupRef.current, draftNpcs, 0x5b8cff, "npc");
      if (obsGroupRef.current) {
        syncActorGroup(obsGroupRef.current, draftObstacles, 0xfbbf24, "obstacle");
      }
    }
    if (!editMode) {
      if (handlesRef.current) {
        while (handlesRef.current.children.length) {
          const c = handlesRef.current.children.pop()!;
          handlesRef.current.remove(c);
        }
      }
      return;
    }
    const handles = handlesRef.current;
    if (!handles) return;
    while (handles.children.length) {
      const c = handles.children.pop()!;
      handles.remove(c);
      const m = c as THREE.Mesh;
      m.geometry?.dispose();
      (m.material as THREE.Material | undefined)?.dispose();
    }
    const handlePts =
      editPhase === "reference"
        ? referencePoints && referencePoints.length
          ? referencePoints
          : trackPoints
        : trackPoints;
    const color = editPhase === "reference" ? 0xffc44d : 0x5eead4;
    const emissive = editPhase === "reference" ? 0x78350f : 0x134e4a;
    for (const p of handlePts) {
      const h = new THREE.Mesh(
        new THREE.SphereGeometry(0.55, 12, 12),
        new THREE.MeshStandardMaterial({ color, emissive })
      );
      h.position.set(p[0], 0.6, -p[1]);
      handles.add(h);
    }
  }, [editMode, editPhase, trackPoints, referencePoints, draftNpcs, draftObstacles]);

  return <div className="viewport-layer" ref={mountRef} />;
}
