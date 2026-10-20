import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

const statusEl = document.getElementById("status");

// ---------- WebSockets ----------
// Control socket: always the server that served this page (the robot Pi).
//   pose data out every XR frame, teleop_state JSON (HUD) in.
// Video socket: optional, a separate camera server (the Orin). Its host:port
//   comes from ?video=<host:port> if given (?video=none turns video off),
//   otherwise from the serving server's /config.json default.
//   Binary frames in only. Its TLS cert must have been accepted once by
//   visiting https://<host:port>/ directly, or the socket fails silently.
// Either socket also accepts binary frames, so a single server that does both
// (server.py --cameras on a dev PC) still works without ?video=.
const wsProto = location.protocol === "https:" ? "wss" : "ws";
let videoHost = new URLSearchParams(location.search).get("video");
if (videoHost === null) {
  try {
    videoHost = (await (await fetch("/config.json")).json()).video || null;
  } catch (e) {
    videoHost = null; // older server without /config.json
  }
} else if (videoHost === "" || videoHost === "none") {
  videoHost = null;
}

const links = {
  control: { url: `${wsProto}://${location.host}/ws`, ws: null, state: "connecting" },
  video: videoHost ? { url: `${wsProto}://${videoHost}/ws`, ws: null, state: "connecting" } : null,
};

let framesReceived = 0;
let fps = 0;
let lastFpsTime = performance.now();
let lastVideoFrameTime = 0;

function setStatus() {
  const c = links.control.state;
  let v = "off";
  if (links.video) {
    v = links.video.state === "open" ? `${fps} fps` : links.video.state;
    if (links.video.state === "error") v += ` -- accept the cert at https://${videoHost}/`;
  }
  const ipd = ipdMm === null ? "" : ` | IPD ${ipdMm.toFixed(1)} mm${stereoActive() ? " (stereo)" : ""}`;
  statusEl.textContent = `control: ${c} | video: ${v}${ipd}`;
}

function connect(name) {
  const link = links[name];
  const ws = new WebSocket(link.url);
  ws.binaryType = "arraybuffer";
  link.ws = ws;
  ws.addEventListener("open", () => { link.state = "open"; setStatus(); });
  ws.addEventListener("error", () => { link.state = "error"; setStatus(); });
  ws.addEventListener("close", () => {
    if (link.state !== "error") link.state = "disconnected";
    setStatus();
    setTimeout(() => connect(name), 2000); // servers may restart; keep retrying
  });
  ws.addEventListener("message", (ev) => {
    if (typeof ev.data === "string") {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "teleop_state") handleTeleopState(msg);
        if (msg.type === "stereo_info") handleStereoInfo(msg);
      } catch (e) {
        // ignore malformed status message
      }
      return;
    }
    if (ev.data instanceof ArrayBuffer) handleFrame(ev.data);
  });
}

function sendPose(obj) {
  const ws = links.control.ws;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// ---------- Offscreen canvases that the (demuxed) camera JPEGs draw into ----------
// Wire format: each binary WS message is [1 byte stream id] + [JPEG bytes].
// Stream ids must match server/cameras.py STREAM_IDS.
const STREAMS = {
  0: { name: "head", width: 640, height: 480 },
  1: { name: "gripper_left", width: 640, height: 480 },
  2: { name: "gripper_right", width: 640, height: 480 },
};

const streamCanvases = {};
const streamTextures = {};
for (const [id, info] of Object.entries(STREAMS)) {
  const canvas = document.createElement("canvas");
  canvas.width = info.width;
  canvas.height = info.height;
  const ctx = canvas.getContext("2d");
  // Placeholder until the first frame arrives
  ctx.fillStyle = "#1e1e1e";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#888888";
  ctx.font = "bold 40px monospace";
  ctx.textAlign = "center";
  ctx.fillText("NO VIDEO", canvas.width / 2, canvas.height / 2 - 10);
  ctx.font = "24px monospace";
  ctx.fillText(info.name, canvas.width / 2, canvas.height / 2 + 30);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  streamCanvases[id] = { canvas, ctx };
  streamTextures[id] = texture;
}

async function handleFrame(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 2) return;
  const streamId = bytes[0];
  if (streamId === STEREO_STREAM_ID) {
    framesReceived++;
    lastVideoFrameTime = performance.now();
    handleStereoFrame(bytes.subarray(1));
    return;
  }
  const entry = streamCanvases[streamId];
  if (!entry) return;

  framesReceived++;
  lastVideoFrameTime = performance.now();
  const blob = new Blob([bytes.subarray(1)], { type: "image/jpeg" });
  try {
    const bitmap = await createImageBitmap(blob);
    entry.ctx.drawImage(bitmap, 0, 0, entry.canvas.width, entry.canvas.height);
    streamTextures[streamId].needsUpdate = true;
    bitmap.close();
  } catch (e) {
    // ignore malformed frame
  }
}

setInterval(() => {
  const now = performance.now();
  fps = Math.round((framesReceived * 1000) / (now - lastFpsTime));
  framesReceived = 0;
  lastFpsTime = now;
  setStatus();
}, 1000);

connect("control");
if (links.video) connect("video");

function videoOk() {
  return performance.now() - lastVideoFrameTime < 1000;
}

// ---------- Three.js scene ----------
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101418);

const camera = new THREE.PerspectiveCamera(
  70,
  window.innerWidth / window.innerHeight,
  0.01,
  50
);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType("local-floor");
document.body.appendChild(renderer.domElement);
document.body.appendChild(
  VRButton.createButton(renderer, { optionalFeatures: ["local-floor"] })
);

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// Light so controller meshes are visible
scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.5));

// Head-locked screens: one big "head/ZED camera" view straight ahead, plus
// two small gripper-camera insets in the lower corners. Each is
// repositioned/reoriented every frame to a fixed offset from the head pose
// (see animation loop), so they follow you like a HUD instead of staying
// fixed in the room.
function makeScreen(streamId, width, height) {
  const geo = new THREE.PlaneGeometry(width, height);
  const mat = new THREE.MeshBasicMaterial({ map: streamTextures[streamId] });
  const mesh = new THREE.Mesh(geo, mat);
  scene.add(mesh);
  return mesh;
}

const headScreen = makeScreen(0, 1.0, 0.75);
const leftGripperScreen = makeScreen(1, 0.32, 0.24);
const rightGripperScreen = makeScreen(2, 0.32, 0.24);

// Offsets from the head, in head-local space (meters): main view straight
// ahead/slightly below eye line; gripper insets low in the left/right
// periphery.
const HEAD_SCREEN_OFFSET = new THREE.Vector3(0, -0.05, -1.2);
const LEFT_GRIPPER_OFFSET = new THREE.Vector3(-0.45, -0.3, -1.0);
const RIGHT_GRIPPER_OFFSET = new THREE.Vector3(0.45, -0.3, -1.0);

// ---------- Stereo ZED view (stream 3: rectified left|right side by side) ----------
// Each eye gets its own head-locked plane showing its half of the frame:
// three.js's WebXR left-eye camera renders layer 1 only, the right-eye camera
// layer 2 only. Each plane is centred in front of its eye (planes ipd apart),
// sized from the ZED calibration so the view is true scale in angle: an object
// at infinity (zero disparity) lands straight ahead of each eye.
// IPD: right stick up/down adjusts it (not in base mode), right stick click
// resets to the headset's own measured IPD; remembered on this headset;
// ?ipd=<mm> in the URL sets it.
const STEREO_STREAM_ID = 3;
const STEREO_DISTANCE = 3.0; // m; any distance works -- angular size is what's matched
const IPD_MIN_MM = 40;
const IPD_MAX_MM = 90;
const IPD_RATE_MM_S = 8; // at full stick deflection
const IPD_STORAGE_KEY = "vrteleop.ipd_mm";

// Until the video server sends calibration: ZED 2 at HD720 (fx ~ 530 px at 1280x720).
let stereoInfo = { width: 1280, height: 720, fx: 530, fy: 530, cx: 640, cy: 360, baseline_m: 0.12, model: "default" };
let lastStereoFrameTime = 0;

const stereoCanvas = document.createElement("canvas");
stereoCanvas.width = 2 * stereoInfo.width;
stereoCanvas.height = stereoInfo.height;
const stereoCtx = stereoCanvas.getContext("2d");
const stereoTexture = new THREE.CanvasTexture(stereoCanvas);
stereoTexture.colorSpace = THREE.SRGBColorSpace;

function makeEyePlane(layer, u0) {
  const geo = new THREE.PlaneGeometry(1, 1);
  const uv = geo.attributes.uv;
  for (let i = 0; i < uv.count; i++) uv.setX(i, u0 + 0.5 * uv.getX(i)); // this eye's half
  uv.needsUpdate = true;
  // Background: drawn first with no depth test, so HUD, insets and controllers draw over it.
  const mat = new THREE.MeshBasicMaterial({ map: stereoTexture, depthTest: false, depthWrite: false });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.layers.set(layer);
  mesh.visible = false;
  mesh.renderOrder = -1;
  scene.add(mesh);
  return mesh;
}
const leftEyePlane = makeEyePlane(1, 0.0);
const rightEyePlane = makeEyePlane(2, 0.5);

function handleStereoInfo(msg) {
  stereoInfo = msg;
  console.log("stereo_info", msg);
}

async function handleStereoFrame(jpegBytes) {
  try {
    const bitmap = await createImageBitmap(new Blob([jpegBytes], { type: "image/jpeg" }));
    if (stereoCanvas.width !== bitmap.width || stereoCanvas.height !== bitmap.height) {
      stereoCanvas.width = bitmap.width;
      stereoCanvas.height = bitmap.height;
      stereoTexture.dispose(); // reallocate at the new size
    }
    stereoCtx.drawImage(bitmap, 0, 0);
    stereoTexture.needsUpdate = true;
    bitmap.close();
    lastStereoFrameTime = performance.now();
  } catch (e) {
    // ignore malformed frame
  }
}

function stereoActive() {
  return performance.now() - lastStereoFrameTime < 1000;
}

// --- IPD setting
let measuredIpdMm = null; // from the headset's own eye poses
let ipdMm = null;
let ipdShownUntil = 0;
let ipdSaveTimer = null;
{
  const fromUrl = parseFloat(new URLSearchParams(location.search).get("ipd"));
  let stored = NaN;
  try {
    stored = parseFloat(localStorage.getItem(IPD_STORAGE_KEY));
  } catch (e) {
    // storage unavailable: fall back to the headset's measured IPD
  }
  if (Number.isFinite(fromUrl)) ipdMm = fromUrl;
  else if (Number.isFinite(stored)) ipdMm = stored;
}

function setIpd(mm, show = true) {
  ipdMm = Math.min(IPD_MAX_MM, Math.max(IPD_MIN_MM, mm));
  if (show) {
    ipdShownUntil = performance.now() + 2000;
    refreshHud();
  }
  clearTimeout(ipdSaveTimer);
  ipdSaveTimer = setTimeout(() => {
    try {
      localStorage.setItem(IPD_STORAGE_KEY, ipdMm.toFixed(1));
    } catch (e) {
      // not persisted; still applies for this session
    }
  }, 500);
}

let prevIpdReset = false;
function updateIpdFromStick(rightGamepad, dtS) {
  if (!rightGamepad) return;
  const inBaseMode = !!(lastTeleopState && String(lastTeleopState.state).includes("BASE"));
  const reset = !!(rightGamepad.buttons[3] && rightGamepad.buttons[3].pressed);
  if (reset && !prevIpdReset && measuredIpdMm !== null) setIpd(measuredIpdMm);
  prevIpdReset = reset;
  if (inBaseMode) return;
  const y = rightGamepad.axes.length >= 4 ? rightGamepad.axes[3] : 0;
  if (Math.abs(y) < 0.3) return;
  // stick forward (y < 0) -> wider
  const v = (Math.abs(y) - 0.3) / 0.7;
  setIpd(ipdMm - Math.sign(y) * v * IPD_RATE_MM_S * dtS);
}

const eyeOffset = new THREE.Vector3();
function placeEyePlanes(headPos, headQuat) {
  const on = stereoActive();
  leftEyePlane.visible = rightEyePlane.visible = on;
  headScreen.visible = !on;
  floorGrid.visible = !on; // the grid would cut through the video
  if (!on) return;
  const { width: W, height: H, fx, fy, cx, cy } = stereoInfo;
  const D = STEREO_DISTANCE;
  const w = (D * W) / fx;
  const h = (D * H) / fy;
  const ox = (D * (W / 2 - cx)) / fx; // principal point offset (image y is down)
  const oy = (D * (cy - H / 2)) / fy;
  const half = (ipdMm ?? 63) / 2000; // m
  for (const [mesh, side] of [[leftEyePlane, -1], [rightEyePlane, 1]]) {
    mesh.scale.set(w, h, 1);
    mesh.quaternion.copy(headQuat);
    eyeOffset.set(side * half + ox, oy, -D).applyQuaternion(headQuat);
    mesh.position.copy(headPos).add(eyeOffset);
  }
}

function headLock(mesh, offset, headPos, headQuat) {
  mesh.quaternion.copy(headQuat);
  mesh.position.copy(offset).applyQuaternion(headQuat).add(headPos);
}

// Simple floor grid for spatial reference
const floorGrid = new THREE.GridHelper(6, 12, 0x3a4a5a, 0x222a33);
scene.add(floorGrid);

// ---------- Controllers ----------
function makeControllerMesh(color) {
  const geo = new THREE.ConeGeometry(0.02, 0.08, 16);
  geo.rotateX(-Math.PI / 2);
  const mat = new THREE.MeshStandardMaterial({ color });
  return new THREE.Mesh(geo, mat);
}

// Small enable-state indicator sphere per controller: dim gray = disabled,
// green = arm enabled, red = warning (e.g. dead-man's switch tripped).
function makeIndicatorMesh() {
  const geo = new THREE.SphereGeometry(0.015, 12, 12);
  const mat = new THREE.MeshStandardMaterial({ color: 0x555555, emissive: 0x222222 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(0, 0.05, 0);
  return mesh;
}

const INDICATOR_COLORS = { off: 0x555555, on: 0x33ff66, warning: 0xff3333 };

const controllers = { left: null, right: null };

for (let i = 0; i < 2; i++) {
  const controller = renderer.xr.getController(i);
  scene.add(controller);

  const grip = renderer.xr.getControllerGrip(i);
  scene.add(grip);

  controller.addEventListener("connected", (event) => {
    const handedness = event.data.handedness; // 'left' | 'right'
    const color = handedness === "left" ? 0x3ddc84 : 0xff6b6b;
    const mesh = makeControllerMesh(color);
    grip.add(mesh);
    const indicator = makeIndicatorMesh();
    grip.add(indicator);
    controllers[handedness] = { controller, grip, inputSource: event.data, indicator };
  });

  controller.addEventListener("disconnected", () => {
    for (const hand of ["left", "right"]) {
      if (controllers[hand] && controllers[hand].controller === controller) {
        controllers[hand] = null;
      }
    }
  });
}

function setIndicator(hand, state) {
  const entry = controllers[hand];
  if (!entry) return;
  entry.indicator.material.color.setHex(INDICATOR_COLORS[state]);
  entry.indicator.material.emissive.setHex(INDICATOR_COLORS[state]);
}

function hapticPulse(hand, intensity, durationMs) {
  const actuator = controllers[hand]?.inputSource?.gamepad?.hapticActuators?.[0];
  if (actuator) {
    try {
      actuator.pulse(intensity, durationMs);
    } catch (e) {
      // not all browsers/controllers support this -- fine to skip
    }
  }
}

// ---------- Head-locked status HUD ----------
const hudCanvas = document.createElement("canvas");
hudCanvas.width = 512;
hudCanvas.height = 128;
const hudCtx = hudCanvas.getContext("2d");
const hudTexture = new THREE.CanvasTexture(hudCanvas);
const hudMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(0.5, 0.125),
  new THREE.MeshBasicMaterial({ map: hudTexture, transparent: true })
);
scene.add(hudMesh);
const HUD_OFFSET = new THREE.Vector3(0, 0.38, -1.2);

function drawHud(text, color) {
  hudCtx.clearRect(0, 0, hudCanvas.width, hudCanvas.height);
  hudCtx.fillStyle = "rgba(10,12,15,0.6)";
  hudCtx.fillRect(0, 0, hudCanvas.width, hudCanvas.height);
  hudCtx.font = "bold 36px monospace";
  hudCtx.fillStyle = color;
  hudCtx.textAlign = "center";
  hudCtx.textBaseline = "middle";
  hudCtx.fillText(text, hudCanvas.width / 2, hudCanvas.height / 2);
  hudTexture.needsUpdate = true;
}

drawHud("STATE: IDLE", "#aaaaaa");

let lastTeleopState = null;

function handleTeleopState(msg) {
  const prevEnabled = lastTeleopState ? lastTeleopState.arm_enabled : { left: false, right: false };

  for (const hand of ["left", "right"]) {
    const isEnabled = !!msg.arm_enabled[hand];
    const wasEnabled = !!prevEnabled[hand];
    setIndicator(hand, isEnabled ? "on" : "off");
    if (isEnabled && !wasEnabled) hapticPulse(hand, 0.3, 80);
    if (!isEnabled && wasEnabled) {
      hapticPulse(hand, 0.5, 60);
      setTimeout(() => hapticPulse(hand, 0.5, 60), 120);
    }
  }

  for (const w of msg.warnings || []) {
    const hand = w.startsWith("left") ? "left" : w.startsWith("right") ? "right" : null;
    if (hand) {
      setIndicator(hand, "warning");
      hapticPulse(hand, 0.9, 200);
    }
  }

  lastTeleopState = msg;
  refreshHud();
}

function refreshHud() {
  const msg = lastTeleopState;
  let text = "STATE: --";
  let color = "#aaaaaa";
  if (msg) {
    const l = msg.arm_enabled.left ? "ON" : "off";
    const r = msg.arm_enabled.right ? "ON" : "off";
    color = msg.warnings && msg.warnings.length ? "#ff5555" : "#eeeeee";
    text = `${msg.state}  L:${l}  R:${r}`;
  }
  if (!videoOk()) text += "  NO VIDEO";
  if (performance.now() < ipdShownUntil && ipdMm !== null) {
    text = `IPD ${ipdMm.toFixed(1)} mm`;
    color = "#66ccff";
  }
  drawHud(text, color);
}

// ---------- Pose extraction + send loop ----------
const tmpPos = new THREE.Vector3();
const tmpQuat = new THREE.Quaternion();

function poseFromXRPose(xrPose) {
  const p = xrPose.transform.position;
  const o = xrPose.transform.orientation;
  return { pos: [p.x, p.y, p.z], quat: [o.x, o.y, o.z, o.w] };
}

function gamepadAxes(inputSource) {
  const gp = inputSource.gamepad;
  if (!gp) return { trigger: 0, grip: 0, axes: [], buttons: {} };
  const trigger = gp.buttons[0] ? gp.buttons[0].value : 0;
  const grip = gp.buttons[1] ? gp.buttons[1].value : 0;
  const buttons = {};
  gp.buttons.forEach((b, idx) => (buttons[idx] = { pressed: b.pressed, value: b.value }));
  return { trigger, grip, axes: Array.from(gp.axes), buttons };
}

let lastLoopTime = null;
renderer.setAnimationLoop((timestamp, frame) => {
  const dtS = lastLoopTime === null ? 0 : Math.min(0.1, (timestamp - lastLoopTime) / 1000);
  lastLoopTime = timestamp;
  if (frame) {
    const refSpace = renderer.xr.getReferenceSpace();
    const session = renderer.xr.getSession();

    let headPayload = null;
    const viewerPose = frame.getViewerPose(refSpace);
    if (viewerPose) {
      // The viewer pose is the point between the eyes (views[0] is the left eye).
      const p = viewerPose.transform.position;
      const o = viewerPose.transform.orientation;
      tmpPos.set(p.x, p.y, p.z);
      tmpQuat.set(o.x, o.y, o.z, o.w);
      headPayload = { pos: [p.x, p.y, p.z], quat: [o.x, o.y, o.z, o.w] };

      if (viewerPose.views.length >= 2) {
        const a = viewerPose.views[0].transform.position;
        const b = viewerPose.views[1].transform.position;
        const mm = 1000 * Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
        if (mm > 30 && mm < 100) {
          measuredIpdMm = mm;
          if (ipdMm === null) setIpd(mm, false); // first run on this headset: start at its IPD
        }
      }
      placeEyePlanes(tmpPos, tmpQuat);

      // Head-lock all three screens + status HUD to this frame's head pose.
      headLock(headScreen, HEAD_SCREEN_OFFSET, tmpPos, tmpQuat);
      headLock(leftGripperScreen, LEFT_GRIPPER_OFFSET, tmpPos, tmpQuat);
      headLock(rightGripperScreen, RIGHT_GRIPPER_OFFSET, tmpPos, tmpQuat);
      headLock(hudMesh, HUD_OFFSET, tmpPos, tmpQuat);
    }

    let leftPayload = null;
    let rightPayload = null;
    for (const inputSource of session.inputSources) {
      if (!inputSource.gripSpace && !inputSource.targetRaySpace) continue;
      const space = inputSource.gripSpace || inputSource.targetRaySpace;
      const pose = frame.getPose(space, refSpace);
      if (!pose) continue;
      const payload = { ...poseFromXRPose(pose), ...gamepadAxes(inputSource) };
      if (inputSource.handedness === "left") leftPayload = payload;
      if (inputSource.handedness === "right") {
        rightPayload = payload;
        updateIpdFromStick(inputSource.gamepad, dtS);
      }
    }

    sendPose({ t: Date.now(), head: headPayload, left: leftPayload, right: rightPayload });
  }

  renderer.render(scene, camera);
});
