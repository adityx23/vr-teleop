import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";
import { latestFrameDecoder } from "./latest_frame.mjs";

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

let framesDisplayed = 0;
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
    if (name === "video" || !links.video) {
      for (const decoder of Object.values(frameDecoders)) decoder.reset();
      stereoDecoder.reset();
    }
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
  // 1 / 2 (gripper_left / gripper_right) are not shown; their frames are ignored.
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

function markDisplayed(streamId) {
  if (streamId === STEREO_STREAM_ID || !stereoActive()) {
    framesDisplayed++;
    lastVideoFrameTime = performance.now();
  }
}

const decodeJpeg = (bytes) => createImageBitmap(new Blob([bytes], { type: "image/jpeg" }));
const frameDecoders = {};
for (const [id, entry] of Object.entries(streamCanvases)) {
  frameDecoders[id] = latestFrameDecoder(decodeJpeg, (bitmap) => {
    // Match the canvas to the stream, so a higher-res camera isn't squashed to 640x480.
    if (entry.canvas.width !== bitmap.width || entry.canvas.height !== bitmap.height) {
      entry.canvas.width = bitmap.width;
      entry.canvas.height = bitmap.height;
      streamTextures[id].dispose(); // reallocate at the new size
    }
    entry.ctx.drawImage(bitmap, 0, 0);
    streamTextures[id].needsUpdate = true;
    markDisplayed(Number(id));
  });
}

function handleFrame(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 2) return;
  const streamId = bytes[0];
  if (streamId === STEREO_STREAM_ID) {
    stereoDecoder.push(bytes.subarray(1));
    return;
  }
  frameDecoders[streamId]?.push(bytes.subarray(1));
}

setInterval(() => {
  const now = performance.now();
  fps = Math.round((framesDisplayed * 1000) / (now - lastFpsTime));
  framesDisplayed = 0;
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

// Head-locked screen: the mono "head" camera view straight ahead (hidden while
// the stereo ZED view is live). Repositioned every frame to a fixed offset
// from the head pose (see animation loop), so it follows you like a HUD.
function makeScreen(streamId, width, height) {
  const geo = new THREE.PlaneGeometry(width, height);
  const mat = new THREE.MeshBasicMaterial({ map: streamTextures[streamId] });
  const mesh = new THREE.Mesh(geo, mat);
  scene.add(mesh);
  return mesh;
}

const headScreen = makeScreen(0, 1.0, 0.75);

// Offset from the head, in head-local space (meters): straight ahead, slightly below eye line.
const HEAD_SCREEN_OFFSET = new THREE.Vector3(0, -0.05, -1.2);

// ---------- Stereo ZED view (stream 3: rectified left|right side by side) ----------
// Each eye gets its own head-locked plane showing its half of the frame:
// three.js's WebXR left-eye camera renders layer 1 only, the right-eye camera
// layer 2 only. Each plane is centred in front of its eye (planes ipd apart),
// sized from the ZED calibration so the view is true scale in angle: an object
// at infinity (zero disparity) lands straight ahead of each eye.
// IPD: right stick up/down adjusts it (not in base mode); starts at the
// headset's own measured IPD; remembered on this headset; ?ipd=<mm> in the
// URL sets it. (Right stick click is the episode start/stop button.)
const STEREO_STREAM_ID = 3;
const STEREO_DISTANCE = 3.0; // m; any distance works -- angular size is what's matched
const IPD_MIN_MM = 40;
const IPD_MAX_MM = 90;
const IPD_RATE_MM_S = 8; // at full stick deflection
const IPD_STORAGE_KEY = "vrteleop.ipd_mm";
// Headset per-eye views are vertically asymmetric (Quest sees further down
// than up), so a plane centred on "straight ahead" leaves a bigger empty band
// at the bottom. Shift both planes (same amount, so no vertical disparity) to
// the vertical centre of the eye frustums. ?center=0 turns this off.
const CENTER_IN_VIEW = new URLSearchParams(location.search).get("center") !== "0";
let viewCenterTanY = 0; // tan of the frustum's vertical centre angle (+ = up)

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
  // Background: drawn first with no depth test, so HUD and controllers draw over it.
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

const stereoDecoder = latestFrameDecoder(decodeJpeg, (bitmap) => {
    if (stereoCanvas.width !== bitmap.width || stereoCanvas.height !== bitmap.height) {
      stereoCanvas.width = bitmap.width;
      stereoCanvas.height = bitmap.height;
      stereoTexture.dispose(); // reallocate at the new size
    }
    stereoCtx.drawImage(bitmap, 0, 0);
    stereoTexture.needsUpdate = true;
    lastStereoFrameTime = performance.now();
    markDisplayed(STEREO_STREAM_ID);
});

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

// --- view tilt: how far the robot's camera looks down (- ) or up (+) relative to your head,
// so the operator can look straight ahead while the robot looks at the table. Right stick
// left/right while teleop is OFF (left = down); the server adds it to the head pitch
// (view_pitch_deg in every pose message; it takes effect when teleop turns on, since the
// neck holds while off). Remembered on this headset; ?tilt=<deg> in the URL sets it.
const TILT_STORAGE_KEY = "vrteleop.view_tilt_deg";
const TILT_MIN_DEG = -60; // matches whole_body.py view_pitch_limits_deg
const TILT_MAX_DEG = 30;
const TILT_RATE_DEG_S = 20; // at full deflection
let viewTiltDeg = 0;
let tiltShownUntil = 0;
let tiltSaveTimer = null;
{
  const fromUrl = parseFloat(new URLSearchParams(location.search).get("tilt"));
  let stored = NaN;
  try {
    stored = parseFloat(localStorage.getItem(TILT_STORAGE_KEY));
  } catch (e) {
    // storage unavailable: start level
  }
  if (Number.isFinite(fromUrl)) viewTiltDeg = fromUrl;
  else if (Number.isFinite(stored)) viewTiltDeg = stored;
  viewTiltDeg = Math.min(TILT_MAX_DEG, Math.max(TILT_MIN_DEG, viewTiltDeg));
}

function updateTiltFromStick(rightGamepad, dtS) {
  if (!rightGamepad) return;
  if (lastTeleopState && lastTeleopState.enabled) return; // teleop on: right stick x turns the base
  const x = rightGamepad.axes.length >= 4 ? rightGamepad.axes[2] : 0;
  const y = rightGamepad.axes.length >= 4 ? rightGamepad.axes[3] : 0;
  if (Math.abs(x) < 0.3 || Math.abs(y) >= Math.abs(x)) return; // up/down is the IPD
  // stick right (x > 0) -> look further up
  const v = (Math.abs(x) - 0.3) / 0.7;
  viewTiltDeg = Math.min(TILT_MAX_DEG, Math.max(TILT_MIN_DEG, viewTiltDeg + Math.sign(x) * v * TILT_RATE_DEG_S * dtS));
  tiltShownUntil = performance.now() + 2000;
  refreshHud();
  clearTimeout(tiltSaveTimer);
  tiltSaveTimer = setTimeout(() => {
    try {
      localStorage.setItem(TILT_STORAGE_KEY, viewTiltDeg.toFixed(1));
    } catch (e) {
      // not persisted; still applies for this session
    }
  }, 500);
}

function updateIpdFromStick(rightGamepad, dtS) {
  if (!rightGamepad) return;
  const inBaseMode = !!(lastTeleopState && String(lastTeleopState.state).includes("BASE"));
  if (inBaseMode) return;
  const x = rightGamepad.axes.length >= 4 ? rightGamepad.axes[2] : 0;
  const y = rightGamepad.axes.length >= 4 ? rightGamepad.axes[3] : 0;
  if (Math.abs(y) < 0.3 || Math.abs(x) > Math.abs(y)) return; // left/right is the view tilt
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
  const oy = (D * (cy - H / 2)) / fy + (CENTER_IN_VIEW ? D * viewCenterTanY : 0);
  const half = (ipdMm ?? 63) / 2000; // m
  for (const [mesh, side] of [[leftEyePlane, -1], [rightEyePlane, 1]]) {
    mesh.scale.set(w, h, 1);
    mesh.quaternion.copy(headQuat);
    eyeOffset.set(side * half + ox, oy, -D).applyQuaternion(headQuat);
    mesh.position.copy(headPos).add(eyeOffset);
  }
}

// tan of the angle (+ up) from straight ahead to the camera image's bottom edge
function videoBottomTan() {
  if (!stereoActive()) return (HEAD_SCREEN_OFFSET.y - 0.75 / 2) / -HEAD_SCREEN_OFFSET.z; // mono screen
  const { height: H, fy, cy } = stereoInfo;
  const centre = (cy - H / 2) / fy + (CENTER_IN_VIEW ? viewCenterTanY : 0); // as in placeEyePlanes
  return centre - H / (2 * fy);
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
// The whole controller marker (cone + sphere) is hidden while that arm is clutched.
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

// ---------- Halo HUD ----------
// Head-locked status ring around the edge of view; the centre stays clear for
// the video. Panels: top (mode, links, REC/IPD/view tilt), bottom (base motion, cycle
// subtask or task -- each only when it has data), lift height just below the camera
// image, plus a pulsing alert banner. (No left/right arm panels: arm state shows as the
// mode chip and alerts; a controller's marker hides while its arm is clutched.) Each panel is a small canvas texture, redrawn
// only when what it shows changes (checked at HUD_RATE_HZ).
// Robot fields come from teleop_state.robot (robot/whole_body_teleop.py
// RobotLink.hud()) and teleop_state.recording; both optional -- whatever the
// server doesn't send is shown as "--" or left out.
const HUD_DIST = 1.4; // m in front of the eyes
const HUD_RATE_HZ = 10;
const C = {
  cyan: "#5ff4ff", dim: "rgba(95,244,255,0.60)", faint: "rgba(95,244,255,0.14)",
  text: "#dffcff", green: "#6dff9c", amber: "#ffb347", red: "#ff4d5e", grey: "#8a99a3",
};
const HUD_FONT = '"Roboto Mono", "DejaVu Sans Mono", "Courier New", monospace';
const Y_AXIS = new THREE.Vector3(0, 1, 0);

const hudPanels = [];
function makeHudPanel({ px, widthM, offset, yawDeg = 0, belowVideo = false, view, draw }) {
  const canvas = document.createElement("canvas");
  canvas.width = px[0];
  canvas.height = px[1];
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(widthM, (widthM * px[1]) / px[0]),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false, depthWrite: false })
  );
  mesh.renderOrder = 10; // over the video
  scene.add(mesh);
  const panel = {
    ctx: canvas.getContext("2d"), canvas, texture, mesh, offset, view, draw, key: null, belowVideo,
    halfH: (widthM * px[1]) / px[0] / 2,
    turn: new THREE.Quaternion().setFromAxisAngle(Y_AXIS, THREE.MathUtils.degToRad(yawDeg)),
  };
  hudPanels.push(panel);
  return panel;
}

// --- drawing helpers
function glowText(ctx, text, x, y, { size = 28, color = C.cyan, align = "left", weight = 600 } = {}) {
  ctx.font = `${weight} ${size}px ${HUD_FONT}`;
  ctx.textAlign = align;
  ctx.textBaseline = "middle";
  ctx.lineJoin = "round";
  ctx.shadowBlur = 0;
  ctx.lineWidth = Math.max(2, size / 8);
  ctx.strokeStyle = "rgba(0,8,12,0.75)"; // dark outline: readable over bright video
  ctx.strokeText(text, x, y);
  ctx.shadowColor = color;
  ctx.shadowBlur = size / 3;
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
  ctx.shadowBlur = 0;
}

function glowPath(ctx, color, width, path) {
  ctx.shadowBlur = 0;
  ctx.lineWidth = width + 4;
  ctx.strokeStyle = "rgba(0,8,12,0.5)";
  ctx.beginPath();
  path();
  ctx.stroke();
  ctx.lineWidth = width;
  ctx.strokeStyle = color;
  ctx.shadowColor = color;
  ctx.shadowBlur = 10;
  ctx.beginPath();
  path();
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function chamfer(ctx, x, y, w, h, c) {
  ctx.moveTo(x + c, y);
  ctx.lineTo(x + w - c, y);
  ctx.lineTo(x + w, y + c);
  ctx.lineTo(x + w, y + h - c);
  ctx.lineTo(x + w - c, y + h);
  ctx.lineTo(x + c, y + h);
  ctx.lineTo(x, y + h - c);
  ctx.lineTo(x, y + c);
  ctx.closePath();
}

function scrim(ctx, x, y, w, h, c = 14) {
  ctx.shadowBlur = 0;
  ctx.fillStyle = "rgba(2,14,20,0.42)";
  ctx.beginPath();
  chamfer(ctx, x, y, w, h, c);
  ctx.fill();
}

function triangle(ctx, x, y, size, down, color) {
  ctx.fillStyle = color;
  ctx.shadowColor = color;
  ctx.shadowBlur = 8;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x - size, y + (down ? -size * 1.4 : size * 1.4));
  ctx.lineTo(x + size, y + (down ? -size * 1.4 : size * 1.4));
  ctx.closePath();
  ctx.fill();
  ctx.shadowBlur = 0;
}

const sgn = (v) => (v >= 0 ? "+" : "-");
const fmtDeg = (v) => `${sgn(v)}${String(Math.abs(Math.round(v))).padStart(2, "0")}°`;
const clamp01 = (v) => Math.min(1, Math.max(0, v));

// --- model: everything the panels show, gathered once per HUD tick
const WARNING_TEXT = {
  left_tracking_lost: ["L CONTROLLER TRACKING LOST", "red"],
  right_tracking_lost: ["R CONTROLLER TRACKING LOST", "red"],
  left_no_ee_pose: ["L ARM POSE UNAVAILABLE", "amber"],
  right_no_ee_pose: ["R ARM POSE UNAVAILABLE", "amber"],
};

const MODE_COLOR = { OFF: C.grey, ON: C.cyan, HOMING: C.amber };
function modeColor(state) {
  if (MODE_COLOR[state]) return MODE_COLOR[state];
  if (state.includes("LIFT ")) return C.amber;
  if (state.includes("BASE")) return C.amber;
  if (state.includes("ARMS")) return C.green;
  return C.cyan;
}

function hudModel() {
  const s = lastTeleopState;
  const r = (s && s.robot) || null;
  const alerts = [];
  if (links.control.state !== "open") alerts.push([`CONTROL LINK ${links.control.state.toUpperCase()}`, "red"]);
  if (r && !r.rpc_ok) alerts.push([r.error ? `ROBOT: ${r.error}`.slice(0, 44).toUpperCase() : "ROBOT LINK STALLED", "red"]);
  for (const w of (s && s.warnings) || []) alerts.push(WARNING_TEXT[w] || [w.replace(/_/g, " ").toUpperCase(), "amber"]);
  for (const side of ["left", "right"]) {
    if (s && s.arm_enabled && s.arm_enabled[side] && r && r.ik_ok && r.ik_ok[side] === false) {
      alerts.push([`${side[0].toUpperCase()} ARM AT REACH / JOINT LIMIT`, "amber"]);
    }
  }
  if (links.video && !videoOk()) alerts.push(["NO VIDEO", "amber"]);
  const cy = s && s.recording && s.recording.cycle;
  if (cy && cy.stale) alerts.push(["ORIN COLLECTOR NOT HEARD", "amber"]);
  if (cy && cy.pending) alerts.push([`CYCLE -> ${String(cy.pending).toUpperCase()} AFTER THIS EPISODE`.slice(0, 44), "amber"]);
  // lift_height is only absolute (travel above the bottom stop) once the lift is homed
  if (r && r.lift && r.lift.homed === false) {
    const rec = s && s.recording && s.recording.active;
    alerts.push([r.lift.homing ? "LIFT HOMING" : "LIFT NOT HOMED", rec ? "red" : "amber"]);
  }
  return { s, r, alerts };
}

// --- top: mode chip, links, REC / IPD / view tilt
const topPanel = makeHudPanel({
  px: [1024, 120], widthM: 0.9, offset: new THREE.Vector3(0, 0.45, -HUD_DIST),
  view: ({ s, r }) => {
    const rec = s && s.recording && s.recording.active ? s.recording : null;
    return {
      mode: s ? String(s.state) : "NO LINK",
      dry: !!(r && r.dry_run),
      ctrl: links.control.state,
      vid: !links.video ? "OFF" : videoOk() ? `${fps} FPS` : links.video.state.toUpperCase(),
      vidOk: !links.video || videoOk(),
      ipd: performance.now() < ipdShownUntil && ipdMm !== null ? ipdMm.toFixed(1) : null,
      tilt: performance.now() < tiltShownUntil ? Math.round(viewTiltDeg) : null,
      rec: rec ? { ep: rec.episode ?? null, t: Math.floor(rec.elapsed_s || 0) } : null,
    };
  },
  draw: (ctx, W, H, v) => {
    // links (left), mode chip (centre), REC / IPD (right)
    const ctrlOk = v.ctrl === "open";
    ctx.fillStyle = ctrlOk ? C.green : C.red;
    ctx.shadowColor = ctx.fillStyle;
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.arc(48, 44, 9, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    glowText(ctx, "CTRL", 66, 44, { size: 26, color: ctrlOk ? C.cyan : C.red });
    glowText(ctx, `VID ${v.vid}`, 160, 44, { size: 26, color: v.vidOk ? C.cyan : C.amber });

    const color = modeColor(v.mode);
    const modeSize = v.mode.length > 12 ? 28 : 36;
    ctx.font = `700 ${modeSize}px ${HUD_FONT}`;
    const chipW = Math.min(340, Math.max(240, ctx.measureText(v.mode).width + 48));
    const chipX = 512 - chipW / 2;
    glowPath(ctx, color, 3, () => chamfer(ctx, chipX, 12, chipW, 64, 16));
    ctx.fillStyle = "rgba(2,14,20,0.45)";
    ctx.beginPath();
    chamfer(ctx, chipX, 12, chipW, 64, 16);
    ctx.fill();
    glowText(ctx, v.mode, 512, 45, { size: modeSize, color, align: "center", weight: 700 });
    if (v.dry) glowText(ctx, "DRY RUN", 512, 92, { size: 20, color: C.amber, align: "center" });

    if (v.tilt !== null) {
      const t = v.tilt === 0 ? "LEVEL" : `${Math.abs(v.tilt)}° ${v.tilt < 0 ? "DOWN" : "UP"}`;
      glowText(ctx, `VIEW TILT ${t}`, W - 40, 44, { size: 28, color: C.text, align: "right" });
    } else if (v.ipd !== null) {
      glowText(ctx, `IPD ${v.ipd} MM`, W - 40, 44, { size: 28, color: C.text, align: "right" });
    } else if (v.rec) {
      const mm = String(Math.floor(v.rec.t / 60)).padStart(2, "0");
      const ss = String(v.rec.t % 60).padStart(2, "0");
      const ep = v.rec.ep === null ? "" : ` EP ${v.rec.ep}`;
      glowText(ctx, `REC${ep}  ${mm}:${ss}`, W - 40, 44, { size: 28, color: C.red, align: "right", weight: 700 });
      const tw = ctx.measureText(`REC${ep}  ${mm}:${ss}`).width;
      ctx.fillStyle = C.red;
      ctx.shadowColor = C.red;
      ctx.shadowBlur = 12;
      ctx.beginPath();
      ctx.arc(W - 40 - tw - 20, 44, 10, 0, 2 * Math.PI);
      ctx.fill();
      ctx.shadowBlur = 0;
    }
  },
});

// --- bottom: base motion, and the cycle's current subtask (always, once the Orin
// collector has announced a cycle) or else the task prompt while recording
const bottomPanel = makeHudPanel({
  px: [1024, 200], widthM: 0.8, offset: new THREE.Vector3(0, -0.4, -HUD_DIST),
  view: ({ s, r }) => {
    const base = s && String(s.state).includes("BASE") && r && r.base_vel ? r.base_vel : null;
    const rec = s && s.recording && s.recording.active ? s.recording : null;
    const cy = s && s.recording && s.recording.cycle ? s.recording.cycle : null;
    return {
      cycle: cy ? { k: cy.idx + 1, n: cy.n, sub: String(cy.subtask).slice(0, 52), state: cy.state, stale: cy.stale } : null,
      base: base ? base.map((x) => Math.round(x * 100) / 100) : null,
      task: rec && rec.task ? String(rec.task).slice(0, 70) : null,
    };
  },
  draw: (ctx, W, H, v) => {
    // base: direction dial (robot x forward = up, y left = left) + numbers, only in base mode
    const dx = 690, dy = 92, dr = 52;
    if (v.base) {
      glowPath(ctx, C.dim, 2, () => ctx.arc(dx, dy, dr, 0, 2 * Math.PI));
      glowText(ctx, "BASE", W - 60, 46, { size: 24, color: C.amber, align: "right" });
      const [vx, vy, wz] = v.base;
      const speed = Math.hypot(vx, vy);
      if (speed > 0.005) {
        const k = Math.min(1, speed / 0.3) * (dr - 6) / speed;
        glowPath(ctx, C.amber, 5, () => {
          ctx.moveTo(dx, dy);
          ctx.lineTo(dx - vy * k, dy - vx * k);
        });
      }
      if (Math.abs(wz) > 0.01) { // turn: arc along the dial, counter-clockwise = left
        const a = Math.min(Math.PI, Math.abs(wz) * 1.5);
        glowPath(ctx, C.amber, 4, () => ctx.arc(dx, dy, dr + 10, -Math.PI / 2, -Math.PI / 2 - Math.sign(wz) * a, wz > 0));
      }
      glowText(ctx, `${speed.toFixed(2)} M/S`, W - 60, 92, { size: 26, color: C.text, align: "right" });
      glowText(ctx, `${sgn(wz)}${Math.abs(wz).toFixed(2)} RAD/S`, W - 60, 128, { size: 22, color: C.dim, align: "right" });
    }

    if (v.cycle) {
      const c = v.cycle;
      const [label, color] = c.state === "rec" ? ["● REC", C.red] : c.state === "retry" ? ["RETRY", C.amber] : ["READY", C.green];
      scrim(ctx, 40, 160, W - 80, 36, 10);
      glowText(ctx, `${c.k}/${c.n}`, 60, 178, { size: 20, color: C.dim });
      glowText(ctx, c.sub.toUpperCase(), W / 2, 178, { size: 20, color: c.stale ? C.dim : C.text, align: "center", weight: 700 });
      glowText(ctx, label, W - 60, 178, { size: 20, color, align: "right", weight: 700 });
    } else if (v.task) {
      scrim(ctx, 40, 160, W - 80, 36, 10);
      glowText(ctx, `TASK  ${v.task.toUpperCase()}`, W / 2, 178, { size: 20, color: C.text, align: "center", weight: 400 });
    }
  },
});

// --- lift: height bar just below the camera image (placeHud moves it to the image's
// bottom edge every frame), so it never covers the view
const liftPanel = makeHudPanel({
  px: [640, 112], widthM: 0.4, offset: new THREE.Vector3(0, -0.6, -HUD_DIST), belowVideo: true,
  view: ({ s, r }) => {
    const lift = r && r.lift ? r.lift : null;
    return {
      lift: lift ? { h: Math.round(lift.height * 1000), min: lift.min, max: lift.max } : null,
      liftDir: s && String(s.state).includes("LIFT UP") ? "up" : s && String(s.state).includes("LIFT DOWN") ? "down" : null,
    };
  },
  draw: (ctx, W, H, v) => {
    if (!v.lift) return; // hidden with no height reading
    const bx = 20, by = 54, bw = 420, bh = 26, n = 20;
    glowText(ctx, "LIFT", bx, 26, { size: 24, color: C.dim });
    if (v.liftDir) glowText(ctx, v.liftDir === "up" ? "▲ UP" : "▼ DOWN", bx + 90, 26, { size: 24, color: C.amber, weight: 700 });
    const frac = clamp01((v.lift.h / 1000 - v.lift.min) / (v.lift.max - v.lift.min));
    for (let i = 0; i < n; i++) {
      const lit = (i + 0.5) / n <= frac;
      ctx.fillStyle = lit ? C.cyan : C.faint;
      ctx.shadowColor = C.cyan;
      ctx.shadowBlur = lit ? 8 : 0;
      ctx.fillRect(bx + (i * bw) / n + 2, by, bw / n - 4, bh);
    }
    ctx.shadowBlur = 0;
    glowText(ctx, `${(v.lift.h / 1000).toFixed(3)} M`, W - 20, by + bh / 2, { size: 26, color: C.text, align: "right" });
  },
});

// --- alert banner: top two alerts, pulses while any is red
const alertPanel = makeHudPanel({
  px: [1024, 96], widthM: 0.72, offset: new THREE.Vector3(0, 0.24, -HUD_DIST),
  view: ({ alerts }) => ({ alerts: alerts.slice(0, 2) }),
  draw: (ctx, W, H, v) => {
    if (!v.alerts.length) return;
    const color = v.alerts.some((a) => a[1] === "red") ? C.red : C.amber;
    scrim(ctx, 8, 8, W - 16, H - 16, 18);
    glowPath(ctx, color, 3, () => chamfer(ctx, 8, 8, W - 16, H - 16, 18));
    glowText(ctx, `⚠ ${v.alerts.map((a) => a[0]).join("  ·  ")}`, W / 2, H / 2, { size: 28, color, align: "center", weight: 700 });
  },
});

let lastHudUpdate = -Infinity;
function refreshHud() {
  lastHudUpdate = -Infinity; // redraw on the next frame
}

function updateHud(now) {
  if (now - lastHudUpdate < 1000 / HUD_RATE_HZ) return;
  lastHudUpdate = now;
  const model = hudModel();
  for (const p of hudPanels) {
    const v = p.view(model);
    const key = JSON.stringify(v);
    if (key === p.key) continue;
    p.key = key;
    p.ctx.clearRect(0, 0, p.canvas.width, p.canvas.height);
    p.draw(p.ctx, p.canvas.width, p.canvas.height, v);
    p.texture.needsUpdate = true;
  }
  alertPanel.mesh.visible = model.alerts.length > 0;
  alertPanel.red = model.alerts.some((a) => a[1] === "red");
}

const hudOffset = new THREE.Vector3();
function placeHud(headPos, headQuat, now) {
  updateHud(now);
  alertPanel.mesh.material.opacity = alertPanel.red ? 0.6 + 0.4 * Math.abs(Math.sin(now * 0.005)) : 1;
  const bottomTan = videoBottomTan();
  for (const p of hudPanels) {
    hudOffset.copy(p.offset);
    if (p.belowVideo) hudOffset.y = HUD_DIST * bottomTan - p.halfH - 0.01; // top edge just under the image
    else hudOffset.y += hudYOffsetM;
    headLock(p.mesh, hudOffset, headPos, headQuat);
    p.mesh.quaternion.multiply(p.turn);
  }
}

let lastTeleopState = null;

// ---------- HUD height ----------
// Left stick up/down moves the whole HUD up/down, only while teleop is OFF (or before
// the robot link is up): with teleop on the left stick drives the base. Remembered on
// this headset; ?hud_y=<m> in the URL sets it.
const HUD_Y_STORAGE_KEY = "vrteleop.hud_y_m";
const HUD_Y_LIMIT_M = 0.6;
const HUD_Y_RATE_M_S = 0.3; // at full deflection
let hudYOffsetM = 0;
let hudYSaveTimer = null;
{
  const fromUrl = parseFloat(new URLSearchParams(location.search).get("hud_y"));
  let stored = NaN;
  try {
    stored = parseFloat(localStorage.getItem(HUD_Y_STORAGE_KEY));
  } catch (e) {
    // storage unavailable: start centred
  }
  if (Number.isFinite(fromUrl)) hudYOffsetM = fromUrl;
  else if (Number.isFinite(stored)) hudYOffsetM = stored;
  hudYOffsetM = Math.min(HUD_Y_LIMIT_M, Math.max(-HUD_Y_LIMIT_M, hudYOffsetM));
}

function updateHudYFromStick(leftGamepad, dtS) {
  if (!leftGamepad) return;
  if (lastTeleopState && lastTeleopState.enabled) return; // teleop on: the stick is the base's
  const y = leftGamepad.axes.length >= 4 ? leftGamepad.axes[3] : 0;
  if (Math.abs(y) < 0.3) return;
  // stick forward (y < 0) -> HUD up
  const v = (Math.abs(y) - 0.3) / 0.7;
  hudYOffsetM = Math.min(HUD_Y_LIMIT_M, Math.max(-HUD_Y_LIMIT_M,
    hudYOffsetM - Math.sign(y) * v * HUD_Y_RATE_M_S * dtS));
  clearTimeout(hudYSaveTimer);
  hudYSaveTimer = setTimeout(() => {
    try {
      localStorage.setItem(HUD_Y_STORAGE_KEY, hudYOffsetM.toFixed(3));
    } catch (e) {
      // not persisted; still applies for this session
    }
  }, 500);
}

function handleTeleopState(msg) {
  const prevEnabled = lastTeleopState ? lastTeleopState.arm_enabled : { left: false, right: false };

  for (const hand of ["left", "right"]) {
    const isEnabled = !!msg.arm_enabled[hand];
    const wasEnabled = !!prevEnabled[hand];
    setIndicator(hand, isEnabled ? "on" : "off");
    if (controllers[hand]) controllers[hand].grip.visible = !isEnabled; // clutched: marker out of the way
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
        // Vertical frustum centre, averaged over both eyes. Column-major
        // projection: m[9] / m[5] = (top + bottom) / (2 * near).
        let tanY = 0;
        for (const v of viewerPose.views) tanY += v.projectionMatrix[9] / v.projectionMatrix[5];
        viewCenterTanY = tanY / viewerPose.views.length;
        if (mm > 30 && mm < 100) {
          measuredIpdMm = mm;
          if (ipdMm === null) setIpd(mm, false); // first run on this headset: start at its IPD
        }
      }
      placeEyePlanes(tmpPos, tmpQuat);

      // Head-lock the screen + status HUD to this frame's head pose.
      headLock(headScreen, HEAD_SCREEN_OFFSET, tmpPos, tmpQuat);
      placeHud(tmpPos, tmpQuat, timestamp);
    }

    let leftPayload = null;
    let rightPayload = null;
    for (const inputSource of session.inputSources) {
      if (!inputSource.gripSpace && !inputSource.targetRaySpace) continue;
      const space = inputSource.gripSpace || inputSource.targetRaySpace;
      const pose = frame.getPose(space, refSpace);
      if (!pose) continue;
      const payload = { ...poseFromXRPose(pose), ...gamepadAxes(inputSource) };
      if (inputSource.handedness === "left") {
        leftPayload = payload;
        updateHudYFromStick(inputSource.gamepad, dtS);
      }
      if (inputSource.handedness === "right") {
        rightPayload = payload;
        updateIpdFromStick(inputSource.gamepad, dtS);
        updateTiltFromStick(inputSource.gamepad, dtS);
      }
    }

    sendPose({ t: Date.now(), head: headPayload, left: leftPayload, right: rightPayload, view_pitch_deg: viewTiltDeg });
  }

  if (!frame) {
    // Not in VR (desktop preview): lock the screens + HUD to the page camera.
    headLock(headScreen, HEAD_SCREEN_OFFSET, camera.position, camera.quaternion);
    placeHud(camera.position, camera.quaternion, timestamp);
  }
  renderer.render(scene, camera);
});
