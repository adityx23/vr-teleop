import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

const statusEl = document.getElementById("status");

// ---------- WebSocket: camera frames in, pose data out ----------
const wsProto = location.protocol === "https:" ? "wss" : "ws";
const ws = new WebSocket(`${wsProto}://${location.host}/ws`);
ws.binaryType = "arraybuffer";

let framesReceived = 0;
let lastFrameLogTime = 0;

ws.addEventListener("open", () => setStatus("connected, waiting for camera frames..."));
ws.addEventListener("close", () => setStatus("disconnected"));
ws.addEventListener("error", () => setStatus("websocket error"));

function setStatus(msg) {
  statusEl.textContent = msg;
}

function sendPose(obj) {
  if (ws.readyState === WebSocket.OPEN) {
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
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  streamCanvases[id] = { canvas, ctx: canvas.getContext("2d") };
  streamTextures[id] = texture;
}

ws.addEventListener("message", async (ev) => {
  if (typeof ev.data === "string") {
    try {
      const msg = JSON.parse(ev.data);
      if (msg.type === "teleop_state") handleTeleopState(msg);
    } catch (e) {
      // ignore malformed status message
    }
    return;
  }
  if (!(ev.data instanceof ArrayBuffer)) return;
  const bytes = new Uint8Array(ev.data);
  if (bytes.length < 2) return;
  const streamId = bytes[0];
  const entry = streamCanvases[streamId];
  if (!entry) return;

  framesReceived++;
  const blob = new Blob([bytes.subarray(1)], { type: "image/jpeg" });
  try {
    const bitmap = await createImageBitmap(blob);
    entry.ctx.drawImage(bitmap, 0, 0, entry.canvas.width, entry.canvas.height);
    streamTextures[streamId].needsUpdate = true;
    bitmap.close();
    const now = performance.now();
    if (now - lastFrameLogTime > 1000) {
      setStatus(`streaming (${framesReceived} frames received)`);
      lastFrameLogTime = now;
    }
  } catch (e) {
    // ignore malformed frame
  }
});

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

function headLock(mesh, offset, headPos, headQuat) {
  mesh.quaternion.copy(headQuat);
  mesh.position.copy(offset).applyQuaternion(headQuat).add(headPos);
}

// Simple floor grid for spatial reference
scene.add(new THREE.GridHelper(6, 12, 0x3a4a5a, 0x222a33));

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

  const l = msg.arm_enabled.left ? "ON" : "off";
  const r = msg.arm_enabled.right ? "ON" : "off";
  const color = msg.warnings && msg.warnings.length ? "#ff5555" : "#eeeeee";
  drawHud(`${msg.state}  L:${l}  R:${r}`, color);

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

renderer.setAnimationLoop((timestamp, frame) => {
  if (frame) {
    const refSpace = renderer.xr.getReferenceSpace();
    const session = renderer.xr.getSession();

    let headPayload = null;
    const viewerPose = frame.getViewerPose(refSpace);
    if (viewerPose) {
      camera.matrixWorld.fromArray(viewerPose.views[0].transform.matrix);
      camera.matrixWorld.decompose(tmpPos, tmpQuat, new THREE.Vector3());
      headPayload = { pos: [tmpPos.x, tmpPos.y, tmpPos.z], quat: [tmpQuat.x, tmpQuat.y, tmpQuat.z, tmpQuat.w] };

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
      if (inputSource.handedness === "right") rightPayload = payload;
    }

    sendPose({ t: Date.now(), head: headPayload, left: leftPayload, right: rightPayload });
  }

  renderer.render(scene, camera);
});
