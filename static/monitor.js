// Read-only robot monitor (static/monitor.html). Polls /monitor.json 5x a second; never
// sends commands. Field meanings: docs/EPISODE_FORMAT.md (same names as the episode frame).
import { latestFrameDecoder } from "./latest_frame.mjs";

const POLL_MS = 200;
const STALE_S = 1.0; // robot state older than this is flagged
const SIDES = ["left", "right"];
const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;

const fmt = (v, digits = 2) => (v === null || v === undefined || Number.isNaN(v) ? "—" : Number(v).toFixed(digits));
const cell = (v, digits, cls = "") => `<td class="${v === null || v === undefined ? "na" : cls}">${fmt(v, digits)}</td>`;
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

function pill(el, text, cls = "") {
  el.textContent = text;
  el.className = `pill ${cls}`;
}

// ---------------------------------------------------------------- arms
function renderArm(side, m) {
  const st = m.link?.robot?.state || {};
  const arm = m.commands?.arms?.[side] || {};
  const clutched = arm.clutched;
  const ik = st[`${side}_ik_ok`];
  const q = st[`${side}_joints`];
  const qc = st[`${side}_joints_cmd`];
  const ee = st[`${side}_ee`];
  const tgt = arm.target;
  const g = st[`${side}_gripper`];
  const gc = st[`${side}_gripper_cmd`];
  const gerr = st[`${side}_gripper_error`];

  let joints = "";
  for (let i = 0; i < 7; i++) {
    const a = q ? q[i] * DEG : null;
    const c = qc ? qc[i] * DEG : null;
    joints += `<tr><td>J${i + 1}</td>${cell(a, 1)}${cell(c, 1, "cmd")}${cell(a !== null && c !== null ? c - a : null, 1)}</tr>`;
  }
  let eeRows = "";
  ["x", "y", "z"].forEach((axis, i) => {
    const a = ee ? ee[4 + i] * 100 : null;
    const t = tgt ? tgt[4 + i] * 100 : null;
    eeRows += `<tr><td>${axis}</td>${cell(a, 1)}${cell(t, 1, "cmd")}${cell(a !== null && t !== null ? t - a : null, 1)}</tr>`;
  });
  const eeNote = tgt ? "" : `<div class="sub">no target: ${esc(
    !m.commands ? "no teleop commands yet" : !m.commands.enabled ? "teleop off" : `${side} arm not clutched`)}</div>`;

  const fill = g === null || g === undefined ? 0 : g * 100;
  const mark = gc === null || gc === undefined ? "" : `<div class="mark" style="left:${gc * 100}%"></div>`;
  $(`arm-${side}`).innerHTML = `
    <h2>${side[0].toUpperCase() + side.slice(1)} arm
      <span class="pill ${clutched ? "ok" : ""}">${clutched ? "clutched" : "not clutched"}</span>
      <span class="pill ${ik === false ? "bad" : ik ? "ok" : ""}">IK ${ik === null || ik === undefined ? "—" : ik ? "ok" : "FAIL"}</span>
    </h2>
    <div class="sub" style="margin-top:0">Gripper (0 closed … 1 open)
      <span class="pill ${gc === null || gc === undefined ? "" : "ok"}" style="margin-left:6px">${gc === null || gc === undefined ? "limp" : "holding"}</span></div>
    <div class="bar"><div class="fill" style="width:${fill}%"></div>${mark}</div>
    <div class="legend"><span><b style="background:var(--info)"></b>measured ${fmt(g)}</span>
      <span><b style="background:var(--cmd)"></b>commanded ${gc === null || gc === undefined ? "— (limp)" : fmt(gc)}</span></div>
    ${gerr ? `<div class="err">${esc(gerr)}</div>` : ""}
    <div class="sub">Joints (deg)</div>
    <table><tr><th></th><th>measured</th><th>commanded</th><th>cmd − meas</th></tr>${joints}</table>
    <div class="sub">End effector (cm, robot base frame)</div>
    <table><tr><th></th><th>measured</th><th>VR target</th><th>tgt − meas</th></tr>${eeRows}</table>
    ${eeNote}`;
}

// ---------------------------------------------------------------- base, lift, neck
function renderBase(m) {
  const st = m.link?.robot?.state || {};
  const cmd = st.base_cmd_vel;
  const enc = st.enc_vel;
  const rows = [["vx (m/s)", 0], ["vy (m/s)", 1], ["wz (rad/s)", 2]]
    .map(([n, i]) => `<tr><td>${n}</td>${cell(cmd ? cmd[i] : null, 3, "cmd")}${cell(enc ? enc[i] : null, 3)}</tr>`).join("");
  $("base").innerHTML = `<tr><th></th><th>commanded</th><th>encoders</th></tr>${rows}`;
  const on = m.commands?.base_mode;
  pill($("base-mode"), on ? "base mode ON" : "base mode off", on ? "ok" : "");
}

function renderLiftNeck(m) {
  const st = m.link?.robot?.state || {};
  const h = m.link?.robot?.lift_height;
  const lc = st.lift_cmd;
  const manual = m.commands?.lift_manual;
  const lcText = lc === 1 ? "↑ up" : lc === -1 ? "↓ down" : lc === 0 ? "stop" : "—";
  $("lift").innerHTML = `<span>Lift height</span><span>${h === null || h === undefined ? "— (no reading)" : `${fmt(h, 3)} m`}</span>
    <span>Lift command</span><span style="color:var(--cmd)">${lcText}${manual ? ` (VR: ${esc(manual)})` : ""}</span>`;
  const max = 0.66;
  $("lift-bar").firstElementChild.style.width = `${h ? Math.min(100, (h / max) * 100) : 0}%`;
  const n = st.neck_deg;
  const nc = st.neck_cmd_deg;
  $("neck").innerHTML = `<tr><th>Neck (deg)</th><th>measured</th><th>commanded</th></tr>
    <tr><td>pan</td>${cell(n ? n[0] : null, 1)}${cell(nc ? nc[0] : null, 1, "cmd")}</tr>
    <tr><td>tilt</td>${cell(n ? n[1] : null, 1)}${cell(nc ? nc[1] : null, 1, "cmd")}</tr>`;
}

function renderEpisode(m) {
  const ep = m.episode;
  if (!ep) {
    $("episode").innerHTML = "<span>Publishing</span><span>off (--no-episodes or port busy)</span>";
    $("episode-errors").innerHTML = "";
    return;
  }
  const kv = [
    ["State", ep.active ? `<b style="color:var(--bad)">RECORDING</b>` : "idle"],
    ["Episode", esc(ep.episode || ep.last_id || "—") + (ep.active ? "" : ep.last_id ? " (last)" : "")],
    ["Task", esc(ep.task || ep.meta?.task || "— (no --task)")],
  ];
  if (ep.active) kv.push(["Elapsed", `${fmt(ep.elapsed_s, 1)} s`], ["Frames", `${ep.frames} @ ${ep.frame_hz ?? "—"} Hz`]);
  if (ep.frames_late !== undefined) kv.push(["Late / failed reads", `${ep.frames_late} / ${ep.frames_failed}`]);
  $("episode").innerHTML = kv.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  const errs = ep.errors || [];
  $("episode-errors").innerHTML = errs.length
    ? `<div class="sub">Signal errors this episode</div>` + errs.map((e) =>
      `<div class="err">#${e.seq} ${esc(e.field)}: ${e.error === null ? "recovered" : esc(e.error)}</div>`).join("")
    : "";
}

// ---------------------------------------------------------------- header + alerts
function renderHeader(m, ageS) {
  pill($("p-feed"), `monitor ${ageS < 1 ? "live" : `stale ${fmt(ageS, 1)} s`}`, ageS < 1 ? "ok" : "bad");
  const t = m.teleop;
  pill($("p-teleop"), `teleop ${t?.state ?? "—"}`, t?.state === "HOMING" ? "warn" : t?.enabled ? "ok" : "");
  const ep = m.episode;
  if (ep?.active) pill($("p-rec"), `● REC ${fmt(ep.elapsed_s, 0)} s`, "rec");
  else pill($("p-rec"), "no episode");
  if (m.dry_run) pill($("p-link"), "DRY RUN (no robot)", "warn");
  else if (!m.link?.rpc_ok) pill($("p-link"), "robot link DOWN", "bad");
  else pill($("p-link"), m.link.homing ? "robot HOMING" : "robot link ok", m.link.homing ? "warn" : "ok");
  const hs = m.headset;
  if (!hs || hs.clients === 0) pill($("p-headset"), "headset not connected");
  else pill($("p-headset"), `headset ${hs.rx_hz} Hz${hs.fresh ? "" : " STALE"}`, hs.fresh ? "ok" : "warn");
}

function renderAlerts(m) {
  const out = [];
  const st = m.link?.robot?.state || {};
  if (m.link?.error) out.push(`robot link: ${m.link.error}`);
  if (m.link?.robot && m.time - m.link.robot.read_at > STALE_S)
    out.push(`robot state is ${fmt(m.time - m.link.robot.read_at, 1)} s old (yor.py slow or not answering)`);
  if (m.link && !m.link.robot) out.push("no robot state read yet");
  for (const side of SIDES) {
    const e = st[`${side}_gripper_error`];
    if (e && m.link?.gripper_sides?.includes(side)) out.push(`${side} gripper: ${e}`);
  }
  for (const w of m.teleop?.warnings || []) out.push(`teleop: ${w}`);
  const el = $("alerts");
  el.style.display = out.length ? "block" : "none";
  el.innerHTML = out.map((s) => `<div>${esc(s)}</div>`).join("");
}

// ---------------------------------------------------------------- poll loop
let lastOk = 0;
let last = null;

async function poll() {
  try {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), 1500);
    const r = await fetch("/monitor.json", { cache: "no-store", signal: ctrl.signal });
    clearTimeout(timer);
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    last = await r.json();
    lastOk = performance.now();
    renderAll(last);
  } catch (e) {
    const ageS = (performance.now() - lastOk) / 1000;
    pill($("p-feed"), `monitor feed lost (${lastOk ? `${fmt(ageS, 0)} s` : "never connected"}) — is the bridge running?`, "bad");
  }
  setTimeout(poll, POLL_MS);
}

function renderAll(m) {
  renderHeader(m, m.loop_age_s ?? 99);
  renderAlerts(m);
  for (const side of SIDES) renderArm(side, m);
  renderBase(m);
  renderLiftNeck(m);
  renderEpisode(m);
}

poll();

// ---------------------------------------------------------------- optional camera
const canvas = $("cam-canvas");
const ctx = canvas.getContext("2d");
let videoHost = null;
let ws = null;
let camOn = false;
let drawn = 0;

try {
  videoHost = (await (await fetch("/config.json")).json()).video || null;
} catch (e) {
  videoHost = null;
}
if (videoHost) $("cam-cert").href = `https://${videoHost}/`;
else {
  $("cam-toggle").disabled = true;
  pill($("cam-state"), "no video server configured");
}

const decoder = latestFrameDecoder(
  (bytes) => createImageBitmap(new Blob([bytes], { type: "image/jpeg" })),
  (bitmap) => {
    const stereo = Number($("cam-stream").value) === 3;
    const w = stereo ? bitmap.width / 2 : bitmap.width; // stereo is left|right side by side
    if (canvas.width !== w || canvas.height !== bitmap.height) {
      canvas.width = w;
      canvas.height = bitmap.height;
    }
    ctx.drawImage(bitmap, 0, 0, w, bitmap.height, 0, 0, w, bitmap.height);
    drawn++;
  },
);

function connectVideo() {
  if (!camOn) return;
  pill($("cam-state"), "connecting…", "warn");
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${videoHost}/ws`);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => pill($("cam-state"), "connected", "ok");
  ws.onmessage = (ev) => {
    if (!(ev.data instanceof ArrayBuffer)) return; // stereo_info etc.
    const bytes = new Uint8Array(ev.data);
    if (bytes.length > 1 && bytes[0] === Number($("cam-stream").value)) decoder.push(bytes.subarray(1));
  };
  ws.onclose = () => {
    decoder.reset();
    ws = null;
    if (camOn) {
      pill($("cam-state"), "disconnected, retrying (cert accepted?)", "bad");
      setTimeout(connectVideo, 2000);
    }
  };
}

$("cam-toggle").onclick = () => {
  camOn = !camOn;
  $("cam-toggle").textContent = camOn ? "Hide camera" : "Show camera";
  canvas.style.display = camOn ? "block" : "none";
  if (camOn) connectVideo();
  else {
    if (ws) ws.close();
    decoder.reset();
    pill($("cam-state"), "off");
  }
};
$("cam-stream").onchange = () => decoder.reset();

setInterval(() => {
  if (camOn && ws?.readyState === WebSocket.OPEN) pill($("cam-state"), `connected · ${drawn} fps`, drawn ? "ok" : "warn");
  drawn = 0;
}, 1000);
