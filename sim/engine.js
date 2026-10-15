/* ORBIT-EYE simulation engine
 * One engine, used by both the Node validation suite and the browser simulator.
 * Units: mm, deg, s, N (linear), N·m (rotary). Joint order: X, Y, Z, Yaw, Pitch.
 */
'use strict';
(function (root) {
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const nowMs = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
function rngOf(seed) { let a = seed >>> 0; return function () { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function gauss(r) { let u = r(); if (u < 1e-12) u = 1e-12; return Math.sqrt(-2 * Math.log(u)) * Math.cos(6.283185307179586 * r()); }
const clamp = (x, a, b) => (x < a ? a : x > b ? b : x);
const sgn = x => (x > 0 ? 1 : x < 0 ? -1 : 0);

/* ---------------- Geometry ---------------- */
const G = {
  Y0: 140, L: 90, WD: 22,                       // pivot Y home offset, pivot->front lens, working distance
  spheres: [[-30, 22], [0, 22], [30, 18], [60, 15], [90, 12]], // [distance along optical axis, radius]
  faceC: [0, -70, 0], faceR: [72, 80, 112],     // head ellipsoid (patient frame)
  noseA: [0, 8, -2], noseB: [0, 28, -26], noseR: 9,
  eyes: { R: [32, 8, 0], L: [-32, 8, 0] },      // corneal apex (patient right eye = +X seen from device)
  margin: 10, band: 30
};
const LO = [-150, -100, -75, -30, -30], HI = [150, 100, 75, 30, 30];
const W = [1, 1, 1, 2, 2];                       // C-space metric: 1 deg ~ 2 mm
const VAX = [150, 150, 150, 60, 60];             // per-axis velocity limits
const PARK = [0, 90, 20, 0, 0];
const NAMES = ['X', 'Y', 'Z', 'Yaw', 'Pitch'];

function dirOf(q) { const ps = q[3] * D2R, th = q[4] * D2R, ct = Math.cos(th); return [ct * Math.sin(ps), -ct * Math.cos(ps), -Math.sin(th)]; }
function pivotOf(q) { return [q[0], q[1] + G.Y0, q[2]]; }
function tipOf(q) { const d = dirOf(q); return [q[0] + G.L * d[0], q[1] + G.Y0 + G.L * d[1], q[2] + G.L * d[2]]; }
function eyePos(side, f) { const e = G.eyes[side]; return [e[0] + f[0], e[1] + f[1], e[2] + f[2]]; }
function ikEye(eye) { return [eye[0], eye[1] + G.WD + G.L - G.Y0, eye[2], 0, 0]; }

function sdEll(px, py, pz, cx, cy, cz, r) {
  const x = (px - cx) / r[0], y = (py - cy) / r[1], z = (pz - cz) / r[2];
  const k0 = Math.sqrt(x * x + y * y + z * z);
  const k1 = Math.sqrt(x * x / (r[0] * r[0]) + y * y / (r[1] * r[1]) + z * z / (r[2] * r[2]));
  return k1 < 1e-12 ? -Math.min(r[0], r[1], r[2]) : k0 * (k0 - 1) / k1;
}
function sdCap(px, py, pz, ax, ay, az, bx, by, bz, r) {
  const bax = bx - ax, bay = by - ay, baz = bz - az, pax = px - ax, pay = py - ay, paz = pz - az;
  const h = clamp((pax * bax + pay * bay + paz * baz) / (bax * bax + bay * bay + baz * baz), 0, 1);
  const dx = pax - bax * h, dy = pay - bay * h, dz = paz - baz * h;
  return Math.sqrt(dx * dx + dy * dy + dz * dz) - r;
}
/* surface-to-surface clearance between imaging head and face (mm); negative = contact */
function clearance(q, f) {
  const d = dirOf(q), gx = q[0], gy = q[1] + G.Y0, gz = q[2];
  const cx = G.faceC[0] + f[0], cy = G.faceC[1] + f[1], cz = G.faceC[2] + f[2];
  const A = G.noseA, B = G.noseB;
  let m = 1e9;
  for (let i = 0; i < G.spheres.length; i++) {
    const t = G.spheres[i][0], r = G.spheres[i][1];
    const px = gx + t * d[0], py = gy + t * d[1], pz = gz + t * d[2];
    const s = Math.min(sdEll(px, py, pz, cx, cy, cz, G.faceR),
      sdCap(px, py, pz, A[0] + f[0], A[1] + f[1], A[2] + f[2], B[0] + f[0], B[1] + f[1], B[2] + f[2], G.noseR)) - r;
    if (s < m) m = s;
  }
  return m;
}
function clrGrad(q, f, mask) { // weighted gradient of clearance in C-space
  const g = [0, 0, 0, 0, 0], c0 = clearance(q, f);
  for (let i = 0; i < 5; i++) { if (mask && !mask[i]) continue; const h = 0.2 / W[i]; const qq = q.slice(); qq[i] += h; g[i] = (clearance(qq, f) - c0) / h; }
  return g;
}

/* ---------------- Planner: RRT* + shortcut + clamped cubic B-spline ---------------- */
function inLim(q) { for (let i = 0; i < 5; i++) if (q[i] < LO[i] - 1e-9 || q[i] > HI[i] + 1e-9) return false; return true; }
function wdist(a, b) { let s = 0; for (let i = 0; i < 5; i++) { const d = (a[i] - b[i]) * W[i]; s += d * d; } return Math.sqrt(s); }
function lerpQ(a, b, t) { const o = new Array(5); for (let i = 0; i < 5; i++) o[i] = a[i] + (b[i] - a[i]) * t; return o; }
function edgeOK(a, b, f, thr) { return edgeCost(a, b, f, thr) >= 0; }
/* edge cost = length weighted by proximity to the face, so the planner prefers wide arcs it can fly fast */
function edgeCost(a, b, f, thr) {
  const L = wdist(a, b), n = Math.max(1, Math.ceil(L / 2.5)); let pen = 0;
  for (let k = 1; k <= n; k++) { const q = lerpQ(a, b, k / n); if (!inLim(q)) return -1; const c = clearance(q, f); if (c < thr) return -1; pen += clamp((G.margin + G.band - c) / G.band, 0, 1); }
  return L * (1 + 3 * pen / n);
}

function bspline(ctrl, perSeg) {
  const P = [ctrl[0], ctrl[0], ...ctrl, ctrl[ctrl.length - 1], ctrl[ctrl.length - 1]];
  const out = [];
  for (let i = 0; i + 3 < P.length; i++) {
    for (let k = 0; k < perSeg; k++) {
      const u = k / perSeg, u2 = u * u, u3 = u2 * u;
      const b0 = (1 - u) ** 3 / 6, b1 = (3 * u3 - 6 * u2 + 4) / 6, b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6, b3 = u3 / 6;
      const q = new Array(5);
      for (let j = 0; j < 5; j++) q[j] = b0 * P[i][j] + b1 * P[i + 1][j] + b2 * P[i + 2][j] + b3 * P[i + 3][j];
      out.push(q);
    }
  }
  out.push(ctrl[ctrl.length - 1].slice());
  return out;
}
function densify(wps, h) { const out = [wps[0].slice()]; for (let i = 1; i < wps.length; i++) { const n = Math.max(1, Math.ceil(wdist(wps[i - 1], wps[i]) / h)); for (let k = 1; k <= n; k++) out.push(lerpQ(wps[i - 1], wps[i], k / n)); } return out; }
function resampleArc(pts, ds) {
  const cum = [0]; for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + wdist(pts[i - 1], pts[i]));
  const Ltot = cum[cum.length - 1], n = Math.max(2, Math.ceil(Ltot / ds) + 1), out = [];
  let j = 0;
  for (let k = 0; k < n; k++) { const s = Ltot * k / (n - 1); while (j < pts.length - 2 && cum[j + 1] < s) j++; const seg = cum[j + 1] - cum[j]; out.push(lerpQ(pts[j], pts[j + 1], seg > 1e-12 ? (s - cum[j]) / seg : 0)); }
  return { pts: out, ds: Ltot / (n - 1), L: Ltot };
}
function smoothPath(wps, f, thr) {
  if (wps.length === 2) return resampleArc(densify(wps, 2), 0.5);
  for (const h of [25, 12, 6, 3]) {
    const pts = bspline(densify(wps, h), 48);
    let ok = true; for (const q of pts) if (!inLim(q) || clearance(q, f) < thr) { ok = false; break; }
    if (ok) return resampleArc(pts, 0.5);
  }
  return resampleArc(densify(wps, 1), 0.5); // polyline fallback (corners -> curvature limit stops there)
}
function planPath(qs, qg, f, opt = {}) {
  const t0 = nowMs(), rng = rngOf(opt.seed || 1), margin = opt.margin ?? G.margin;
  const cs = clearance(qs, f), thr = Math.min(margin, cs - 0.3);
  if (thr < 3) return { ok: false, reason: 'start pose already inside keep-out', ms: nowMs() - t0 };
  if (!inLim(qg) || clearance(qg, f) < margin) return { ok: false, reason: 'goal pose violates keep-out or limits', ms: nowMs() - t0 };
  let wps, iters = 0, method = 'direct';
  if (edgeOK(qs, qg, f, thr)) wps = [qs.slice(), qg.slice()];
  else {
    method = 'RRT*';
    const N = opt.iters || 3000, step = 20, R = 45, nodes = [qs.slice()], par = [-1], cost = [0];
    let goalIdx = -1, extra = 0;
    const ylo = Math.max(LO[1], -60);
    for (let it = 0; it < N; it++) {
      iters = it + 1;
      let x;
      if (rng() < 0.12) x = qg;
      else { x = new Array(5); for (let i = 0; i < 5; i++) { const lo = i === 1 ? ylo : LO[i]; x[i] = lo + rng() * (HI[i] - lo); } }
      let ni = 0, nd = 1e18; for (let i = 0; i < nodes.length; i++) { const d = wdist(nodes[i], x); if (d < nd) { nd = d; ni = i; } }
      if (nd < 1e-9) continue;
      const q = lerpQ(nodes[ni], x, Math.min(1, step / nd));
      if (!inLim(q) || clearance(q, f) < thr) continue;
      const nb = []; for (let i = 0; i < nodes.length; i++) if (wdist(nodes[i], q) < R) nb.push(i);
      let bp = -1, bc = 1e18;
      for (const i of nb) { if (cost[i] + wdist(nodes[i], q) >= bc) continue; const ec = edgeCost(nodes[i], q, f, thr); if (ec >= 0 && cost[i] + ec < bc) { bc = cost[i] + ec; bp = i; } }
      if (bp < 0) { const ec = edgeCost(nodes[ni], q, f, thr); if (ec < 0) continue; bp = ni; bc = cost[ni] + ec; }
      const id = nodes.length; nodes.push(q); par.push(bp); cost.push(bc);
      for (const i of nb) { if (bc + wdist(q, nodes[i]) >= cost[i]) continue; const ec = edgeCost(q, nodes[i], f, thr); if (ec >= 0 && bc + ec < cost[i]) { par[i] = id; cost[i] = bc + ec; } }
      const dg = wdist(q, qg);
      const eg = dg < step * 1.5 ? edgeCost(q, qg, f, thr) : -1;
      if (eg >= 0) {
        const cg = bc + eg;
        if (goalIdx < 0) { goalIdx = nodes.length; nodes.push(qg.slice()); par.push(id); cost.push(cg); }
        else if (cg < cost[goalIdx]) { par[goalIdx] = id; cost[goalIdx] = cg; }
      }
      if (goalIdx >= 0 && ++extra > 500) break;
    }
    if (goalIdx < 0) return { ok: false, reason: 'planner found no collision-free path', ms: nowMs() - t0, iters };
    const p = []; let k = goalIdx, guard = 0; while (k >= 0 && guard++ < 10000) { p.push(nodes[k]); k = par[k]; } p.reverse();
    const pc = (i, j) => { let c = 0; for (let k = i; k < j; k++) c += edgeCost(p[k], p[k + 1], f, thr); return c; };
    for (let s = 0; s < 300 && p.length > 2; s++) { const i = Math.floor(rng() * (p.length - 2)), j = i + 2 + Math.floor(rng() * (p.length - i - 2)); if (j >= p.length) continue; const ec = edgeCost(p[i], p[j], f, thr); if (ec >= 0 && ec <= pc(i, j) * 1.02) p.splice(i + 1, j - i - 1); }
    wps = p;
  }
  const path = smoothPath(wps, f, thr);
  return { ok: true, wps, path, method, iters, thr, ms: nowMs() - t0 };
}

/* ---------------- Time parameterisation: SSM speed limit -> accel-limited profile -> FIR S-curve -> ZVD ---------------- */
function zvdImpulses(fHz, z) { const K = Math.exp(-z * Math.PI / Math.sqrt(1 - z * z)), Td = 1 / (fHz * Math.sqrt(1 - z * z)), n = (1 + K) ** 2; return [[0, 1 / n], [Td / 2, 2 * K / n], [Td, K * K / n]]; }
function buildTraj(path, f, opt) {
  const { vmax = 150, amax = 1500, alat = 900, Tj = 0.025, zvd = true, fZvd = 45, zZvd = 0.02, margin = G.margin, band = G.band, dt = 0.001 } = opt;
  const P = path.pts, ds = path.ds, n = P.length;
  const vl = new Float64Array(n), clr = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    clr[i] = clearance(P[i], f);
    let v = vmax * clamp((clr[i] - margin) / band, 0.2, 1);              // speed & separation monitoring
    const k = 4, a = Math.max(0, i - k), b = Math.min(n - 1, i + k);
    if (b - a >= 2) { const m = (a + b) >> 1; let s2 = 0; for (let j = 0; j < 5; j++) { const dd = (P[b][j] - 2 * P[m][j] + P[a][j]) * W[j] / (((b - a) / 2 * ds) ** 2); s2 += dd * dd; } const kap = Math.sqrt(s2); if (kap > 1e-9) v = Math.min(v, Math.sqrt(alat / kap)); }
    const i0 = Math.max(0, i - 1), i1 = Math.min(n - 1, i + 1);
    for (let j = 0; j < 5; j++) { const dq = Math.abs(P[i1][j] - P[i0][j]) / ((i1 - i0) * ds || 1); if (dq > 1e-9) v = Math.min(v, VAX[j] / dq); }
    vl[i] = v;
  }
  vl[0] = 0; vl[n - 1] = 0;
  for (let i = 1; i < n; i++) vl[i] = Math.min(vl[i], Math.sqrt(vl[i - 1] ** 2 + 2 * amax * ds));
  for (let i = n - 2; i >= 0; i--) vl[i] = Math.min(vl[i], Math.sqrt(vl[i + 1] ** 2 + 2 * amax * ds));
  const tg = new Float64Array(n); for (let i = 1; i < n; i++) { const vv = vl[i] + vl[i - 1]; tg[i] = tg[i - 1] + (vv > 1e-9 ? 2 * ds / vv : Math.sqrt(2 * ds / amax)); }
  const T = tg[n - 1], N0 = Math.ceil(T / dt) + 1, s0 = new Float64Array(N0); let j = 0;
  for (let k = 0; k < N0; k++) { const t = Math.min(T, k * dt); while (j < n - 2 && tg[j + 1] < t) j++; const h = tg[j + 1] - tg[j]; s0[k] = ds * (j + (h > 0 ? (t - tg[j]) / h : 0)); }
  const Lp = ds * (n - 1);
  const nJ = Math.max(1, Math.round(Tj / dt)), imp = zvd ? zvdImpulses(fZvd, zZvd) : [[0, 1]];
  const tail = nJ + Math.ceil(imp[imp.length - 1][0] / dt) + 2, N = N0 + tail;
  const at = (arr, k) => (k < 0 ? arr[0] : k >= arr.length ? arr[arr.length - 1] : arr[k]);
  const s1 = new Float64Array(N); let acc = 0; for (let k = 0; k < N; k++) { acc += at(s0, k) - at(s0, k - nJ); s1[k] = acc / nJ; }   // box FIR => jerk-limited S-curve
  for (let k = 0; k < nJ; k++) { } // (running sum starts at s0[0]=0)
  const s2 = new Float64Array(N);
  for (let k = 0; k < N; k++) { let v = 0; for (const [td, A] of imp) { const x = k - td / dt, i0 = Math.floor(x), fr = x - i0; v += A * (at(s1, i0) * (1 - fr) + at(s1, i0 + 1) * fr); } s2[k] = Math.min(Lp, v); }
  const q = [0, 1, 2, 3, 4].map(() => new Float64Array(N)), v = q.map(() => new Float64Array(N)), a = q.map(() => new Float64Array(N));
  for (let k = 0; k < N; k++) { const x = s2[k] / ds, i0 = Math.min(n - 2, Math.floor(x)), fr = x - i0; for (let jj = 0; jj < 5; jj++) q[jj][k] = P[i0][jj] + (P[i0 + 1][jj] - P[i0][jj]) * fr; }
  // 9 ms centred smoothing of each joint (same kernel on every axis -> stays on the path to within µm)
  for (let jj = 0; jj < 5; jj++) for (let pass = 0; pass < 2; pass++) { const src = Float64Array.from(q[jj]); for (let k = 0; k < N; k++) { let acc2 = 0; for (let d = -4; d <= 4; d++) acc2 += at(src, k + d); q[jj][k] = acc2 / 9; } }
  for (let jj = 0; jj < 5; jj++) for (let k = 0; k < N; k++) { const km = Math.max(0, k - 1), kp = Math.min(N - 1, k + 1); v[jj][k] = (q[jj][kp] - q[jj][km]) / ((kp - km) * dt); a[jj][k] = (q[jj][kp] - 2 * q[jj][k] + q[jj][km]) / (dt * dt); }
  let minClr = 1e9; for (let i = 0; i < n; i++) minClr = Math.min(minClr, clr[i]);
  return { q, v, a, N, T: N * dt, Tplan: T, s: s2, L: Lp, minClr, vlim: vl, clr };
}

/* ---------------- Drivetrain / axis models ---------------- */
// m: load mass (kg | kg·m²), M: reflected motor+screw inertia, k: drivetrain stiffness (N/mm | N·m/deg)
const AXDEF = [
  { lin: 1, m: 9.0, M: 107, k: 15000, z: 0.05, Fs: 18, Fc: 15, fm: 8, Umax: 700 },
  { lin: 1, m: 5.5, M: 100, k: 15000, z: 0.05, Fs: 15, Fc: 12, fm: 8, Umax: 700 },
  { lin: 1, m: 3.0, M: 95, k: 15000, z: 0.05, Fs: 12, Fc: 10, fm: 8, Umax: 700 },
  { lin: 0, m: 0.004, M: 0.018, k: 13.96, z: 0.05, Fs: 0.03, Fc: 0.025, fm: 0.01, Umax: 4 },
  { lin: 0, m: 0.003, M: 0.018, k: 13.96, z: 0.05, Fs: 0.03, Fc: 0.025, fm: 0.01, Umax: 4 }
];
const ENC = {
  motorQ: [5 / 16384, 5 / 16384, 5 / 16384, 360 / 30 / 16384, 360 / 30 / 16384],   // AS5048A 14-bit on motor shaft
  load: {
    lin: { q: 2 / 4096, n: 0.00025 },                       // AS5311 magnetic linear scale on carriage
    MT6835: { q: 360 / 2097152, n: 0.0012 },                // 21-bit magnetic angle IC on output
    AMT223: { q: 360 / 16384, n: 0.0 }                      // 14-bit (as originally specified)
  }
};

class Sim {
  constructor(o = {}) {
    this.o = Object.assign({
      seed: 7, fHead: 45, zHead: 0.02, zvd: true, fZvd: 45, zZvd: 0.02, rotEnc: 'MT6835', nut: 'DFU1605',
      estopCat: 1, breathing: true, camNoise: 0.3, trackFace: true, autoRecover: true,
      vmax: 150, amax: 1500, Tj: 0.025, faceOffset: [0, 0, 0], slipCheck: true, log: true
    }, o);
    this.rng = rngOf(this.o.seed);
    this.ax = AXDEF.map((d, i) => {
      const a = Object.assign({}, d);
      a.s = a.lin ? 1000 : R2D;
      a.b = a.lin ? (this.o.nut === 'SFU1605' ? 0.020 : 0.003) : 0.004; // total backlash / lost motion
      a.mu = a.m * a.M / (a.m + a.M); a.w = Math.sqrt(a.s * a.k / a.mu); a.c = 2 * a.z * a.mu * a.w / a.s;
      const wv = 2 * Math.PI * (a.lin ? 100 : 35), wp = 2 * Math.PI * (a.lin ? 20 : 10), wi = 2 * Math.PI * (a.lin ? 3 : 2);
      a.Kv = (a.M + a.m) * wv / a.s; a.Kp = wp; a.Ki = wp * wi; a.Ilim = a.lin ? 0.02 : 0.02;
      a.qm = ENC.motorQ[i];
      const le = a.lin ? ENC.load.lin : ENC.load[this.o.rotEnc]; a.ql = le.q; a.nl = le.n;
      a.slipThr = a.b / 2 + (a.lin ? 0.008 : Math.max(0.008, 1.2 * a.ql)) + 4 * a.nl;
      a.astop = a.lin ? 5000 : 2500;
      return a;
    });
    const q0 = PARK.slice();
    const z5 = () => new Float64Array(5);
    Object.assign(this, {
      xm: Float64Array.from(q0), vm: z5(), xl: Float64Array.from(q0), vl: z5(), cpl: z5(),
      xh: Float64Array.from(q0), vh: z5(), integ: z5(), vref: z5(), u: z5(),
      mMeas: Float64Array.from(q0), lMeas: Float64Array.from(q0), refOff: z5(), slipCnt: z5(),
      qc: q0.slice(), vc: [0, 0, 0, 0, 0], ac: [0, 0, 0, 0, 0],
      flex: [0, 0, 0], flexV: [0, 0, 0], tb: null, tb1: null, tb2: null,
      t: 0, tick: 0, enabled: [1, 1, 1, 1, 1], power: 1, brake: 0, stoAt: null
    });
    this.fBase = this.o.faceOffset.slice(); this.fGoal = this.fBase.slice(); this.fTrue = this.fBase.slice();
    this.fe = [0, 0, 0]; this.feV = [0, 0, 0]; this.pe = null; this.peV = [0, 0, 0]; this.registered = false; this.afterRetract = null;
    this.state = 'INIT'; this.stateT = 0; this.msg = 'Power on — face not registered';
    this.events = []; this.inj = null; this.fault = null;
    this.traj = null; this.tau = 0; this.r = 1; this.rT = 1; this.replanPending = false; this.planFace = null;
    this.goal = null; this.goalQ = null; this.trackF = null;
    this.adm = { v: [0, 0, 0, 0, 0], F: [0, 0, 0, 0, 0], deadman: false, active: false };
    this.stopV0 = null; this.stopT = 0;
    this.m = { lastPlanMs: null, lastPlanMethod: '', slipLatencyMs: null, stopTimeMs: null, faultAt: null, injAt: null, minClr: 1e9, lastPathL: 0 };
    this.LOGN = 1200; this.logEvery = 5;
    this.lg = { i: 0, n: 0, t: new Float64Array(this.LOGN), err: new Float64Array(this.LOGN), clr: new Float64Array(this.LOGN), spd: new Float64Array(this.LOGN), res: [0, 1, 2, 3, 4].map(() => new Float64Array(this.LOGN)), thr: [0, 1, 2, 3, 4].map(() => new Float64Array(this.LOGN)) };
    this.tipTrue = tipOf(q0); this.tipCmd = tipOf(q0); this.clrTrue = clearance(q0, this.fTrue); this.clrEst = this.clrTrue;
    this.resid = z5();
    this.ev('INIT', 'System powered, homed to park pose');
  }
  ev(type, text) { this.events.push({ t: this.t, type, text }); if (this.events.length > 200) this.events.shift(); }
  setState(s, msg) { if (this.state !== s) { this.state = s; this.stateT = this.t; } if (msg) this.msg = msg; }

  /* ---------- operator commands ---------- */
  cmd(c, arg) {
    const S = this.state;
    const busyFault = ['FAULT_SLIP', 'FAULT_NOGO', 'ESTOP', 'RETRACT'].includes(S);
    switch (c) {
      case 'register': if (!busyFault) { this.setState('FACE_REGISTER', 'Depth camera registering face…'); this.regT = this.t; this.ev('REG', 'Face registration started'); } break;
      case 'goR': case 'goL': case 'park': {
        if (busyFault || S === 'MANUAL_JOG') { this.ev('WARN', 'Command ignored in ' + S); return false; }
        if (!this.registered && c !== 'park') { this.ev('WARN', 'Register face first'); return false; }
        this.goal = c === 'goR' ? 'R' : c === 'goL' ? 'L' : 'PARK';
        if (S === 'MOVE_ARC') { this.rT = 0; this.replanPending = true; this.ev('PLAN', 'New goal mid-move: easing to stop then replanning'); }
        else this.doPlan();
        return true;
      }
      case 'goto': if (busyFault || S === 'MANUAL_JOG') return false; this.goal = 'X'; this.customQ = arg.slice(); if (S === 'MOVE_ARC') { this.rT = 0; this.replanPending = true; } else this.doPlan(); return true;
      case 'estop': this.estop(); break;
      case 'reset': if (S === 'ESTOP') { this.power = 1; this.brake = 0; this.enabled = [1, 1, 1, 1, 1]; this.syncCmd(); this.ev('RESET', 'E-stop released, power restored → retract'); this.startRetract(); } break;
      case 'manual': if (!busyFault) { if (S === 'MOVE_ARC') { this.rT = 0; this.wantManual = true; this.ev('MAN', 'Hand-over requested: easing to stop'); } else this.enterManual(); } break;
      case 'auto': if (S === 'MANUAL_JOG') { this.adm.active = false; this.ev('MAN', 'Operator released → automatic control'); this.syncCmd(true); if (this.goal) this.doPlan(); else this.setState('READY', 'Ready'); } break;
      case 'deadman': this.adm.deadman = !!arg; break;
      case 'force': this.adm.F = arg.slice(); break;
      case 'inject': this.inject(arg.type, arg.axis, arg.ms || 250); break;
      case 'shiftFace': this.fGoal = [this.fGoal[0] + arg[0], this.fGoal[1] + arg[1], this.fGoal[2] + arg[2]]; this.ev('PAT', `Patient moved by (${arg.join(', ')}) mm`); break;
    }
    return true;
  }
  enterManual() { this.syncCmd(true); this.adm.v = [0, 0, 0, 0, 0]; this.adm.active = true; this.setState('MANUAL_JOG', 'Compliant hand-over: hold deadman and push the handle'); this.ev('MAN', 'Manual (admittance) mode'); }
  inject(type, axis, ms) { this.inj = { type, axis, until: this.t + ms / 1000, frozen: this.lMeas[axis] }; this.m.injAt = this.t; this.m.slipLatencyMs = null; this.ev('INJ', `Injected ${type} on ${NAMES[axis]} (${ms} ms)`); }
  estop() {
    if (this.state === 'ESTOP') return;
    this.ev('ESTOP', `E-stop pressed (Category ${this.o.estopCat})`); this.m.faultAt = this.t; this.m.stopTimeMs = null;
    this.adm.active = false; this.traj = null;
    this.setState('ESTOP', this.o.estopCat === 0 ? 'E-STOP: motor power cut instantly (Cat 0) — axes coasting' : 'E-STOP: controlled stop then power removal (Cat 1)');
    if (this.o.estopCat === 0) { this.power = 0; this.stoAt = this.t; } else this.beginStop();
  }
  syncCmd(fromLoad) { for (let i = 0; i < 5; i++) { if (fromLoad) this.qc[i] = this.lMeas[i]; this.vc[i] = 0; this.ac[i] = 0; this.integ[i] = 0; } }
  goalConfig() { if (this.goal === 'PARK') return PARK.slice(); if (this.goal === 'X') return this.customQ.slice(); return ikEye(eyePos(this.goal, this.fe)); }
  doPlan() {
    this.setState('PLAN', 'Planning collision-free path…');
    const qg = this.goalConfig(); const qs = this.qc.slice();
    if (wdist(qs, qg) < 0.05) { this.goalQ = qg; this.traj = null; if (this.goal === 'PARK' || this.goal === 'X') this.setState('READY', 'Already at goal'); else { this.planFace = this.fe.slice(); this.trackF = qs.slice(); this.trackV = [0, 0, 0, 0, 0]; this.setState('FINE_ALIGN', 'Fine alignment'); } return true; }
    const pr = planPath(qs, qg, this.fe, { seed: (this.rng() * 1e9) | 0 });
    this.m.lastPlanMs = pr.ms; this.m.lastPlanMethod = pr.method || '';
    if (!pr.ok) { this.ev('PLAN', 'Plan failed: ' + pr.reason); this.setState('READY', 'Plan failed: ' + pr.reason); this.traj = null; return false; }
    this.traj = buildTraj(pr.path, this.fe, { vmax: this.o.vmax, amax: this.o.amax, Tj: this.o.Tj, zvd: this.o.zvd, fZvd: this.o.fZvd, zZvd: this.o.zZvd });
    this.traj.wps = pr.wps; this.traj.pts = pr.path.pts;
    this.planFace = this.fe.slice(); this.goalQ = qg; this.cmdMode = 'traj'; this.tau = 0; this.r = 1; this.rT = 1; this.replanPending = false; this.wantManual = false;
    this.m.lastPathL = pr.path.L;
    this.ev('PLAN', `${pr.method} path ${pr.path.L.toFixed(0)} mm-eq, ${this.traj.T.toFixed(2)} s, min clearance ${this.traj.minClr.toFixed(1)} mm, planned in ${pr.ms.toFixed(1)} ms`);
    this.setState('MOVE_ARC', this.goal === 'PARK' ? 'Moving to park' : this.goal === 'X' ? 'Moving to commanded pose' : `Swinging to ${this.goal === 'R' ? 'right' : 'left'} eye`);
    return true;
  }
  beginStop() { this.traj = null; this.stopV0 = this.vc.slice(); let Ts = 0; for (let i = 0; i < 5; i++) Ts = Math.max(Ts, Math.abs(this.vc[i]) / this.ax[i].astop); this.stopT = 0; this.stopTs = Math.max(Ts, 0.002); this.cmdMode = 'stop'; }
  startRetract(after) {
    this.afterRetract = after || null;
    const mask = this.enabled.map(e => !!e);
    const q = this.lMeas.slice ? Array.from(this.lMeas) : this.lMeas.slice();
    const g = clrGrad(q, this.fe, mask); let n = 0; for (let i = 0; i < 5; i++) n += (g[i] * W[i]) ** 2; n = Math.sqrt(n);
    let d; if (n < 1e-6) { d = [0, mask[1] ? 1 : 0, 0, 0, 0]; } else d = g.map((x, i) => x * W[i] / n / W[i]);
    let qr = q.map((x, i) => clamp(x + 20 * d[i], LO[i], HI[i]));
    this.syncCmd(true);
    const path = resampleArc(densify([this.qc.slice(), qr], 1), 0.5);
    this.traj = buildTraj(path, this.fe, { vmax: 60, amax: 800, Tj: this.o.Tj, zvd: this.o.zvd, fZvd: this.o.fZvd, zZvd: this.o.zZvd, margin: -100 });
    this.tau = 0; this.r = 1; this.rT = 1; this.cmdMode = 'traj';
    this.setState('RETRACT', after === 'plan' ? 'Reflex retreat — patient moved toward the head' : 'Retracting along safe vector (away from face)'); this.ev('RETRACT', `Safe retract ${NAMES.filter((_, i) => Math.abs(d[i]) > 0.15).join('+') || 'Y'}`);
  }
  setFault(kind, axis, text) {
    if (this.fault) return;
    this.fault = { kind, axis, t: this.t }; this.m.faultAt = this.t; this.m.stopTimeMs = null;
    if (this.inj && this.m.injAt != null) this.m.slipLatencyMs = (this.t - this.m.injAt) * 1000;
    if (axis != null) this.enabled[axis] = 0;
    this.adm.active = false;
    this.setState(kind === 'SLIP' ? 'FAULT_SLIP' : 'FAULT_NOGO', text);
    this.ev('FAULT', text + (this.m.slipLatencyMs != null && kind === 'SLIP' ? ` — detected ${this.m.slipLatencyMs.toFixed(2)} ms after onset` : ''));
    this.beginStop();
  }

  /* ---------- 1 kHz supervisory tick + 20 kHz physics ---------- */
  stepMs() {
    const dtc = 0.001, S = this.state;
    // patient motion: lean ramps at 50 mm/s; breathing
    for (let i = 0; i < 3; i++) { const d = this.fGoal[i] - this.fBase[i]; this.fBase[i] += clamp(d, -0.05, 0.05); }
    const br = this.o.breathing ? [0, 0.8 * Math.sin(2 * Math.PI * 0.25 * this.t), 0.4 * Math.sin(2 * Math.PI * 0.25 * this.t + 1)] : [0, 0, 0];
    for (let i = 0; i < 3; i++) this.fTrue[i] = this.fBase[i] + br[i];
    // depth camera @30 Hz (registration + tracking)
    if (this.tick % 33 === 0) { // alpha-beta filter: position + velocity of the face
      const reg = this.state === 'FACE_REGISTER', al = reg ? 0.5 : 0.3, be = reg ? 0.0 : 0.04, T = 0.033;
      for (let i = 0; i < 3; i++) { const pred = this.fe[i] + this.feV[i] * T, r = this.fTrue[i] + this.o.camNoise * gauss(this.rng) - pred; this.fe[i] = pred + al * r; this.feV[i] = reg ? 0 : this.feV[i] + be * r / T; }
    }
    // on-head pupil camera (lateral) + corneal-reflex focus (axial) at 100 Hz once within 4 mm of the eye
    if (!((this.state === 'FINE_ALIGN' || this.state === 'IMAGING') && this.goal in G.eyes)) this.pe = null;
    else if (this.tick % 10 === 0) {
      const eT = eyePos(this.goal, this.fTrue), tip = this.tipTrue, tg = [eT[0], eT[1] + G.WD, eT[2]];
      if (Math.hypot(tip[0] - tg[0], tip[2] - tg[2]) < 4 && Math.abs(tip[1] - tg[1]) < 6) {
        const z = [eT[0] + 0.012 * gauss(this.rng), eT[1] + 0.025 * gauss(this.rng), eT[2] + 0.012 * gauss(this.rng)];
        if (!this.pe) { this.pe = z.slice(); this.peV = [0, 0, 0]; }
        else for (let i = 0; i < 3; i++) { const pred = this.pe[i] + this.peV[i] * 0.01, r = z[i] - pred; this.pe[i] = pred + 0.5 * r; this.peV[i] += 0.1 * r / 0.01; }
      } else this.pe = null;
    }

    // ----- supervisor -----
    if (S === 'FACE_REGISTER' && this.t - this.regT > 1.0) { this.registered = true; this.setState('READY', 'Face registered — patient frame locked'); this.ev('REG', `Face registered at offset (${this.fe.map(x => x.toFixed(1)).join(', ')}) mm`); }
    if (this.inj && this.t > this.inj.until) { this.ev('INJ', `${this.inj.type} on ${NAMES[this.inj.axis]} cleared`); this.inj = null; }

    // ----- command generation -----
    if (S === 'ESTOP' && this.power === 0) { /* no command */ }
    else if (this.cmdMode === 'stop' && (S === 'FAULT_SLIP' || S === 'FAULT_NOGO' || S === 'ESTOP')) {
      this.stopT += dtc; const k = Math.max(0, 1 - this.stopT / this.stopTs);
      for (let i = 0; i < 5; i++) { const v = this.stopV0[i] * k; this.qc[i] += 0.5 * (this.vc[i] + v) * dtc; this.ac[i] = (v - this.vc[i]) / dtc; this.vc[i] = v; }
    } else if (this.adm.active) this.admittance(dtc);
    else if (this.traj && (S === 'MOVE_ARC' || S === 'RETRACT')) {
      this.r += clamp(this.rT - this.r, -dtc / 0.12, dtc / 0.12);
      const rs = this.r * this.r * (3 - 2 * this.r); // smoothstep -> continuous accel at pause/resume
      this.tau = Math.min(this.traj.N - 1, this.tau + rs);
      const k0 = Math.floor(this.tau), k1 = Math.min(this.traj.N - 1, k0 + 1), fr = this.tau - k0;
      for (let i = 0; i < 5; i++) { const T = this.traj; this.qc[i] = T.q[i][k0] + (T.q[i][k1] - T.q[i][k0]) * fr; this.vc[i] = rs * (T.v[i][k0] + (T.v[i][k1] - T.v[i][k0]) * fr); this.ac[i] = rs * rs * T.a[i][k0]; }
      if (S === 'MOVE_ARC') {
        // software tier: look-ahead keep-out check against latest face estimate + goal drift
        if (!this.replanPending && this.tick % 33 === 0) {
          let bad = false; const T = this.traj; const kE = Math.min(T.N - 1, Math.floor(this.tau) + 400);
          for (let k = Math.floor(this.tau); k <= kE; k += 10) { const q = [T.q[0][k], T.q[1][k], T.q[2][k], T.q[3][k], T.q[4][k]]; if (clearance(q, this.fe) < G.margin - 1.0) { bad = true; break; } }
          const drift = Math.hypot(this.fe[0] - this.planFace[0], this.fe[1] - this.planFace[1], this.fe[2] - this.planFace[2]);
          if (bad || (drift > 2.0 && this.goal in G.eyes && this.traj.N - this.tau > 250)) { this.rT = 0; this.replanPending = true; this.ev('PLAN', bad ? 'Face moved into look-ahead window → easing to stop, replanning' : `Patient shifted ${drift.toFixed(1)} mm → replanning to follow`); }
        }
        if ((this.replanPending || this.wantManual) && this.r < 1e-3) { if (this.wantManual) { this.wantManual = false; this.replanPending = false; this.enterManual(); } else this.doPlan(); }
        else if (this.tau >= this.traj.N - 1) {
          this.traj = null; for (let i = 0; i < 5; i++) { this.vc[i] = 0; this.ac[i] = 0; }
          if (this.goal === 'PARK' || this.goal === 'X') this.setState('READY', this.goal === 'PARK' ? 'Parked' : 'At commanded pose'); else { this.setState('FINE_ALIGN', 'Fine alignment to corneal apex'); this.trackF = this.goalQ.slice(); this.trackV = [0, 0, 0, 0, 0]; }
          this.ev('MOVE', 'Arc complete');
        }
      } else if (S === 'RETRACT' && this.tau >= this.traj.N - 1) {
        this.traj = null; for (let i = 0; i < 5; i++) { this.vc[i] = 0; this.ac[i] = 0; }
        if (this.afterRetract === 'hold') { this.afterRetract = null; this.setState('HOLD', 'Holding clear of the face until the patient is still'); this.stillSince = null; }
        else if (this.afterRetract === 'plan') { this.afterRetract = null; if (this.goal) this.doPlan(); else this.setState('READY', 'Ready'); }
        else this.setState('RECOVER', 'Retracted — waiting for fault to clear');
      }
    } else if (S === 'FINE_ALIGN' || S === 'IMAGING') {
      // track the eye (patient-relative) with a 2nd-order critically-damped reference filter
      let tgt, tgV = [0, 0, 0, 0, 0];
      if (!this.o.trackFace) tgt = this.goalQ;
      else if (this.pe) { tgt = ikEye(this.pe); tgV = [this.peV[0], this.peV[1], this.peV[2], 0, 0]; }
      else { tgt = ikEye(eyePos(this.goal, this.fe)); tgV = [this.feV[0], this.feV[1], this.feV[2], 0, 0]; }
      this.trackTgt = tgt;
      const w = 2 * Math.PI * 3.0;
      for (let i = 0; i < 5; i++) { const a = w * w * (tgt[i] - this.trackF[i]) + 2 * w * (tgV[i] - this.trackV[i]); this.trackV[i] = clamp(this.trackV[i] + a * dtc, -20, 20); this.trackF[i] += this.trackV[i] * dtc; this.qc[i] = this.trackF[i]; this.vc[i] = this.trackV[i]; this.ac[i] = a; }
      const tq = Array.from(this.lMeas), e = tipOf(tq), te = tipOf(tgt); this.alignErr = Math.hypot(e[0] - te[0], e[1] - te[1], e[2] - te[2]); const err = Math.hypot(e[0] - te[0], e[1] - te[1], e[2] - te[2]);
      if (S === 'FINE_ALIGN') { if (err < 0.05 && (this.pe || !this.o.trackFace)) { this.alignOk = (this.alignOk || 0) + 1; if (this.alignOk > 100) { const dtA = (this.t - this.stateT - 0.1) * 1000; this.m.settleMs = dtA; this.setState('IMAGING', 'On target — imaging (tracking breathing)'); this.ev('ALIGN', `Within ±50 µm of target ${Math.max(0, dtA).toFixed(0)} ms after arc end (held 100 ms)`); } } else this.alignOk = 0; }
      if (S === 'IMAGING' && err > 1.0) { this.setState('FINE_ALIGN', 'Re-acquiring eye'); }
      const fv = Math.hypot(this.feV[0], this.feV[1], this.feV[2]);
      if (fv > 8 && this.o.trackFace) { this.ev('AVOID', `Patient moving at ${fv.toFixed(0)} mm/s → retreat and hold until still`); this.startRetract('hold'); }
      else if (this.clrEst < G.margin + 1) { this.ev('AVOID', `Patient approaching head (clearance ${this.clrEst.toFixed(1)} mm) → reflex retreat`); this.startRetract('plan'); }
      else if (Math.hypot(...this.fe.map((x, i) => x - (this.planFace ? this.planFace[i] : x))) > 6) { this.ev('PLAN', 'Large patient shift during imaging → replanning'); this.doPlan(); }
    } else if (S === 'HOLD') {
      for (let i = 0; i < 5; i++) { this.vc[i] = 0; this.ac[i] = 0; }
      if (this.clrEst < G.margin + 1) this.startRetract('hold');
      else if (Math.hypot(this.feV[0], this.feV[1], this.feV[2]) < 2.5) { if (this.stillSince == null) this.stillSince = this.t; else if (this.t - this.stillSince > 0.3) { this.ev('PLAN', 'Patient still → re-registering and replanning'); this.doPlan(); } } else this.stillSince = null;
    } else if (S === 'RECOVER') {
      if (!this.inj && this.t - this.stateT > 0.3) {
        for (let i = 0; i < 5; i++) { this.refOff[i] = this.mMeas[i] - this.lMeas[i]; }
        this.enabled = [1, 1, 1, 1, 1]; this.fault = null; this.syncCmd(true); this.power = 1; this.brake = 0;
        for (let i = 0; i < 5; i++) { this.xh[i] = this.mMeas[i]; this.vh[i] = 0; }
        this.ev('RECOVER', 'Encoders re-referenced (motor ↔ load), fault cleared');
        if (this.o.autoRecover && this.goal) this.doPlan(); else this.setState('READY', 'Recovered — ready');
      }
    }
    // fault stop finished -> STO / retract
    if ((S === 'FAULT_SLIP' || S === 'FAULT_NOGO' || S === 'ESTOP')) {
      let still = true; for (let i = 0; i < 5; i++) if (Math.abs(this.vl[i]) > (this.ax[i].lin ? 0.5 : 0.2) || Math.abs(this.vm[i]) > (this.ax[i].lin ? 0.5 : 0.2)) still = false;
      if (still && this.m.stopTimeMs == null) { this.m.stopTimeMs = (this.t - this.m.faultAt) * 1000; this.ev('STOP', `All axes at rest ${this.m.stopTimeMs.toFixed(1)} ms after ${S === 'ESTOP' ? 'e-stop' : 'fault'}`); this.stillAt = this.t; }
      if (S === 'ESTOP' && this.power === 1 && this.m.stopTimeMs != null && this.t - this.stillAt > 0.02) { this.power = 0; this.stoAt = this.t; this.ev('STO', 'Safe Torque Off engaged, Z brake set'); }
      if (S !== 'ESTOP' && this.m.stopTimeMs != null && this.t - this.stillAt > 0.15) this.startRetract();
    }
    if (this.power === 0 && this.stoAt != null && this.t - this.stoAt > 0.015) this.brake = 1;

    // ----- position loop (1 kHz) on load encoders -----
    for (let i = 0; i < 5; i++) {
      const a = this.ax[i];
      if (!this.enabled[i] || !this.power) { this.vref[i] = 0; this.integ[i] = 0; continue; }
      const e = this.qc[i] - this.lMeas[i];
      if (Math.abs(e) > 0.5 * a.ql) this.integ[i] = clamp(this.integ[i] + e * dtc, -a.Ilim, a.Ilim);
      this.vref[i] = this.vc[i] + a.Kp * e + a.Ki * this.integ[i];
    }
    // ----- 20 kHz physics, 10 kHz velocity loop + encoder cross-check -----
    const dt = 0.00005;
    for (let sub = 0; sub < 20; sub++) {
      if ((sub & 1) === 0) this.fastLoop(0.0001);
      this.physics(dt);
    }
    // ----- monitors -----
    const ql = Array.from(this.lMeas);
    this.clrEst = clearance(ql, this.fe);
    const qt = Array.from(this.xl);
    const tb = tipOf(qt); this.tipTrue = [tb[0] + this.flex[0], tb[1] + this.flex[1], tb[2] + this.flex[2]];
    this.tipCmd = tipOf(this.qc.map((x, i) => x + this.vc[i] * dtc)); this.clrTrue = clearance(qt, this.fTrue);
    if (this.clrTrue < this.m.minClr) this.m.minClr = this.clrTrue;
    // firmware tier: measured pose must never breach (margin - 4 mm)
    if (!this.fault && this.state !== 'ESTOP' && this.state !== 'RETRACT' && this.state !== 'RECOVER' && this.clrEst < G.margin - 4) this.setFault('NOGO', null, `KEEP-OUT BREACH: clearance ${this.clrEst.toFixed(1)} mm`);
    // soft limits
    if (!this.fault && this.power) for (let i = 0; i < 5; i++) if (this.lMeas[i] < LO[i] - 2 || this.lMeas[i] > HI[i] + 2) { this.setFault('NOGO', null, `Soft limit on ${NAMES[i]}`); break; }

    this.t += dtc; this.tick++;
    if (this.o.log && this.tick % this.logEvery === 0) this.logSample();
  }
  fastLoop(Ts) {
    for (let i = 0; i < 5; i++) {
      const a = this.ax[i];
      // encoders
      this.mMeas[i] = Math.round(this.xm[i] / a.qm) * a.qm;
      const frozen = this.inj && this.inj.type === 'freeze' && this.inj.axis === i;
      if (!frozen) this.lMeas[i] = Math.round((this.xl[i] + a.nl * gauss(this.rng)) / a.ql) * a.ql;
      // alpha-beta velocity observer on motor encoder
      const pred = this.xh[i] + this.vh[i] * Ts + 0.5 * this.ac[i] * Ts * Ts, r = this.mMeas[i] - pred;
      this.xh[i] = pred + 0.45 * r; this.vh[i] += this.ac[i] * Ts + 0.07 * r / Ts;
      // velocity loop + feed-forward -> force/torque command
      if (!this.power) { this.u[i] = 0; }
      else if (!this.enabled[i]) { this.u[i] = clamp(a.Kv * (0 - this.vh[i]), -a.Umax, a.Umax); } // faulted axis: hold motor at zero speed
      else { const ff = (a.M + a.m) / a.s * this.ac[i] + (a.Fc + a.fm) * Math.tanh(this.vc[i] / (a.lin ? 0.5 : 0.2)); this.u[i] = clamp(ff + a.Kv * (this.vref[i] - this.vh[i]), -a.Umax, a.Umax); }
      // dual-encoder cross-check: expected elastic deflection from commanded accel + friction
      const expD = (a.m * this.ac[i] / a.s + a.Fc * Math.tanh(this.vc[i] / (a.lin ? 0.5 : 0.2))) / a.k;
      const raw = this.mMeas[i] - this.lMeas[i] - this.refOff[i] - expD; const res = this.resid[i] + 0.47 * (raw - this.resid[i]); this.resid[i] = res; // 1 kHz low-pass
      if (this.o.slipCheck && this.enabled[i] && this.power && !this.fault) {
        if (Math.abs(res) > a.slipThr) { if (++this.slipCnt[i] >= 4) this.setFault('SLIP', i, `SLIP DETECTED on ${NAMES[i]}: motor/load mismatch ${(res * (a.lin ? 1000 : 1)).toFixed(a.lin ? 0 : 3)} ${a.lin ? 'µm' : '°'}`); }
        else this.slipCnt[i] = 0;
      }
    }
  }
  physics(dt) {
    for (let i = 0; i < 5; i++) {
      const a = this.ax[i];
      const h = a.b / 2; let d = this.xm[i] - this.xl[i] - this.cpl[i];
      const slipping = this.inj && this.inj.type === 'slip' && this.inj.axis === i;
      let F;
      if (d > h) F = a.k * (d - h) + a.c * (this.vm[i] - this.vl[i]); else if (d < -h) F = a.k * (d + h) + a.c * (this.vm[i] - this.vl[i]); else F = 0;
      if (slipping) { const Fs = a.lin ? 4 : 0.004; if (Math.abs(F) > Fs) { F = Fs * sgn(F); this.cpl[i] = this.xm[i] - this.xl[i] - (h + Fs / a.k) * sgn(d); } }
      // motor side (with brake on Z when power is off)
      const fm = this.power ? a.fm : (a.lin ? 30 : 0.02);
      let am = a.s * (this.u[i] - F - fm * Math.tanh(this.vm[i] / 0.3)) / a.M;
      let vmn = this.vm[i] + am * dt;
      if (this.brake && i === 2) { const db = a.s * 1500 / a.M * dt; vmn = Math.abs(vmn) <= db ? 0 : vmn - db * sgn(vmn); }
      this.vm[i] = vmn; this.xm[i] += this.vm[i] * dt;
      // load side with Karnopp stiction
      const Fd = F + (this.adm.ext ? this.adm.ext[i] : 0);
      const vs = a.lin ? 0.005 : 0.002;
      if (Math.abs(this.vl[i]) < vs && Math.abs(Fd) <= a.Fs) this.vl[i] = 0;
      else { const fr = Math.abs(this.vl[i]) < vs ? a.Fc * sgn(Fd) : a.Fc * sgn(this.vl[i]); const vn = this.vl[i] + a.s * (Fd - fr) / a.m * dt; this.vl[i] = (vn * this.vl[i] < 0) ? 0 : vn; }
      this.xl[i] += this.vl[i] * dt;
    }
    // cantilevered head flex mode driven by tip base acceleration
    const qt = [this.xl[0], this.xl[1], this.xl[2], this.xl[3], this.xl[4]];
    const tb = tipOf(qt);
    if (this.tb1) {
      const w = 2 * Math.PI * this.o.fHead, z = this.o.zHead;
      for (let j = 0; j < 3; j++) { const ab = (tb[j] - 2 * this.tb1[j] + this.tb2[j]) / (dt * dt); const af = -w * w * this.flex[j] - 2 * z * w * this.flexV[j] - ab; this.flexV[j] += af * dt; this.flex[j] += this.flexV[j] * dt; }
    }
    this.tb2 = this.tb1 || tb; this.tb1 = tb;
  }
  admittance(dtc) {
    const A = this.adm; const Bv = [0.04, 0.04, 0.04, 0.02, 0.02], Mv = [0.0015, 0.0015, 0.0015, 0.0008, 0.0008], db = [0.3, 0.3, 0.3, 0.02, 0.02];
    const v = A.v, q = this.qc.slice();
    for (let i = 0; i < 5; i++) {
      let F = A.deadman ? A.F[i] : 0; F = Math.abs(F) < db[i] ? 0 : F - db[i] * sgn(F);
      const B = A.deadman ? Bv[i] : Bv[i] * 8;
      const acc = (F - B * v[i]) / Mv[i]; v[i] += acc * dtc;
    }
    // virtual fixture: never let the operator push the head into the keep-out band
    const qn = q.map((x, i) => x + v[i] * dtc * 40); const cn = clearance(qn, this.fe);
    if (cn < G.margin + 2) {
      const g = clrGrad(q, this.fe); let gn = 0, vg = 0; for (let i = 0; i < 5; i++) { gn += g[i] * g[i] * W[i] * W[i]; }
      gn = Math.sqrt(gn) || 1; const gu = g.map((x, i) => x * W[i] / gn);
      for (let i = 0; i < 5; i++) vg += v[i] * W[i] * gu[i];
      if (vg < 0) { for (let i = 0; i < 5; i++) v[i] -= vg * gu[i] / W[i]; A.blocked = true; } else A.blocked = false;
    } else A.blocked = false;
    for (let i = 0; i < 5; i++) { v[i] = clamp(v[i], -VAX[i] * 0.6, VAX[i] * 0.6); const qn2 = clamp(this.qc[i] + v[i] * dtc, LO[i], HI[i]); const vn = (qn2 - this.qc[i]) / dtc; this.ac[i] = (vn - this.vc[i]) / dtc; this.vc[i] = vn; this.qc[i] = qn2; }
  }
  logSample() {
    const L = this.lg, i = L.i;
    L.t[i] = this.t; L.err[i] = Math.hypot(this.tipTrue[0] - this.tipCmd[0], this.tipTrue[1] - this.tipCmd[1], this.tipTrue[2] - this.tipCmd[2]) * 1000;
    L.clr[i] = this.clrTrue; L.spd[i] = Math.hypot(this.vl[0], this.vl[1], this.vl[2]);
    for (let j = 0; j < 5; j++) { L.res[j][i] = this.resid[j] / this.ax[j].slipThr; }
    L.i = (i + 1) % this.LOGN; L.n = Math.min(this.LOGN, L.n + 1);
  }
  run(ms, until) { for (let k = 0; k < ms; k++) { this.stepMs(); if (until && until(this)) return true; } return false; }
}

const OE = { Sim, G, LO, HI, W, PARK, NAMES, VAX, tipOf, pivotOf, dirOf, eyePos, ikEye, clearance, planPath, buildTraj, rngOf, gauss, wdist, inLim, zvdImpulses };
if (typeof module !== 'undefined' && module.exports) module.exports = OE; else root.OE = OE;
})(typeof window !== 'undefined' ? window : globalThis);
