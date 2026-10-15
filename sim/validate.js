// ORBIT-EYE validation suite — runs the same engine as the browser simulator.
const OE = require('./engine.js');
const { Sim, tipOf, ikEye, eyePos, clearance, planPath, G, LO, HI, rngOf } = OE;
const R = {};
const stats = a => { const n = a.length, m = a.reduce((x, y) => x + y, 0) / n, sd = Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / Math.max(1, n - 1)); return { n, mean: m, sd, max: Math.max(...a), min: Math.min(...a), p95: [...a].sort((x, y) => x - y)[Math.floor(0.95 * (n - 1))] }; };
const log = (...a) => console.log(...a);
function ready(o = {}) { const s = new Sim(Object.assign({ log: false }, o)); s.cmd('register'); s.run(1200); return s; }
function goAndWait(s, c, ms = 8000, st = 'IMAGING') { s.cmd(c); return s.run(ms, x => x.state === st); }

/* T1 — bidirectional repeatability to the same eye pose (20 cycles), several hardware variants */
function repeatability(variant) {
  const s = ready(Object.assign({ breathing: false, trackFace: false, camNoise: 0 }, variant));
  const tips = [], qs = []; let trips = 0;
  for (let c = 0; c < 20; c++) {
    goAndWait(s, c % 2 ? 'goL' : 'park', 8000, c % 2 ? 'IMAGING' : 'READY');
    goAndWait(s, 'goR'); s.run(400);
    if (s.events.some(e => e.type === 'FAULT' && e.t > (s._lastT || 0))) trips++; s._lastT = s.t;
    if (s.state !== 'IMAGING') { s.run(8000, x => x.state === 'IMAGING' || x.state === 'READY'); }
    tips.push(s.tipTrue.slice()); qs.push(Array.from(s.xl));
  }
  const out = {};
  ['x', 'y', 'z'].forEach((k, j) => { const v = tips.map(t => t[j] * 1000); const m = v.reduce((a, b) => a + b) / v.length; out['tip_' + k + '_um'] = Math.max(...v.map(x => Math.abs(x - m))); });
  ['yaw', 'pitch'].forEach((k, j) => { const v = qs.map(q => q[3 + j]); const m = v.reduce((a, b) => a + b) / v.length; out[k + '_deg'] = Math.max(...v.map(x => Math.abs(x - m))); });
  out.nuisanceSlipTrips = trips;
  return out;
}
log('T1 repeatability…');
R.repeat = {
  recommended: repeatability({ rotEnc: 'MT6835', nut: 'DFU1605' }),
  asSpecifiedRotaryEncoder: repeatability({ rotEnc: 'AMT223', nut: 'DFU1605' }),
  nonPreloadedNut_slipCheckOn: repeatability({ rotEnc: 'MT6835', nut: 'SFU1605' }),
  nonPreloadedNut_slipCheckOff: repeatability({ rotEnc: 'MT6835', nut: 'SFU1605', slipCheck: false })
};
log(R.repeat);

/* T2 — settling after a park → right-eye arc: ZVD on/off and mistuned */
function settle(o) {
  const s = ready(Object.assign({ breathing: false, trackFace: false, camNoise: 0 }, o));
  const qT = [90, 40, -40, 20, 10]; s.cmd('goto', qT); let tEnd = null, last = null, peak = 0; const target = tipOf(qT);
  for (let k = 0; k < 6000; k++) {
    s.stepMs();
    if (tEnd == null && s.state !== 'MOVE_ARC') tEnd = s.t;
    if (tEnd != null) { const e = Math.hypot(...s.tipTrue.map((v, i) => v - target[i])) * 1000; if (e > 50) last = s.t; peak = Math.max(peak, e); if (s.t - tEnd > 1.0) break; }
  }
  return { settle_ms_to_50um: last == null ? 0 : Math.max(0, (last - tEnd) * 1000), peak_after_end_um: peak, moveTime_s: tEnd - 1.2 };
}
function eyeLock(n) { const v = []; for (let k = 0; k < n; k++) { const s = ready({ seed: 40 + k }); goAndWait(s, k % 2 ? 'goL' : 'goR'); v.push(s.m.settleMs); } return stats(v); }
log('T2 settling…');
R.settle = {
  zvd_on: settle({ zvd: true }), zvd_off: settle({ zvd: false }),
  zvd_on_head_40Hz_mistuned: settle({ zvd: true, fHead: 40 }), zvd_on_head_52Hz_mistuned: settle({ zvd: true, fHead: 52 }),
  zvd_off_trapezoid_like: settle({ zvd: false, Tj: 0.001 }),
  eyeLock_ms_after_arc_with_breathing: eyeLock(20)
};
log(R.settle);

/* T3 — 1000 randomised moves (random patient offset, start and goal); full dynamics */
log('T3 1000 randomised moves…');
{
  const rng = rngOf(2026); let viol10 = 0, contact = 0, planFail = 0, falseTrips = 0, done = 0; const minC = [], pms = [], rms = [], maxE = [], durs = [];
  const randFree = f => { for (let k = 0; k < 500; k++) { const q = [0, 1, 2, 3, 4].map(i => LO[i] + rng() * (HI[i] - LO[i])); q[1] = -40 + rng() * 140; if (clearance(q, f) > G.margin + 2) return q; } return null; };
  const t0 = Date.now();
  for (let n = 0; n < 1000; n++) {
    const f = [(rng() - 0.5) * 30, (rng() - 0.5) * 30, (rng() - 0.5) * 30];
    const s = new Sim({ log: false, seed: 100 + n, faceOffset: f, trackFace: false, breathing: true });
    const q0 = randFree(f); if (!q0) continue;
    for (const arr of [s.xm, s.xl, s.xh, s.mMeas, s.lMeas]) for (let i = 0; i < 5; i++) arr[i] = q0[i];
    s.qc = q0.slice(); s.tb1 = s.tb2 = null; for (let i = 0; i < 3; i++) s.fe[i] = f[i];
    s.registered = true; s.setState('READY');
    const r = rng(); let ok;
    if (r < 0.7) ok = s.cmd(r < 0.35 ? 'goR' : 'goL');
    else { const qg = randFree(f); if (!qg) continue; ok = s.cmd('goto', qg); }
    if (s.state !== 'MOVE_ARC') { planFail++; continue; }
    pms.push(s.m.lastPlanMs); s.m.minClr = 1e9;
    const errs = []; const T0 = s.t;
    while (s.state === 'MOVE_ARC' && s.t - T0 < 15) { s.stepMs(); errs.push(Math.hypot(...s.tipTrue.map((v, i) => v - s.tipCmd[i])) * 1000); }
    s.run(150);
    if (s.fault) falseTrips++;
    durs.push(s.t - T0); minC.push(s.m.minClr); if (s.m.minClr < G.margin - 0.5) viol10++; if (s.m.minClr < 0) contact++;
    rms.push(Math.sqrt(errs.reduce((a, b) => a + b * b, 0) / errs.length)); maxE.push(Math.max(...errs)); done++;
  }
  R.random = { moves: done, planFailures: planFail, contact, below_margin_minus_0p5: viol10, falseSlipTrips: falseTrips, minClearance: stats(minC), planMs: stats(pms), pathRMS_um: stats(rms), pathMax_um: stats(maxE), moveTime_s: stats(durs), wall_s: (Date.now() - t0) / 1000 };
  log(R.random);
}

/* T4 — fault injection: coupling slip and frozen load encoder mid-arc */
function faults(type, N) {
  const rng = rngOf(type === 'slip' ? 11 : 12); const lat = [], stop = [], minC = [], recovered = [], undetected = [];
  for (let n = 0; n < N; n++) {
    const s = ready({ seed: 500 + n, breathing: true });
    s.cmd(n % 2 ? 'goR' : 'goL'); if (n % 2 === 0) { goAndWait(s, 'goR'); s.cmd('goL'); }
    const tInj = 0.2 + rng() * 0.9 * (s.traj ? s.traj.T : 1); s.run(Math.floor(tInj * 1000));
    if (s.state !== 'MOVE_ARC') { continue; }
    const mv = [0, 1, 2, 3, 4].filter(i => Math.abs(s.vc[i]) > (i < 3 ? 10 : 4)); if (!mv.length) continue;
    const ax = mv[Math.floor(rng() * mv.length)]; s.m.minClr = 1e9;
    s.cmd('inject', { type, axis: ax, ms: 300 });
    const det = s.run(300, x => !!x.fault);
    if (!det) { undetected.push(ax); continue; }
    lat.push(s.m.slipLatencyMs);
    s.run(400, x => x.m.stopTimeMs != null); stop.push(s.m.stopTimeMs);
    const rec = s.run(10000, x => x.state === 'IMAGING'); recovered.push(rec ? 1 : 0); minC.push(s.m.minClr);
  }
  return { trials: lat.length + undetected.length, detected: lat.length, undetected: undetected.length, latency_ms: stats(lat), stop_ms: stats(stop), minClearance: stats(minC), autoRecovered: recovered.reduce((a, b) => a + b, 0) };
}
log('T4 fault injection…');
R.slip = faults('slip', 60); log('slip', R.slip);
R.freeze = faults('freeze', 60); log('freeze', R.freeze);

/* T5 — e-stop at peak arc speed: Cat 0 (as specified: cut power) vs Cat 1 (controlled stop, then STO) */
function estop(cat) {
  const out = [];
  for (let n = 0; n < 20; n++) {
    const s = ready({ seed: 900 + n, estopCat: cat, breathing: false });
    s.cmd('goR'); let vpk = 0, kpk = 0; const T = s.traj; for (let k = 0; k < T.N; k++) { const v = Math.hypot(T.v[0][k], T.v[1][k], T.v[2][k]); if (v > vpk) { vpk = v; kpk = k; } }
    s.run(kpk); const p0 = s.tipTrue.slice(); const sp = Math.hypot(s.vl[0], s.vl[1], s.vl[2]); s.m.minClr = 1e9;
    s.cmd('estop'); let tStop = null;
    for (let k = 0; k < 3000; k++) { s.stepMs(); if (tStop == null && Math.hypot(s.vl[0], s.vl[1], s.vl[2]) < 0.5 && Math.abs(s.vm[0]) < 0.5 && Math.abs(s.vm[1]) < 0.5 && Math.abs(s.vm[2]) < 0.5) { tStop = k + 1; break; } }
    s.run(200);
    out.push({ speed: sp, stop_ms: tStop, dist_mm: Math.hypot(...s.tipTrue.map((v, i) => v - p0[i])), minClr: s.m.minClr });
  }
  return { speed_mm_s: stats(out.map(o => o.speed)), stop_ms: stats(out.map(o => o.stop_ms ?? 3000)), coast_mm: stats(out.map(o => o.dist_mm)), minClearance: stats(out.map(o => o.minClr)) };
}
log('T5 e-stop…');
R.estop = { cat0_as_specified: estop(0), cat1_recommended: estop(1) }; log(R.estop);

/* T6 — compliant hand-over: steady speed vs operator force; virtual fixture holds the keep-out */
log('T6 hand-over…');
{
  const rows = [];
  for (const F of [0.2, 0.5, 1, 2, 5]) {
    const s = ready({ breathing: false }); s.cmd('manual'); s.cmd('deadman', true); s.cmd('force', [F, 0, 0, 0, 0]);
    const x0 = s.tipTrue[0]; let t1mm = null; for (let k = 0; k < 1500; k++) { s.stepMs(); if (t1mm == null && Math.abs(s.tipTrue[0] - x0) > 1) t1mm = k + 1; }
    rows.push({ force_N: F, speed_mm_s: Math.abs(s.vl[0]), ms_to_move_1mm: t1mm });
  }
  // push straight at the face for 4 s with 8 N: clearance must never drop below margin
  const s = ready({ breathing: true }); goAndWait(s, 'goR'); s.cmd('manual'); s.run(200); s.cmd('deadman', true); s.cmd('force', [0, -8, 0, 0, 0]); s.m.minClr = 1e9; s.run(4000);
  const pushFace = { force_N: 8, minClearance_mm: s.m.minClr, blocked: !!s.adm.blocked };
  s.cmd('deadman', false); const p = s.tipTrue.slice(); s.run(300);
  R.handover = { rows, pushFace, deadmanReleaseDrift_mm: Math.hypot(...s.tipTrue.map((v, i) => v - p[i])) };
  log(R.handover);
}

/* T7 — patient leans 20 mm during imaging: platform follows; re-acquire time and final error */
log('T7 patient shift…');
{
  const out = [];
  for (const d of [[20, 0, 0], [0, -20, 0], [0, 0, 20], [-14, 10, -10]]) {
    const s = ready({ breathing: false, camNoise: 0.3 }); goAndWait(s, 'goR'); s.run(500);
    const t0 = s.t; s.cmd('shiftFace', d); s.run(800);
    const ok = s.run(8000, x => x.state === 'IMAGING' && x.t - t0 > 0.6); const tR = s.t - t0; s.run(300);
    const eye = eyePos('R', s.fTrue), tgt = tipOf(ikEye(eye));
    out.push({ shift: d.join(','), reacquired: ok, reacquire_s: tR, finalErr_um: Math.hypot(...s.tipTrue.map((v, i) => v - tgt[i])) * 1000, minClr: s.m.minClr, faults: s.events.filter(e => e.type === 'FAULT').length });
  }
  R.patientShift = out; log(out);
}

/* T8 — imaging stability with breathing + camera noise (what the eye actually sees) */
log('T8 tracking…');
{
  const s = ready({ breathing: true, camNoise: 0.3 }); goAndWait(s, 'goR'); s.run(1000); const e = [];
  for (let k = 0; k < 8000; k++) { s.stepMs(); const tgt = tipOf(ikEye(eyePos('R', s.fTrue))); e.push(Math.hypot(...s.tipTrue.map((v, i) => v - tgt[i])) * 1000); }
  const noCam = ready({ breathing: true, camNoise: 0.05 }); goAndWait(noCam, 'goR'); noCam.run(1000); const e2 = [];
  for (let k = 0; k < 8000; k++) { noCam.stepMs(); const tgt = tipOf(ikEye(eyePos('R', noCam.fTrue))); e2.push(Math.hypot(...noCam.tipTrue.map((v, i) => v - tgt[i])) * 1000); }
  R.tracking = { camNoise_0p3mm: stats(e), camNoise_0p05mm: stats(e2) }; log(R.tracking);
}

/* T9 — sensitivity: settle time vs actual head mode with ZVD tuned at 45 Hz */
log('T9 robustness…');
R.zvdSweep = [30, 35, 40, 45, 50, 55, 60, 80, 120].map(fh => ({ fHead: fh, zvd_peak_um: settle({ zvd: true, fHead: fh }).peak_after_end_um, none_peak_um: settle({ zvd: false, fHead: fh }).peak_after_end_um, trapezoid_peak_um: settle({ zvd: false, Tj: 0.001, fHead: fh }).peak_after_end_um }));
log(R.zvdSweep);

require('fs').writeFileSync(__dirname + '/../results/results.json', JSON.stringify(R, null, 1));
log('done');
