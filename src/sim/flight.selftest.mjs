import assert from 'node:assert/strict';
import {
  createFlightState,
  stepFlight,
  F35_LIGHTNING,
  FLIGHT_DT,
  FLIGHT_EVT_GUN_FIRED,
  FLIGHT_EVT_AGM_FIRED,
  FLIGHT_EVT_STALLED,
  FLIGHT_EVT_CRASHED,
} from './flight.ts';

const flat = () => 0;
const input = (over = {}) => ({
  throttle: 0.7, pitchCmd: 0, rollCmd: 0, yawCmd: 0, fireGun: false, fireAgm: false, ...over,
});
const cruise = (over = {}) => {
  const s = createFlightState(0, 200, 0, 0, F35_LIGHTNING, over);
  return s;
};
const finite = (s) => [s.pos.x, s.pos.y, s.pos.z, s.vel.x, s.vel.y, s.vel.z,
  s.yaw, s.pitch, s.roll].every(Number.isFinite);

// 1. Determinism: identical inputs -> identical states.
{
  const run = () => {
    const s = cruise();
    for (let i = 0; i < 600; i++) {
      stepFlight(s, input({ pitchCmd: Math.sin(i * 0.02), rollCmd: 0.4, throttle: 0.85 }), F35_LIGHTNING, FLIGHT_DT, flat);
    }
    return s;
  };
  const a = run(), b = run();
  assert.deepEqual([a.pos.x, a.pos.y, a.pos.z], [b.pos.x, b.pos.y, b.pos.z]);
  assert.deepEqual([a.yaw, a.pitch, a.roll], [b.yaw, b.pitch, b.roll]);
  assert.ok(finite(a));
}

// 2. Level cruise holds altitude hands-off (within a loose band).
// Throttle 0.5 is the neutral cruise setting (thrust ≈ drag, lift ≈ weight).
{
  const s = cruise();
  const y0 = s.pos.y;
  for (let i = 0; i < 600; i++) stepFlight(s, input({ throttle: 0.5 }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.ok(Math.abs(s.pos.y - y0) < 25, `cruise held (${s.pos.y.toFixed(1)} vs ${y0})`);
  assert.ok(!s.stalled && !s.crashed);
}

// 3. Stall: chopped throttle + hard pull bleeds speed into the danger
// zone, the wing drops the nose by itself, and the event fires airborne.
{
  const s = createFlightState(0, 800, 0, 0, F35_LIGHTNING);
  let stalled = false;
  const y0 = s.pos.y;
  for (let i = 0; i < 600; i++) {
    stepFlight(s, input({ throttle: 0, pitchCmd: 1 }), F35_LIGHTNING, FLIGHT_DT, flat);
    if (s._evt & FLIGHT_EVT_STALLED) stalled = true;
    if (s.crashed) break;
  }
  const sank = s.pos.y < y0 - 20;
  assert.ok(stalled, 'stall event raised');
  assert.ok(!s.crashed, 'stall happens airborne, not in the dirt');
  assert.ok(sank, 'stalled jet sinks');
  assert.ok(finite(s));
}

// 4. Banked turn: sustained right bank yaws right without rudder.
{
  const s = cruise();
  const yaw0 = s.yaw;
  for (let i = 0; i < 300; i++) stepFlight(s, input({ rollCmd: 1 }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.ok(s.yaw > yaw0 + 0.15, `banked turn yaws (${(s.yaw - yaw0).toFixed(2)} rad)`);
  assert.ok(s.roll > 0.8, 'bank established');
}

// 5. Terrain strike is lethal and deterministic.
{
  const s = createFlightState(0, 30, 0, 0, F35_LIGHTNING);
  let crashed = false;
  for (let i = 0; i < 1200 && !crashed; i++) {
    stepFlight(s, input({ throttle: 0, pitchCmd: -1 }), F35_LIGHTNING, FLIGHT_DT, flat);
    if (s._evt & FLIGHT_EVT_CRASHED) crashed = true;
  }
  assert.ok(crashed && s.crashed, 'ground strike crashes');
  const y = s.pos.y;
  stepFlight(s, input({ throttle: 1, pitchCmd: 1 }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.equal(s.pos.y, y, 'crashed jet is inert');
}

// 6. Weapon gating: gun cycles, AGM fires once per 40 s reload cadence
// (no double-tap alpha strike — the anti-carpet-bombing doctrine).
{
  const s = cruise();
  stepFlight(s, input({ fireGun: true }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.ok(s._evt & FLIGHT_EVT_GUN_FIRED, 'gun fires');
  assert.ok(s.gunReloadT > 0, 'gun reload starts');
  stepFlight(s, input({ fireAgm: true }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.ok(s._evt & FLIGHT_EVT_AGM_FIRED, 'agm fires');
  assert.equal(s.agmLeft, 1);
  stepFlight(s, input({ fireAgm: true }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.ok(!(s._evt & FLIGHT_EVT_AGM_FIRED), 'reload gates the second round');
  for (let i = 0; i < Math.ceil(40 / FLIGHT_DT); i++) stepFlight(s, input(), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.equal(s.agmLeft, 2, 'one round reloaded per 40 s');
  stepFlight(s, input({ fireAgm: true }), F35_LIGHTNING, FLIGHT_DT, flat);
  assert.ok(s._evt & FLIGHT_EVT_AGM_FIRED, 'reloaded round fires');
  assert.equal(s.agmLeft, 1);
}

// 7. Perf: 60 s of flight steps runs in a blink with no allocation blowup.
{
  const s = cruise();
  const t0 = performance.now();
  for (let i = 0; i < 3600; i++) stepFlight(s, input({ rollCmd: 0.3 }), F35_LIGHTNING, FLIGHT_DT, flat);
  const ms = performance.now() - t0;
  assert.ok(ms < 500, `3600 steps in ${ms.toFixed(0)} ms`);
  assert.ok(finite(s));
}

// 8. Spec sanity: documented top speed matches the thrust/drag wall.
{
  const expect = Math.sqrt(F35_LIGHTNING.thrustAccel / F35_LIGHTNING.dragK);
  assert.ok(Math.abs(expect - F35_LIGHTNING.maxSpeed) < 1, 'maxSpeed matches drag wall');
  assert.ok(F35_LIGHTNING.hp < 500, 'fragile vs tank guns');
  assert.ok(F35_LIGHTNING.gunDamage <= 15, 'gun chips, never deletes MBTs');
}

console.log('flight.selftest: deterministic fixed-wing model, stall/turn/crash/weapon gates and spec doctrine passed');
