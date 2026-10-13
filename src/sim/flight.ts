/**
 * flight.ts — deterministic fixed-wing flight model for air support.
 *
 * Phase 1 of mixed tank/air battles. Owns ONLY the flight dynamics and
 * weapon-gating counters; armor, ballistics resolution, rendering, bots and
 * roster wiring arrive in later phases.
 *
 * Conventions (ARCHITECTURE §1.1, shared with movement.ts): meters /
 * seconds / radians, +Y up, yaw = 0 faces +Z (forward = [sin yaw, 0, cos yaw]
 * level), positive pitch = nose up, positive roll = right side up (matches
 * the locked renderer composition rotation.set(-pitch, yaw, roll, 'YXZ')).
 * Fixed-step integration at SIM_DT (1/60); no wall-clock, no Math.random,
 * no DOM, no renderer — runs under plain node. Module scratch makes the
 * stepper allocation-free but non-reentrant: advance aircraft sequentially
 * inside a fixed-step owner (same contract as movement.ts).
 *
 * Balance doctrine (F-35-class vs MBTs, see F35_LIGHTNING below):
 * - Speed is life, ammo is scarce: the gun chips armor (threat vs light
 *   vehicles/rear shots only) and the 2 ready AGMs hit hard but reload
 *   slowly. A jet cannot loiter over the battlefield carpet-bombing.
 * - The jet is fragile (low HP) and the ground is lethal: terrain strike =
 *   kill, stall below stallSpeed bleeds authority, and every attack run
 *   spends energy that must be rebuilt in level flight.
 * - Counter-play levers reserved for later phases: roof MG vs air chip,
 *   dedicated AA vehicles, and spotting/AI engagement of air contacts.
 */

import { Vector3 } from 'three';

/** Per-step pilot/AI command. All commands are unitless in [-1, 1]. */
export interface FlightInput {
  /** -1 idle … +1 full military thrust. */
  throttle: number;
  /** -1 full push (nose down) … +1 full pull (nose up). */
  pitchCmd: number;
  /** -1 full left … +1 full right bank. */
  rollCmd: number;
  /** Rudder: weak direct yaw, mainly for fine aim on gun runs. */
  yawCmd: number;
  /** Gun trigger held. */
  fireGun: boolean;
  /** AGM release pressed (edge-triggered by the caller). */
  fireAgm: boolean;
}

/** Authored per-airframe performance and weapon data. */
export interface FlightSpec {
  /** Thrust acceleration at full throttle (m/s^2). */
  thrustAccel: number;
  /** Parasitic drag coefficient: drag = dragK * v^2 (m/s^2 at v). */
  dragK: number;
  /**
   * Induced (high-alpha) drag: extra dragK scaled by sin^2(pitch).
   * Level cruise is untouched; sustained nose-high hanging bleeds energy
   * into the stall instead of settling in a mushing descent.
   */
  inducedK: number;
  /** Level-flight top speed, derived: sqrt(thrust/drag). Kept explicit. */
  maxSpeed: number;
  /** Below this speed control authority collapses (m/s). */
  stallSpeed: number;
  /** Full-stick pitch rate at flying speed (rad/s). */
  maxPitchRate: number;
  /** Full-stick roll rate at flying speed (rad/s). */
  maxRollRate: number;
  /** Rudder yaw rate at flying speed (rad/s, deliberately weak). */
  maxYawRate: number;
  /** Sustained load factor limit (G). Caps banked-turn rate. */
  maxG: number;
  /** Lift accel straight up at q = 1 (cruise q ≈ 0.5 balances weight). */
  liftK: number;
  /** Soft altitude ceiling (m). Control softens above, never snaps. */
  ceilingM: number;
  /** Minimum survivable clearance over terrain (m). */
  minClearanceM: number;
  /** Gun: 25 mm-class chip damage per resolved hit. */
  gunDamage: number;
  /** Gun cycle time (s). */
  gunReloadS: number;
  /** Ready AGM rounds. */
  agmCount: number;
  /** AGM reload per round (s). */
  agmReloadS: number;
  /** Airframe HP (fragile vs tank guns). */
  hp: number;
}

/** Event bits raised on state._evt by stepFlight; caller reads and clears. */
export const FLIGHT_EVT_GUN_FIRED = 1;
export const FLIGHT_EVT_AGM_FIRED = 2;
export const FLIGHT_EVT_STALLED = 4;
export const FLIGHT_EVT_CRASHED = 8;

export interface FlightState {
  pos: Vector3;
  vel: Vector3;
  yaw: number;
  pitch: number;
  roll: number;
  throttle: number;
  stalled: boolean;
  crashed: boolean;
  gunReloadT: number;
  agmLeft: number;
  agmReloadT: number;
  hp: number;
  _evt: number;
}

export function createFlightState(x: number, y: number, z: number, yaw: number, spec: FlightSpec): FlightState {
  const cruise = (spec.maxSpeed + spec.stallSpeed) * 0.5;
  return {
    pos: new Vector3(x, y, z),
    vel: new Vector3(Math.sin(yaw) * cruise, 0, Math.cos(yaw) * cruise),
    yaw,
    pitch: 0,
    roll: 0,
    throttle: 0.7,
    stalled: false,
    crashed: false,
    gunReloadT: 0,
    agmLeft: spec.agmCount,
    agmReloadT: 0,
    hp: spec.hp,
    _evt: 0,
  };
}

// Module scratch (see non-reentrancy contract above).
const _fwd = new Vector3();
const _acc = new Vector3();

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * Advance one fixed step. heightAt(x, z) is the terrain height sampler.
 * Pure deterministic math; the ONLY state writes are on `state`.
 */
export function stepFlight(
  state: FlightState,
  input: FlightInput,
  spec: FlightSpec,
  dt: number,
  heightAt: (x: number, z: number) => number,
): void {
  if (state.crashed) return;
  state._evt = 0;

  state.throttle = clamp(Number.isFinite(input.throttle) ? input.throttle : 0, 0, 1);
  const speed = state.vel.length();
  // Control authority scales with dynamic pressure; gone below stall speed.
  const q = clamp((speed - spec.stallSpeed) / Math.max(1, spec.maxSpeed - spec.stallSpeed), 0, 1);
  const authority = q * q * (3 - 2 * q); // smoothstep: no authority cliff at stall
  const wasStalled = state.stalled;
  state.stalled = speed < spec.stallSpeed;
  if (state.stalled && !wasStalled) state._evt |= FLIGHT_EVT_STALLED;
  // Stall break: a stalled wing drops the nose by itself (authority is ~0
  // so the stick cannot hold it up). The dive rebuilds flying speed.
  if (state.stalled) state.pitch = Math.max(state.pitch - 0.6 * dt, -0.35);
  // Ceiling softening: control fades in the last 15% below the ceiling.
  const ceilK = clamp((spec.ceilingM - state.pos.y) / (spec.ceilingM * 0.15), 0, 1);

  const pitchCmd = clamp(Number.isFinite(input.pitchCmd) ? input.pitchCmd : 0, -1, 1);
  const rollCmd = clamp(Number.isFinite(input.rollCmd) ? input.rollCmd : 0, -1, 1);
  const yawCmd = clamp(Number.isFinite(input.yawCmd) ? input.yawCmd : 0, -1, 1);

  state.pitch += pitchCmd * spec.maxPitchRate * authority * ceilK * dt;
  state.pitch = clamp(state.pitch, -1.1, 1.1);
  state.roll += rollCmd * spec.maxRollRate * authority * dt;
  state.roll = clamp(state.roll, -1.35, 1.35);
  // Coordinated banked turn, G-limited; rudder adds a weak direct term.
  const gTurn = 9.81 * Math.tan(state.roll) / Math.max(speed, spec.stallSpeed);
  const maxTurn = (spec.maxG * 9.81) / Math.max(speed, spec.stallSpeed);
  state.yaw += (clamp(gTurn, -maxTurn, maxTurn)
    + yawCmd * spec.maxYawRate * authority) * dt;

  // Forces along body forward + drag + gravity + lift correction.
  const cp = Math.cos(state.pitch);
  _fwd.set(
    Math.sin(state.yaw) * cp,
    Math.sin(state.pitch),
    Math.cos(state.yaw) * cp,
  );
  _acc.copy(_fwd).multiplyScalar(state.throttle * spec.thrustAccel);
  const sinPitch = Math.sin(state.pitch);
  _acc.addScaledVector(state.vel, -(spec.dragK + spec.inducedK * sinPitch * sinPitch) * speed);
  _acc.y -= 9.81;
  // Lift: straight up, proportional to dynamic pressure. At cruise q the
  // lift roughly balances weight (holds altitude hands-off); below stall
  // speed it collapses and the jet sinks (the stall sink).
  _acc.y += spec.liftK * q;
  state.vel.addScaledVector(_acc, dt);

  // Hard speed cap (dive): drag wall, no energy creation.
  const v2 = state.vel.length();
  if (v2 > spec.maxSpeed * 1.25) state.vel.multiplyScalar((spec.maxSpeed * 1.25) / v2);

  state.pos.addScaledVector(state.vel, dt);

  // Terrain strike: lethal, deterministic.
  const ground = heightAt(state.pos.x, state.pos.z) + spec.minClearanceM;
  if (state.pos.y <= ground) {
    state.pos.y = ground;
    state.crashed = true;
    state._evt |= FLIGHT_EVT_CRASHED;
    return;
  }

  // Weapon gating counters.
  if (state.gunReloadT > 0) state.gunReloadT -= dt;
  if (input.fireGun && state.gunReloadT <= 0) {
    state.gunReloadT = spec.gunReloadS;
    state._evt |= FLIGHT_EVT_GUN_FIRED;
  }
  if (state.agmReloadT > 0) {
    state.agmReloadT -= dt;
    if (state.agmReloadT <= 0 && state.agmLeft < spec.agmCount) state.agmLeft += 1;
  }
  if (input.fireAgm && state.agmLeft > 0 && state.agmReloadT <= 0) {
    state.agmLeft -= 1;
    state.agmReloadT = spec.agmReloadS;
    state._evt |= FLIGHT_EVT_AGM_FIRED;
  }
}

/**
 * F-35-class multirole jet, tuned so tanks keep the upper hand in a
 * straight fight (see doctrine above): fast, fragile, ammo-starved.
 * Display naming follows the fleet's real-vehicle convention.
 */
export const F35_LIGHTNING: FlightSpec = {
  thrustAccel: 22,
  dragK: 0.00062,
  inducedK: 0.0022,
  maxSpeed: 188, // sqrt(22 / 0.00062) ≈ 188 m/s level top speed
  // Gameplay stall (above the real ~60 m/s): the mushing-descent
  // equilibrium sits in the 70s, so slow flight must already be the
  // danger zone — speed is life, no runway landings in this doctrine.
  stallSpeed: 80,
  maxPitchRate: 0.9,
  maxRollRate: 2.4,
  maxYawRate: 0.25,
  maxG: 7,
  liftK: 20,
  ceilingM: 900,
  minClearanceM: 3,
  gunDamage: 12, // chip vs MBT armor; threat vs light vehicles/rear
  gunReloadS: 0.12,
  agmCount: 2,
  agmReloadS: 40,
  hp: 350, // one clean MBT hit cripples; two kill
};

export const FLIGHT_DT = 1 / 60;
