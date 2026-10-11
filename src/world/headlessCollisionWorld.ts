import { Vector3 } from 'three';
import { createObstacleGrid, rayCollisionRecord } from './collision.ts';
import type { CollisionRecord } from './collision.ts';
import type { HeightField } from './terrain.ts';

export type PackedSimpleShape =
  | readonly ['o', number, number, number, number, number]
  | readonly ['c', number, number, number]
  | readonly ['v', ...number[]];

type PackedShape = PackedSimpleShape | readonly ['m', ...PackedSimpleShape[]];

export interface PackedCollisionRecord {
  b: readonly [number, number, number, number, number, number];
  s?: PackedShape;
  q?: boolean | 0 | 1;
  m?: number | null;
  e?: number | null;
  k?: string | null;
  t?: number | null;
  p?: number | null;
}

interface ConcealmentRecord {
  x: number;
  z: number;
  r: number;
  add: number;
}

export interface CollisionManifest {
  obstacles: PackedCollisionRecord[];
  colliders: PackedCollisionRecord[];
  concealers?: Array<readonly [number, number, number, number]>;
}

interface HeadlessCollisionWorldOptions {
  mapId?: string;
  heightField?: HeightField;
  manifest?: CollisionManifest;
}

export interface HeadlessRayHit {
  point: Vector3;
  normal: Vector3;
  dist: number;
  kind: 'terrain' | 'prop';
  record: CollisionRecord | null;
}

/** Caller-owned raycast result. `raycastInto` writes point/normal in place. */
export interface HeadlessRayHitOut {
  point: Vector3;
  normal: Vector3;
  dist: number;
  kind: 'terrain' | 'prop';
  record: CollisionRecord | null;
}

export interface HeadlessCollisionWorld {
  mapId?: string;
  heightField: HeightField;
  raycast(origin: Vector3, direction: Vector3, maxDistance: number): HeadlessRayHit | null;
  /**
   * Allocation-free raycast. Writes into caller-owned `out` and returns it,
   * or null on a miss (`out` untouched). The same `out` must not be retained
   * across a second `raycastInto` call — consume it synchronously.
   */
  raycastInto(
    origin: Vector3, direction: Vector3, maxDistance: number, out: HeadlessRayHitOut,
  ): HeadlessRayHitOut | null;
  /** Allocation-free miss test: hit distance, or -1 on a miss. */
  raycastDistance(origin: Vector3, direction: Vector3, maxDistance: number): number;
  getObstacles(): CollisionRecord[];
  getColliders(): CollisionRecord[];
  getConcealment(): ConcealmentRecord[];
  queryObstacles(
    minX: number, minZ: number, maxX: number, maxZ: number, out: CollisionRecord[],
  ): CollisionRecord[];
  crushObstacle(obstacle: CollisionRecord | null | undefined): boolean;
}

function unpackRecord(packed: PackedCollisionRecord): CollisionRecord {
  const bounds = packed.b;
  const record: CollisionRecord = {
    min: [bounds[0], bounds[1], bounds[2]],
    max: [bounds[3], bounds[4], bounds[5]],
  };
  const shape = packed.s;
  const unpackSimpleShape = (value: PackedSimpleShape) => {
    if (value[0] === 'o') return {
      kind: 'obb' as const, cx: value[1], cz: value[2], hw: value[3], hl: value[4], yaw: value[5],
    };
    if (value[0] === 'c') return {
      kind: 'circle' as const, cx: value[1], cz: value[2], r: value[3],
    };
    const points = value.slice(1) as number[];
    let cx = 0;
    let cz = 0;
    for (let index = 0; index < points.length; index += 2) {
      cx += points[index];
      cz += points[index + 1];
    }
    const count = Math.max(1, points.length / 2);
    return { kind: 'convex' as const, cx: cx / count, cz: cz / count, points };
  };
  if (shape?.[0] === 'm') {
    const parts = shape.slice(1).map((part) => unpackSimpleShape(part as PackedSimpleShape));
    record.shape2 = {
      kind: 'compound',
      cx: (bounds[0] + bounds[3]) * 0.5,
      cz: (bounds[2] + bounds[5]) * 0.5,
      parts,
    };
  } else if (shape) {
    record.shape2 = unpackSimpleShape(shape);
  }
  if (packed.q) record.crushable = true;
  if (packed.m != null) record.crushMin = packed.m;
  if (packed.e != null) record.crushKeep = packed.e;
  if (packed.k != null) record.kind = packed.k;
  if (record.kind === 'tree') {
    record.crushMin = 0;
    record.crushKeep = 1;
  }
  if (packed.t != null) record.treeIdx = packed.t;
  if (packed.p != null) record.propIdx = packed.p;
  return record;
}

/** Inflate one captured visual-world manifest into a match-local facade. */
export function createHeadlessCollisionWorld(
  { mapId, heightField, manifest }: HeadlessCollisionWorldOptions = {},
): HeadlessCollisionWorld {
  if (!heightField || typeof heightField.getHeightAt !== 'function') {
    throw new TypeError('heightField is required');
  }
  if (!manifest || !Array.isArray(manifest.obstacles) || !Array.isArray(manifest.colliders)) {
    throw new TypeError('collision manifest is required');
  }
  const worldHeightField = heightField;
  const obstacles = manifest.obstacles.map(unpackRecord);
  const colliders = manifest.colliders.map(unpackRecord);
  const packedTreeIds = new Set(
    colliders.filter((record) => record.treeIdx != null).map((record) => record.treeIdx),
  );
  for (const obstacle of obstacles) {
    if (obstacle.treeIdx != null && !packedTreeIds.has(obstacle.treeIdx)) {
      colliders.push(obstacle);
    }
  }
  const concealers = (manifest.concealers || []).map(([x, z, r, add]) => ({ x, z, r, add }));
  const queryObstacles = createObstacleGrid(obstacles);
  const queryColliders = createObstacleGrid(colliders);
  const candidates: CollisionRecord[] = [];
  const point = new Vector3();
  const bisectPoint = new Vector3();
  const hitNormal = new Vector3();
  const bestNormal = new Vector3();
  const fastHeightAt = worldHeightField.getHeightAtFast || worldHeightField.getHeightAt;

  function nearestColliderHit(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
  ): { distance: number; record: CollisionRecord | null } {
    let bestDistance = Infinity;
    let bestRecord: CollisionRecord | null = null;
    const endX = origin.x + direction.x * maxDistance;
    const endZ = origin.z + direction.z * maxDistance;
    queryColliders(
      Math.min(origin.x, endX), Math.min(origin.z, endZ),
      Math.max(origin.x, endX), Math.max(origin.z, endZ),
      candidates,
    );
    for (const collider of candidates) {
      if (collider.dead) continue;
      const distance = rayCollisionRecord(
        origin, direction, collider, Math.min(maxDistance, bestDistance), hitNormal,
      );
      if (distance >= 0 && distance < bestDistance) {
        bestDistance = distance;
        bestRecord = collider;
        bestNormal.copy(hitNormal);
      }
    }
    return { distance: bestDistance, record: bestRecord };
  }

  function terrainHitDistance(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
  ): number {
    const refineHit = (lowDistance: number, highDistance: number): number => {
      let low = lowDistance;
      let high = highDistance;
      for (let index = 0; index < 6; index++) {
        const mid = (low + high) * 0.5;
        bisectPoint.copy(direction).multiplyScalar(mid).add(origin);
        if (bisectPoint.y - fastHeightAt(bisectPoint.x, bisectPoint.z) <= 0) high = mid;
        else low = mid;
      }
      return (low + high) * 0.5;
    };
    let distance = 0;
    let clearance = origin.y - fastHeightAt(origin.x, origin.z);
    if (clearance <= 0) return 0;
    while (distance < maxDistance) {
      const step = Math.min(Math.max(clearance * 0.5, 0.5), 2);
      const priorDistance = distance;
      distance = Math.min(distance + step, maxDistance);
      point.copy(direction).multiplyScalar(distance).add(origin);
      if (direction.y > 0 && point.y > worldHeightField.maxY + 2) return -1;
      clearance = point.y - fastHeightAt(point.x, point.z);
      if (clearance <= 0) return refineHit(priorDistance, distance);
      if (distance >= maxDistance) return -1;
    }
    return -1;
  }

  function raycast(origin: Vector3, direction: Vector3, maxDistance: number): HeadlessRayHit | null {
    return raycastInto(origin, direction, maxDistance, {
      point: new Vector3(),
      normal: new Vector3(),
      dist: 0,
      kind: 'terrain',
      record: null,
    });
  }

  function raycastInto(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    out: HeadlessRayHitOut,
  ): HeadlessRayHitOut | null {
    const propHit = nearestColliderHit(origin, direction, maxDistance);
    const terrainDistance = terrainHitDistance(
      origin,
      direction,
      Math.min(maxDistance, propHit.distance),
    );

    let hitDistance: number;
    let kind: 'terrain' | 'prop';
    if (terrainDistance >= 0 && terrainDistance < propHit.distance) {
      hitDistance = terrainDistance;
      kind = 'terrain';
    } else if (propHit.record && propHit.distance <= maxDistance) {
      hitDistance = propHit.distance;
      kind = 'prop';
    } else {
      return null;
    }
    out.point.copy(direction).multiplyScalar(hitDistance).add(origin);
    if (kind === 'terrain') {
      const n = worldHeightField.getNormalAt(out.point.x, out.point.z);
      out.normal.set(n.x, n.y, n.z);
    } else {
      out.normal.copy(bestNormal);
    }
    out.dist = hitDistance;
    out.kind = kind;
    out.record = kind === 'prop' ? propHit.record : null;
    return out;
  }

  function raycastDistance(origin: Vector3, direction: Vector3, maxDistance: number): number {
    const propHit = nearestColliderHit(origin, direction, maxDistance);
    const terrainDistance = terrainHitDistance(
      origin,
      direction,
      Math.min(maxDistance, propHit.distance),
    );
    if (terrainDistance >= 0 && terrainDistance < propHit.distance) return terrainDistance;
    if (propHit.record && propHit.distance <= maxDistance) return propHit.distance;
    return -1;
  }

  function crushObstacle(obstacle: CollisionRecord | null | undefined) {
    if (!obstacle || obstacle.crushed) return false;
    obstacle.crushed = true;
    if (obstacle.propIdx != null || obstacle.treeIdx != null) {
      const sameRecord = (record: CollisionRecord): boolean => (
        obstacle.propIdx != null
          ? record.propIdx === obstacle.propIdx
          : record.treeIdx === obstacle.treeIdx
      );
      for (const record of obstacles) {
        if (sameRecord(record)) record.crushed = true;
      }
      for (const record of colliders) {
        if (sameRecord(record)) {
          record.crushed = true;
          record.dead = true;
        }
      }
    }
    return true;
  }

  return {
    mapId,
    heightField: worldHeightField,
    raycast,
    raycastInto,
    raycastDistance,
    getObstacles: () => obstacles,
    getColliders: () => colliders,
    getConcealment: () => concealers,
    queryObstacles,
    crushObstacle,
  };
}
