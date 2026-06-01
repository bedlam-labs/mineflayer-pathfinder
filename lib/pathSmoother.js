'use strict'

const Vec3 = require('vec3').Vec3

const AGENT_HEIGHT = 1.8

class SmoothedPoint extends Vec3 {
  constructor (x, y, z) {
    super(x, y, z)
    this.toBreak = []
    this.toPlace = []
    this.parkour = false
  }
}

class PathSmoother {
  constructor (bot, options = {}) {
    this.bot = bot
    this.cornerRadius = options.cornerRadius ?? 2.0
    this.samplesPerCorner = options.samplesPerCorner ?? 4
  }

  smooth (rawPath) {
    if (!this.bot.pathfinder.useSplineSmoothing || rawPath.length < 3) return rawPath
    return this._processPath(rawPath)
  }

  // Split into pure-movement runs and fixed action nodes (dig/place/parkour).
  _processPath (path) {
    const result = []
    let runStart = 0

    for (let i = 0; i <= path.length; i++) {
      const isEnd = i === path.length
      const node = isEnd ? null : path[i]
      const isFixed = !isEnd && (node.toBreak.length > 0 || node.toPlace.length > 0 || node.parkour)

      if (isFixed || isEnd) {
        const run = path.slice(runStart, i)
        result.push(...(run.length >= 3 ? this._smoothRun(run) : run))
        if (!isEnd) {
          result.push(node)
          runStart = i + 1
        }
      }
    }

    return result
  }

  _smoothRun (waypoints) {
    const pruned = this._stringPull(waypoints)
    if (pruned.length < 3) return pruned
    return this._roundCorners(pruned)
  }

  // At each interior waypoint, insert a Bézier arc of radius `cornerRadius`.
  //
  // For a corner C with incoming direction dIn and outgoing direction dOut:
  //   hi = C - dIn * r          (handle-in:  last point on the straight approach)
  //   ho = C + dOut * r         (handle-out: first point on the straight exit)
  //   p1 = hi + dIn * chord/3   (Bézier control: continues incoming tangent)
  //   p2 = ho - dOut * chord/3  (Bézier control: prepares outgoing tangent)
  //
  // Control points lie along the incoming/outgoing straight segments, so the
  // convex hull of {hi, p1, p2, ho} is always within the navigable floor area.
  // C¹ continuity holds at hi and ho: tangent direction matches the straight path.
  _roundCorners (waypoints) {
    const n = waypoints.length
    const result = [new SmoothedPoint(waypoints[0].x, waypoints[0].y, waypoints[0].z)]
    let prevHo = waypoints[0]

    for (let i = 1; i < n - 1; i++) {
      const C = waypoints[i]
      const B = waypoints[i + 1]

      const dIn  = this._normalize(C.x - prevHo.x, C.y - prevHo.y, C.z - prevHo.z)
      const dOut = this._normalize(B.x - C.x,      B.y - C.y,      B.z - C.z)

      const r = Math.min(this.cornerRadius, this._dist(prevHo, C) / 2, this._dist(C, B) / 2)

      const hi = new Vec3(C.x - dIn.x * r,  C.y - dIn.y * r,  C.z - dIn.z * r)
      const ho = new Vec3(C.x + dOut.x * r, C.y + dOut.y * r, C.z + dOut.z * r)

      const tangentLen = this._dist(hi, ho) / 3
      const p1 = new Vec3(hi.x + dIn.x  * tangentLen, hi.y + dIn.y  * tangentLen, hi.z + dIn.z  * tangentLen)
      const p2 = new Vec3(ho.x - dOut.x * tangentLen, ho.y - dOut.y * tangentLen, ho.z - dOut.z * tangentLen)

      result.push(new SmoothedPoint(hi.x, hi.y, hi.z))
      for (let s = 1; s <= this.samplesPerCorner; s++) {
        const pt = this._evalCubicBezier(hi, p1, p2, ho, s / this.samplesPerCorner)
        result.push(new SmoothedPoint(pt.x, pt.y, pt.z))
      }

      prevHo = ho
    }

    result.push(new SmoothedPoint(waypoints[n - 1].x, waypoints[n - 1].y, waypoints[n - 1].z))
    return result
  }

  // DDA raycast: floor must be solid and agent-height air must be clear at each step.
  // Only pulls same-Y nodes to avoid shortcutting across jumps/drops.
  _hasLineOfSight (from, to) {
    if (Math.abs(to.y - from.y) > 0.01) return false

    const dx = to.x - from.x
    const dz = to.z - from.z
    const steps = Math.max(Math.ceil(Math.sqrt(dx * dx + dz * dz) * 4), 2)
    const floorY = Math.floor(from.y - 1)

    for (let s = 1; s < steps; s++) {
      const t = s / steps
      const bx = Math.floor(from.x + dx * t)
      const bz = Math.floor(from.z + dz * t)

      const floor = this.bot.blockAt(new Vec3(bx, floorY, bz))
      if (!floor || floor.boundingBox !== 'block') return false

      for (const checkY of [Math.floor(from.y), Math.floor(from.y + AGENT_HEIGHT - 0.2)]) {
        const block = this.bot.blockAt(new Vec3(bx, checkY, bz))
        if (block && block.boundingBox === 'block') return false
      }
    }
    return true
  }

  // Greedy string pulling: from each anchor, reach the furthest visible same-Y node.
  _stringPull (path) {
    if (path.length <= 2) return [...path]

    const result = [path[0]]
    let anchor = 0

    while (anchor < path.length - 1) {
      let reach = path.length - 1
      while (reach > anchor + 1 && !this._hasLineOfSight(path[anchor], path[reach])) {
        reach--
      }
      result.push(path[reach])
      anchor = reach
    }

    return result
  }

  _normalize (dx, dy, dz) {
    const len = Math.sqrt(dx * dx + dy * dy + dz * dz)
    if (len < 0.001) return new Vec3(0, 0, 0)
    return new Vec3(dx / len, dy / len, dz / len)
  }

  _dist (a, b) {
    const dx = b.x - a.x
    const dy = b.y - a.y
    const dz = b.z - a.z
    return Math.sqrt(dx * dx + dy * dy + dz * dz)
  }

  _evalCubicBezier (p0, p1, p2, p3, t) {
    const u = 1 - t
    const uu = u * u
    const tt = t * t
    return new Vec3(
      uu * u * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + tt * t * p3.x,
      uu * u * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + tt * t * p3.y,
      uu * u * p0.z + 3 * uu * t * p1.z + 3 * u * tt * p2.z + tt * t * p3.z
    )
  }
}

module.exports = { PathSmoother }
