/**
 * AUBO i5H (aubo_i5H_10) kinematics from its URDF: forward kinematics for every
 * link frame, the TCP pose as the arm bridge reports it, and a damped-least-
 * squares IK. Plain TypeScript with no Lens Studio types (metres, radians,
 * row-major 3x3 rotations), so it can be tested outside the editor.
 *
 * Frames and conventions match the station:
 * - Arm base frame = the URDF base_link (verified on the arm to < 1 mm / 0.5 deg).
 * - Joint angles from the controller drive the URDF directly, no offsets.
 * - The bridge's TCP is the flange (wrist3_Link) turned by the station's
 *   aubo.tcp_offset yaw (+90 deg), and its RPY is R = Rz(yaw) Ry(pitch) Rx(roll).
 */

export type Vec3 = [number, number, number]
/** Row-major 3x3 rotation. */
export type Mat3 = number[]

export interface Frame {
  R: Mat3
  p: Vec3
}

/** TCP pose as on the wire: mm and RPY degrees. */
export interface Pose {
  pos: Vec3
  rpy: Vec3
}

const DEG = Math.PI / 180

/** URDF joint origins (xyz m, rpy rad); every joint turns about its local +Z. */
export const JOINTS: {name: string, xyz: Vec3, rpy: Vec3}[] = [
  {name: "shoulder_joint", xyz: [0, 0, 0.122], rpy: [0, 0, Math.PI]},
  {name: "upperArm_joint", xyz: [0, 0.1215, 0], rpy: [-Math.PI / 2, -Math.PI / 2, 0]},
  {name: "foreArm_joint", xyz: [0.408, 0, 0], rpy: [-Math.PI, 0, 0]},
  {name: "wrist1_joint", xyz: [0.376, 0, 0], rpy: [Math.PI, 0, Math.PI / 2]},
  {name: "wrist2_joint", xyz: [0, 0.1025, 0], rpy: [-Math.PI / 2, 0, 0]},
  {name: "wrist3_joint", xyz: [0, -0.094, 0], rpy: [Math.PI / 2, 0, 0]}
]

/**
 * Bridge-owned joint limits (config/arm_bridge.yaml aubo.joint_min/max_deg):
 * the bridge holds a target outside these, so the twin never asks for one.
 */
export const JOINT_MIN_DEG = [-355, -160, -150, -355, -355, -355]
export const JOINT_MAX_DEG = [355, 160, 150, 355, 355, 355]
/** The bridge's elbow guard (aubo.elbow_min_deg): |J3| below this is held. */
export const ELBOW_MIN_DEG = 15

/** AUBO's ready pose (aubo-control-notes.md §9), degrees. */
export const READY_DEG = [0, -15, 100, 25, 90, 0]

// ---- small linear algebra ---------------------------------------------------
export function matMul(a: Mat3, b: Mat3): Mat3 {
  const r = new Array(9)
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j]
    }
  }
  return r
}

export function matVec(a: Mat3, v: Vec3): Vec3 {
  return [
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
    a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2]
  ]
}

export function transpose(a: Mat3): Mat3 {
  return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]]
}

function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/** R = Rz(yaw) Ry(pitch) Rx(roll), radians (URDF rpy and the bridge's RPY). */
export function rpyToMat(roll: number, pitch: number, yaw: number): Mat3 {
  const cr = Math.cos(roll), sr = Math.sin(roll)
  const cp = Math.cos(pitch), sp = Math.sin(pitch)
  const cy = Math.cos(yaw), sy = Math.sin(yaw)
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr
  ]
}

/** Principal RPY (radians) of R = Rz Ry Rx. */
export function matToRpy(m: Mat3): Vec3 {
  const r20 = Math.max(-1, Math.min(1, m[6]))
  const pitch = Math.asin(-r20)
  if (Math.abs(r20) > 1 - 1e-9) {     // gimbal lock: put it all in yaw
    return [0, pitch, Math.atan2(-m[1], m[4])]
  }
  return [Math.atan2(m[7], m[8]), pitch, Math.atan2(m[3], m[0])]
}

function rotZ(a: number): Mat3 {
  const c = Math.cos(a), s = Math.sin(a)
  return [c, -s, 0, s, c, 0, 0, 0, 1]
}

/** Rotation vector (axis * angle, radians) of R. */
export function matLog(m: Mat3): Vec3 {
  const c = Math.max(-1, Math.min(1, (m[0] + m[4] + m[8] - 1) / 2))
  const angle = Math.acos(c)
  const v: Vec3 = [m[7] - m[5], m[2] - m[6], m[3] - m[1]]
  if (angle < 1e-9) {
    return [v[0] / 2, v[1] / 2, v[2] / 2]
  }
  if (Math.PI - angle < 1e-6) {
    // Near 180 deg: axis from the diagonal, signs from the off-diagonals.
    let x = Math.sqrt(Math.max(0, (m[0] + 1) / 2))
    let y = Math.sqrt(Math.max(0, (m[4] + 1) / 2))
    let z = Math.sqrt(Math.max(0, (m[8] + 1) / 2))
    if (x >= y && x >= z) {
      y = m[1] + m[3] < 0 ? -y : y
      z = m[2] + m[6] < 0 ? -z : z
    } else if (y >= z) {
      x = m[1] + m[3] < 0 ? -x : x
      z = m[5] + m[7] < 0 ? -z : z
    } else {
      x = m[2] + m[6] < 0 ? -x : x
      y = m[5] + m[7] < 0 ? -y : y
    }
    return [x * angle, y * angle, z * angle]
  }
  const k = angle / (2 * Math.sin(angle))
  return [v[0] * k, v[1] * k, v[2] * k]
}

// ---- kinematics -----------------------------------------------------------------
const JOINT_ORIGINS: Mat3[] = JOINTS.map((j) => rpyToMat(j.rpy[0], j.rpy[1], j.rpy[2]))

/** Frames of base_link (index 0) and the six moving links (1..6), base frame, metres. */
export function linkFrames(qRad: number[]): Frame[] {
  const frames: Frame[] = [{R: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [0, 0, 0]}]
  let R: Mat3 = frames[0].R
  let p: Vec3 = frames[0].p
  for (let i = 0; i < 6; i++) {
    p = add(p, matVec(R, JOINTS[i].xyz))
    R = matMul(matMul(R, JOINT_ORIGINS[i]), rotZ(qRad[i]))
    frames.push({R: R, p: p})
  }
  return frames
}

/** The TCP frame: the flange turned by `tcpYawDeg` about its own Z. */
export function tcpFrame(qRad: number[], tcpYawDeg: number): Frame {
  const f = linkFrames(qRad)[6]
  return {R: matMul(f.R, rotZ(tcpYawDeg * DEG)), p: f.p}
}

/** The TCP pose as the bridge reports it (mm, RPY degrees). */
export function tcpPose(qRad: number[], tcpYawDeg: number): Pose {
  const f = tcpFrame(qRad, tcpYawDeg)
  const rpy = matToRpy(f.R)
  return {
    pos: [f.p[0] * 1000, f.p[1] * 1000, f.p[2] * 1000],
    rpy: [rpy[0] / DEG, rpy[1] / DEG, rpy[2] / DEG]
  }
}

export interface IkResult {
  q: number[]
  ok: boolean
  posErrMm: number
  oriErrDeg: number
}

/**
 * DLS IK for a TCP pose (mm, RPY deg), seeded from `seedRad` so the solution
 * stays on the seed's branch (the bridge seeds its own IK the same way).
 * `ok` = converged within 0.5 mm / 0.5 deg.
 */
export function solveIk(target: Pose, tcpYawDeg: number, seedRad: number[],
                        iterations: number = 60): IkResult {
  const pT: Vec3 = [target.pos[0] / 1000, target.pos[1] / 1000, target.pos[2] / 1000]
  const RT = matMul(rpyToMat(target.rpy[0] * DEG, target.rpy[1] * DEG, target.rpy[2] * DEG),
                    rotZ(-tcpYawDeg * DEG))            // target flange orientation
  const q = seedRad.slice()
  const lambda = 0.02
  let posErr = Infinity
  let oriErr = Infinity
  for (let it = 0; it < iterations; it++) {
    const frames = linkFrames(q)
    const flange = frames[6]
    const ep = sub(pT, flange.p)
    const eo = matLog(matMul(RT, transpose(flange.R)))
    posErr = Math.hypot(ep[0], ep[1], ep[2]) * 1000
    oriErr = Math.hypot(eo[0], eo[1], eo[2]) / DEG
    if (posErr < 0.05 && oriErr < 0.05) {
      break
    }
    // Geometric Jacobian (6x6): rows = [linear; angular], column i = joint i.
    const J: number[][] = [[], [], [], [], [], []]
    for (let i = 0; i < 6; i++) {
      const z: Vec3 = [frames[i + 1].R[2], frames[i + 1].R[5], frames[i + 1].R[8]]
      const lin = cross(z, sub(flange.p, frames[i + 1].p))
      J[0][i] = lin[0]; J[1][i] = lin[1]; J[2][i] = lin[2]
      J[3][i] = z[0]; J[4][i] = z[1]; J[5][i] = z[2]
    }
    const e = [ep[0], ep[1], ep[2], eo[0] * 0.5, eo[1] * 0.5, eo[2] * 0.5]
    for (let r = 3; r < 6; r++) {                    // weight orientation like the error
      for (let c = 0; c < 6; c++) J[r][c] *= 0.5
    }
    // dq = J^T (J J^T + lambda^2 I)^-1 e
    const A: number[][] = []
    for (let r = 0; r < 6; r++) {
      A.push([])
      for (let c = 0; c < 6; c++) {
        let s = 0
        for (let k = 0; k < 6; k++) s += J[r][k] * J[c][k]
        A[r].push(s + (r === c ? lambda * lambda : 0))
      }
    }
    const y = solve6(A, e)
    if (y === null) {
      break
    }
    for (let i = 0; i < 6; i++) {
      let dq = 0
      for (let r = 0; r < 6; r++) dq += J[r][i] * y[r]
      q[i] += Math.max(-0.2, Math.min(0.2, dq))     // at most ~11 deg per iteration
    }
  }
  return {q: q, ok: posErr < 0.5 && oriErr < 0.5, posErrMm: posErr, oriErrDeg: oriErr}
}

/** Gaussian elimination with partial pivoting for a 6x6 system. */
function solve6(A: number[][], b: number[]): number[] | null {
  const n = 6
  const M = A.map((row, i) => row.concat([b[i]]))
  for (let col = 0; col < n; col++) {
    let piv = col
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r
    }
    if (Math.abs(M[piv][col]) < 1e-12) {
      return null
    }
    const tmp = M[col]; M[col] = M[piv]; M[piv] = tmp
    for (let r = col + 1; r < n; r++) {
      const f = M[r][col] / M[col][col]
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c]
    }
  }
  const x = new Array(n).fill(0)
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n]
    for (let c = r + 1; c < n; c++) s -= M[r][c] * x[c]
    x[r] = s / M[r][r]
  }
  return x
}

/** Clamp joints (radians) into the bridge's limits and outside its elbow guard. */
export function clampToBridge(qRad: number[]): number[] {
  const out = qRad.map((v, i) => Math.max(JOINT_MIN_DEG[i] * DEG, Math.min(JOINT_MAX_DEG[i] * DEG, v)))
  const elbow = ELBOW_MIN_DEG * DEG
  if (Math.abs(out[2]) < elbow) {
    out[2] = out[2] < 0 ? -elbow : elbow
  }
  return out
}
