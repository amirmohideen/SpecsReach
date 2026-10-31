import {SIK} from "SpectaclesInteractionKit.lspkg/SIK"
import {BaseHand} from "SpectaclesInteractionKit.lspkg/Providers/HandInputData/BaseHand"
import {HandType} from "SpectaclesInteractionKit.lspkg/Providers/HandInputData/HandType"
import WorldCameraFinderProvider from "SpectaclesInteractionKit.lspkg/Providers/CameraProvider/WorldCameraFinderProvider"
import {Interactable} from "SpectaclesInteractionKit.lspkg/Components/Interaction/Interactable/Interactable"
import {InteractorEvent} from "SpectaclesInteractionKit.lspkg/Core/Interactor/InteractorEvent"
import {InteractorInputType} from "SpectaclesInteractionKit.lspkg/Core/Interactor/Interactor"
import {ArmTeleop, TwinSource} from "./ArmTeleop"
import * as K from "./AuboKinematics"

/**
 * Digital twin of the AUBO i5H: a 3D model of the arm that mirrors the real
 * arm, and that you pinch and drag to move the real arm.
 *
 * - Pinch the TOOL (the flange, link6) and drag: the tool tip moves with your
 *   hand (orientation kept); the arm's joints follow by IK.
 * - Pinch an ARM SEGMENT (shoulder, upper arm, forearm, wrist parts) and drag
 *   around its joint: that joint turns.
 * - Pinch the BASE (link0) and drag: the model moves in the room; the real arm
 *   does not.
 * - Let go: the real arm stops where it is.
 *
 * The model always shows the REAL arm (its pose comes back from the relay at
 * 10 Hz); a see-through ghost shows where you are dragging it, when a Ghost
 * Material is set. The target goes to the relay through ArmTeleop's
 * connection (the relay takes one Lens connection), as a `twin` message; the
 * relay moves the arm by the target's motion since the grab, with its usual
 * limits, and the bridge's safety on top.
 *
 * The rig is built in code from the URDF (AuboKinematics.ts): each link mesh
 * is placed at its link frame every frame. The meshes are the converted GLBs
 * (link0..link6, metres, URDF link frames); how Lens Studio scaled or rotated
 * them on import is detected from their bounds.
 */

const DEG = Math.PI / 180
/**
 * Arm base (Z up, metres) -> model space (Lens axes, Y up): arm X -> -Z (away
 * from you when the model appears), arm Y -> -X (your left), arm Z -> +Y.
 * A proper rotation, like the relay's view-mode mapping at view_yaw_deg 0.
 */
const C: K.Mat3 = [0, -1, 0, 0, 0, 1, -1, 0, 0]
const CT: K.Mat3 = K.transpose(C)
const IDENTITY: K.Mat3 = [1, 0, 0, 0, 1, 0, 0, 0, 1]
const ROT_X_POS90: K.Mat3 = [1, 0, 0, 0, 0, -1, 0, 1, 0]
const ROT_X_NEG90: K.Mat3 = [1, 0, 0, 0, 0, 1, 0, -1, 0]

/** Expected link2 (upper arm) X extent and link3 (forearm) bounds centre, metres (URDF meshes). */
const LINK2_X_EXTENT = 0.526
const TOOL_LINK = 6
const BASE_LINK = 0
const RELEASE_GRACE_S = 0.1   // the grabbing hand open this long = let go
const LINK_NAMES = ["base", "shoulder", "upper arm", "forearm", "wrist 1", "wrist 2", "tool"]

function matToQuat(m: K.Mat3): quat {
  const tr = m[0] + m[4] + m[8]
  let w: number, x: number, y: number, z: number
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2
    w = 0.25 * s; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2
    w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s
  } else if (m[4] > m[8]) {
    const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2
    w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s
  } else {
    const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2
    w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s
  }
  return new quat(w, x, y, z)
}

function toVec(a: K.Vec3): vec3 {
  return new vec3(a[0], a[1], a[2])
}

interface Drag {
  link: number
  by: InteractorInputType
  interactor: any
  pinch0: vec3          // hand (or pointer) position at the grab, world
  grab0: vec3           // grabbed point on the model, world
  q0: number[]          // joints at the grab (the model as shown)
  modelPos0: vec3
}

@component
export class ArmTwin extends BaseScriptComponent implements TwinSource {
  @input
  @hint("The ArmTeleop component: the twin sends through its relay connection.")
  teleop: ArmTeleop

  @input
  @hint("The 7 imported link meshes in order: link0 (base) ... link6 (flange), from ModelImport/link0.glb ... link6.glb.")
  linkMeshes: RenderMesh[]

  @input
  @hint("Material for the model (e.g. a grey PBR or Unlit material).")
  armMaterial: Material

  @input
  @allowUndefined
  @hint("Optional per-part materials, same order as Link Meshes (element 0 = link0, the base ... 6 = the flange). An empty slot, or a shorter list, uses Arm Material for that part.")
  linkMaterials: Material[] = []

  @input
  @allowUndefined
  @hint("Optional see-through material for the ghost that shows where you are dragging the arm. Leave empty for no ghost.")
  ghostMaterial: Material

  @input
  @allowUndefined
  @hint("Optional: SIK's outline material (Packages > SpectaclesInteractionKit > Assets > Materials > TargetOutlineMaterial). Outlines the part your hand is on, like Snap's own InteractableOutlineFeedback.")
  outlineMaterial: Material

  @input("vec4", "{1, 1, 0.04, 1}")
  @widget(new ColorWidget())
  @hint("Outline colour while a hand hovers a part (SIK's default yellow).")
  hoverColor: vec4 = new vec4(1, 1, 0.04, 1)

  @input("vec4", "{1, 1, 1, 1}")
  @widget(new ColorWidget())
  @hint("Outline colour while a part is pinched (SIK's default white).")
  grabColor: vec4 = new vec4(1, 1, 1, 1)

  @input
  @hint("Outline thickness in cm (SIK's default 0.25). Change it if the outline looks too thick or thin.")
  outlineWeight: number = 0.25

  @input
  @allowUndefined
  @hint("Optional: a material the hovered / pinched part switches to (e.g. a brighter copy of the Arm Material). Can be used with or without the outline.")
  hoverMaterial: Material

  @input
  @hint("Model size relative to the real arm (0.4 = 40%). Bigger = easier to grab small parts.")
  modelScale: number = 0.4

  @input
  @hint("Must equal the yaw in the station's aubo.tcp_offset (config/arm_bridge.yaml): 90.")
  tcpYawDeg: number = 90

  @input
  @hint("Where the model appears when shown: distance in front of you (cm).")
  spawnDistanceCm: number = 55

  @input
  @hint("Where the model appears when shown: height of its base relative to your eyes (cm, negative = below).")
  spawnHeightCm: number = -40

  @input
  @allowUndefined
  @hint("Optional Text for twin hints and status.")
  twinText: Text

  private camera: WorldCameraFinderProvider | null = null
  private root: SceneObject | null = null
  private links: SceneObject[] = []
  private visuals: RenderMeshVisual[] = []
  private hoverBits: number[] = [0, 0, 0, 0, 0, 0, 0]   // hands hovering each part (InteractorInputType)
  private highlight: string[] = ["", "", "", "", "", "", ""]
  private outlineHover: Material | null = null
  private outlineGrab: Material | null = null
  private ghosts: SceneObject[] = []
  private built: boolean = false
  private buildNote: string = ""
  private meshFix: K.Mat3 = IDENTITY     // raw link coords = meshFix * imported coords
  private meshScale: number = 1          // Lens units per imported mesh unit, before modelScale

  private modelPos: vec3 = vec3.zero()
  private modelYaw: number = 0
  private qShown: number[] = K.READY_DEG.map((v) => v * DEG)
  private qActual: number[] | null = null
  private lastPoseKey: string = ""
  private qCmd: number[] = this.qShown.slice()
  private drag: Drag | null = null
  private openSince: number = -1        // dragging, but the grabbing hand stopped pinching at
  private dragNote: string = ""
  private visible: boolean = false
  private lastTextAt: number = 0

  onAwake() {
    this.createEvent("OnStartEvent").bind(() => {
      this.camera = WorldCameraFinderProvider.getInstance()
    })
    this.createEvent("UpdateEvent").bind(() => this.onUpdate())
  }

  // ---- TwinSource (ArmTeleop pulls this every send tick) ----------------
  /** Active while a moving part (not the base) is held; pose = the model's TCP target. */
  public twinMessage(): {active: boolean, pose: number[]} {
    const active = this.visible && this.drag !== null && this.drag.link !== BASE_LINK
    const q = active ? this.qCmd : (this.qActual !== null ? this.qActual : this.qShown)
    const p = K.tcpPose(q, this.tcpYawDeg)
    return {active: active, pose: p.pos.concat(p.rpy).map((v) => Math.round(v * 100) / 100)}
  }

  // ---- show / hide (ArmMenu) --------------------------------------------
  /** Show the model in front of you, mirroring the arm, and take over the relay link. */
  public show() {
    if (!this.built) {
      this.build()
    }
    if (this.root === null) {
      return
    }
    this.placeInFront()
    this.root.enabled = true
    this.visible = true
    if (this.teleop) {
      this.teleop.attachTwin(this)
    }
  }

  public hide() {
    this.endDrag("model hidden")
    this.hoverBits = this.hoverBits.map(() => 0)   // no hover-exit arrives for a hidden part
    this.visible = false
    if (this.root !== null) {
      this.root.enabled = false
    }
    if (this.teleop) {
      this.teleop.attachTwin(null)
    }
  }

  /** Put the model back in front of you (e.g. a Recenter button). */
  public recenter() {
    if (this.visible) {
      this.placeInFront()
    }
  }

  // ---- building the rig ---------------------------------------------------
  private build() {
    this.built = true
    if (!this.linkMeshes || this.linkMeshes.length < 7 || this.linkMeshes.some((m) => !m)) {
      this.buildNote = "set all 7 Link Meshes (link0..link6)"
      print(`[ArmTwin] ${this.buildNote}`)
      return
    }
    if (!this.armMaterial) {
      this.buildNote = "set the Arm Material"
      print(`[ArmTwin] ${this.buildNote}`)
      return
    }
    this.detectImport()
    if (this.outlineMaterial) {
      this.outlineHover = this.makeOutline(this.hoverColor)
      this.outlineGrab = this.makeOutline(this.grabColor)
    }
    const parent = this.getSceneObject()
    this.root = global.scene.createSceneObject("ArmTwinModel")
    this.root.setParent(parent)
    this.root.layer = parent.layer
    for (let i = 0; i < 7; i++) {
      this.links.push(this.makeLink(i, this.partMaterial(i), true))
      if (this.ghostMaterial) {
        const g = this.makeLink(i, this.ghostMaterial, false)
        g.enabled = false
        this.ghosts.push(g)
      }
    }
    this.root.enabled = false
    print(`[ArmTwin] built: mesh scale ${this.meshScale.toFixed(3)} ${this.buildNote}`)
  }

  private makeLink(i: number, material: Material, grabbable: boolean): SceneObject {
    const obj = global.scene.createSceneObject(`ArmTwin${grabbable ? "" : "Ghost"}_link${i}`)
    obj.setParent(this.root)
    obj.layer = this.root.layer
    const visual = obj.createComponent("Component.RenderMeshVisual") as RenderMeshVisual
    visual.mesh = this.linkMeshes[i]
    visual.mainMaterial = material
    if (grabbable) {
      this.visuals.push(visual)
      const collider = obj.createComponent("Physics.ColliderComponent") as ColliderComponent
      collider.shape = Shape.createBoxShape()
      collider.fitVisual = true
      const interactable = obj.createComponent(Interactable.getTypeName()) as Interactable
      interactable.onTriggerStart.add((e: InteractorEvent) => this.startDrag(i, e))
      interactable.onTriggerEnd.add((e: InteractorEvent) => this.onRelease(i))
      // A drag usually ends with the hand away from where the part was: SIK
      // reports that release as EndOutside, not End.
      interactable.onTriggerEndOutside.add((e: InteractorEvent) => this.onRelease(i))
      interactable.onTriggerCanceled.add((e: InteractorEvent) => this.onRelease(i))
      interactable.onHoverEnter.add((e: InteractorEvent) => {
        this.hoverBits[i] |= e && e.interactor ? e.interactor.inputType : 0
      })
      interactable.onHoverExit.add((e: InteractorEvent) => {
        this.hoverBits[i] &= ~(e && e.interactor ? e.interactor.inputType : 0)
      })
    }
    return obj
  }

  /** Part i's own material: its Link Materials slot if set, else Arm Material. */
  private partMaterial(i: number): Material {
    const m = this.linkMaterials && i < this.linkMaterials.length ? this.linkMaterials[i] : null
    return m ? m : this.armMaterial
  }

  private makeOutline(color: vec4): Material {
    const m = this.outlineMaterial.clone()
    m.mainPass.lineWeight = this.outlineWeight
    m.mainPass.lineColor = color
    return m
  }

  /**
   * Hover / pinch feedback, the way SIK's InteractableOutlineFeedback does it:
   * an outline material drawn as an extra pass in front of the part's own
   * material, plus (optionally) a swap to the hover material. The pinched part
   * keeps its highlight even when the hand moves off it.
   */
  private updateHighlights() {
    for (let i = 0; i < this.visuals.length; i++) {
      const state = this.drag !== null && this.drag.link === i ? "grab"
        : (this.hoverBits[i] !== 0 && this.drag === null ? "hover" : "none")
      if (state === this.highlight[i]) {
        continue
      }
      this.highlight[i] = state
      const v = this.visuals[i]
      const base = state !== "none" && this.hoverMaterial ? this.hoverMaterial : this.partMaterial(i)
      v.clearMaterials()
      if (state === "hover" && this.outlineHover !== null) {
        v.addMaterial(this.outlineHover)
      } else if (state === "grab" && this.outlineGrab !== null) {
        v.addMaterial(this.outlineGrab)
      }
      v.addMaterial(base)
    }
  }

  /**
   * How Lens Studio imported the GLBs: their units (metres kept as-is, or
   * scaled to cm) from the upper arm's length, and whether it turned them
   * Y-up from where the forearm's bulk sits (URDF: +Z of its link frame).
   */
  private detectImport() {
    const upper = this.linkMeshes[2]
    const xExtent = upper.aabbMax.x - upper.aabbMin.x
    const unit = xExtent / LINK2_X_EXTENT          // imported units per metre
    if (!(unit > 0.3 && unit < 3) && !(unit > 30 && unit < 300)) {
      this.buildNote = `(unexpected mesh size: upper arm ${xExtent.toFixed(3)} units)`
    }
    this.meshScale = 100 / (unit > 0 ? unit : 1)   // -> cm, before modelScale
    const fore = this.linkMeshes[3]
    const cy = (fore.aabbMin.y + fore.aabbMax.y) / 2
    const cz = (fore.aabbMin.z + fore.aabbMax.z) / 2
    if (Math.abs(cz) >= Math.abs(cy)) {
      this.meshFix = IDENTITY                      // coordinates kept as exported
    } else if (cy > 0) {
      this.meshFix = ROT_X_POS90                   // imported = Rx(-90) raw
      this.buildNote += " (meshes were turned Y-up on import: corrected)"
    } else {
      this.meshFix = ROT_X_NEG90
      this.buildNote += " (meshes were turned on import: corrected)"
    }
  }

  // ---- placing ------------------------------------------------------------
  private placeInFront() {
    if (this.camera === null) {
      this.camera = WorldCameraFinderProvider.getInstance()
    }
    const t = this.camera.getTransform()
    const look = t.forward.uniformScale(-1)
    let flat = new vec3(look.x, 0, look.z)
    flat = flat.length > 0.1 ? flat.normalize() : new vec3(0, 0, -1)
    this.modelPos = t.getWorldPosition().add(flat.uniformScale(this.spawnDistanceCm))
      .add(new vec3(0, this.spawnHeightCm, 0))
    // Arm +X (model -Z) points away from you, like standing behind the real arm.
    this.modelYaw = Math.atan2(-flat.x, -flat.z)
  }

  private modelRot(): quat {
    return quat.angleAxis(this.modelYaw, vec3.up())
  }

  private lensPerMetre(): number {
    return 100 * this.modelScale
  }

  /** Arm-base point (metres) -> world. */
  private toWorldPoint(p: K.Vec3): vec3 {
    return this.modelPos.add(this.modelRot().multiplyVec3(toVec(K.matVec(C, p))).uniformScale(this.lensPerMetre()))
  }

  /** Arm-base direction -> world. */
  private toWorldDir(d: K.Vec3): vec3 {
    return this.modelRot().multiplyVec3(toVec(K.matVec(C, d)))
  }

  /** World displacement -> arm-base displacement in mm. */
  private toArmMm(d: vec3): K.Vec3 {
    const m = this.modelRot().invert().multiplyVec3(d).uniformScale(1 / this.lensPerMetre())
    const a = K.matVec(CT, [m.x, m.y, m.z])
    return [a[0] * 1000, a[1] * 1000, a[2] * 1000]
  }

  private poseLinks(objs: SceneObject[], qRad: number[]) {
    const frames = K.linkFrames(qRad)
    const rot = this.modelRot()
    const scale = this.meshScale * this.modelScale
    for (let i = 0; i < objs.length; i++) {
      const t = objs[i].getTransform()
      t.setWorldPosition(this.toWorldPoint(frames[i].p))
      t.setWorldRotation(rot.multiply(matToQuat(K.matMul(K.matMul(C, frames[i].R), this.meshFix))))
      t.setWorldScale(new vec3(scale, scale, scale))
    }
  }

  // ---- dragging -------------------------------------------------------------
  private handFor(type: InteractorInputType): BaseHand | null {
    try {
      if (type & InteractorInputType.LeftHand) return SIK.HandInputData.getHand("left" as HandType)
      if (type & InteractorInputType.RightHand) return SIK.HandInputData.getHand("right" as HandType)
    } catch (e) {
      // no hand data
    }
    return null
  }

  /** Where the grabbing hand (or the editor pointer) is now; null if lost. */
  private pointerPos(drag: {by: InteractorInputType, interactor: any}): vec3 | null {
    const hand = this.handFor(drag.by)
    if (hand !== null) {
      if (!hand.isTracked()) return null
      return hand.thumbTip.position.add(hand.indexTip.position).uniformScale(0.5)
    }
    const it = drag.interactor
    if (it) {
      const p = it.planecastPoint || it.endPoint
      if (p) return p
    }
    return null
  }

  private startDrag(link: number, e: InteractorEvent) {
    if (!this.visible || this.drag !== null || !e || !e.interactor) {
      return
    }
    const by = e.interactor.inputType
    const probe = {by: by, interactor: e.interactor}
    const pinch = this.pointerPos(probe)
    if (pinch === null) {
      return
    }
    const hit = e.interactor.targetHitInfo
    const grab = hit && hit.hit ? hit.hit.position : pinch
    this.drag = {link: link, by: by, interactor: e.interactor, pinch0: pinch, grab0: grab,
                 q0: this.qShown.slice(), modelPos0: this.modelPos}
    this.openSince = -1
    this.qCmd = this.qShown.slice()
    this.dragNote = link === BASE_LINK ? "moving the model" : `moving the arm by its ${LINK_NAMES[link]}`
    print(`[ArmTwin] grab ${LINK_NAMES[link]}`)
  }

  private onRelease(link: number) {
    if (this.drag !== null && this.drag.link === link) {
      this.endDrag("")
    }
  }

  private endDrag(why: string) {
    if (this.drag === null) {
      return
    }
    this.drag = null
    this.dragNote = why
    for (const g of this.ghosts) {
      g.enabled = false
    }
  }

  private updateDrag() {
    const d = this.drag
    // Safety net on top of SIK's release events: the drag (and so the real
    // arm) ends when the grabbing hand stops pinching, whatever SIK reports.
    // A short grace rides out a single frame of pinch flicker.
    const hand = this.handFor(d.by)
    if (hand !== null) {
      if (hand.isPinching()) {
        this.openSince = -1
      } else if (this.openSince < 0) {
        this.openSince = getTime()
      } else if (getTime() - this.openSince > RELEASE_GRACE_S) {
        this.endDrag("")
        return
      }
    }
    const now = this.pointerPos(d)
    if (now === null) {
      this.endDrag("let go: hand lost")
      return
    }
    const delta = now.sub(d.pinch0)
    if (d.link === BASE_LINK) {
      this.modelPos = d.modelPos0.add(delta)
      return
    }
    if (d.link === TOOL_LINK) {
      // The tool tip follows the hand; its orientation is kept; IK finds the joints.
      const start = K.tcpPose(d.q0, this.tcpYawDeg)
      const dm = this.toArmMm(delta)
      const target: K.Pose = {pos: [start.pos[0] + dm[0], start.pos[1] + dm[1], start.pos[2] + dm[2]],
                              rpy: start.rpy}
      const ik = K.solveIk(target, this.tcpYawDeg, this.qCmd)
      if (ik.ok) {
        this.qCmd = K.clampToBridge(ik.q)
        this.dragNote = "moving the tool"
      } else {
        this.dragNote = "out of reach: the arm holds at the last reachable point"
      }
      return
    }
    // An arm segment: turn its joint by the angle the hand swept around the joint axis.
    const frame = K.linkFrames(d.q0)[d.link]
    const axis = this.toWorldDir([frame.R[2], frame.R[5], frame.R[8]]).normalize()
    const origin = this.toWorldPoint(frame.p)
    const v0 = d.grab0.sub(origin)
    const v1 = d.grab0.add(delta).sub(origin)
    const p0 = v0.sub(axis.uniformScale(v0.dot(axis)))
    const p1 = v1.sub(axis.uniformScale(v1.dot(axis)))
    if (p0.length < 0.5 || p1.length < 0.5) {
      this.dragNote = "grab further from the joint to turn it"
      return
    }
    const angle = Math.atan2(axis.dot(p0.cross(p1)), p0.dot(p1))
    const q = d.q0.slice()
    q[d.link - 1] += angle
    this.qCmd = K.clampToBridge(q)
    this.dragNote = `turning the ${LINK_NAMES[d.link]} joint`
  }

  // ---- per frame ------------------------------------------------------------
  private onUpdate() {
    if (!this.visible || this.root === null) {
      this.renderText()
      return
    }
    this.updateActual()
    if (this.drag !== null) {
      this.updateDrag()
    }
    // The solid model eases toward the real arm (10 Hz updates); without a
    // relay pose it shows the last target, so it still previews offline.
    const goal = this.qActual !== null ? this.qActual : (this.drag !== null ? this.qCmd : this.qShown)
    const k = 1 - Math.exp(-getDeltaTime() / 0.08)
    this.qShown = this.qShown.map((v, i) => v + (goal[i] - v) * k)
    this.poseLinks(this.links, this.qShown)
    this.updateHighlights()

    const showGhost = this.drag !== null && this.drag.link !== BASE_LINK && this.ghosts.length === 7
    for (const g of this.ghosts) {
      g.enabled = showGhost
    }
    if (showGhost) {
      this.poseLinks(this.ghosts, this.qCmd)
    }
    this.renderText()
  }

  /** IK the relay's actual TCP pose into joints for the model (seeded: same branch). */
  private updateActual() {
    const st = this.teleop ? this.teleop.getRelayState() : null
    const pose = st && Array.isArray(st.pose) && st.pose.length === 6 ? st.pose : null
    if (pose === null) {
      this.qActual = null
      this.lastPoseKey = ""
      return
    }
    const key = pose.join(",")
    if (key === this.lastPoseKey) {
      return
    }
    this.lastPoseKey = key
    const seed = this.qActual !== null ? this.qActual : this.qShown
    const ik = K.solveIk({pos: [pose[0], pose[1], pose[2]], rpy: [pose[3], pose[4], pose[5]]},
                         this.tcpYawDeg, seed)
    if (ik.ok) {
      this.qActual = ik.q
    }
  }

  private renderText() {
    if (!this.twinText) {
      return
    }
    const now = getTime()
    if (now - this.lastTextAt < 0.2) {
      return
    }
    this.lastTextAt = now
    const st = this.teleop ? this.teleop.getRelayState() : null
    const arm = st ? `${st.arm}${st.err ? ` err ${st.err}` : ""}${st.engaged ? "  DRIVING" : ""}` : "no relay"
    let hint = this.buildNote || this.dragNote
    if (!hint) {
      hint = "pinch the tool to move it, a segment to turn its joint, the base to move the model"
    }
    this.twinText.text =
      `DIGITAL TWIN\n` +
      `arm   : ${arm}\n` +
      `model : ${this.qActual !== null ? "mirroring the real arm" : "no arm pose (preview only)"}\n` +
      `note  : ${st && st.note && this.drag !== null ? st.note : hint}`
  }
}
