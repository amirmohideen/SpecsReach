/**
 * Draws the arm's X/Y/Z axes as three short bars at a point (the drive wrist):
 * X red, Y green, Z blue, with optional X/Y/Z labels at the tips that turn to
 * face the camera. ArmTeleop decides the directions and calls update() every
 * frame; this class only builds and places the visuals.
 *
 * Everything is created in code, so the scene only needs one Unlit material:
 * it is cloned once per axis and recoloured. "Dim" is darker colours rather
 * than transparency, so the material needs no blend-mode setup.
 */

const AXIS_NAMES = ["X", "Y", "Z"]
const AXIS_COLORS = [
  new vec4(1.0, 0.25, 0.25, 1.0),
  new vec4(0.25, 1.0, 0.35, 1.0),
  new vec4(0.35, 0.55, 1.0, 1.0)
]
const DIM = 0.35

export interface ArmAxesOptions {
  lengthCm: number
  thicknessCm: number
  labels: boolean
  labelSize: number
  /** Font for the labels; null = the default font. */
  labelFont: Font | null
}

/** A box from y = 0 to y = length, `half` wide each side of the Y axis. */
function barMesh(length: number, half: number): RenderMesh {
  const b = new MeshBuilder([
    {name: "position", components: 3},
    {name: "normal", components: 3, normalized: true},
    {name: "texture0", components: 2}
  ])
  b.topology = MeshTopology.Triangles
  b.indexType = MeshIndexType.UInt16
  const x0 = -half, x1 = half, y0 = 0, y1 = length, z0 = -half, z1 = half
  // Each face: its normal, then four corners counter-clockwise seen from outside.
  const faces: number[][][] = [
    [[0, 0, 1], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
    [[0, 0, -1], [x1, y0, z0], [x0, y0, z0], [x0, y1, z0], [x1, y1, z0]],
    [[1, 0, 0], [x1, y0, z1], [x1, y0, z0], [x1, y1, z0], [x1, y1, z1]],
    [[-1, 0, 0], [x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0]],
    [[0, 1, 0], [x0, y1, z1], [x1, y1, z1], [x1, y1, z0], [x0, y1, z0]],
    [[0, -1, 0], [x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]]
  ]
  const uv = [[0, 0], [1, 0], [1, 1], [0, 1]]
  faces.forEach((face, f) => {
    const n = face[0]
    for (let i = 0; i < 4; i++) {
      const p = face[i + 1]
      b.appendVerticesInterleaved([p[0], p[1], p[2], n[0], n[1], n[2], uv[i][0], uv[i][1]])
    }
    const base = f * 4
    b.appendIndices([base, base + 1, base + 2, base, base + 2, base + 3])
  })
  b.updateMesh()
  return b.getMesh()
}

export class ArmAxes {
  private root: SceneObject
  private bars: SceneObject[] = []
  private materials: Material[] = []
  private labels: SceneObject[] = []
  private texts: Text[] = []
  private length: number
  private bright: boolean | null = null
  private visible: boolean = true

  constructor(parent: SceneObject, material: Material, opts: ArmAxesOptions) {
    this.length = opts.lengthCm
    this.root = global.scene.createSceneObject("ArmAxes")
    this.root.setParent(parent)
    this.root.layer = parent.layer
    const mesh = barMesh(opts.lengthCm, opts.thicknessCm / 2)

    for (let i = 0; i < 3; i++) {
      const bar = global.scene.createSceneObject(`ArmAxis${AXIS_NAMES[i]}`)
      bar.setParent(this.root)
      bar.layer = parent.layer
      const visual = bar.createComponent("Component.RenderMeshVisual") as RenderMeshVisual
      visual.mesh = mesh
      const mat = material.clone()
      visual.mainMaterial = mat
      this.bars.push(bar)
      this.materials.push(mat)

      if (opts.labels) {
        const label = global.scene.createSceneObject(`ArmAxisLabel${AXIS_NAMES[i]}`)
        label.setParent(this.root)
        label.layer = parent.layer
        const text = label.createComponent("Component.Text") as Text
        text.text = AXIS_NAMES[i]
        text.size = opts.labelSize
        text.twoSided = true
        text.horizontalOverflow = HorizontalOverflow.Overflow
        text.verticalOverflow = VerticalOverflow.Overflow
        text.worldSpaceRect = Rect.create(-1.5, 1.5, -1.0, 1.0)
        if (opts.labelFont !== null) {
          text.font = opts.labelFont
        }
        this.labels.push(label)
        this.texts.push(text)
      }
    }
    this.setBright(false)
  }

  setVisible(on: boolean): void {
    if (on === this.visible) {
      return
    }
    this.visible = on
    this.root.enabled = on
  }

  /**
   * Place the axes at `origin` (world, cm) along the three unit directions.
   * `bright` = full colour (frame frozen), else dim. `faceRotation` turns the
   * labels to face the viewer (the camera's world rotation).
   */
  update(origin: vec3, dirs: vec3[], bright: boolean, faceRotation: quat | null): void {
    this.setBright(bright)
    const up = vec3.up()
    for (let i = 0; i < 3; i++) {
      const dir = dirs[i]
      const t = this.bars[i].getTransform()
      t.setWorldPosition(origin)
      t.setWorldRotation(quat.rotationFromTo(up, dir))
      if (i < this.labels.length) {
        const lt = this.labels[i].getTransform()
        lt.setWorldPosition(origin.add(dir.uniformScale(this.length + 1.2)))
        if (faceRotation !== null) {
          lt.setWorldRotation(faceRotation)
        }
      }
    }
  }

  private setBright(bright: boolean): void {
    if (bright === this.bright) {
      return
    }
    this.bright = bright
    const k = bright ? 1.0 : DIM
    for (let i = 0; i < 3; i++) {
      const c = AXIS_COLORS[i]
      const color = new vec4(c.x * k, c.y * k, c.z * k, 1.0)
      this.materials[i].mainPass.baseColor = color
      if (i < this.texts.length) {
        this.texts[i].textFill.color = color
      }
    }
  }
}
