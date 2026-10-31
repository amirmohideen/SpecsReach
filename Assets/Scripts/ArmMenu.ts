import {ArmTeleop} from "./ArmTeleop"
import {ArmTwin} from "./ArmTwin"

/**
 * The main menu: choose Teleop (hand tracking drives the arm) or Digital Twin
 * (drag a 3D model of the arm). Wire UIKit buttons' "On Trigger Up
 * Callbacks" to startTeleop / startTwin / showMenu (and recenterTwin).
 *
 * Switching mode always tells ArmTeleop first, which lets go of any grab
 * before the new mode can drive (ArmTeleop.setAppMode).
 */
@component
export class ArmMenu extends BaseScriptComponent {
  @input
  @hint("The ArmTeleop component.")
  teleop: ArmTeleop

  @input
  @allowUndefined
  @hint("The ArmTwin component (the digital twin model).")
  twin: ArmTwin

  @input
  @allowUndefined
  @hint("The menu panel with the Teleop / Digital Twin buttons; shown only in the menu.")
  menuPanel: SceneObject

  @input
  @hint("Objects shown only in Teleop mode (e.g. the DRIVE / clutch panel).")
  teleopObjects: SceneObject[] = []

  @input
  @hint("Objects shown only in Digital Twin mode (e.g. a Back / Recenter panel).")
  twinObjects: SceneObject[] = []

  @input
  @widget(new ComboBoxWidget([new ComboBoxItem("menu"), new ComboBoxItem("teleop"), new ComboBoxItem("twin")]))
  @hint("What the Lens opens in.")
  startIn: string = "menu"

  onAwake() {
    this.createEvent("OnStartEvent").bind(() => {
      if (this.startIn === "teleop") this.startTeleop()
      else if (this.startIn === "twin") this.startTwin()
      else this.showMenu()
    })
  }

  public showMenu() {
    this.apply("menu")
  }

  public startTeleop() {
    this.apply("teleop")
  }

  public startTwin() {
    this.apply("twin")
  }

  /** Put the twin model back in front of you. */
  public recenterTwin() {
    if (this.twin) {
      this.twin.recenter()
    }
  }

  private apply(mode: "menu" | "teleop" | "twin") {
    if (this.teleop) {
      this.teleop.setAppMode(mode)
    }
    if (this.twin) {
      if (mode === "twin") this.twin.show()
      else this.twin.hide()
    }
    this.setEnabled([this.menuPanel], mode === "menu")
    this.setEnabled(this.teleopObjects, mode === "teleop")
    this.setEnabled(this.twinObjects, mode === "twin")
  }

  private setEnabled(objs: SceneObject[] | undefined, on: boolean) {
    if (!objs) {
      return
    }
    for (const o of objs) {
      if (o) {
        o.enabled = on
      }
    }
  }
}
