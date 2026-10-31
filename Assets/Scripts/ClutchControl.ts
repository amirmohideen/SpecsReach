/**
 * The clutch for ArmTeleop: what goes into the `clutch` field of each hand
 * message, plus when the wrist axes freeze. Plain TypeScript with no Lens
 * Studio APIs (the caller passes the time and the hand state), so the rules
 * can be tested outside the editor.
 *
 * The relay drives only while clutch.tracked AND clutch.pinch are true, starts
 * a grab only on a false -> true change of that, and lets go when pinch has
 * been false for 0.15 s, the drive hand is lost, the arm faults, or the Lens
 * goes silent. After any let-go it needs another false -> true change.
 *
 * Modes:
 * - "pinch" (default): exactly the original behaviour, clutch = the clutch
 *   hand's tracking and pinch. After a switch INTO pinch mode, pinch is sent
 *   false until the hand has been seen open once, so changing modes mid-pinch
 *   can never start a grab.
 * - "toggle": clutch.tracked is always true and clutch.pinch is the DRIVE
 *   toggle. The toggle starts off and switches itself off when the relay lets
 *   go after grabbing, when the relay has not grabbed within ENGAGE_WAIT_S of
 *   switching on, when no relay state has arrived for STATE_STALE_S, when the
 *   connection drops or reconnects, and when the mode changes.
 */

export type ClutchMode = "pinch" | "toggle"

export const ENGAGE_WAIT_S = 3.0   // toggle on (or a clutch edge) without the relay grabbing
export const STATE_STALE_S = 2.0   // toggle on without any relay state message

export interface ClutchOut {
  tracked: boolean
  pinch: boolean
}

/**
 * The on/off value from a UIKit callback. A Switch calls back repeatedly while
 * its knob animates, with values between 0 and 1, so the state is set from the
 * value instead of flipped: a number means on = value > 0.5, a boolean is used
 * as is, and anything else (no argument, an event object) flips `current`.
 */
export function resolveSwitch(value: unknown, current: boolean): boolean {
  if (typeof value === "boolean") {
    return value
  }
  if (typeof value === "number") {
    return value > 0.5
  }
  return !current
}

export class ClutchControl {
  mode: ClutchMode
  toggleOn: boolean = false
  /** Why the toggle last switched itself off, or why DRIVE was refused; "" = nothing to say. */
  note: string = ""
  /**
   * Whether the arm frame is frozen: from the clutch edge (the relay freezes
   * its frame when it grabs) until the clutch goes off, the relay lets go, or
   * the relay has not grabbed within ENGAGE_WAIT_S. Re-set when the relay
   * starts a grab while the clutch is on.
   */
  frozen: boolean = false

  private armed: boolean = true
  private toggleOnAt: number = 0
  private toggleSawEngaged: boolean = false
  private wasActive: boolean = false
  private activeAt: number = 0
  private engagedThisClutch: boolean = false
  private lastEngaged: boolean = false
  private lastStateAt: number = -1e9
  private relayNote: string = ""

  constructor(mode: ClutchMode = "pinch") {
    this.mode = mode
  }

  /** Pinch mode: false while waiting to see the hand open after a mode switch. */
  get pinchArmed(): boolean {
    return this.armed
  }

  /** Returns whether the mode changed. */
  setMode(mode: ClutchMode): boolean {
    if (mode === this.mode) {
      return false
    }
    this.switchOff("clutch mode changed")
    this.mode = mode
    if (mode === "pinch") {
      this.armed = false
    }
    return true
  }

  /** A DRIVE on/off request. Refusals and their reason land in `note`. */
  requestDrive(on: boolean, now: number, connected: boolean, driveTracked: boolean): void {
    if (this.mode !== "toggle") {
      this.note = "DRIVE works in toggle mode"
      return
    }
    if (!on) {
      if (this.toggleOn) {
        this.toggleOn = false
        this.note = ""
      }
      return
    }
    if (this.toggleOn) {
      return
    }
    if (!connected) {
      this.note = "DRIVE refused: not connected to the relay"
      return
    }
    if (!driveTracked) {
      this.note = "DRIVE refused: show your drive hand first"
      return
    }
    this.toggleOn = true
    this.toggleOnAt = now
    this.toggleSawEngaged = false
    this.note = ""
  }

  /** The socket closed, errored or (re)connected. */
  connectionChanged(reason: string): void {
    this.switchOff(reason)
  }

  /** A relay state message arrived. */
  relayState(now: number, engaged: boolean, note: string): void {
    this.lastStateAt = now
    this.relayNote = note || ""
    if (this.toggleOn) {
      if (engaged) {
        this.toggleSawEngaged = true
      } else if (this.toggleSawEngaged) {
        this.switchOff(`relay let go: ${this.relayNote || "no reason given"}`)
      }
    }
    if (this.wasActive) {
      if (engaged) {
        if (!this.lastEngaged) {
          this.frozen = true            // the relay froze its frame just now
        }
        this.engagedThisClutch = true
      } else if (this.engagedThisClutch) {
        this.frozen = false             // the relay let go: its frame is gone
      }
    }
    this.lastEngaged = engaged
  }

  /** Once per frame: the clutch to send, given the clutch hand right now. */
  frame(now: number, clutchTracked: boolean, clutchPinching: boolean): ClutchOut {
    let out: ClutchOut
    if (this.mode === "toggle") {
      if (this.toggleOn) {
        if (!this.toggleSawEngaged && now - this.toggleOnAt > ENGAGE_WAIT_S) {
          this.switchOff(`relay did not grab: ${this.relayNote || "no reason given"}`)
        } else if (now - Math.max(this.lastStateAt, this.toggleOnAt) > STATE_STALE_S) {
          this.switchOff("no state from the relay for 2 s")
        }
      }
      out = {tracked: true, pinch: this.toggleOn}
    } else {
      if (!this.armed && clutchTracked && !clutchPinching) {
        this.armed = true
      }
      out = {tracked: clutchTracked, pinch: clutchTracked && clutchPinching && this.armed}
    }

    const active = out.tracked && out.pinch
    if (active && !this.wasActive) {
      this.activeAt = now
      this.engagedThisClutch = false
      this.frozen = true
    } else if (!active && this.wasActive) {
      this.frozen = false
    } else if (active && this.frozen && !this.engagedThisClutch && now - this.activeAt > ENGAGE_WAIT_S) {
      this.frozen = false               // the relay never grabbed: follow the head again
    }
    this.wasActive = active
    return out
  }

  private switchOff(reason: string): void {
    if (this.toggleOn) {
      this.toggleOn = false
      this.note = reason
    }
  }
}
