# specs_relay — Spectacles hand tracking → arm bridge

The station half of **SpecsReach**. The Spectacles Lens (`Assets/Scripts/ArmRelayClient.ts`
in the SpecsReach Lens project) can open a WebSocket but not a WebRTC
PeerConnection. This relay sits between the Lens and the bridge and appears to the
bridge as an ordinary headset:

```
Spectacles ──ws:// JSON, 50 Hz──► specs_relay ──WebRTC `arm` + `armctl`──► arm bridge :8444 ──► AUBO
```

- **The bridge is unchanged.** Its safety still applies: workspace box, reach,
  keep-outs, IK holds, ramps, elbow guard, watchdog and smooth stops.
- **The Lens only measures.** It sends the drive hand's wrist and three knuckles,
  whether the clutch hand is pinching, and the head direction.
- **The relay holds the logic.** Clutch, gates and the mimicry mapping are in
  `teleop_map.py`. Tuning is in `specs_relay.json`: edit it and restart, with no
  Lens rebuild.

## Control

| Hand | Does |
|---|---|
| Left pinch (clutch) | Pinch **and hold** to grab the arm; a quick pinch-tap does nothing useful. Open the pinch to let go: the arm stops smoothly where it is. |
| Right wrist (drive) | While grabbed, the tool moves and turns as the wrist does, 1:1 (`position_scale`). |
| Right fingers | Drive the DG-5F through SpecsGrip's `HandBridge.ts`, which is independent of this relay. |

To reach further, release, move your hand back, and pinch again. The same
"re-indexing" happens on the Quest.

An engage is refused unless all of these hold:
- the pinch is a fresh press;
- the bridge STATUS is fresher than 150 ms and reads `SERVO`;
- there is no fault and no latched hold;
- the drive hand is tracked.

Any of the following drops the deadman: the pinch staying open for 150 ms, losing
either hand, a gate failing, or no Lens message for `lens_timeout_s` (1 s in
`specs_relay.json`, see Tuning).

Only the newest Lens connection is listened to. A second Lens (for example Lens
Studio's Preview on the Mac, which runs the same script) replaces the first, and
the two keep kicking each other off, so close the Preview while driving.

## Frames

- **view** (default, no calibration): your horizontal look direction at the
  moment you pinch becomes the arm-base direction `view_yaw_deg`. Set it to the
  way you face the arm from your usual spot: `0` means you look along arm +X.
  Turning your head while grabbed changes nothing, because the frame is frozen
  at engage.
- **room** (calibrated, keyboard `c` twice):
  1. Hold the right hand over the arm base and press `c`.
  2. Move the hand at least 20 cm along arm +X and press `c` again.

  From then on, moving your hand toward the window moves the tool toward the
  window, wherever you stand. The Lens world restarts with every Lens launch, so
  the relay drops the calibration when a new launch connects.

## Run (on the station)

Start the arm bridge (`station/start_bridge.sh`, or an arm bridge that already
runs on the station), then, from the repo root:

```bash
station/start_relay.sh
```

That runs `specs_relay.py --bridge-host 127.0.0.1 --port 8766` in the
`specsreach-station` image; extra arguments are passed through.

1. Put the station's **Wi-Fi** address in **Server Url** of the
   `ArmRelayClient` component: `ws://<wifi-ip>:8766`. `python3
   station/netcheck.py` prints it. The relay also prints a `Lens URL` line per
   address, including the arm's cable network, which the glasses can't reach.
2. Open the Lens on the glasses. The relay then connects to the bridge (only
   while a Lens is connected, plus 10 s), and the HUD shows `bridge: connected`.
3. Type `e` + Enter to enable the arm. On the AUBO this is power-on plus brake
   release, and takes a few seconds. The HUD turns green once the arm reads
   `SERVO`.
4. Show your right hand, pinch with your left, and move.

Keyboard (each followed by Enter):

| Key | Action |
|---|---|
| `e` / `d` | Enable / disable. On the AUBO, disable is a **power-off**. |
| `h` / `x` | Home / cancel home |
| `r` | Recover (clear a fault) |
| `t` | Retreat into the envelope (warn 1006) |
| `c` | Calibrate the room frame (press twice) |
| `v` | Switch to view mode |
| `s` | Status |
| `q` | Quit |

**The bridge has one peer slot.** While the relay holds it, the Quest cannot
connect, and the reverse is also true.

**Network:**
- The glasses and the station must be on the same private Wi-Fi.
- The station's wired port (to the AUBO) never takes the default route, so use
  the station's **Wi-Fi** address.
- Open TCP 8766 if a firewall is on.
- Test on the glasses. Lens Studio's Preview can connect too (when its device
  type is Spectacles), and then takes over from the glasses: close it while
  driving.

## Digital twin mode

The Lens can drive the arm from a 3D model of it instead of the hand: the
operator pinches a part of the model and drags it (`ArmDigitalTwin.ts` in the
SpecsReach Lens). The Lens then sends, instead of `hand` messages:

```json
{"type": "twin", "t": 12345, "active": true, "pose": [x, y, z, roll, pitch, yaw]}
```

`pose` is the model's TCP in the arm base frame (mm, RPY degrees, the bridge's
convention); `active` is the twin's clutch (true while a part is held).

- Same clutch rules as the hand: a grab starts on an `active` false -> true
  edge and only when the arm is ready; it ends at once when `active` turns
  false, when twin messages stop for `lens_timeout_s`, or on a fault.
- The arm follows the target's motion **since the grab** (relative), so a model
  drawn a little off the real arm cannot make it jump. The usual speed, range
  and rotation limits apply, and the bridge's own safety on top.
- `twin_orientation` (default true) follows the tool rotation too; the hand's
  `enable_orientation` does not apply, because the twin's targets are noise-free.
- `twin_max_speed_mm_s` / `twin_max_orient_speed_deg_s` (default 60 / 20) set
  how fast the arm chases the model, separate from the hand's limits: a small
  model turns small hand moves into big arm moves. The model shows the real
  arm catching up; letting go stops it wherever it has got to.
- While twin messages flow the twin owns the arm: a hand grab is released and
  needs a fresh pinch afterwards.

The relay's `state` message carries `pose`: the arm's actual TCP (or null
without a fresh STATUS), which the Lens uses to pose the model like the real arm.

## Test without glasses

From the repo root, in the station image:

```bash
docker run --rm -v "$PWD":/workspace -w /workspace specsreach-station \
  python3 -m unittest discover -s specs_relay/tests     # mapping + clutch logic
```

The whole station path with a mock arm (`station/mock.yaml`), on spare ports
so it never meets the real bridge or relay:

```bash
CONFIG=station/mock.yaml SIGNALING_PORT=8454 station/start_bridge.sh
NAME=specsreach-relay-test station/start_relay.sh --bridge-port 8454 --port 8776 --auto-enable
# in a second terminal:
docker run --rm --network host -v "$PWD":/workspace -w /workspace specsreach-station \
  python3 specs_relay/fake_lens.py --url ws://127.0.0.1:8776 --axis forward --cm 3
docker rm -f specsreach-arm-mock                        # when done
```

- `fake_lens.py` plays a 50 Hz Lens: a rest, a left pinch, the right wrist
  moved `--cm` out and back, then a release. `fake_twin.py <url>` does the same
  for the digital twin.
- `--auto-enable` powers the arm on by itself: use it with the mock only.
- **Real arm:** keep `--cm` small and a hand on the e-stop.

## Tuning (`specs_relay.json`)

| Key | Default | Meaning |
|---|---|---|
| `mode` / `view_yaw_deg` | view / 0 | Frame, as described above |
| `position_scale` | 1.0 | Hand → tool translation scale |
| `filter_tau_s` | 0.06 | Low-pass on the wrist pose. Raise it for less jitter, at the cost of more lag. |
| `deadzone_cm` / `orient_deadzone_deg` | 1.5 / 8 | Subtractive deadzones around the engage pose |
| `max_range_mm` | 500 | Per-axis box around the engage pose |
| `max_speed_mm_s` / `max_orient_speed_deg_s` | 250 / 90 | Glitch armour. The bridge's own ramp is lower (×0.7). |
| `max_orient_deg` | 120 | Geodesic cone around the engage orientation |
| `enable_orientation` | true | `false` translates only and holds the tool orientation |
| `lens_timeout_s` / `status_fresh_s` | 0.3 / 0.15 | Liveness gates: no Lens message / no fresh bridge STATUS for this long releases the grab. |
| `clutch_release_s` | 0.15 | How long the pinch must stay open to let go. Hand tracking drops a held pinch for a frame or two; a silent Lens or a lost drive hand still releases at once. |

**This station's `specs_relay.json` overrides three defaults** (2026-10-30):
- `lens_timeout_s` 1.0: the glasses deliver hand data in bursts. Measured on
  the station, nothing had arrived for > 100 ms 70% of the time, > 300 ms 26%,
  and at most 974 ms, although the glasses timestamp a message every 34 ms and
  the Wi-Fi itself is fine (ping max 60 ms, no loss). At 0.3 every grab dropped
  within a second. 1 s matches the bridge's own `watchdog_ms` for the Quest.
  While the data is late, the arm keeps gliding toward the last reported hand
  position (never past it), and a pinch release is seen when the late data lands.
- `filter_tau_s` 0.25: smooths the bursts into continuous motion, at about
  0.2 s more lag.
- `enable_orientation` false: position only for the first sessions.

## Verified, and not yet

- **Verified on the station (2026-10-30/31):** Spectacles teleop of the AUBO
  through this relay and the arm bridge, with the movement directions right in
  view mode (`view_yaw_deg` 0), so the Lens-axes assumption (right-handed, +Y
  up, cm) holds. If "forward" ever drives the arm sideways, fix `view_yaw_deg`;
  if up/down or a rotation comes out mirrored, report it rather than tuning
  around it.
- **(HW)** With the DG-5F mounted on the flange, the AUBO `tcp_offset` and
  `payload` in `station/arm_bridge.yaml` must be re-commissioned.
