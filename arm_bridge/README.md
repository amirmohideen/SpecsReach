# arm_bridge — the AUBO arm bridge

The program that drives the AUBO i5H. It takes one WebRTC client at a time
(the relay, `specs_relay/`), checks every target against its safety limits,
and streams the arm through AUBO's SDK at 50 Hz.

It comes from the teleop-server project (commit `4cd4187`, where a Quest
headset drives the arm), trimmed to what SpecsReach needs:

- **Kept:** the WebRTC transport and signaling, the session and safety checks,
  the servo loop (50 Hz here), the AUBO driver, and the `mock` backend for
  testing without hardware.
- **Left out:** the web dashboard and its Programs tab, the xArm and simulator
  backends, and everything for the Quest's video and recording.

The robot model the driver checks against lives at
`arm_bridge/dashboard/static/assets/aubo_i5H_10/` (its manifest and URDF only;
the path is the one the code expects). Comments in the code sometimes cite
teleop-server documents that aren't part of this repo.

## Run

From the repo root, with the `specsreach-station` image built:

```bash
station/start_bridge.sh                                        # the AUBO
CONFIG=station/mock.yaml SIGNALING_PORT=8454 station/start_bridge.sh   # no hardware
```

That runs `python3 -m arm_bridge.main --signaling-port 8444 --config
station/arm_bridge.yaml` in a container on the host network. The config's keys
and defaults are documented in `arm_bridge/config.py`.

## Safety

- Session = intent, servo loop = sole actuator: network callbacks only record
  the latest validated target; the servo loop is the only thing that commands
  the arm, so the network can never block it.
- Out-of-box or non-finite targets are rejected, never clamped, and every step
  is limited in speed and acceleration.
- The joint limits, the elbow guard and the IK checks hold any pose the arm
  should not reach; RETREAT (relay key `t`) brings the arm back inside.
- No heartbeat for `watchdog_ms` holds the arm. Power-on is always an explicit
  command (relay key `e`).
