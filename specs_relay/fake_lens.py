#!/usr/bin/env python3
"""Stand-in for the SpecsReach Lens: drives the relay without glasses.

Sends the Lens wire format at 50 Hz: a flat right hand at rest, then a left
pinch (clutch), a slow cosine move of the right wrist `--cm` along one
operator axis and back, then a release. Prints the relay's state replies.

    python3 specs_relay/fake_lens.py --url ws://127.0.0.1:8766 --axis forward --cm 3

Axes are the operator's (view mode, looking along the Lens world -Z):
forward / right / up. With the mock bridge (`arm_backend: mock`) and
`specs_relay.py --auto-enable` this exercises the whole station path; on the
real arm keep --cm small and a hand on the e-stop.
"""

import argparse
import asyncio
import json
import math
import time

import websockets

AXES = {"forward": (0.0, 0.0, -1.0), "right": (1.0, 0.0, 0.0), "up": (0.0, 1.0, 0.0),
        "roll": None}   # roll: tilt the hand about the forward axis, --cm x 10 degrees


def hand_msg(wrist, pinch, roll_deg=0.0):
    x, y, z = wrist
    c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))

    def at(dx, dy, dz):     # knuckle offset, rolled about -Z through the wrist
        return [x + c * dx + s * dy, y - s * dx + c * dy, z + dz]

    return {
        "type": "hand", "t": int(time.monotonic() * 1000),
        "drive": {"tracked": True, "wrist": [x, y, z], "index": at(-2.0, 0.0, -8.0),
                  "middle": at(0.0, 0.0, -8.5), "pinky": at(3.5, 0.0, -7.0)},
        "clutch": {"tracked": True, "pinch": pinch},
        "head": {"fwd": [0.0, 0.0, -1.0]},
    }


async def run(args) -> int:
    axis = AXES[args.axis]
    base = (0.0, -20.0, -40.0)
    states = []

    async with websockets.connect(args.url, open_timeout=5) as ws:
        async def reader():
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get("type") == "state":
                    states.append(msg)

        rd = asyncio.create_task(reader())
        await ws.send(json.dumps({"type": "hello", "boot": "fake-lens", "drive": "right",
                                  "clutch": "left"}))
        hz, t0 = 50.0, time.monotonic()
        rest, move = 1.0, args.seconds
        total = rest + move + 1.0
        last_print = 0.0
        while (t := time.monotonic() - t0) < total:
            pinch = rest <= t < rest + move
            off = 0.0
            if pinch:
                u = (t - rest) / move                       # out and back
                off = args.cm * (1 - math.cos(2 * math.pi * u)) / 2
            if axis is None:
                await ws.send(json.dumps(hand_msg(base, pinch, roll_deg=off * 10.0)))
            else:
                wrist = tuple(b + a * off for b, a in zip(base, axis))
                await ws.send(json.dumps(hand_msg(wrist, pinch)))
            if states and t - last_print >= 0.5:
                last_print = t
                s = states[-1]
                print(f"t={t:4.1f}s pinch={int(pinch)} off={off:5.2f}cm  bridge={s['bridge']} "
                      f"arm={s['arm']} engaged={s['engaged']} note={s['note']!r}")
            await asyncio.sleep(1.0 / hz)
        rd.cancel()
    engaged = any(s["engaged"] for s in states)
    print(f"=== fake_lens: {len(states)} state messages, engaged={engaged} ===")
    return 0 if states else 1


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--url", default="ws://127.0.0.1:8766")
    p.add_argument("--axis", choices=sorted(AXES), default="forward")
    p.add_argument("--cm", type=float, default=3.0, help="peak wrist offset (cm), <= 10")
    p.add_argument("--seconds", type=float, default=4.0, help="duration of the out-and-back move")
    args = p.parse_args()
    if not 0 < args.cm <= 10:
        p.error("--cm must be in (0, 10]")
    raise SystemExit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()
