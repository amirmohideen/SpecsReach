#!/usr/bin/env python3
"""Stand-in for the SpecsReach digital twin (ArmDigitalTwin.ts): drives the relay's
`twin` path without glasses.

Waits for the arm's actual pose in the relay state, then plays a model drawn
3 mm off it (to show the grab does not jump the arm): idle, grab, move the
model's TCP +30 mm X, turn it 15 deg about Z, let go. Prints what the relay
reports back.

    python3 tools/specs_relay/fake_twin.py ws://127.0.0.1:8766

On the real arm keep a hand on the e-stop: this moves it 30 mm.
"""
import asyncio, json, sys, time
import websockets

async def main(url):
    states = []
    async with websockets.connect(url, open_timeout=5) as ws:
        async def reader():
            async for raw in ws:
                m = json.loads(raw)
                if m.get("type") == "state":
                    states.append(m)
        rd = asyncio.create_task(reader())
        await ws.send(json.dumps({"type": "hello", "boot": "fake-twin", "device": "fake-twin"}))
        while not states or states[-1].get("pose") is None or states[-1]["arm"] != "SERVO":
            await ws.send(json.dumps({"type": "twin", "t": 0, "active": False, "pose": [0, 0, 0, 0, 0, 0]}))
            await asyncio.sleep(0.05)
        p0 = states[-1]["pose"]
        print("actual pose from relay:", p0)
        model = list(p0)          # the twin would draw the arm here
        model[0] += 3.0           # ... 3 mm off, to show there is no jump
        # (active, seconds, dx from -> to, yaw from -> to): a drag starts where
        # it grabs and moves from there, like a hand dragging the model.
        plan = [(False, 0.3, (0, 0), (0, 0)), (True, 0.3, (0, 0), (0, 0)),
                (True, 2.5, (0, 30.0), (0, 0)), (True, 2.0, (30.0, 30.0), (0, 15.0)),
                (False, 1.0, (30.0, 30.0), (15.0, 15.0))]
        for active, dur, (x0, x1), (y0, y1) in plan:
            t0 = time.monotonic()
            while (u := (time.monotonic() - t0) / dur) < 1.0:
                dx, dyaw = x0 + (x1 - x0) * u, y0 + (y1 - y0) * u
                pose = [model[0] + dx, model[1], model[2], model[3], model[4], model[5] + dyaw]
                await ws.send(json.dumps({"type": "twin", "t": int(time.monotonic() * 1000),
                                          "active": active, "pose": pose}))
                await asyncio.sleep(0.02)
            await asyncio.sleep(0.3)        # let the relay report the settled pose
            s = states[-1]
            print(f"after active={active} dx->{x1} yaw->{y1}: engaged={s['engaged']} "
                  f"note={s['note']!r} pose={s['pose']}")
        rd.cancel()

asyncio.run(main(sys.argv[1]))
