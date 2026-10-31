"""Tests for the Spectacles -> arm mapping (stdlib unittest, no hardware).

    python3 -m unittest discover -s tools/specs_relay/tests   (or pytest, as in run_py_tests.sh)
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from teleop_map import (LensSample, Teleop, Tuning, TwinSample, basis, hand_rotation,  # noqa: E402
                        yaw_from_points)
from arm_bridge import rotation  # noqa: E402
from arm_bridge.protocol import ArmState, Status  # noqa: E402

DT = 0.02
LOOK = (0.0, 0.0, -1.0)       # Lens camera looks along world -Z


def status(x=400.0, y=0.0, z=300.0, rpy=(180.0, 0.0, 0.0), state=ArmState.SERVO,
           err=0, hold=False, permit=7):
    return Status(state, False, True, False, x, y, z, *rpy, 84.0,
                  error_code=err, motion_permit_id=permit, hold_latched=hold)


def hand(wrist=(0.0, 0.0, -40.0), pinch=False, tracked=True, clutch_tracked=True,
         yaw_deg=0.0, head=LOOK):
    """A flat right hand, palm down, fingers pointing away (world -Z),
    optionally turned by yaw_deg about world +Y around the wrist."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))

    def turn(v):   # rotate about +Y (right-handed)
        return (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])

    def at(off):
        o = turn(off)
        return (wrist[0] + o[0], wrist[1] + o[1], wrist[2] + o[2])

    return LensSample(drive_tracked=tracked, wrist=wrist,
                      index_k=at((-2.0, 0.0, -8.0)), middle_k=at((0.0, 0.0, -8.5)),
                      pinky_k=at((3.5, 0.0, -7.0)), clutch_tracked=clutch_tracked,
                      pinch=pinch, head_fwd=head)


def tilted(roll_deg, pinch=False):
    """The flat hand rolled by roll_deg about the operator's forward axis
    (world -Z, which view mode maps onto arm +X) around the wrist."""
    base = hand(pinch=pinch)
    c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))

    def roll(p):   # rotate about -Z through the wrist
        x, y, z = (p[0] - base.wrist[0], p[1] - base.wrist[1], p[2] - base.wrist[2])
        x, y = c * x + s * y, -s * x + c * y
        return (base.wrist[0] + x, base.wrist[1] + y, base.wrist[2] + z)

    return LensSample(drive_tracked=True, wrist=base.wrist, index_k=roll(base.index_k),
                      middle_k=roll(base.middle_k), pinky_k=roll(base.pinky_k),
                      clutch_tracked=True, pinch=pinch, head_fwd=LOOK)


class Rig:
    """Drives a Teleop with a fake clock."""

    def __init__(self, **tuning):
        self.t = 100.0
        self.tele = Teleop(tuning=Tuning(**tuning))
        self.tele.on_status(status(), self.t)

    def step(self, sample=None, st=None, fresh_status=True):
        self.t += DT
        if fresh_status:
            self.tele.on_status(st or self.tele.status, self.t)
        if sample is not None:
            self.tele.on_lens(sample, self.t)
        return self.tele.tick(self.t)

    def engage(self, **kw):
        self.step(hand(**kw))
        return self.step(hand(pinch=True, **kw))


class BasisTest(unittest.TestCase):
    def test_view_basis_maps_operator_axes(self):
        m = basis((0.0, 0.0, -1.0), 0.0)
        mv = lambda v: tuple(round(sum(m[i][k] * v[k] for k in range(3)), 9) for i in range(3))
        self.assertEqual(mv((0, 0, -1)), (1, 0, 0))     # forward -> arm +X
        self.assertEqual(mv((1, 0, 0)), (0, -1, 0))     # right   -> arm -Y
        self.assertEqual(mv((0, 1, 0)), (0, 0, 1))      # up      -> arm +Z
        det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
               - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
               + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
        self.assertAlmostEqual(det, 1.0)

    def test_view_yaw_turns_the_mapping(self):
        m = basis((0.0, 0.0, -1.0), 90.0)
        fwd = tuple(sum(m[i][k] * (0, 0, -1)[k] for k in range(3)) for i in range(3))
        self.assertAlmostEqual(fwd[1], 1.0)              # forward -> arm +Y

    def test_hand_rotation_is_proper(self):
        s = hand()
        r = hand_rotation(s.wrist, s.index_k, s.middle_k, s.pinky_k)
        rt_r = rotation._mul(rotation._transpose(r), r)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(rt_r[i][j], 1.0 if i == j else 0.0)

    def test_calibration_needs_distance(self):
        self.assertIsNone(yaw_from_points((0, 0, 0), (10, 30, 0), 20.0))   # mostly vertical
        f = yaw_from_points((0, 5, 0), (30, 0, 0), 20.0)
        self.assertAlmostEqual(f[0], 1.0)


class ClutchTest(unittest.TestCase):
    def test_idle_echoes_actual_pose(self):
        rig = Rig()
        cmd = rig.step(hand())
        self.assertFalse(cmd.deadman)
        self.assertEqual(cmd.pose, (400.0, 0.0, 300.0, 180.0, 0.0, 0.0))
        self.assertEqual(cmd.permit, 7)
        self.assertEqual(cmd.gripper, 84.0)

    def test_no_status_sends_nothing(self):
        tele = Teleop()
        self.assertIsNone(tele.tick(1.0))

    def test_press_edge_engages(self):
        rig = Rig()
        self.assertTrue(rig.engage().deadman)

    def test_pinch_held_from_the_start_does_not_engage(self):
        rig = Rig()
        for _ in range(5):
            self.assertFalse(rig.step(hand(pinch=True)).deadman)

    def test_clutch_hand_entering_view_pinched_does_not_engage(self):
        rig = Rig()
        rig.step(hand(clutch_tracked=False))
        self.assertFalse(rig.step(hand(pinch=True)).deadman)
        rig.step(hand())
        self.assertTrue(rig.step(hand(pinch=True)).deadman)

    def test_refuses_without_servo_and_needs_new_press(self):
        rig = Rig()
        rig.tele.on_status(status(state=ArmState.READY), rig.t)
        rig.step(hand(), st=status(state=ArmState.READY))
        self.assertFalse(rig.step(hand(pinch=True), st=status(state=ArmState.READY)).deadman)
        self.assertIn("READY", rig.tele.note)
        # SERVO arrives while the pinch is still held: no engage until re-pressed.
        self.assertFalse(rig.step(hand(pinch=True), st=status()).deadman)
        rig.step(hand(), st=status())
        self.assertTrue(rig.step(hand(pinch=True), st=status()).deadman)

    def test_refuses_on_fault_or_hold(self):
        for st in (status(err=31), status(hold=True)):
            rig = Rig()
            rig.step(hand(), st=st)
            self.assertFalse(rig.step(hand(pinch=True), st=st).deadman)

    def test_stale_status_refuses_and_releases(self):
        rig = Rig()
        rig.step(hand())
        rig.t += 0.5      # STATUS now 0.5 s old
        self.assertFalse(rig.step(hand(pinch=True), fresh_status=False).deadman)
        rig = Rig()
        rig.engage()
        rig.t += 0.5
        self.assertFalse(rig.step(hand(pinch=True), fresh_status=False).deadman)

    def test_pinch_flicker_keeps_the_grab(self):
        rig = Rig()
        rig.engage()
        for _ in range(3):                         # 60 ms open: tracking flicker
            self.assertTrue(rig.step(hand(pinch=False)).deadman)
        self.assertTrue(rig.step(hand(pinch=True)).deadman)
        self.assertTrue(rig.tele.engaged)

    def test_release_and_hand_loss_drop_the_deadman(self):
        rig = Rig()
        rig.engage()
        cmds = [rig.step(hand(pinch=False)) for _ in range(9)]   # 180 ms open
        self.assertTrue(cmds[0].deadman)           # still inside clutch_release_s
        self.assertFalse(cmds[-1].deadman)         # released after 150 ms
        self.assertIn("pinch released", rig.tele.note)
        rig.step(hand())
        rig.step(hand(pinch=True))
        self.assertFalse(rig.step(hand(pinch=True, tracked=False)).deadman)
        rig.step(hand(pinch=True))
        rig.step(hand())
        rig.step(hand(pinch=True))
        # The clutch hand leaving view counts as an open pinch (same grace).
        cmds = [rig.step(hand(pinch=True, clutch_tracked=False)) for _ in range(9)]
        self.assertTrue(cmds[0].deadman)
        self.assertFalse(cmds[-1].deadman)

    def test_silent_lens_releases(self):
        rig = Rig()
        rig.engage()
        rig.t += 0.2      # 220 ms without a Lens message: Wi-Fi jitter, still held
        self.assertTrue(rig.step(None).deadman)
        rig.t += 0.15     # 390 ms: past lens_timeout_s (0.3)
        self.assertFalse(rig.step(None).deadman)
        self.assertIn("Lens silent", rig.tele.note)

    def test_room_mode_needs_calibration(self):
        rig = Rig(mode="room")
        self.assertFalse(rig.engage().deadman)
        self.assertIn("calibration", rig.tele.note)


class MappingTest(unittest.TestCase):
    def run_to(self, rig, sample, n=150):
        cmd = None
        for _ in range(n):
            cmd = rig.step(sample)
        return cmd

    def test_forward_hand_moves_arm_plus_x(self):
        rig = Rig()
        rig.engage()
        cmd = self.run_to(rig, hand(wrist=(0.0, 0.0, -50.0), pinch=True))   # 10 cm forward
        self.assertTrue(cmd.deadman)
        self.assertAlmostEqual(cmd.pose[0], 400.0 + 85.0, delta=0.5)   # minus 1.5 cm deadzone
        self.assertAlmostEqual(cmd.pose[1], 0.0, delta=0.1)
        self.assertAlmostEqual(cmd.pose[2], 300.0, delta=0.1)

    def test_right_and_up(self):
        rig = Rig()
        rig.engage()
        cmd = self.run_to(rig, hand(wrist=(10.0, 10.0, -40.0), pinch=True))
        d = math.hypot(10, 10)
        k = (d - 1.5) / d * 100.0
        self.assertAlmostEqual(cmd.pose[1], -k, delta=0.5)    # right -> -Y
        self.assertAlmostEqual(cmd.pose[2], 300.0 + k, delta=0.5)

    def test_deadzone_swallows_jitter(self):
        rig = Rig()
        rig.engage()
        cmd = self.run_to(rig, hand(wrist=(0.8, -0.6, -40.5), pinch=True), 30)
        self.assertEqual(cmd.pose[:3], (400.0, 0.0, 300.0))

    def test_speed_limit(self):
        rig = Rig()
        rig.engage()
        cmd = rig.step(hand(wrist=(0.0, 0.0, -140.0), pinch=True))   # 1 m jump
        self.assertLessEqual(cmd.pose[0] - 400.0, 250.0 * DT + 1e-6)

    def test_range_box(self):
        rig = Rig()
        rig.engage()
        cmd = self.run_to(rig, hand(wrist=(0.0, 0.0, -140.0), pinch=True), 400)
        self.assertAlmostEqual(cmd.pose[0], 900.0, delta=0.5)        # anchor + 500

    def test_scale(self):
        rig = Rig(position_scale=0.5)
        rig.engage()
        cmd = self.run_to(rig, hand(wrist=(0.0, 0.0, -50.0), pinch=True))
        self.assertAlmostEqual(cmd.pose[0], 400.0 + 42.5, delta=0.5)

    def test_yaw_left_turns_tool_about_arm_z(self):
        rig = Rig()
        rig.engage()
        cmd = self.run_to(rig, hand(pinch=True, yaw_deg=30.0))
        w = rotation.rotvec_between((180.0, 0.0, 0.0), cmd.pose[3:])
        self.assertAlmostEqual(w[2], 30.0 - 8.0, delta=0.3)          # minus the deadzone
        self.assertAlmostEqual(w[0], 0.0, delta=0.3)
        self.assertAlmostEqual(w[1], 0.0, delta=0.3)
        self.assertAlmostEqual(cmd.pose[0], 400.0, delta=0.5)        # wrist pivot: no translation

    def test_tilt_past_roll_180_stays_in_the_bridge_box(self):
        # Tool-down anchor sits at roll +-180; a roll the other way must wrap,
        # not go to -190 (the bridge rejects |roll| > 180 and the arm freezes).
        for deg in (5.0, 10.0, -5.0, -10.0, 20.0, -20.0):
            rig = Rig()
            rig.tele.on_status(status(rpy=(-180.0, 0.0, 0.0)), rig.t)
            rig.step(tilted(0.0))
            rig.step(tilted(0.0, pinch=True))
            cmd = None
            for _ in range(150):
                cmd = rig.step(tilted(deg, pinch=True))
            self.assertTrue(cmd.deadman)
            for v in cmd.pose[3:]:
                self.assertTrue(-180.0 <= v <= 180.0, cmd.pose)
            turned = rotation.angle_between((-180.0, 0.0, 0.0), cmd.pose[3:])
            self.assertAlmostEqual(turned, abs(deg) - 8.0 if abs(deg) > 8 else 0.0, delta=0.3)

    def test_orientation_off_holds_anchor(self):
        rig = Rig(enable_orientation=False)
        rig.engage()
        cmd = self.run_to(rig, hand(pinch=True, yaw_deg=40.0))
        self.assertEqual(cmd.pose[3:], (180.0, 0.0, 0.0))

    def test_view_frame_frozen_at_engage(self):
        rig = Rig()
        rig.engage()
        # Turning the head after engaging must not re-map the hand.
        cmd = self.run_to(rig, hand(wrist=(0.0, 0.0, -50.0), pinch=True, head=(1.0, 0.0, 0.0)))
        self.assertAlmostEqual(cmd.pose[0], 485.0, delta=0.5)

    def test_room_mode_after_calibration(self):
        rig = Rig()
        self.assertTrue(rig.tele.calibrate_from_points((0, 0, 0), (25, 0, 0)))   # arm +X = world +X
        self.assertEqual(rig.tele.tuning.mode, "room")
        rig.engage(head=(0.0, 0.0, 1.0))         # head direction is ignored in room mode
        cmd = self.run_to(rig, hand(wrist=(10.0, 0.0, -40.0), pinch=True))
        self.assertAlmostEqual(cmd.pose[0], 485.0, delta=0.5)
        self.assertAlmostEqual(cmd.pose[1], 0.0, delta=0.1)


class ParseTest(unittest.TestCase):
    MSG = {"type": "hand", "drive": {"tracked": True, "wrist": [1, 2, 3], "index": [1, 2, 4],
                                     "middle": [1, 2, 5], "pinky": [2, 2, 4]},
           "clutch": {"tracked": True, "pinch": True}, "head": {"fwd": [0, 0, -1]}}

    def test_parses(self):
        s = LensSample.from_json(self.MSG)
        self.assertTrue(s.drive_tracked and s.pinch and s.clutch_tracked)
        self.assertEqual(s.wrist, (1.0, 2.0, 3.0))
        self.assertEqual(s.head_fwd, (0.0, 0.0, -1.0))

    def test_untracked_needs_no_points(self):
        s = LensSample.from_json({"drive": {"tracked": False}, "clutch": {}})
        self.assertFalse(s.drive_tracked or s.pinch)

    def test_rejects_bad_vectors(self):
        bad = dict(self.MSG, drive=dict(self.MSG["drive"], wrist=[1, float("nan"), 3]))
        with self.assertRaises(ValueError):
            LensSample.from_json(bad)
        bad = dict(self.MSG, drive=dict(self.MSG["drive"], wrist=[1, 2]))
        with self.assertRaises(ValueError):
            LensSample.from_json(bad)

    def test_tuning_rejects_unknown_keys(self):
        with self.assertRaises(ValueError):
            Tuning.from_dict({"positon_scale": 2})
        self.assertEqual(Tuning.from_dict({"_note": "x", "view_yaw_deg": 90}).view_yaw_deg, 90.0)


def twin(active, pose=(400.0, 0.0, 300.0, 180.0, 0.0, 0.0)):
    return TwinSample.from_json({"type": "twin", "active": active, "pose": list(pose)})


class TwinTest(unittest.TestCase):
    """Digital twin: absolute model TCP targets, relative arm motion, same clutch rules."""

    def step(self, rig, sample=None, st=None):
        rig.t += DT
        rig.tele.on_status(st or rig.tele.status, rig.t)
        if sample is not None:
            rig.tele.on_twin(sample, rig.t)
        return rig.tele.tick(rig.t)

    def grab(self, rig, pose=(400.0, 0.0, 300.0, 180.0, 0.0, 0.0)):
        self.step(rig, twin(False, pose))
        return self.step(rig, twin(True, pose))

    def test_press_edge_engages_and_idle_echoes_actual(self):
        rig = Rig()
        self.assertFalse(self.step(rig, twin(False)).deadman)
        self.assertTrue(self.grab(rig).deadman)
        self.assertIn("twin", rig.tele.note)

    def test_active_from_the_start_does_not_engage(self):
        rig = Rig()
        for _ in range(5):
            self.assertFalse(self.step(rig, twin(True)).deadman)

    def test_arm_follows_the_models_motion_relative_to_the_grab(self):
        rig = Rig()                                     # actual arm at (400, 0, 300)
        # The model's TCP is drawn 5 mm off the actual pose: no jump at the grab.
        cmd = self.grab(rig, (405.0, 0.0, 300.0, 180.0, 0.0, 0.0))
        self.assertAlmostEqual(cmd.pose[0], 400.0, delta=1e-6)
        for _ in range(100):
            cmd = self.step(rig, twin(True, (435.0, -20.0, 310.0, 180.0, 0.0, 0.0)))
        self.assertAlmostEqual(cmd.pose[0], 430.0, delta=0.01)
        self.assertAlmostEqual(cmd.pose[1], -20.0, delta=0.01)
        self.assertAlmostEqual(cmd.pose[2], 310.0, delta=0.01)

    def test_orientation_follows_even_with_hand_orientation_off(self):
        rig = Rig(enable_orientation=False)
        self.grab(rig)
        for _ in range(100):
            cmd = self.step(rig, twin(True, (400.0, 0.0, 300.0, 180.0, 0.0, 20.0)))
        w = rotation.rotvec_between((180.0, 0.0, 0.0), cmd.pose[3:])
        self.assertAlmostEqual(w[2], 20.0, delta=0.05)
        for v in cmd.pose[3:]:
            self.assertTrue(-180.0 <= v <= 180.0)      # principal RPY for the bridge box

    def test_twin_orientation_off_holds_the_tool_rotation(self):
        rig = Rig(twin_orientation=False)
        self.grab(rig)
        for _ in range(50):
            cmd = self.step(rig, twin(True, (420.0, 0.0, 300.0, 180.0, 0.0, 20.0)))
        self.assertAlmostEqual(rotation.angle_between((180.0, 0.0, 0.0), cmd.pose[3:]), 0.0, delta=1e-6)
        self.assertAlmostEqual(cmd.pose[0], 420.0, delta=0.01)

    def test_speed_limit_and_range_box(self):
        rig = Rig()
        self.grab(rig)
        cmd = self.step(rig, twin(True, (1400.0, 0.0, 300.0, 180.0, 0.0, 0.0)))
        self.assertLessEqual(cmd.pose[0] - 400.0, 60.0 * DT + 1e-6)      # twin_max_speed_mm_s
        for _ in range(500):                                             # 10 s at 60 mm/s
            cmd = self.step(rig, twin(True, (1400.0, 0.0, 300.0, 180.0, 0.0, 0.0)))
        self.assertAlmostEqual(cmd.pose[0], 900.0, delta=0.01)          # anchor + max_range_mm

    def test_twin_speeds_are_separate_from_the_hand_and_tunable(self):
        rig = Rig(twin_max_speed_mm_s=20.0, twin_max_orient_speed_deg_s=5.0)
        self.grab(rig)
        cmd = None
        for _ in range(50):                                              # 1 s
            cmd = self.step(rig, twin(True, (500.0, 0.0, 300.0, 180.0, 0.0, 30.0)))
        self.assertAlmostEqual(cmd.pose[0], 420.0, delta=0.01)          # 20 mm/s x 1 s
        turned = rotation.angle_between((180.0, 0.0, 0.0), cmd.pose[3:])
        self.assertAlmostEqual(turned, 5.0, delta=0.05)                 # 5 deg/s x 1 s

    def test_release_on_let_go_silence_and_fault(self):
        rig = Rig()
        self.grab(rig)
        self.assertFalse(self.step(rig, twin(False)).deadman)       # immediate, no grace
        self.assertIn("let go", rig.tele.note)
        rig = Rig()
        self.grab(rig)
        rig.t += 0.5                                                 # > lens_timeout_s (0.3)
        self.assertFalse(self.step(rig).deadman)
        self.assertIn("silent", rig.tele.note)
        rig = Rig()
        self.grab(rig)
        self.assertFalse(self.step(rig, twin(True), st=status(err=31)).deadman)

    def test_refused_without_servo_then_needs_a_new_edge(self):
        rig = Rig()
        ready = status(state=ArmState.READY)
        self.step(rig, twin(False), st=ready)
        self.assertFalse(self.step(rig, twin(True), st=ready).deadman)
        self.assertIn("READY", rig.tele.note)
        self.assertFalse(self.step(rig, twin(True), st=status()).deadman)  # still held
        self.step(rig, twin(False))
        self.assertTrue(self.step(rig, twin(True)).deadman)

    def test_twin_takes_over_from_a_hand_grab(self):
        rig = Rig()
        self.assertTrue(rig.engage().deadman)                        # hand grab
        cmd = self.step(rig, twin(False))
        self.assertFalse(cmd.deadman)
        self.assertIn("twin took over", rig.tele.note)
        # twin quiet again: the hand needs a fresh pinch edge, a held pinch does nothing
        rig.t += 0.5
        self.assertFalse(rig.step(hand(pinch=True)).deadman)
        rig.step(hand())
        self.assertTrue(rig.step(hand(pinch=True)).deadman)

    def test_parse_rejects_bad_poses(self):
        with self.assertRaises(ValueError):
            TwinSample.from_json({"type": "twin", "active": True, "pose": [1, 2, 3]})
        with self.assertRaises(ValueError):
            TwinSample.from_json({"type": "twin", "active": True, "pose": [1, 2, 3, 4, float("nan"), 6]})


if __name__ == "__main__":
    unittest.main()
