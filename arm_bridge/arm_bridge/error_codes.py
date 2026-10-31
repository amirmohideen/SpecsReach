"""Teleop error codes: the bridge's own vendor-neutral fault vocabulary.

STATUS `errorCode` / `warnCode` (ARM-BRIDGE-QUEST-HANDOFF.md §5) carry codes
from THIS table, whatever arm is behind the bridge; each backend maps its
native faults onto it here (`from_xarm_*`, `aubo_*`). Ranges:

  error codes
    1-999      robot/controller conditions. Numbering == the UFACTORY
               controller error codes (xArm-Python-SDK doc/api/xarm_api_code.md,
               "Controller Error Code"), so the xArm mapping is the identity and
               every label the Quest already shows stays valid.
    1000-1999  robot/controller conditions UFACTORY has no code for (derived
               from AUBO's safety/robot modes).
    2000-2999  bridge/driver faults (SDK call failed, identity mismatch, ...).
  warn codes
    1-999      == UFACTORY controller warn codes (identity for the xArm).
    1000-1999  teleop warnings (target held: no IK solution, joint limit, ...).

Legacy namespace: STATUS flag bit4 (`errorCode` is an SDK API code) is still
how the xArm backend reports a failed SDK call -- its ArmFault codes are xArm
API return codes, a different table. New backends raise ArmFault with a
2000-2999 teleop code instead, and `is_teleop_code()` keeps bit4 off for them.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CodeInfo:
    code: int
    name: str
    label: str


def _table(*rows: tuple[int, str, str]) -> dict[int, CodeInfo]:
    table = {code: CodeInfo(code, name, label) for code, name, label in rows}
    assert len(table) == len(rows), "duplicate code"
    return table


# ---- error codes ------------------------------------------------------------

ERRORS: dict[int, CodeInfo] = _table(
    # 1-999: UFACTORY controller error codes (identity for the xArm).
    (1, "ESTOP_BUTTON", "Emergency stop button is pushed"),
    (2, "ESTOP_IO", "Emergency stop input of the control box is triggered"),
    (3, "ESTOP_ENABLING_DEVICE", "Emergency stop of the three-state switch is pressed"),
    (10, "SERVO_MOTOR", "Servo motor error"),
    (11, "SERVO_MOTOR_1", "Joint 1 servo motor error"),
    (12, "SERVO_MOTOR_2", "Joint 2 servo motor error"),
    (13, "SERVO_MOTOR_3", "Joint 3 servo motor error"),
    (14, "SERVO_MOTOR_4", "Joint 4 servo motor error"),
    (15, "SERVO_MOTOR_5", "Joint 5 servo motor error"),
    (16, "SERVO_MOTOR_6", "Joint 6 servo motor error"),
    (17, "SERVO_MOTOR_7", "Joint 7 servo motor error"),
    (18, "FT_SENSOR_COMM", "Force/torque sensor communication error"),
    (19, "END_MODULE_COMM", "End module communication error"),
    (21, "KINEMATIC", "Kinematic error"),
    (22, "SELF_COLLISION", "Self-collision"),
    (23, "JOINT_LIMIT", "Joint angle exceeds limit"),
    (24, "SPEED_LIMIT", "Speed exceeds limit"),
    (25, "PLANNING", "Planning error"),
    (26, "LINUX_RT", "Linux RT error"),
    (27, "COMMAND_REPLY", "Command reply error"),
    (28, "END_MODULE_COMM_2", "End module communication error"),
    (29, "OTHER", "Other controller error"),
    (30, "FEEDBACK_SPEED_LIMIT", "Feedback speed exceeds limit"),
    (31, "COLLISION", "Collision caused abnormal current"),
    (32, "CIRCLE_CALC", "Three-point circle calculation error"),
    (33, "CONTROLLER_GPIO", "Controller GPIO error"),
    (34, "RECORDING_TIMEOUT", "Recording timeout"),
    (35, "SAFETY_BOUNDARY", "Safety boundary limit"),
    (36, "DELAY_COMMANDS", "Too many delay commands"),
    (37, "MANUAL_MODE_MOTION", "Abnormal movement in manual mode"),
    (38, "JOINT_ANGLE_ABNORMAL", "Abnormal joint angle"),
    (39, "POWER_BOARD_COMM", "Power board master/slave communication error"),
    (40, "NO_IK", "No inverse kinematics solution"),
    (50, "FT_SENSOR_READ", "Six-axis force/torque sensor read error"),
    (51, "FT_SENSOR_MODE", "Six-axis force/torque sensor set-mode error"),
    (52, "FT_SENSOR_ZERO", "Six-axis force/torque sensor set-zero error"),
    (53, "FT_SENSOR_OVERLOAD", "Six-axis force/torque sensor overloaded"),
    (110, "BASE_BOARD_COMM", "Arm base board communication error"),
    (111, "EXTERNAL_485_COMM", "Control box external RS-485 device communication error"),
    # 1000-1999: conditions without a UFACTORY code (from AUBO's modes).
    (1001, "POWERED_OFF", "Robot is powered off or its brakes are engaged"),
    (1002, "SAFEGUARD_STOP", "Safeguard stop input is active"),
    (1003, "PROTECTIVE_STOP", "Protective stop"),
    (1004, "SAFETY_VIOLATION", "Safety limit violation"),
    (1005, "SAFETY_FAULT", "Safety system fault"),
    (1006, "RECOVERY_MODE", "Outside the safety limits: recovery mode"),
    (1007, "ROBOT_ERROR", "Robot controller reports an error state"),
    (1008, "CONTROLLER_DISCONNECTED", "Arm is disconnected from its controller"),
    # 2000-2999: bridge/driver faults.
    (2001, "SDK_CALL_FAILED", "Robot SDK call failed"),
    (2002, "CONTROLLER_UNREACHABLE", "Robot controller unreachable"),
    (2003, "IDENTITY_MISMATCH", "Connected robot does not match robot_model"),
    (2004, "VERSION_MISMATCH", "SDK and controller interface versions differ"),
    (2005, "NOT_READY", "Robot is not ready for this command"),
    (2006, "TIMEOUT", "Robot did not reach the expected state in time"),
    (2007, "PROGRAM_RUNNING", "A controller/pendant program is running"),
    (2008, "UNSUPPORTED", "Not supported by this arm backend"),
    (2009, "PROGRAM_MODE", "Station is in Program mode (switch on the dashboard)"),
    (2010, "OUTSIDE_ENVELOPE", "Arm posture is outside the configured envelope"),
)

# Named constants for the codes the code base refers to.
ESTOP_BUTTON = 1
ESTOP_IO = 2
KINEMATIC = 21
COLLISION = 31
JOINT_LIMIT = 23
POWERED_OFF = 1001
SAFEGUARD_STOP = 1002
PROTECTIVE_STOP = 1003
SAFETY_VIOLATION = 1004
SAFETY_FAULT = 1005
RECOVERY_MODE = 1006
ROBOT_ERROR = 1007
CONTROLLER_DISCONNECTED = 1008
SDK_CALL_FAILED = 2001
CONTROLLER_UNREACHABLE = 2002
IDENTITY_MISMATCH = 2003
VERSION_MISMATCH = 2004
NOT_READY = 2005
TIMEOUT = 2006
PROGRAM_RUNNING = 2007
UNSUPPORTED = 2008
PROGRAM_MODE = 2009
OUTSIDE_ENVELOPE = 2010


def servo_motor_error(joint_index: int) -> int:
    """Per-joint servo motor error for a 0-based joint index (11..17)."""
    return 11 + joint_index if 0 <= joint_index < 7 else 10


# ---- warn codes -------------------------------------------------------------

WARNINGS: dict[int, CodeInfo] = _table(
    # 1-999: UFACTORY controller warn codes (identity for the xArm).
    (11, "CMD_QUEUE_FULL", "Controller command queue is full"),
    (12, "PARAMETER", "Parameter error"),
    (13, "NO_SUCH_INSTRUCTION", "Instruction does not exist"),
    (14, "NO_SOLUTION", "Command has no solution"),
    (15, "MODBUS_FULL", "Modbus command queue is full"),
    # 1000-1999: teleop warnings. "Target held" = the arm stays at its last
    # good command and moves again once the target becomes valid.
    (1001, "TARGET_NO_IK", "Target held: no inverse kinematics solution"),
    (1002, "TARGET_JOINT_LIMIT", "Target held: a joint would exceed its limit"),
    (1003, "TARGET_IK_FLIP", "Target held: arm configuration would flip (singularity)"),
    (1004, "REDUCED_MODE", "Robot is in reduced (slow) safety mode"),
    (1005, "TARGET_SELF_COLLISION", "Target held: controller predicts a self collision"),
    (1006, "OUTSIDE_ENVELOPE", "Arm is outside the envelope: RETREAT or hand-guide it before driving"),
)

TARGET_NO_IK = 1001
TARGET_JOINT_LIMIT = 1002
TARGET_IK_FLIP = 1003
REDUCED_MODE = 1004
TARGET_SELF_COLLISION = 1005
WARN_OUTSIDE_ENVELOPE = 1006


def is_teleop_code(code: int) -> bool:
    """True for codes in this table's own ranges (1000+). Codes below 1000 are
    UFACTORY-numbered; an ArmFault code there may still be a legacy xArm SDK
    API code (STATUS bit4)."""
    return 1000 <= code < 3000


def error_label(code: int) -> str:
    info = ERRORS.get(code)
    return info.label if info else f"error {code}"


def warn_label(code: int) -> str:
    info = WARNINGS.get(code)
    return info.label if info else f"warning {code}"


# ---- brand mappings ---------------------------------------------------------

def from_xarm_controller_error(code: int) -> int:
    """UFACTORY controller error -> teleop error: the identity (same numbering)."""
    return int(code)


def from_xarm_controller_warn(code: int) -> int:
    """UFACTORY controller warn -> teleop warn: the identity (same numbering)."""
    return int(code)


def aubo_fault(robot_mode: str, safety_mode: str, collision: bool = False,
               faulted_joint: int | None = None) -> tuple[int, bool]:
    """Map AUBO ARCS modes (enum `.name` strings, aubo-control-notes.md §5.3)
    to (teleop error code, protective_stop). (0, False) = no fault.

    Safety modes win over robot modes: an e-stop or protective stop applies
    whatever the robot mode is."""
    if safety_mode == "RobotEmergencyStop":
        return ESTOP_BUTTON, False
    if safety_mode == "SystemEmergencyStop":
        return ESTOP_IO, False
    if safety_mode == "ProtectiveStop":
        return (COLLISION if collision else PROTECTIVE_STOP), True
    if safety_mode == "SafeguardStop":
        return SAFEGUARD_STOP, True
    if safety_mode == "Violation":
        return SAFETY_VIOLATION, False
    if safety_mode == "Fault":
        return (servo_motor_error(faulted_joint) if faulted_joint is not None
                else SAFETY_FAULT), False
    if safety_mode == "Recovery":
        return RECOVERY_MODE, False
    if robot_mode == "Error":
        return (servo_motor_error(faulted_joint) if faulted_joint is not None
                else ROBOT_ERROR), False
    if robot_mode == "Disconnected":
        return CONTROLLER_DISCONNECTED, False
    if collision:
        return COLLISION, True
    return 0, False
