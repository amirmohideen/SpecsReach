"""Arm bridge entrypoint.

Wires config -> Session -> ArmController -> ServoLoop (250 Hz) and the WebRTC
DataChannel transport + signaling server, then runs until interrupted. The
servo loop and transport run on their own threads; main just waits.

    python3 -m arm_bridge.main --signaling-port 8444 [--config bridge.yaml]
        [--stun-server host:port]

The arm backend is chosen by the **config** (`arm_backend: mock|sim|xarm|aubo`),
not a CLI flag. Default `mock` runs with no hardware. `xarm` / `aubo` select the
real controllers (SDK-gated); they fail closed if the config is uncommissioned or
the SDK/arm is missing -- never falling back to mock.

Config precedence: built-in defaults < `--config` YAML (schema in config.py)
< `--stun-server` CLI override.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading

from .arm_interface import ArmFault, MockArmController
from .capture_tap import ArmCaptureTap
from .config import ArmBridgeConfig, ConfigError, resolve_config
from .instrument import LoggingArmController
from .logbus import LogBus, wire_logging
from .program_supervisor import ProgramSupervisor, ViewerCounter
from .robot_models import default_catalog
from .servo import ServoLoop
from .session import Session
from .signaling import SignalingServer
from .statehub import StateHub
from .transport_webrtc import WebRtcArmTransport

log = logging.getLogger("arm_bridge.main")


# How long to wait between arm-connect attempts on the dashboard path (see
# connect_arm_with_retry). Loose: a powered-off / unreachable arm is common and
# non-urgent; the dashboard stays observable and self-heals when it returns.
ARM_RECONNECT_INTERVAL_S = 5.0


def construct_arm(config: ArmBridgeConfig):
    """Construct the ArmController selected by `config.arm_backend` WITHOUT
    connecting. Splitting construct from connect lets the caller bring the
    dashboard up first, so a connect failure is surfaced there instead of taking
    the (in-process) dashboard down with it. Unknown backend still fails closed."""
    if config.arm_backend == "mock":
        return MockArmController()
    if config.arm_backend == "sim":
        from .sim_arm import SimArmController   # lazy: only imports pybullet here
        return SimArmController(config)
    if config.arm_backend == "xarm":
        from .xarm_controller import XArmController   # lazy: only touches the SDK here
        return XArmController(config)
    if config.arm_backend == "aubo":
        from .aubo_controller import AuboController   # lazy: pyaubo_sdk loads at connect
        return AuboController(config)
    # config.validate() already rejects unknown backends; belt and braces.
    print(f"arm: unknown arm_backend '{config.arm_backend}'", file=sys.stderr)
    sys.exit(2)


def build_arm(config: ArmBridgeConfig):
    """Construct AND connect the arm, failing closed (exit non-zero) on a
    connect/commissioning failure -- never silently falls back to the mock. Used
    on the HEADLESS path (no dashboard). With a dashboard configured, main()
    connects via `connect_arm_with_retry` instead so an unreachable arm keeps the
    dashboard observable rather than crash-looping the container."""
    arm = construct_arm(config)
    try:
        arm.connect()
    except ArmFault as exc:
        if config.arm_backend == "sim":
            print(f"arm: sim connect failed: {exc.message} "
                  "(pybullet + vendored URDF required; see tools/fetch_urdf.sh)",
                  file=sys.stderr)
        else:
            print(f"arm: {config.arm_backend} connect failed: {exc.message}",
                  file=sys.stderr)
        sys.exit(2)
    return arm


def connect_arm_with_retry(arm, bus: LogBus, stop: threading.Event,
                           config: ArmBridgeConfig) -> bool:
    """Connect `arm`, retrying every `ARM_RECONNECT_INTERVAL_S` until it succeeds
    or `stop` fires. Keeps the in-process dashboard alive so a powered-off /
    unreachable arm is observable (and self-heals when it returns) instead of
    crash-looping the container. Returns True once connected, False if stopped
    first. Each attempt also logs through the arm's `robot` source."""
    attempt = 0
    while not stop.is_set():
        try:
            arm.connect()
            if attempt:
                bus.publish("system", "info",
                            f"arm connected after {attempt} failed attempt(s)")
            return True
        except ArmFault as exc:
            attempt += 1
            bus.publish("system", "error",
                        f"arm ({config.arm_backend}) connect failed "
                        f"(attempt {attempt}): {exc.message}; retrying in "
                        f"{ARM_RECONNECT_INTERVAL_S:.0f}s")
            stop.wait(ARM_RECONNECT_INTERVAL_S)
    return False


def repo_root_for(config_path: str):
    """Repo root: the parent of the config dir when --config was given (the
    editable files and scripts/ are siblings of it), else cwd's parent (the
    containers run with -w /workspace/arm_bridge)."""
    from pathlib import Path
    config_dir = Path(config_path).resolve().parent if config_path else None
    return config_dir, (config_dir.parent if config_dir else Path.cwd().parent)


def build_programs(config: ArmBridgeConfig, config_path: str, arm, viewers):
    """Programs tab plumbing -- only with a dashboard and a backend that runs
    controller programs. Returns (supervisor, store) or (None, None)."""
    if config.dashboard_port is None or not getattr(arm, "supports_programs", False):
        return None, None
    from pathlib import Path

    from .dashboard.programs_store import ProgramsStore   # stdlib only
    _, repo_root = repo_root_for(config_path)
    root = Path(config.programs.dir)
    root = root if root.is_absolute() else repo_root / root
    try:
        store = ProgramsStore(root, history_keep=config.programs.history_keep)
    except OSError as exc:
        # Fail closed with the fix in the message: a programs/ created by an
        # earlier root container is the usual cause (scripts/start_all.sh).
        print(f"arm bridge: programs.dir {root} is not usable ({exc}); make it "
              f"writable by this user (sudo chown -R $(id -u):$(id -g) {root}) or "
              "set programs.dir", file=sys.stderr)
        sys.exit(2)
    supervisor = ProgramSupervisor(arm, watchdog_s=config.programs.watchdog_s,
                                   viewer_count=viewers.get, on_run_end=store.record_run)
    log.info("programs: %s (features: %s)", root,
             ", ".join(sorted(supervisor.features)) or "run/stop only")
    return supervisor, store


def build_dashboard(config: ArmBridgeConfig, hub: StateHub, bus: LogBus,
                    config_path: str, arm, session, servo, on_restart,
                    programs_store=None, viewers=None):
    """Import-gated like the xArm SDK: fastapi/uvicorn are only required when
    `dashboard_port` is configured. Fail closed -- a configured dashboard that
    cannot start is an error, never a silent skip."""
    if config.dashboard_port is None:
        return None
    try:
        from .dashboard.recording import RecordingSupervisor
        from .dashboard.server import DashboardServer
    except ImportError as exc:
        print("arm bridge: dashboard_port is set but the dashboard deps are "
              f"missing (pip: fastapi uvicorn websockets): {exc}", file=sys.stderr)
        sys.exit(2)
    from .control_pin import pin_file
    from .dashboard.control_auth import ControlAuth
    config_dir, repo_root = repo_root_for(config_path)
    recorder = None
    if (repo_root / "scripts" / "record_rosbag.sh").exists():
        recorder = RecordingSupervisor(
            repo_root, bus,
            extra_meta={"arm_backend": config.arm_backend,
                        "bridge_boot_id": hub.boot_id})
    # The PIN file sits next to the config (never inside it: the config
    # editor could rewrite it). No --config -> no PIN -> control refused.
    auth = ControlAuth(pin_file(config_dir) if config_dir else None)
    return DashboardServer(config, hub, bus, config_dir=config_dir, arm=arm,
                           session=session, servo=servo, recorder=recorder,
                           on_restart=on_restart, programs_store=programs_store,
                           control_auth=auth, viewers=viewers)


def main() -> None:
    parser = argparse.ArgumentParser(description="Quest -> xArm bridge (WebRTC DataChannels).")
    parser.add_argument("--signaling-port", type=int, default=8444)
    parser.add_argument("--signaling-allow-ip", default="")
    parser.add_argument("--config", default="",
                        help="YAML config file (schema documented in config.py); "
                             "arm_backend and xarm_ip live here.")
    parser.add_argument("--stun-server", default=None, metavar="HOST:PORT",
                        help="STUN server (bare host:port); overrides the config "
                             "file. Pass an empty string to force LAN mode.")
    args = parser.parse_args()

    try:
        config = resolve_config(args.config or None, args.stun_server)
    except ConfigError as exc:
        print(f"arm bridge: config error: {exc}", file=sys.stderr)
        sys.exit(2)

    # LogBus + journal logging exist unconditionally (the bus is cheap with no
    # subscribers); the StateHub + dashboard server only when configured.
    bus = LogBus()
    wire_logging(bus)
    dashboard_on = config.dashboard_port is not None

    # With a dashboard, defer the arm connect (construct only) so observability
    # comes up first; headless keeps the fail-closed connect-or-exit. The arm
    # SDK, servo loop, transport, and dashboard all just hold references -- none
    # need a *connected* arm to be constructed.
    raw_arm = construct_arm(config) if dashboard_on else build_arm(config)
    arm = LoggingArmController(raw_arm, bus)
    # The hub stamps the arm's robot model (config override, else what the
    # backend reports -- possibly learned at connect) on every snapshot.
    hub = StateHub(config.arm_backend, config.servo_hz, config.status_hz,
                   catalog=default_catalog(), model_source=arm.robot_model,
                   model_override=config.robot_model,
                   power_source=arm.power_info if arm.supports_power else None) \
        if dashboard_on else None
    if hub is not None:
        for error in default_catalog().errors:
            bus.publish("system", "warn", f"robot model skipped: {error}")
    session = Session(config)
    viewers = ViewerCounter()
    programs, programs_store = build_programs(config, args.config, arm, viewers)
    # Data capture (CAPTURE-SYNC-PLAN.md S3): arm records to the capture node.
    # Inert until a recording's capture node listens; never touches motion.
    capture_tap = None
    if config.capture.enabled:
        capture_tap = ArmCaptureTap(config.capture.socket_name, config.capture.queue_max)
        capture_tap.start()
        raw_arm.set_capture_tap(capture_tap)
    servo = ServoLoop(config, session, arm, programs=programs, capture_tap=capture_tap,
                      capture_snapshots=not raw_arm.captures_state)
    signaling = SignalingServer(args.signaling_port, args.signaling_allow_ip or None)
    transport = WebRtcArmTransport(config, session, servo, signaling, state_hub=hub,
                                   capture_tap=capture_tap)
    # Dashboard-requested restart = the normal clean shutdown; the container
    # restart policy (start_all.sh applies unless-stopped) does the respawn.
    stop = threading.Event()
    dashboard = build_dashboard(config, hub, bus, args.config, arm, session,
                                servo, on_restart=stop.set,
                                programs_store=programs_store, viewers=viewers) \
        if hub is not None else None

    # Register signal handlers before the (possibly long) arm-connect retry so a
    # container stop / Ctrl-C interrupts it cleanly.
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    # Bring the dashboard, signaling and transport up FIRST -- neither
    # observability NOR the Quest's ability to connect may be gated on the arm.
    # The signaling listener (the port the Quest dials) and the WebRTC transport
    # start unconditionally, so a powered-off / unreachable arm no longer blocks
    # the connection: the Quest connects, and STATUS reports armState=UNKNOWN
    # (read_snapshot() returns UNKNOWN until connected) which the client renders
    # as "arm offline" rather than a failed connection. Only the servo loop --
    # the single actuator -- waits for a connected arm.
    if dashboard is not None:
        dashboard.start()
    # transport.start() FIRST: it registers the signaling handlers
    # (set_handlers) that _accept_loop/_dispatch need. Starting signaling before
    # them would drop a peer's HELLO that arrives in the gap (no _on_peer/
    # _on_message set yet) and the armctl DataChannel would never open.
    transport.start()
    signaling.start()

    # `servo_started` gates teardown of the actuation path: True once the servo
    # loop is running (arm connected). Read on the main thread, set on either the
    # connector thread (dashboard path) or here (headless).
    servo_started = threading.Event()

    def _connect_then_servo() -> None:
        # Off the main thread so an absent/unreachable arm never blocks the
        # signaling/transport bring-up above. Start the actuator once connected,
        # unless we've already begun shutting down.
        if connect_arm_with_retry(arm, bus, stop, config) and not stop.is_set():
            servo.start()
            servo_started.set()

    if hub is not None:
        connector = threading.Thread(target=_connect_then_servo,
                                     name="arm-connect", daemon=True)
        connector.start()
    else:
        # Headless: build_arm already connected (or exited non-zero), so the arm
        # is live now -- start the servo loop immediately, no connector thread.
        connector = None
        servo.start()
        servo_started.set()
    log.info("arm bridge: signaling ws://:%s arm=%s servo=%.0fHz watchdog=%.0fms "
             "stun=%s dashboard=%s connect=%s",
             args.signaling_port, config.arm_backend, config.servo_hz,
             config.watchdog_ms, config.stun_server or "off",
             f"http://{config.dashboard_bind}:{config.dashboard_port}"
             if dashboard is not None else "off",
             "deferred" if connector is not None else "eager")

    try:
        stop.wait()
    finally:
        log.info("arm bridge: shutting down")
        # Let the connector's retry loop observe `stop` and exit (it may still be
        # mid-connect); bounded join so a blocking SDK connect can't hang us.
        if connector is not None:
            connector.join(timeout=ARM_RECONNECT_INTERVAL_S)
        if dashboard is not None:
            dashboard.stop()
        # Signaling + transport always came up, so always tear them down.
        signaling.stop()
        transport.stop()
        # Actuation only if the servo loop actually started (arm connected);
        # arm.disconnect() is best-effort/idempotent, so it is safe to call for
        # any arm the connector may have connected without arming the servo.
        if servo_started.is_set():
            servo.stop()
        arm.disconnect()
        if capture_tap is not None:
            capture_tap.stop()


if __name__ == "__main__":
    main()
