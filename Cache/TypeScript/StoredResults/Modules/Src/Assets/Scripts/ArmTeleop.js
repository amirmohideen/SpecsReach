"use strict";
var __esDecorate = (this && this.__esDecorate) || function (ctor, descriptorIn, decorators, contextIn, initializers, extraInitializers) {
    function accept(f) { if (f !== void 0 && typeof f !== "function") throw new TypeError("Function expected"); return f; }
    var kind = contextIn.kind, key = kind === "getter" ? "get" : kind === "setter" ? "set" : "value";
    var target = !descriptorIn && ctor ? contextIn["static"] ? ctor : ctor.prototype : null;
    var descriptor = descriptorIn || (target ? Object.getOwnPropertyDescriptor(target, contextIn.name) : {});
    var _, done = false;
    for (var i = decorators.length - 1; i >= 0; i--) {
        var context = {};
        for (var p in contextIn) context[p] = p === "access" ? {} : contextIn[p];
        for (var p in contextIn.access) context.access[p] = contextIn.access[p];
        context.addInitializer = function (f) { if (done) throw new TypeError("Cannot add initializers after decoration has completed"); extraInitializers.push(accept(f || null)); };
        var result = (0, decorators[i])(kind === "accessor" ? { get: descriptor.get, set: descriptor.set } : descriptor[key], context);
        if (kind === "accessor") {
            if (result === void 0) continue;
            if (result === null || typeof result !== "object") throw new TypeError("Object expected");
            if (_ = accept(result.get)) descriptor.get = _;
            if (_ = accept(result.set)) descriptor.set = _;
            if (_ = accept(result.init)) initializers.unshift(_);
        }
        else if (_ = accept(result)) {
            if (kind === "field") initializers.unshift(_);
            else descriptor[key] = _;
        }
    }
    if (target) Object.defineProperty(target, contextIn.name, descriptor);
    done = true;
};
var __runInitializers = (this && this.__runInitializers) || function (thisArg, initializers, value) {
    var useValue = arguments.length > 2;
    for (var i = 0; i < initializers.length; i++) {
        value = useValue ? initializers[i].call(thisArg, value) : initializers[i].call(thisArg);
    }
    return useValue ? value : void 0;
};
var __setFunctionName = (this && this.__setFunctionName) || function (f, name, prefix) {
    if (typeof name === "symbol") name = name.description ? "[".concat(name.description, "]") : "";
    return Object.defineProperty(f, "name", { configurable: true, value: prefix ? "".concat(prefix, " ", name) : name });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.ArmTeleop = void 0;
var __selfType = requireType("./ArmTeleop");
function component(target) {
    target.getTypeName = function () { return __selfType; };
    if (target.prototype.hasOwnProperty("getTypeName"))
        return;
    Object.defineProperty(target.prototype, "getTypeName", {
        value: function () { return __selfType; },
        configurable: true,
        writable: true
    });
}
const SIK_1 = require("SpectaclesInteractionKit.lspkg/SIK");
const WorldCameraFinderProvider_1 = require("SpectaclesInteractionKit.lspkg/Providers/CameraProvider/WorldCameraFinderProvider");
const Interactable_1 = require("SpectaclesInteractionKit.lspkg/Components/Interaction/Interactable/Interactable");
const Interactor_1 = require("SpectaclesInteractionKit.lspkg/Core/Interactor/Interactor");
const ArmAxes_1 = require("./ArmAxes");
const ClutchControl_1 = require("./ClutchControl");
/**
 * Streams Spectacles hand tracking to the SpecsReach relay, which drives the
 * AUBO arm through the teleop arm bridge.
 *
 * Like HandBridge (SpecsGrip), this script is deliberately "dumb": it sends the
 * raw keypoints of the DRIVE hand (wrist + three knuckles), whether the CLUTCH
 * hand is pinching, and where the head is looking. Anchoring, the frame
 * mapping, filtering, gating and limits all live in the relay
 * (teleop/teleop-server/tools/specs_relay), so retuning is a JSON edit on the
 * station, not a Lens rebuild.
 *
 * Control: pinch with the clutch hand (default left) to grab the arm; the
 * arm's tool then follows the drive hand's wrist (default right). Release the
 * pinch to let go, re-pinch from a comfortable spot to reach further.
 * Alternatively, toggle mode: a DRIVE button (pressed with the clutch hand)
 * starts and stops the grab instead of a held pinch (ClutchControl.ts).
 *
 * The arm's X/Y/Z axes are drawn at the drive wrist (ArmAxes.ts), using the
 * relay's view-mode mapping: dim and following the head before a grab, bright
 * and frozen while grabbing.
 *
 * Wire format (one JSON text frame per tick, positions in cm, Lens world):
 *   {"type":"hand","t":<ms>,
 *    "drive":{"tracked":true,"wrist":[x,y,z],"index":[..],"middle":[..],"pinky":[..]},
 *    "clutch":{"tracked":true,"pinch":false,"mode":"pinch"},
 *    "head":{"fwd":[x,y,z]}}
 * plus one {"type":"hello","boot":"<id>","drive":"right","clutch":"left"} on
 * connect. In toggle mode clutch.tracked is always true and clutch.pinch is
 * the DRIVE state. The relay answers ~10 Hz with {"type":"state",...} for the
 * HUD.
 */
const COLOR_OK = new vec4(0.2, 0.9, 0.4, 1.0);
const COLOR_WARN = new vec4(1.0, 0.75, 0.2, 1.0);
const COLOR_BAD = new vec4(1.0, 0.35, 0.35, 1.0);
const COLOR_DRIVE = new vec4(0.35, 0.75, 1.0, 1.0);
function v3(p) {
    return [Math.round(p.x * 100) / 100, Math.round(p.y * 100) / 100, Math.round(p.z * 100) / 100];
}
/** Teleop warn codes from arm_bridge/error_codes.py, the ones an operator can act on. */
const WARN_LABELS = {
    1001: "target held: out of reach (no IK)",
    1002: "target held: joint limit / arm stretched - pull back",
    1003: "target held: singularity",
    1004: "reduced mode",
    1005: "target held: would self-collide",
    1006: "arm outside envelope - retreat it (relay: t)"
};
let ArmTeleop = (() => {
    let _classDecorators = [component];
    let _classDescriptor;
    let _classExtraInitializers = [];
    let _classThis;
    let _classSuper = BaseScriptComponent;
    var ArmTeleop = _classThis = class extends _classSuper {
        constructor() {
            super();
            this.serverUrl = this.serverUrl;
            this.driveHand = this.driveHand;
            this.sendRateHz = this.sendRateHz;
            this.statusText = this.statusText;
            this.debugLog = this.debugLog;
            this.clutchMode = this.clutchMode;
            this.driveButton = this.driveButton;
            this.driveStateText = this.driveStateText;
            this.viewYawDeg = this.viewYawDeg;
            this.axesMaterial = this.axesMaterial;
            this.axisLabels = this.axisLabels;
            this.axisLabelSize = this.axisLabelSize;
            this.internetModule = require("LensStudio:InternetModule");
            this.socket = null;
            this.drive = null;
            this.clutch = null;
            this.camera = null;
            this.boot = "";
            this.urls = [];
            this.urlIndex = 0;
            this.connected = false;
            this.connecting = false;
            this.nextRetryAt = 0;
            this.retryDelay = 1.0;
            this.lastSendAt = 0;
            this.lastLogAt = 0;
            this.lastHudAt = 0;
            // --- diagnostics surfaced on the HUD ---
            this.state = "STARTING";
            this.attempts = 0;
            this.packetsSent = 0;
            this.lastError = "-";
            this.moduleOk = false;
            this.relay = null;
            this.relayAt = 0;
            // --- clutch (pinch / toggle) and the wrist axes ---
            this.clutchCtl = new ClutchControl_1.ClutchControl("pinch");
            this.clutchOut = { tracked: false, pinch: false };
            this.driveHooked = false;
            this.axes = null;
            this.axesNote = "";
            this.liveFwd = new vec3(0, 0, -1); // horizontal look direction, world
            this.frozenFwd = null;
            this.hudNote = "";
            this.hudNoteAt = 0;
            this.driveLabel = "";
            // --- app mode (ArmMenu) and the digital twin ---
            this.appMode = "teleop"; // without a menu, the original behaviour
            this.lastSent = "hand";
            this.twin = null;
            this.lastTwinPose = null;
            this.twinReleaseUntil = 0; // after leaving twin mode: keep saying "not active"
        }
        __initialize() {
            super.__initialize();
            this.serverUrl = this.serverUrl;
            this.driveHand = this.driveHand;
            this.sendRateHz = this.sendRateHz;
            this.statusText = this.statusText;
            this.debugLog = this.debugLog;
            this.clutchMode = this.clutchMode;
            this.driveButton = this.driveButton;
            this.driveStateText = this.driveStateText;
            this.viewYawDeg = this.viewYawDeg;
            this.axesMaterial = this.axesMaterial;
            this.axisLabels = this.axisLabels;
            this.axisLabelSize = this.axisLabelSize;
            this.internetModule = require("LensStudio:InternetModule");
            this.socket = null;
            this.drive = null;
            this.clutch = null;
            this.camera = null;
            this.boot = "";
            this.urls = [];
            this.urlIndex = 0;
            this.connected = false;
            this.connecting = false;
            this.nextRetryAt = 0;
            this.retryDelay = 1.0;
            this.lastSendAt = 0;
            this.lastLogAt = 0;
            this.lastHudAt = 0;
            // --- diagnostics surfaced on the HUD ---
            this.state = "STARTING";
            this.attempts = 0;
            this.packetsSent = 0;
            this.lastError = "-";
            this.moduleOk = false;
            this.relay = null;
            this.relayAt = 0;
            // --- clutch (pinch / toggle) and the wrist axes ---
            this.clutchCtl = new ClutchControl_1.ClutchControl("pinch");
            this.clutchOut = { tracked: false, pinch: false };
            this.driveHooked = false;
            this.axes = null;
            this.axesNote = "";
            this.liveFwd = new vec3(0, 0, -1); // horizontal look direction, world
            this.frozenFwd = null;
            this.hudNote = "";
            this.hudNoteAt = 0;
            this.driveLabel = "";
            // --- app mode (ArmMenu) and the digital twin ---
            this.appMode = "teleop"; // without a menu, the original behaviour
            this.lastSent = "hand";
            this.twin = null;
            this.lastTwinPose = null;
            this.twinReleaseUntil = 0; // after leaving twin mode: keep saying "not active"
        }
        onAwake() {
            // A new id per Lens launch: the world origin restarts with the Lens, so
            // the relay drops a room calibration made in a previous launch.
            this.boot = `${Date.now().toString(36)}-${Math.floor(Math.random() * 1e9).toString(36)}`;
            this.createEvent("OnStartEvent").bind(() => this.onStart());
            this.createEvent("UpdateEvent").bind(() => this.onUpdate());
            this.createEvent("OnDestroyEvent").bind(() => this.closeSocket());
        }
        onStart() {
            this.clutchCtl = new ClutchControl_1.ClutchControl(this.clutchMode === "toggle" ? "toggle" : "pinch");
            if (this.internetModule === undefined || this.internetModule === null) {
                this.state = "NO INTERNET MODULE";
                this.lastError = "require('LensStudio:InternetModule') returned nothing";
                this.renderHud(true);
                return;
            }
            this.moduleOk = true;
            this.urls = this.serverUrl
                .split(",")
                .map((s) => s.trim())
                .filter((s) => s.length > 0);
            if (this.urls.length === 0) {
                this.state = "NO URL SET";
                this.lastError = "Server Url is empty";
                this.renderHud(true);
                return;
            }
            try {
                const driveName = this.driveHand === "left" ? "left" : "right";
                const clutchName = driveName === "left" ? "right" : "left";
                this.drive = SIK_1.SIK.HandInputData.getHand(driveName);
                this.clutch = SIK_1.SIK.HandInputData.getHand(clutchName);
                this.camera = WorldCameraFinderProvider_1.default.getInstance();
            }
            catch (e) {
                this.state = "INIT FAILED";
                this.lastError = `${e}`;
                this.renderHud(true);
                return;
            }
            this.createAxes();
            print(`[ArmTeleop] drive=${this.driveHand} clutch=${this.clutchCtl.mode} -> ${this.urls.join(" | ")}`);
            this.renderHud(true);
            this.connect();
        }
        createAxes() {
            if (this.axesMaterial === undefined || this.axesMaterial === null) {
                this.axesNote = "no Axes Material set";
                return;
            }
            try {
                const font = this.statusText !== undefined && this.statusText !== null ? this.statusText.font : null;
                this.axes = new ArmAxes_1.ArmAxes(this.getSceneObject(), this.axesMaterial, {
                    lengthCm: 6,
                    thicknessCm: 0.4,
                    labels: this.axisLabels,
                    labelSize: this.axisLabelSize,
                    labelFont: font
                });
            }
            catch (e) {
                this.axes = null;
                this.axesNote = `axes failed: ${e}`;
                print(`[ArmTeleop] ${this.axesNote}`);
            }
        }
        /** Lens Studio's Preview can reach the relay too, and through Docker every
         * client arrives from the same address -- so say which one this is. */
        deviceKind() {
            try {
                if (global.deviceInfoSystem.isEditor())
                    return "lens-studio";
                if (global.deviceInfoSystem.isSpectacles())
                    return "spectacles";
            }
            catch (e) {
                // older runtimes: unknown
            }
            return "unknown";
        }
        clutchName() {
            return this.driveHand === "left" ? "right" : "left";
        }
        currentUrl() {
            return this.urls[this.urlIndex % this.urls.length];
        }
        // ---- networking ----------------------------------------------------
        connect() {
            if (this.connecting || this.connected || !this.moduleOk) {
                return;
            }
            this.connecting = true;
            this.attempts += 1;
            this.state = "CONNECTING";
            const url = this.currentUrl();
            this.renderHud(true);
            print(`[ArmTeleop] connect attempt ${this.attempts} -> ${url}`);
            try {
                const sock = this.internetModule.createWebSocket(url);
                sock.binaryType = "blob";
                this.socket = sock;
                sock.onopen = () => {
                    this.connected = true;
                    this.connecting = false;
                    this.retryDelay = 1.0;
                    this.state = "CONNECTED";
                    this.lastError = "-";
                    this.clutchCtl.connectionChanged("reconnected to the relay");
                    this.send({ type: "hello", boot: this.boot, drive: this.driveHand, clutch: this.clutchName(),
                        device: this.deviceKind() });
                    this.renderHud(true);
                    print("[ArmTeleop] connected");
                };
                sock.onmessage = (event) => {
                    if (typeof event.data !== "string") {
                        return;
                    }
                    try {
                        const msg = JSON.parse(event.data);
                        if (msg.type === "state") {
                            this.relay = msg;
                            this.relayAt = getTime();
                            this.clutchCtl.relayState(this.relayAt, msg.engaged === true, `${msg.note || ""}`);
                        }
                    }
                    catch (e) {
                        // ignore a malformed frame; the next one replaces it
                    }
                };
                sock.onclose = (event) => {
                    this.connected = false;
                    this.connecting = false;
                    this.relay = null;
                    this.clutchCtl.connectionChanged("connection to the relay closed");
                    this.lastError = event.code === 1006
                        ? "1006 abnormal: never reached the relay (IP? port? same Wi-Fi?)"
                        : `closed, code ${event.code}`;
                    this.state = "CLOSED";
                    this.scheduleRetry();
                    this.renderHud(true);
                    print(`[ArmTeleop] closed, code ${event.code}`);
                };
                sock.onerror = () => {
                    this.connected = false;
                    this.connecting = false;
                    this.relay = null;
                    this.clutchCtl.connectionChanged("connection to the relay failed");
                    this.state = "SOCKET ERROR";
                    this.lastError = "onerror -- no route to the relay, or ws:// refused";
                    this.scheduleRetry();
                    this.renderHud(true);
                    print("[ArmTeleop] socket error");
                };
            }
            catch (e) {
                this.connecting = false;
                this.state = "CREATE FAILED";
                this.lastError = `createWebSocket threw: ${e}`;
                this.scheduleRetry();
                this.renderHud(true);
                print(`[ArmTeleop] createWebSocket threw: ${e}`);
            }
        }
        send(payload) {
            if (this.socket === null) {
                return false;
            }
            try {
                this.socket.send(JSON.stringify(payload));
                return true;
            }
            catch (e) {
                this.connected = false;
                this.state = "SEND FAILED";
                this.lastError = `${e}`;
                this.clutchCtl.connectionChanged("sending to the relay failed");
                this.scheduleRetry();
                return false;
            }
        }
        scheduleRetry() {
            this.socket = null;
            if (this.urls.length > 1) {
                this.urlIndex = (this.urlIndex + 1) % this.urls.length;
            }
            this.nextRetryAt = getTime() + this.retryDelay;
            this.retryDelay = Math.min(this.retryDelay * 1.6, 5.0);
        }
        closeSocket() {
            if (this.socket !== null) {
                try {
                    this.socket.close();
                }
                catch (e) {
                    // already gone
                }
                this.socket = null;
            }
            this.connected = false;
        }
        // ---- per-frame -----------------------------------------------------
        onUpdate() {
            if (!this.moduleOk) {
                return;
            }
            const now = getTime();
            this.hookDriveButton();
            this.updateClutch(now);
            this.updateDriveLabel();
            this.updateAxes();
            if (!this.connected) {
                this.renderHud(false);
                if (!this.connecting && now >= this.nextRetryAt) {
                    this.connect();
                }
                return;
            }
            const minInterval = 1.0 / Math.max(1, this.sendRateHz);
            if (now - this.lastSendAt < minInterval) {
                this.renderHud(false);
                return;
            }
            this.lastSendAt = now;
            const msg = this.message(now);
            if (msg !== null && this.send(msg)) {
                this.packetsSent += 1;
            }
            this.renderHud(false);
        }
        /** This frame's clutch (ClutchControl), the head direction, and the axes freeze. */
        updateClutch(now) {
            if (this.camera !== null) {
                // Transform.forward points out of the back of the camera.
                const look = this.camera.forward().uniformScale(-1);
                const flat = new vec3(look.x, 0, look.z);
                if (flat.length > 0.1) { // looking straight up/down: keep the last one
                    this.liveFwd = flat.normalize();
                }
            }
            const c = this.clutch;
            const tracked = c !== null && c.isTracked();
            const wasFrozen = this.clutchCtl.frozen;
            this.clutchOut = this.clutchCtl.frame(now, tracked, tracked && c.isPinching());
            if (this.clutchCtl.frozen && !wasFrozen) {
                this.frozenFwd = this.liveFwd; // the relay freezes its frame at this moment too
            }
            else if (!this.clutchCtl.frozen) {
                this.frozenFwd = null;
            }
        }
        /** ON / OFF in driveStateText. Checked every frame because DRIVE can switch
         * itself off (ClutchControl.frame); the Text is only written on a change. */
        updateDriveLabel() {
            if (this.driveStateText === undefined || this.driveStateText === null) {
                return;
            }
            const label = this.clutchCtl.toggleOn ? "ON" : "OFF";
            if (label !== this.driveLabel) {
                this.driveLabel = label;
                this.driveStateText.text = label;
            }
        }
        /** The arm's axes at the drive wrist, in the relay's view-mode mapping. */
        updateAxes() {
            if (this.axes === null) {
                return;
            }
            const d = this.drive;
            const roomMode = this.relay !== null && this.relay.mode === "room";
            if (d === null || !d.isTracked() || roomMode || this.appMode !== "teleop") {
                this.axes.setVisible(false);
                return;
            }
            // f = horizontal look direction (frozen while grabbing), r = f x up (right).
            // X = f cos(phi) + r sin(phi), Y = f sin(phi) - r cos(phi), Z = up; with
            // phi = 0: X = where you look, Y = your left, Z = up.
            const f = this.frozenFwd !== null ? this.frozenFwd : this.liveFwd;
            const up = vec3.up();
            const r = f.cross(up);
            const phi = this.viewYawDeg * Math.PI / 180;
            const cos = Math.cos(phi);
            const sin = Math.sin(phi);
            const x = f.uniformScale(cos).add(r.uniformScale(sin));
            const y = f.uniformScale(sin).sub(r.uniformScale(cos));
            const face = this.camera !== null ? this.camera.getTransform().getWorldRotation() : null;
            this.axes.setVisible(true);
            this.axes.update(d.wrist.position, [x, y, up], this.frozenFwd !== null, face);
        }
        // ---- clutch mode and DRIVE (UIKit hooks) ---------------------------
        /**
         * Wire to a UIKit Switch's "On Value Changed Callbacks": on = toggle mode,
         * off = pinch mode. The Switch calls back repeatedly while its knob
         * animates, so the mode is set from the value; no argument flips it.
         */
        setClutchMode(value) {
            const toggle = (0, ClutchControl_1.resolveSwitch)(value, this.clutchCtl.mode === "toggle");
            const mode = toggle ? "toggle" : "pinch";
            if (this.clutchCtl.setMode(mode)) {
                print(`[ArmTeleop] clutch mode ${mode}`);
                this.renderHud(true);
            }
        }
        /**
         * DRIVE on/off for toggle mode, for wiring a UIKit callback directly: a
         * number means on = value > 0.5, a boolean is used as is, no argument flips.
         * Prefer the driveButton input instead, which also rejects presses by the
         * drive hand -- a callback cannot tell which hand pressed.
         */
        setDrive(value) {
            this.requestDrive((0, ClutchControl_1.resolveSwitch)(value, this.clutchCtl.toggleOn));
        }
        requestDrive(on) {
            const driveTracked = this.drive !== null && this.drive.isTracked();
            this.clutchCtl.requestDrive(on, getTime(), this.connected, driveTracked);
            print(`[ArmTeleop] DRIVE ${this.clutchCtl.toggleOn ? "ON" : "OFF"}` +
                (this.clutchCtl.note ? ` (${this.clutchCtl.note})` : ""));
            this.renderHud(true);
        }
        /** Listen to the DRIVE button's Interactable, which says which hand pressed. */
        hookDriveButton() {
            if (this.driveHooked || this.driveButton === undefined || this.driveButton === null) {
                return;
            }
            // UIKit adds the Interactable during its own start: try again next frame.
            const interactable = this.driveButton.getComponent(Interactable_1.Interactable.getTypeName());
            if (interactable === null || interactable === undefined) {
                return;
            }
            interactable.onTriggerEnd.add((e) => this.onDrivePressed(e));
            this.driveHooked = true;
        }
        onDrivePressed(e) {
            const driveBit = this.driveHand === "left" ? Interactor_1.InteractorInputType.LeftHand : Interactor_1.InteractorInputType.RightHand;
            const by = e !== undefined && e !== null && e.interactor ? e.interactor.inputType : Interactor_1.InteractorInputType.None;
            if ((by & driveBit) !== 0) {
                // The drive hand is the one that moves the arm: never let it press DRIVE.
                this.clutchCtl.note = `DRIVE ignored: press it with your ${this.clutchName()} hand`;
                print(`[ArmTeleop] ${this.clutchCtl.note}`);
                this.renderHud(true);
                return;
            }
            this.requestDrive(!this.clutchCtl.toggleOn);
        }
        // ---- app mode and the digital twin (ArmMenu / ArmTwin) ---------------
        /** Switch between the menu, hand teleop and the digital twin. Any DRIVE toggle switches off. */
        setAppMode(mode) {
            if (mode === this.appMode) {
                return;
            }
            if (this.appMode === "twin") {
                this.twinReleaseUntil = getTime() + 0.3;
            }
            this.appMode = mode;
            this.clutchCtl.connectionChanged("app mode changed");
            print(`[ArmTeleop] app mode ${mode}`);
            this.renderHud(true);
        }
        getAppMode() {
            return this.appMode;
        }
        /** ArmTwin attaches itself when shown (null when hidden). */
        attachTwin(source) {
            this.twin = source;
        }
        /** The relay's latest state message (with the arm's actual `pose`), or null if none for 1 s. */
        getRelayState() {
            return this.connected && this.relay !== null && getTime() - this.relayAt < 1.0 ? this.relay : null;
        }
        /**
         * The message for this tick. teleop: hand. twin: the twin's target. Leaving
         * twin mode, and in the menu after it: "twin, not active" (last pose), so
         * the relay lets go now instead of after its silence timeout. Menu after
         * teleop: hand with the clutch off, for the same reason.
         */
        message(now) {
            if (this.appMode === "twin") {
                if (this.twin === null) {
                    return null;
                }
                const m = this.twin.twinMessage();
                this.lastTwinPose = m.pose;
                this.lastSent = "twin";
                return { type: "twin", t: Math.round(now * 1000), active: m.active, pose: m.pose };
            }
            if (this.lastSent === "twin" && (this.appMode === "menu" || now < this.twinReleaseUntil)) {
                const pose = this.twin !== null ? this.twin.twinMessage().pose : this.lastTwinPose;
                return pose === null ? null : { type: "twin", t: Math.round(now * 1000), active: false, pose: pose };
            }
            if (this.appMode === "menu") {
                const msg = this.sample(now);
                msg.clutch = { tracked: true, pinch: false, mode: this.clutchCtl.mode };
                return msg;
            }
            this.lastSent = "hand";
            return this.sample(now);
        }
        sample(now) {
            const d = this.drive;
            const driveTracked = d !== null && d.isTracked();
            const drive = { tracked: driveTracked };
            if (driveTracked) {
                drive.wrist = v3(d.wrist.position);
                drive.index = v3(d.indexKnuckle.position);
                drive.middle = v3(d.middleKnuckle.position);
                drive.pinky = v3(d.pinkyKnuckle.position);
            }
            const msg = {
                type: "hand",
                t: Math.round(now * 1000),
                drive: drive,
                // Pinch mode: the clutch hand's tracking and pinch, as always. Toggle
                // mode: tracked = true, pinch = DRIVE (ClutchControl).
                clutch: { tracked: this.clutchOut.tracked, pinch: this.clutchOut.pinch, mode: this.clutchCtl.mode }
            };
            if (this.camera !== null) {
                // Transform.forward points out of the back of the camera.
                msg.head = { fwd: v3(this.camera.forward().uniformScale(-1)) };
            }
            if (this.debugLog && now - this.lastLogAt > 1.0) {
                this.lastLogAt = now;
                print(`[ArmTeleop] drive=${driveTracked} clutch=${this.clutchCtl.mode} ` +
                    `tracked=${msg.clutch.tracked} pinch=${msg.clutch.pinch}` +
                    (driveTracked ? ` wrist=${drive.wrist.join(",")}` : ""));
            }
            return msg;
        }
        // ---- HUD -----------------------------------------------------------
        renderHud(force) {
            if (this.statusText === undefined || this.statusText === null) {
                return;
            }
            const now = getTime();
            if (!force && now - this.lastHudAt < 0.2) {
                return;
            }
            this.lastHudAt = now;
            const driveTracked = this.drive !== null && this.drive.isTracked();
            const clutchTracked = this.clutch !== null && this.clutch.isTracked();
            const pinching = clutchTracked && this.clutch.isPinching();
            const r = this.connected && this.relay !== null && now - this.relayAt < 1.0 ? this.relay : null;
            let color = COLOR_BAD;
            let arm = "-";
            let note = "";
            if (r === null) {
                note = this.connected ? "waiting for the relay..." :
                    (this.state === "CONNECTING" ? "waiting for the relay to answer..." :
                        `retry in ${Math.max(0, this.nextRetryAt - now).toFixed(1)}s`);
                color = this.connected || this.state === "CONNECTING" ? COLOR_WARN : COLOR_BAD;
            }
            else {
                arm = `${r.arm}${r.err ? ` err ${r.err}` : ""}${r.hold ? " HOLD" : ""}`;
                note = r.note;
                if (r.warn && WARN_LABELS[r.warn] !== undefined) {
                    note = WARN_LABELS[r.warn];
                }
                else if (r.warn) {
                    note = `warn ${r.warn}`;
                }
                if (r.engaged) {
                    color = COLOR_DRIVE;
                }
                else if (r.bridge === "connected" && r.arm === "SERVO" && !r.err && !r.hold) {
                    color = COLOR_OK;
                    if (this.appMode === "twin")
                        note = r.note || "drag the model to move the arm";
                    else if (this.appMode === "menu")
                        note = "choose a mode";
                    else if (!driveTracked)
                        note = `show your ${this.driveHand} hand`;
                    else
                        note = this.clutchHint(pinching, `${r.note || ""}`);
                }
                else {
                    color = COLOR_WARN;
                    if (r.bridge !== "connected")
                        note = `bridge ${r.bridge}`;
                    else if (r.arm !== "SERVO")
                        note = `arm ${r.arm}: enable on the relay (e)`;
                }
            }
            // Why DRIVE is off (or was refused): shown while it stays off in toggle
            // mode, and for a few seconds otherwise.
            const ctl = this.clutchCtl;
            if (ctl.note !== this.hudNote) {
                this.hudNote = ctl.note;
                this.hudNoteAt = now;
            }
            const showNote = ctl.note !== "" &&
                ((ctl.mode === "toggle" && !ctl.toggleOn) || now - this.hudNoteAt < 5.0);
            const clutchLine = (ctl.mode === "toggle" ? `toggle  DRIVE ${ctl.toggleOn ? "ON" : "OFF"}` : "pinch") +
                (showNote ? `  (${ctl.note})` : "");
            const roomMode = this.relay !== null && this.relay.mode === "room";
            const axesLine = roomMode ? "room mode: axes unknown" : this.axesNote;
            this.statusText.text =
                `SPECSREACH ARM  [${this.appMode.toUpperCase()}]${r !== null && r.engaged ? "  ** DRIVING **" : ""}\n` +
                    `relay : ${this.state}  ${this.currentUrl()}\n` +
                    `bridge: ${r !== null ? r.bridge : "-"}   arm: ${arm}\n` +
                    `mode  : ${r !== null ? r.mode + (r.calibrating ? " (calibrating)" : "") : "-"}   sent: ${this.packetsSent}\n` +
                    `clutch: ${clutchLine}\n` +
                    `hands : ${this.driveHand} ${driveTracked ? "TRACKED" : "not visible"}, ` +
                    `${this.clutchName()} ${clutchTracked ? (pinching ? "PINCH" : "open") : "not visible"}\n` +
                    (axesLine ? `axes  : ${axesLine}\n` : "") +
                    `note  : ${note}\n` +
                    `err   : ${this.lastError}`;
            try {
                this.statusText.textFill.color = color;
            }
            catch (e) {
                // some Text configurations do not expose textFill; the text still updates
            }
        }
        /** The "what to do next" hint while the arm is ready but not driving. */
        clutchHint(pinching, relayNote) {
            const refused = relayNote.startsWith("engage refused");
            const ctl = this.clutchCtl;
            if (ctl.mode === "toggle") {
                if (ctl.toggleOn) {
                    return refused ? relayNote : "DRIVE on: waiting for the relay to grab";
                }
                return `press DRIVE with your ${this.clutchName()} hand`;
            }
            if (!ctl.pinchArmed) {
                return `open your ${this.clutchName()} hand first (clutch mode changed)`;
            }
            if (!pinching && !refused) {
                return `pinch and hold your ${this.clutchName()} hand to drive`;
            }
            return relayNote;
        }
    };
    __setFunctionName(_classThis, "ArmTeleop");
    (() => {
        const _metadata = typeof Symbol === "function" && Symbol.metadata ? Object.create(_classSuper[Symbol.metadata] ?? null) : void 0;
        __esDecorate(null, _classDescriptor = { value: _classThis }, _classDecorators, { kind: "class", name: _classThis.name, metadata: _metadata }, null, _classExtraInitializers);
        ArmTeleop = _classThis = _classDescriptor.value;
        if (_metadata) Object.defineProperty(_classThis, Symbol.metadata, { enumerable: true, configurable: true, writable: true, value: _metadata });
        __runInitializers(_classThis, _classExtraInitializers);
    })();
    return ArmTeleop = _classThis;
})();
exports.ArmTeleop = ArmTeleop;
//# sourceMappingURL=ArmTeleop.js.map