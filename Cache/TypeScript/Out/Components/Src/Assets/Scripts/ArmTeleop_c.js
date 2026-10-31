if (script.onAwake) {
    script.onAwake();
    return;
}
function checkUndefined(property, showIfData) {
    for (var i = 0; i < showIfData.length; i++) {
        if (showIfData[i][0] && script[showIfData[i][0]] != showIfData[i][1]) {
            return;
        }
    }
    if (script[property] == undefined) {
        throw new Error("Input " + property + " was not provided for the object " + script.getSceneObject().name);
    }
}
// @input string serverUrl = "ws://192.168.1.154:8766" {"hint":"Relay address(es), e.g. ws://192.168.1.154:8766. Comma-separate several and the Lens cycles through them on each retry. The relay prints the URL when it starts."}
// @input string driveHand = "right" {"hint":"The hand whose wrist moves the arm. The other hand's pinch is the clutch.", "widget":"combobox", "values":[{"label":"right", "value":"right"}, {"label":"left", "value":"left"}]}
// @input float sendRateHz = 50 {"hint":"Send rate in Hz. The bridge streams to the arm at 50."}
// @input Component.Text statusText {"hint":"Optional but STRONGLY recommended: a Text component for the on-device status readout. Park it under the Camera so it is always visible."}
// @input bool debugLog {"hint":"Also print status to the Logger panel (only visible while tethered to Lens Studio)."}
// @input string clutchMode = "pinch" {"hint":"How the arm is grabbed. pinch: hold a pinch on the clutch hand (the original behaviour). toggle: press DRIVE to start, again to stop. Switchable at run time with setClutchMode().", "widget":"combobox", "values":[{"label":"pinch", "value":"pinch"}, {"label":"toggle", "value":"toggle"}]}
// @input SceneObject driveButton {"hint":"Recommended for toggle mode: the DRIVE button's object (a plain UIKit RectangleButton, Toggleable OFF). The script listens to it directly and IGNORES presses by the drive hand, which would move the arm. Do not also wire its callbacks to setDrive."}
// @input Component.Text driveStateText {"hint":"Optional: a Text that shows ON while DRIVE is on and OFF otherwise (e.g. on the DRIVE button). It follows the real state, including when DRIVE switches itself off."}
// @input float viewYawDeg {"hint":"Must equal view_yaw_deg in the relay's specs_relay.json (0 = arm +X is where you look). Only used to draw the wrist axes."}
// @input Asset.Material axesMaterial {"hint":"An Unlit material for the arm axes drawn at the drive wrist (X red, Y green, Z blue). Leave empty for no axes."}
// @input bool axisLabels = true {"hint":"Draw X/Y/Z labels at the axis tips."}
// @input float axisLabelSize = 40 {"hint":"Font size of the X/Y/Z labels. Lower it if they look too big on the glasses."}
if (!global.BaseScriptComponent) {
    function BaseScriptComponent() {}
    global.BaseScriptComponent = BaseScriptComponent;
    global.BaseScriptComponent.prototype = Object.getPrototypeOf(script);
    global.BaseScriptComponent.prototype.__initialize = function () {};
    global.BaseScriptComponent.getTypeName = function () {
        throw new Error("Cannot get type name from the class, not decorated with @component");
    };
}
var Module = require("../../../../Modules/Src/Assets/Scripts/ArmTeleop");
Object.setPrototypeOf(script, Module.ArmTeleop.prototype);
script.__initialize();
let awakeEvent = script.createEvent("OnAwakeEvent");
awakeEvent.bind(() => {
    checkUndefined("serverUrl", []);
    checkUndefined("driveHand", []);
    checkUndefined("sendRateHz", []);
    checkUndefined("debugLog", []);
    checkUndefined("clutchMode", []);
    checkUndefined("viewYawDeg", []);
    checkUndefined("axisLabels", []);
    checkUndefined("axisLabelSize", []);
    if (script.onAwake) {
       script.onAwake();
    }
});
