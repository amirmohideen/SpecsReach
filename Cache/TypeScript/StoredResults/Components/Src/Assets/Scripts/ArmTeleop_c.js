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
    if (script.onAwake) {
       script.onAwake();
    }
});
