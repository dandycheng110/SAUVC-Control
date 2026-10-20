// Gamepad piloting — a game controller, the DJI motion controller if the Mac
// sees it as one, or a home-made grip that enumerates as a USB gamepad.
//
// This file only turns a gamepad into a demand. KeyboardPilot reads it on its
// own tick (setAnalogSource), so the stick goes through the same speed level,
// ramp, depth-target walk, turn-rate path and Space stop as the keys, and the
// vehicle cannot tell which of the two the operator is using.
//
// Safety, checked every tick:
//
//   Deadman      nothing moves unless the deadman button is held. A pad with
//                no deadman mapped never arms.
//   Fresh press  arming takes a press of the deadman made while everything is
//                ready: a piloting mode on, this window focused and visible,
//                and every stick at rest — an RC transmitter refusing to arm
//                with the throttle up. If anything is missing, the deadman has
//                to be let go and pressed again. A press only counts if the
//                tick before saw the deadman up with this window focused, so a
//                deadman already down when the pad appears, squeezed while the
//                tab was hidden, or held through a replug never arms.
//   Grip neutral a stick axis mapped with its centre "at arm" (a motion grip's
//                tilt) takes the pose at that press as neutral instead of
//                having to be at rest.
//   Disarm       letting go of the deadman, the brake button, Space, losing
//                focus, the tab going hidden, leaving the piloting modes,
//                unplugging and editing the mapping all disarm.
//   Silence      a pad that keeps reporting while nobody moves it (most
//                Bluetooth pads do, and so does anything with an IMU or a
//                noisy stick) and then goes quiet is treated as gone.
//                Bluetooth can take seconds to call a link dead, and until it
//                does the browser keeps showing the last values as live ones.
//                A pad that only reports on change cannot be judged this way;
//                for it the deadman and the backstop below are what is left.
//   Backstop     behind all of this the vehicle stops by itself when the page
//                stops sending: wrench_sum drops the GUI source after 0.5 s and
//                heading_hold treats the turn rate as zero after 0.3 s.
//
// Mappings are per device (gamepad.id) and kept in this browser's
// localStorage. A pad the browser reports with the "standard" mapping gets a
// built-in layout like ArduSub's joystick defaults; anything else starts
// unmapped and is set up in Tuning → Gamepad mapping.

const GAMEPAD_STORAGE_KEY = "orca.gamepad.profiles.v1";
const GAMEPAD_ROLES = Object.freeze(["surge", "sway", "heave", "yaw"]);

// A streaming pad that goes this long without a report is treated as gone.
// Such pads report every few milliseconds, so this is far outside normal.
const GAMEPAD_STALE_MS = 500;
// "Idle" reports — nothing pressed or released, no axis moved by more than
// noise — needed within one second before a pad counts as streaming. Noise
// counts as idle on purpose: an IMU grip or a jittery stick is exactly the
// kind of pad that reports all the time. A hand moving a stick on purpose
// moves it further than this between two ticks.
const GAMEPAD_STREAMING_IDLE_REPORTS = 10;
const GAMEPAD_NOISE = 0.02;

// Button names under the "standard" mapping (W3C Gamepad, figure 1).
const STANDARD_BUTTON_NAMES = Object.freeze([
    "A / ✕", "B / ○", "X / □", "Y / △", "LB / L1", "RB / R1", "LT / L2", "RT / R2",
    "Back / Share", "Start / Options", "L stick", "R stick",
    "D-pad up", "D-pad down", "D-pad left", "D-pad right", "Home",
]);

// Like ArduSub's joystick defaults: the left stick climbs, dives and turns,
// the right stick drives and strafes. Down is positive in this stack, so
// pushing the left stick down (axis +1) goes deeper as it reads. Hold L1 / LB
// to pilot; R1 / RB stops and holds here, like Space.
const STANDARD_GAMEPAD_PROFILE = Object.freeze({
    name: "standard layout (like ArduSub)",
    roles: Object.freeze({
        surge: Object.freeze({axis: 3, invert: true, range: "stick", center: "fixed"}),
        sway: Object.freeze({axis: 2, invert: false, range: "stick", center: "fixed"}),
        heave: Object.freeze({axis: 1, invert: false, range: "stick", center: "fixed"}),
        yaw: Object.freeze({axis: 0, invert: false, range: "stick", center: "fixed"}),
    }),
    deadzone: 0.1,
    expo: 0.3,
    deadman: 4,
    brake: 5,
});

function blankGamepadProfile() {
    const roles = {};
    for (const role of GAMEPAD_ROLES) {
        roles[role] = {axis: -1, invert: false, range: "stick", center: "fixed"};
    }
    return {name: "custom", roles, deadzone: 0.1, expo: 0.3, deadman: -1, brake: -1};
}

// Number() reads "" and null as 0, which here would mean button 0 or a zero
// deadzone. Blank means unset.
function gamepadNumber(value) {
    if (value === null || value === undefined || value === "") {
        return NaN;
    }
    return Number(value);
}

function gamepadIndexOrNone(value) {
    const n = gamepadNumber(value);
    return Number.isInteger(n) && n >= 0 ? n : -1;
}

function gamepadClamp(value, lo, hi, fallback) {
    const n = gamepadNumber(value);
    return Number.isFinite(n) ? Math.min(hi, Math.max(lo, n)) : fallback;
}

/** A clean copy of a stored or edited profile; anything malformed falls back. */
function sanitizeGamepadProfile(input) {
    const profile = blankGamepadProfile();
    if (!input || typeof input !== "object") {
        return profile;
    }
    for (const role of GAMEPAD_ROLES) {
        const mapping = input.roles && input.roles[role];
        if (!mapping || typeof mapping !== "object") {
            continue;
        }
        profile.roles[role] = {
            axis: gamepadIndexOrNone(mapping.axis),
            invert: mapping.invert === true,
            range: mapping.range === "trigger" ? "trigger" : "stick",
            center: mapping.center === "arm" ? "arm" : "fixed",
        };
    }
    if (typeof input.name === "string" && input.name) {
        profile.name = input.name.slice(0, 60);
    }
    // Never quite zero: every stick and IMU has some noise, and with no
    // deadzone it would either block arming or drive the vehicle by itself.
    profile.deadzone = gamepadClamp(input.deadzone, 0.02, 0.5, profile.deadzone);
    profile.expo = gamepadClamp(input.expo, 0, 1, profile.expo);
    profile.deadman = gamepadIndexOrNone(input.deadman);
    profile.brake = gamepadIndexOrNone(input.brake);
    return profile;
}

/** Why a profile cannot arm on a pad of this shape, or "" when it can. */
function gamepadProfileProblem(profile, axisCount, buttonCount) {
    if (profile.deadman < 0) {
        return "no deadman button is set";
    }
    if (profile.deadman >= buttonCount) {
        return `the deadman, button ${profile.deadman}, is not on this pad`;
    }
    if (profile.brake === profile.deadman) {
        return "the deadman and the brake are the same button";
    }
    // A brake the pad does not have is worse than none: the hint would
    // promise a stop that is not there.
    if (profile.brake >= buttonCount) {
        return `the brake, button ${profile.brake}, is not on this pad`;
    }
    let mapped = 0;
    for (const role of GAMEPAD_ROLES) {
        const axis = profile.roles[role].axis;
        if (axis < 0) {
            continue;
        }
        if (axis >= axisCount) {
            return `${role} uses axis ${axis}, which this pad does not have`;
        }
        mapped += 1;
    }
    return mapped ? "" : "no axis is mapped";
}

/**
 * One axis to a demand in −1 … 1: invert, centre, deadzone, expo.
 *
 * "stick" axes rest at 0, or at `neutral` when the centre is taken at arm; each
 * side of the centre is rescaled on its own, so a grip armed slightly tilted
 * still reaches full scale at the end of its travel. "trigger" axes rest at −1
 * and read 0 … 1 — the usual report for a trigger on an axis of its own.
 * Invert flips the raw reading, so it also turns round a trigger that rests
 * at +1.
 */
function shapeGamepadAxis(raw, mapping, neutral, deadzone, expo) {
    const sign = mapping.invert ? -1 : 1;
    let value = sign * Number(raw);
    if (!Number.isFinite(value)) {
        return 0;
    }
    if (mapping.range === "trigger") {
        value = (value + 1) / 2;
    } else {
        const centre = mapping.center === "arm" && Number.isFinite(neutral)
            ? sign * neutral : 0;
        const delta = value - centre;
        const span = delta >= 0 ? 1 - centre : 1 + centre;
        value = span > 0 ? delta / span : 0;
    }
    value = Math.max(-1, Math.min(1, value));
    const magnitude = Math.abs(value);
    if (magnitude <= deadzone) {
        return 0;
    }
    const linear = Math.min(1, (magnitude - deadzone) / (1 - deadzone));
    return Math.sign(value) * ((1 - expo) * linear + expo * linear ** 3);
}

function gamepadStorage() {
    try {
        return window.localStorage;
    } catch (error) {
        // Blocked site data: mappings then last for this page load only.
        return null;
    }
}

class GamepadPilot {
    /**
     * @param options
     *   onState (state) => void — every tick, for the panels
     *   onBrake () => void      — the brake button; the dashboard makes it Space
     *   storage                 — a localStorage stand-in, for tests
     */
    constructor(options = {}) {
        this.onState = options.onState || (() => {});
        this.onBrake = options.onBrake || (() => {});
        this.storage = options.storage !== undefined ? options.storage : gamepadStorage();
        this.profiles = this._load();

        this.pad = null;             // {index, id, mapping} of the pad followed
        this.preferredIndex = null;  // picked in the mapping editor
        this.armed = false;
        this.status = "none";        // none | unmapped | idle | blocked | armed | stale
        this.reason = "";            // why it is not armed, for the hint line
        this.lastStop = "";          // what disarmed it last
        this.neutral = null;         // raw axes taken at arm
        this.raw = null;             // last reading, for the mapping editor
        this.streaming = false;
        this._demand = null;
        this._prevPressed = null;
        // True when the last tick saw the deadman up with this window focused
        // and visible — the only state a press may arm from.
        this._readyToArm = false;
        this._reports = {timestamp: null, at: 0, axes: null, buttons: null, idle: []};
        this._lastError = "";

        // Also forget that the deadman was seen up: a press made while away is
        // not one this page watched, even if no tick ran in the meantime.
        window.addEventListener("blur", () => {
            this.disarm("the window lost focus");
            this._readyToArm = false;
        });
        document.addEventListener("visibilitychange", () => {
            if (document.hidden) {
                this.disarm("the tab was hidden");
                this._readyToArm = false;
            }
        });
        // Connect and disconnect both mean "a new pad, as far as arming goes":
        // a replug between two ticks keeps the same index and id, and a deadman
        // held through it must not read as a fresh press.
        const forgetPad = (reason) => (event) => {
            const index = event.gamepad ? event.gamepad.index : null;
            if (this.pad && index === this.pad.index) {
                this.disarm(reason);
                this.pad = null;
                this._prevPressed = null;
                this._readyToArm = false;
            }
            if (reason === "the gamepad was unplugged" && index === this.preferredIndex) {
                this.preferredIndex = null;
            }
        };
        window.addEventListener("gamepaddisconnected", forgetPad("the gamepad was unplugged"));
        window.addEventListener("gamepadconnected", forgetPad("the gamepad reconnected"));
    }

    // ------------------------------------------------------------ profiles

    /** The profile for the pad being followed: saved, built-in or blank. */
    profile() {
        if (!this.pad) {
            return null;
        }
        return this.profiles[this.pad.id] || this.defaultProfile();
    }

    defaultProfile() {
        return this.pad && this.pad.mapping === "standard"
            ? STANDARD_GAMEPAD_PROFILE : blankGamepadProfile();
    }

    hasSavedProfile() {
        return Boolean(this.pad && this.profiles[this.pad.id]);
    }

    saveProfile(profile) {
        if (!this.pad) {
            return;
        }
        // Never swap the mapping under a stick that is driving, and make the
        // next arm a fresh press on whatever the deadman now is.
        this.disarm("the mapping changed");
        this._readyToArm = false;
        this.profiles[this.pad.id] = sanitizeGamepadProfile(profile);
        this._persist();
    }

    resetProfile() {
        if (!this.pad) {
            return;
        }
        this.disarm("the mapping changed");
        this._readyToArm = false;
        delete this.profiles[this.pad.id];
        this._persist();
    }

    _load() {
        const out = {};
        try {
            const raw = this.storage ? this.storage.getItem(GAMEPAD_STORAGE_KEY) : null;
            const parsed = raw ? JSON.parse(raw) : {};
            for (const [id, profile] of Object.entries(parsed || {})) {
                out[id] = sanitizeGamepadProfile(profile);
            }
        } catch (error) {
            // Unreadable storage is the same as nothing saved.
        }
        return out;
    }

    _persist() {
        try {
            if (this.storage) {
                this.storage.setItem(GAMEPAD_STORAGE_KEY, JSON.stringify(this.profiles));
            }
        } catch (error) {
            // Not persisted; the mapping still holds for this page load.
        }
    }

    // ---------------------------------------------------------------- pads

    pads() {
        return this._list().map((p) => ({index: p.index, id: p.id, mapping: p.mapping}));
    }

    selectPad(index) {
        this.disarm("another gamepad was picked");
        this.preferredIndex = Number.isInteger(index) ? index : null;
    }

    buttonName(index) {
        if (!(index >= 0)) {
            return "no button";
        }
        const named = this.pad && this.pad.mapping === "standard"
            && STANDARD_BUTTON_NAMES[index];
        return named ? `${named} (button ${index})` : `button ${index}`;
    }

    _list() {
        let list = [];
        try {
            list = navigator.getGamepads ? Array.from(navigator.getGamepads()) : [];
        } catch (error) {
            // Refused by a permissions policy: no gamepads, rather than a crash.
            list = [];
        }
        return list.filter((p) => p && p.connected !== false);
    }

    _pick() {
        const pads = this._list();
        const byIndex = (index) => pads.find((p) => p.index === index);
        // An armed pad keeps the controls: another one plugged in, even into
        // the slot that was picked earlier, does not take over mid-run.
        if (this.armed && this.pad && byIndex(this.pad.index)) {
            return byIndex(this.pad.index);
        }
        if (this.preferredIndex !== null && !byIndex(this.preferredIndex)) {
            this.preferredIndex = null;
        }
        return (this.preferredIndex !== null && byIndex(this.preferredIndex))
            || (this.pad && byIndex(this.pad.index))
            || pads[0]
            || null;
    }

    _adopt(gp) {
        this.disarm("a different gamepad appeared");
        this.pad = {index: gp.index, id: gp.id, mapping: gp.mapping};
        // Whatever is held as the pad appears counts as already held, so a
        // deadman squeezed while plugging in is not a press.
        this._prevPressed = null;
        this._readyToArm = false;
        this._reports = {timestamp: null, at: performance.now(), axes: null, buttons: null, idle: []};
        this.streaming = false;
        this.lastStop = "";
        this.status = "idle";
        this.reason = "";
    }

    // Learns whether the pad streams, and when it last said anything.
    _watchReports(gp, now) {
        const reports = this._reports;
        const axes = Array.from(gp.axes, Number);
        const buttons = Array.from(gp.buttons, (b) => (b ? b.value : 0));
        if (gp.timestamp !== reports.timestamp) {
            if (reports.timestamp !== null && reports.axes
                    && axes.length === reports.axes.length
                    && buttons.length === reports.buttons.length
                    && buttons.every((v, i) => v === reports.buttons[i])
                    && axes.every((v, i) => Math.abs(v - reports.axes[i]) <= GAMEPAD_NOISE)) {
                reports.idle.push(now);
            }
            reports.timestamp = gp.timestamp;
            reports.at = now;
        }
        reports.axes = axes;
        reports.buttons = buttons;
        while (reports.idle.length && now - reports.idle[0] > 1000) {
            reports.idle.shift();
        }
        if (reports.idle.length >= GAMEPAD_STREAMING_IDLE_REPORTS) {
            this.streaming = true;
        }
    }

    // ---------------------------------------------------------------- tick

    /** Drop out until the deadman is let go and pressed again. */
    disarm(reason) {
        if (this.armed) {
            this.armed = false;
            this.lastStop = reason;
        }
        this._demand = null;
        this.neutral = null;
    }

    /** {surge, sway, heave, yaw} in −1 … 1 while armed, otherwise null. */
    demand() {
        // Focus is checked here as well as on the events, so a missed blur can
        // never leave the stick driving a window nobody is looking at.
        if (!this.armed || !this._demand || !this._focused()) {
            return null;
        }
        return this._demand;
    }

    /** Called by KeyboardPilot once per tick, before demand(). */
    update(enabled) {
        const now = performance.now();
        const focused = this._focused();
        const gp = this._pick();
        if (!gp) {
            if (this.pad) {
                this.disarm("the gamepad was unplugged");
            }
            this.pad = null;
            this.raw = null;
            this._prevPressed = null;
            this._readyToArm = false;
            this._setStatus("none", "");
            this._emit();
            return;
        }
        if (!this.pad || gp.index !== this.pad.index || gp.id !== this.pad.id) {
            this._adopt(gp);
        }
        this._watchReports(gp, now);

        const pressed = Array.from(gp.buttons, (b) => Boolean(b && b.pressed));
        // No usable history (a fresh pad, or one whose button count changed):
        // count everything held now as already held.
        const reshaped = !this._prevPressed || this._prevPressed.length !== pressed.length;
        const before = reshaped ? pressed : this._prevPressed;
        this._prevPressed = pressed;
        const held = (i) => i >= 0 && i < pressed.length && pressed[i];
        const rose = (i) => held(i) && !before[i];
        this.raw = {axes: Array.from(gp.axes, Number), pressed};

        const profile = this.profile();
        // A press arms only from a tick that saw the deadman up — an actual
        // button on this pad, seen released — while this window was focused
        // and visible. A rising edge alone is not enough: the tick before may
        // have run in a hidden tab, where the pad's data can be stale, before
        // a replug, or before the pad grew the button at all.
        const freshPress = held(profile.deadman) && this._readyToArm && !reshaped;
        this._readyToArm = focused && profile.deadman >= 0
            && profile.deadman < pressed.length && !pressed[profile.deadman];

        const braked = rose(profile.brake);
        if (braked) {
            this.disarm("the brake button");
            this._safely(() => this.onBrake());
        }

        const problem = gamepadProfileProblem(profile, gp.axes.length, gp.buttons.length);
        if (problem) {
            this.disarm("the mapping is incomplete");
            this._setStatus("unmapped", problem);
        } else if (this.streaming && now - this._reports.at > GAMEPAD_STALE_MS) {
            this.disarm("the gamepad stopped reporting");
            this._setStatus("stale", "the gamepad stopped reporting");
        } else if (!held(profile.deadman)) {
            this.disarm("the deadman was let go");
            // Say so before the press rather than refusing it after.
            this._setStatus("idle", enabled ? "" : "piloting needs Depth hold or Manual");
        } else if (this.armed) {
            const blocker = this._blocker(enabled);
            if (blocker) {
                this.disarm(blocker);
                this._setStatus("blocked", blocker);
            } else {
                this._demand = this._shape(gp, profile);
                this._setStatus("armed", "");
            }
        } else if (freshPress && !braked) {
            // (Brake and deadman in the same tick: the brake wins, and the
            // deadman has to come up and go down again.)
            const blocker = this._blocker(enabled) || this._offCentre(gp, profile);
            if (blocker) {
                this._setStatus("blocked", blocker);
            } else {
                this.armed = true;
                this.lastStop = "";
                this.neutral = Array.from(gp.axes, Number);
                this._demand = this._shape(gp, profile);
                this._setStatus("armed", "");
            }
        } else if (this.status !== "blocked") {
            // Held, but not a fresh press: it was already down when the pad
            // appeared, or it stayed down through a stop. It has to come up first.
            this._setStatus("blocked", this.lastStop || "the deadman was already held");
        }
        this._emit();
    }

    _focused() {
        return document.visibilityState === "visible" && document.hasFocus();
    }

    _blocker(enabled) {
        if (!enabled) {
            return "piloting needs Depth hold or Manual";
        }
        if (!this._focused()) {
            return "this window is not focused";
        }
        return "";
    }

    _offCentre(gp, profile) {
        for (const role of GAMEPAD_ROLES) {
            const mapping = profile.roles[role];
            if (mapping.axis < 0) {
                continue;
            }
            // A tilt axis takes its neutral at this press; a trigger and every
            // other stick has to be at rest.
            const raw = Number(gp.axes[mapping.axis]);
            if (!Number.isFinite(raw)) {
                return `${role} reads no value`;
            }
            if (mapping.range === "stick" && mapping.center === "arm") {
                continue;
            }
            const value = shapeGamepadAxis(raw, mapping, 0,
                profile.deadzone, profile.expo);
            if (value !== 0) {
                return `${role} is not at rest`;
            }
        }
        return "";
    }

    _shape(gp, profile) {
        const demand = {};
        for (const role of GAMEPAD_ROLES) {
            const mapping = profile.roles[role];
            const index = mapping.axis;
            demand[role] = index >= 0 && index < gp.axes.length
                ? shapeGamepadAxis(gp.axes[index], mapping,
                    this.neutral ? this.neutral[index] : 0, profile.deadzone, profile.expo)
                : 0;
        }
        return demand;
    }

    _setStatus(status, reason) {
        this.status = status;
        this.reason = reason;
    }

    // The callbacks run inside the keyboard pilot's tick. One that throws —
    // a panel missing from a stale cached page, say — must not take the keys
    // and the stop path down with it. Logged once per distinct error.
    _safely(callback) {
        try {
            callback();
        } catch (error) {
            const text = String(error && error.message || error);
            if (text !== this._lastError) {
                this._lastError = text;
                console.error("gui: gamepad callback failed", error);
            }
        }
    }

    _emit() {
        this._safely(() => this.onState({
            status: this.status,
            reason: this.reason,
            lastStop: this.lastStop,
            armed: this.armed,
            pad: this.pad,
            pads: this.pads(),
            profile: this.profile(),
            saved: this.hasSavedProfile(),
            demand: this._demand || {surge: 0, sway: 0, heave: 0, yaw: 0},
            raw: this.raw,
            streaming: this.streaming,
        }));
    }
}

if (typeof window !== "undefined") {
    window.GamepadPilot = GamepadPilot;
    window.GamepadPilotKit = {
        ROLES: GAMEPAD_ROLES,
        STANDARD_PROFILE: STANDARD_GAMEPAD_PROFILE,
        blankProfile: blankGamepadProfile,
        sanitizeProfile: sanitizeGamepadProfile,
        shapeAxis: shapeGamepadAxis,
    };
}
