// Keyboard piloting.
//
//   W/S  A/D     forward-reverse / strafe        ┐ proportional and ramped, all
//   ← / →        turn                            │ scaled by the speed level and
//   Q / E        up / down                       ┘ by Shift
//   1 … 5        speed level, 20 … 100 %
//   Shift        fine: ×0.3 while held, for lining up on a prop
//   C            cruise: keep the current forward/strafe with no key held
//   Space        stop moving: drop every key and the cruise at once, and hold
//                the depth and heading the vehicle is at (the holds stay on)
//   [ / ]        heading target −/+15°, with Shift 90° (heading hold only)
//
// Where each axis goes depends on what the vehicle is holding, because a key
// should ask for the same *motion* in every mode:
//
//   surge/sway   always an open-loop force on control/wrench_sources/gui.
//   yaw          heading hold on: a turn rate on control/targets/yaw_rate_dps.
//                heading_hold_node walks its target round at that rate and
//                brakes where the key is let go. A torque instead spins the
//                vehicle past 150°/s in two seconds with nothing to stop it —
//                measured in the sim, and why the old keyboard felt like
//                steering on ice.
//                heading hold off: a torque, as before.
//   up/down      Depth hold: walks control/targets/depth_m at a steady rate and
//                the depth PID flies there. Manual: nothing holds depth (Manual
//                deactivates the depth PID), so it is a direct vertical force.
//
// wrench_sum has a single global active flag and no per-mode source filter, so
// the GUI wrench is summed whenever the bus is live — verified in DEPTH_HOLD: a
// GUI force.x of 20 arrives at control/wrench_command intact alongside the
// PID's force.z.
//
// Ramping: a key no longer slams the thrusters from zero to full. Every axis
// moves towards its demand at RAMP_UP_PER_S and back to zero at the faster
// RAMP_DOWN_PER_S, so letting go still stops promptly. Losing focus, Space and
// leaving a piloting mode skip the ramp and zero at once.
//
// Held keys are composed every tick, so W+D is a diagonal rather than whichever
// key won a race.

const TICK_HZ = 20;

const SPEED_LEVELS = Object.freeze([0.2, 0.4, 0.6, 0.8, 1.0]);
const DEFAULT_SPEED_LEVEL = 3;
const FINE_SCALE = 0.3;

// Fractions of full scale per second: 0 → 100 % in 0.35 s, back in 0.15 s.
const RAMP_UP_PER_S = 1 / 0.35;
const RAMP_DOWN_PER_S = 1 / 0.15;

const HEADING_STEP_DEG = 15;
const HEADING_BIG_STEP_DEG = 90;

// How far the depth target may run ahead of the measured depth while Q/E is
// held. Without a limit a held key walks the setpoint metres away from a
// vehicle that cannot keep up, and it keeps going long after the key is up.
const DEPTH_TARGET_MAX_LEAD_M = 0.5;

// Down is positive in this stack, so E (deeper) is +1 on both the setpoint and
// the force axis.
const AXIS_KEYS = Object.freeze({
    KeyW: {axis: "surge", sign: +1},
    KeyS: {axis: "surge", sign: -1},
    KeyD: {axis: "sway", sign: +1},
    KeyA: {axis: "sway", sign: -1},
    ArrowRight: {axis: "yaw", sign: +1},
    ArrowLeft: {axis: "yaw", sign: -1},
    KeyE: {axis: "heave", sign: +1},
    KeyQ: {axis: "heave", sign: -1},
});
const AXES = Object.freeze(["surge", "sway", "heave", "yaw"]);

const SPEED_KEYS = Object.freeze({
    Digit1: 1, Digit2: 2, Digit3: 3, Digit4: 4, Digit5: 5,
    Numpad1: 1, Numpad2: 2, Numpad3: 3, Numpad4: 4, Numpad5: 5,
});
const FINE_KEYS = Object.freeze(new Set(["ShiftLeft", "ShiftRight"]));
const HEADING_KEYS = Object.freeze({BracketLeft: -1, BracketRight: +1});

function zeroAxes() {
    return {surge: 0, sway: 0, heave: 0, yaw: 0};
}

// Move `current` towards `target`, faster when the magnitude is shrinking (or
// the sign is flipping) than when it is growing.
function approach(current, target, dt) {
    const shrinking = Math.abs(target) < Math.abs(current)
        || Math.sign(target) * Math.sign(current) < 0;
    const step = (shrinking ? RAMP_DOWN_PER_S : RAMP_UP_PER_S) * dt;
    if (Math.abs(target - current) <= step) {
        return target;
    }
    return current + Math.sign(target - current) * step;
}

class KeyboardPilot {
    /**
     * @param socket       GuiSocket
     * @param protocol     GuiProtocol
     * @param options
     *   getEnabled       () => bool    — gate; motion keys are ignored when false
     *   getVerticalMode  () => "target" | "force"  — Q/E: depth setpoint or force
     *   getYawMode       () => "rate" | "torque"   — arrows: turn rate or torque
     *   getTargetDepth   () => number  — current depth setpoint
     *   getDepth         () => number  — measured depth, for the setpoint lead limit
     *   getHeading       () => number|null — measured heading, degrees
     *   getHeadingTarget () => number|null — heading hold target, degrees
     *   onState          (state) => void — for the on-screen key map
     */
    constructor(socket, protocol, options = {}) {
        this.socket = socket;
        this.protocol = protocol;
        this.getEnabled = options.getEnabled || (() => true);
        this.getVerticalMode = options.getVerticalMode || (() => "target");
        this.getYawMode = options.getYawMode || (() => "torque");
        this.getTargetDepth = options.getTargetDepth || (() => 0);
        this.getDepth = options.getDepth || (() => null);
        this.getHeading = options.getHeading || (() => null);
        this.getHeadingTarget = options.getHeadingTarget || (() => null);
        this.onState = options.onState || (() => {});

        // Full scale, i.e. what speed level 5 asks for. Level 3 (the default)
        // is 60 % of these: 24 N and 12 N·m, close to the old fixed 20 / 10.
        this.translationForceN = 40;
        this.yawTorqueNm = 20;
        this.yawRateDps = 90;
        this.verticalForceN = 30;
        this.depthRateMps = 0.4;

        this.speedLevel = DEFAULT_SPEED_LEVEL;
        this.held = new Set();
        // Cruise holds a direction per axis (−1, 0, +1), not a force, so the
        // speed keys still change how fast the vehicle cruises.
        this.cruise = null;
        this.output = zeroAxes();

        this._timer = null;
        this._lastTick = null;
        this._wasPublishingWrench = false;
        this._wasPublishingRate = false;
        this._depthTarget = null;
    }

    start() {
        window.addEventListener("keydown", this._onKeyDown);
        window.addEventListener("keyup", this._onKeyUp);
        // Losing focus must release everything. Without this, alt-tabbing while
        // holding W leaves the vehicle driving forward with no key to let go of
        // and no visible cause — the keyup never arrives. Cruise is dropped for
        // the same reason: nobody is watching the page any more.
        window.addEventListener("blur", this._releaseAll);
        document.addEventListener("visibilitychange", this._onVisibility);
        this._lastTick = performance.now();
        this._timer = window.setInterval(this._tick, 1000 / TICK_HZ);
        // Paint the speed level once now; otherwise the meter stays blank
        // until the first key, since nothing ticks out state while disarmed.
        this._emitState();
    }

    stop() {
        window.removeEventListener("keydown", this._onKeyDown);
        window.removeEventListener("keyup", this._onKeyUp);
        window.removeEventListener("blur", this._releaseAll);
        document.removeEventListener("visibilitychange", this._onVisibility);
        window.clearInterval(this._timer);
        this._releaseAll();
    }

    /** Speed level 1–5 times Shift. */
    scale() {
        return SPEED_LEVELS[this.speedLevel - 1] * (this._fine() ? FINE_SCALE : 1);
    }

    setSpeedLevel(level) {
        const clamped = Math.min(SPEED_LEVELS.length, Math.max(1, Math.round(level)));
        if (clamped !== this.speedLevel) {
            this.speedLevel = clamped;
            this._emitState();
        }
    }

    _fine() {
        return [...FINE_KEYS].some((code) => this.held.has(code));
    }

    _onVisibility = () => {
        if (document.hidden) {
            this._releaseAll();
        }
    };

    _isTypingTarget(target) {
        if (!target) {
            return false;
        }
        const tag = target.tagName;
        return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT"
            || target.isContentEditable;
    }

    _onKeyDown = (event) => {
        // Never steal keys from a number field — the operator is typing a gain,
        // not asking the vehicle to strafe.
        if (this._isTypingTarget(event.target)) {
            return;
        }
        if (event.ctrlKey || event.metaKey || event.altKey) {
            return;
        }
        const code = event.code;

        // Space is always ours. Left to the browser it scrolls the page or —
        // worse — clicks whichever button last had focus, which after arming
        // is a mode toggle.
        if (code === "Space") {
            event.preventDefault();
            if (!event.repeat && this.getEnabled()) {
                this.stopMoving();
            }
            return;
        }
        if (event.repeat) {
            if (AXIS_KEYS[code]) {
                event.preventDefault();
            }
            return;
        }

        if (FINE_KEYS.has(code)) {
            this.held.add(code);
            this._emitState();
            return;
        }
        if (SPEED_KEYS[code]) {
            // Works while disarmed too, so the level can be picked beforehand.
            this.setSpeedLevel(SPEED_KEYS[code]);
            return;
        }
        if (!this.getEnabled()) {
            return;
        }

        const mapping = AXIS_KEYS[code];
        if (mapping) {
            event.preventDefault();
            // Touching forward/strafe takes the vehicle back from cruise.
            if (this.cruise && (mapping.axis === "surge" || mapping.axis === "sway")) {
                this.cruise = null;
            }
            this.held.add(code);
            this._emitState();
            return;
        }
        if (code === "KeyC") {
            event.preventDefault();
            this._toggleCruise();
            return;
        }
        if (HEADING_KEYS[code]) {
            event.preventDefault();
            const step = event.shiftKey ? HEADING_BIG_STEP_DEG : HEADING_STEP_DEG;
            this._stepHeading(HEADING_KEYS[code] * step);
        }
    };

    _onKeyUp = (event) => {
        if (event.code === "Space" && !this._isTypingTarget(event.target)) {
            // Buttons fire their click on Space *up*; swallow that too.
            event.preventDefault();
        }
        if (this.held.delete(event.code)) {
            this._emitState();
        }
    };

    _releaseAll = () => {
        const moving = this.held.size > 0 || this.cruise !== null
            || AXES.some((axis) => this.output[axis] !== 0);
        this.held.clear();
        this.cruise = null;
        this._depthTarget = null;
        if (moving) {
            this.output = zeroAxes();
            this._publish();
            this._emitState();
        }
    };

    /** Space: zero every axis now and hold the depth and heading we are at. */
    stopMoving() {
        this._releaseAll();
        this.output = zeroAxes();
        this._publish();

        if (this.getVerticalMode() === "target") {
            const depth = Number(this.getDepth());
            if (Number.isFinite(depth)) {
                this._sendDepthTarget(Math.max(0, depth));
            }
        }
        const heading = this.getHeading();
        if (this.getYawMode() === "rate" && Number.isFinite(heading)) {
            this.socket.sendTopic(this.protocol.topics.targetHeadingDeg,
                {data: Number(heading.toFixed(2))});
        }
        this._emitState();
    }

    _toggleCruise() {
        if (this.cruise) {
            this.cruise = null;
            this._emitState();
            return;
        }
        const keys = this._keyDirections();
        if (keys.surge === 0 && keys.sway === 0) {
            // Nothing to keep. Say so rather than silently doing nothing.
            this._emitState({notice: "Hold W/A/S/D, then press C to cruise"});
            return;
        }
        this.cruise = {surge: keys.surge, sway: keys.sway};
        this._emitState();
    }

    _stepHeading(deltaDeg) {
        if (this.getYawMode() !== "rate") {
            this._emitState({notice: "[ and ] need heading hold — press H"});
            return;
        }
        const base = this.getHeadingTarget() ?? this.getHeading();
        if (!Number.isFinite(base)) {
            return;
        }
        const next = ((base + deltaDeg) % 360 + 360) % 360;
        this.socket.sendTopic(this.protocol.topics.targetHeadingDeg,
            {data: Number(next.toFixed(2))});
    }

    /** −1 / 0 / +1 per axis from the keys held right now. */
    _keyDirections() {
        const totals = zeroAxes();
        for (const code of this.held) {
            const mapping = AXIS_KEYS[code];
            if (mapping) {
                totals[mapping.axis] += mapping.sign;
            }
        }
        // Opposite keys held together cancel, which is what the operator sees
        // on screen and expects the vehicle to do.
        for (const axis of AXES) {
            totals[axis] = Math.sign(totals[axis]);
        }
        return totals;
    }

    /** What every axis is asked for, as a fraction of full scale. */
    demand() {
        const keys = this._keyDirections();
        if (this.cruise) {
            if (keys.surge === 0) keys.surge = this.cruise.surge;
            if (keys.sway === 0) keys.sway = this.cruise.sway;
        }
        // In Depth hold, Q/E move the setpoint instead (see _stepDepthTarget).
        if (this.getVerticalMode() !== "force") {
            keys.heave = 0;
        }
        const scale = this.scale();
        const demand = {};
        for (const axis of AXES) {
            demand[axis] = keys[axis] * scale;
        }
        return demand;
    }

    _tick = () => {
        const now = performance.now();
        const dt = Math.min(0.2, (now - this._lastTick) / 1000);
        this._lastTick = now;

        if (!this.getEnabled()) {
            // Left the piloting modes (or the link dropped): stop at once.
            this._releaseAll();
            return;
        }

        const demand = this.demand();
        for (const axis of AXES) {
            this.output[axis] = approach(this.output[axis], demand[axis], dt);
        }
        this._publish();
        this._stepDepthTarget(dt);
        this._emitState();
    };

    _publish() {
        const rateMode = this.getYawMode() === "rate";
        const out = this.output;
        const wrench = {
            force: {
                x: out.surge * this.translationForceN,
                y: out.sway * this.translationForceN,
                z: out.heave * this.verticalForceN,
            },
            torque: {
                x: 0,
                y: 0,
                z: rateMode ? 0 : out.yaw * this.yawTorqueNm,
            },
        };
        // Publish while anything is non-zero, plus exactly one zero frame after
        // it all reaches zero, so the bus is explicitly cleared instead of
        // relying on the source timeout to fade it out.
        const wrenchActive = Object.values(wrench.force).some((v) => v !== 0)
            || wrench.torque.z !== 0;
        if (wrenchActive || this._wasPublishingWrench) {
            this.socket.sendTopic(this.protocol.topics.wrenchCommand, wrench);
            this._wasPublishingWrench = wrenchActive;
        }

        const rate = rateMode ? out.yaw * this.yawRateDps : 0;
        if (rate !== 0 || this._wasPublishingRate) {
            this.socket.sendTopic(this.protocol.topics.yawRateDps,
                {data: Number(rate.toFixed(2))});
            this._wasPublishingRate = rate !== 0;
        }
    }

    _stepDepthTarget(dt) {
        const direction = this._keyDirections().heave;
        if (this.getVerticalMode() !== "target" || direction === 0) {
            this._depthTarget = null;
            return;
        }
        if (this._depthTarget === null) {
            const current = Number(this.getTargetDepth());
            this._depthTarget = Number.isFinite(current) ? current : 0;
        }
        let next = this._depthTarget + direction * this.depthRateMps * this.scale() * dt;
        const depth = Number(this.getDepth());
        if (Number.isFinite(depth)) {
            next = direction > 0
                ? Math.min(next, Math.max(this._depthTarget, depth + DEPTH_TARGET_MAX_LEAD_M))
                : Math.max(next, Math.min(this._depthTarget, depth - DEPTH_TARGET_MAX_LEAD_M));
        }
        next = Math.max(0, next);
        if (next !== this._depthTarget) {
            this._sendDepthTarget(next);
        }
    }

    _sendDepthTarget(depth) {
        this._depthTarget = depth;
        this.socket.sendTopic(this.protocol.topics.targetDepthM, {
            data: depth.toFixed(3),
        });
    }

    _emitState(extra = {}) {
        this.onState({
            held: new Set(this.held),
            output: {...this.output},
            speedLevel: this.speedLevel,
            fine: this._fine(),
            cruise: this.cruise !== null,
            enabled: this.getEnabled(),
            ...extra,
        });
    }
}

if (typeof window !== "undefined") {
    window.KeyboardPilot = KeyboardPilot;
    window.KeyboardPilotKeys = {AXIS_KEYS, SPEED_LEVELS};
}
