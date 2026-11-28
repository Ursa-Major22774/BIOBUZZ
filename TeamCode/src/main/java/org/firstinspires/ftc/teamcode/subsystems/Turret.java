package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.LongSupplier;

/** Single-servo manual test. Encoder samples come from Intake's shared port 0. */
public class Turret {
    // Maximum manual command; approachPower slows movement near its limit.
    // Reported geometry: clockwise servo rotation turns the turret left (CCW).
    public static final double LEFT_POWER = 1.00;
    public static final double SETUP_POWER = 0.15;
    public static final double UNWIND_POWER = 1.00;
    // Team measurement: 5413 - (-16817), exactly one turret revolution.
    public static final int TICKS_PER_TURRET_REVOLUTION = 22230;
    // Team's measured safe range: 5882 - (-17443). This calibration survives
    // restarts because it is in code; raw endpoint counts do not define a home.
    public static final int CALIBRATED_LEFT_TO_RIGHT_TICKS = 23325;
    // Provisional position-control values, separate from full-speed manual jogging.
    public static final double AIM_KP_POWER_PER_DEG = 0.012;
    public static final double AIM_MAX_POWER = 1.00;
    // Provisional friction compensation, used only outside the angle tolerance.
    public static final double AIM_MIN_MOVING_POWER = 0.08;
    private static final double AIM_TOLERANCE_DEG = 2.0;
    private static final double AIM_SETTLE_SECONDS = 0.20;
    private static final int AIM_LIMIT_MARGIN_TICKS = 150;
    private static final double SLOWDOWN_DISTANCE_TICKS = 2500;
    private static final double UNWIND_TIMEOUT_SECONDS = 30;
    private static final double NO_PROGRESS_SECONDS = 1.5;
    private static final double MIN_SETTLE_SECONDS = 0.20;
    private static final double STATIONARY_SECONDS = 0.10;
    private static final double SETTLE_TIMEOUT_SECONDS = 1.5;

    private final CRServo servo;
    private final LongSupplier clock;
    private int encoderTicks;
    private long deltaTicks;
    private boolean hasSample;
    private Integer leftLimit;
    private Integer rightLimit;
    private Integer forwardReferenceTicks;
    private double power;
    private String status = "OFF";
    private int unwindRequest; // -1: driver held left; +1: driver held right.
    private long unwindTarget;
    private long unwindStartTime;
    private long progressTime;
    private int progressTicks;
    private String fault;
    private String lastPreAimFault;
    private boolean settling;
    private long settleStartTime;
    private long stationarySince;
    private int settleTicks;
    private Double aimTargetTicks;
    private Double aimErrorDeg;
    private long aimInToleranceSince = -1;
    private boolean aimSettled;
    private int aimDirection;
    private int aimProgressTicks;
    private long aimProgressTime;
    private long aimCommandStartTime;

    public Turret(HardwareMap hardwareMap) {
        this(hardwareMap, System::nanoTime);
    }

    public Turret(HardwareMap hardwareMap, LongSupplier clock) {
        this.clock = clock;
        // The broken turretRight is disconnected and is deliberately not mapped.
        servo = hardwareMap.get(CRServo.class, "turretLeft");
        servo.setDirection(CRServo.Direction.FORWARD);
        stop();
    }

    public void sampleEncoder(int ticks) {
        deltaTicks = hasSample ? (long) ticks - encoderTicks : 0;
        encoderTicks = ticks;
        hasSample = true;
    }

    public void saveLeftLimit() {
        if (hasSample) {
            leftLimit = encoderTicks;
            forwardReferenceTicks = null;
            fault = null;
            stop();
        }
    }

    public void saveRightLimit() {
        if (hasSample) {
            rightLimit = encoderTicks;
            forwardReferenceTicks = null;
            fault = null;
            stop();
        }
    }

    /** Operator places the turret at the same physical LEFT reference each run. */
    public void referenceLeft() {
        if (!hasSample) return;
        long right = (long) encoderTicks + CALIBRATED_LEFT_TO_RIGHT_TICKS;
        if (right < Integer.MIN_VALUE || right > Integer.MAX_VALUE) {
            failUnwind("REFERENCE FAULT - ENCODER RANGE");
            return;
        }
        leftLimit = encoderTicks;
        rightLimit = (int) right;
        forwardReferenceTicks = null;
        fault = null;
        stop();
        status = "REFERENCED - SAVED RANGE LOADED";
    }

    /** Operator aligns the shooter with chassis forward, then captures this count. */
    public void referenceForward() {
        stop();
        if (!hasSample || !limitsReady()) {
            status = "SET LEFT REFERENCE BEFORE FORWARD";
            return;
        }
        if (encoderTicks < Math.min(leftLimit, rightLimit)
                || encoderTicks > Math.max(leftLimit, rightLimit)) {
            status = "FORWARD REFERENCE OUTSIDE LIMITS";
            return;
        }
        forwardReferenceTicks = encoderTicks;
        status = "FORWARD REFERENCE SET";
    }

    /** Unwrapped mechanical angle: zero forward, positive left/CCW, negative right. */
    public Double getAngleDegrees() {
        if (forwardReferenceTicks == null || !limitsReady()) return null;
        int towardRight = Integer.compare(rightLimit, leftLimit);
        return -((long) encoderTicks - forwardReferenceTicks) * towardRight
                * 360.0 / TICKS_PER_TURRET_REVOLUTION;
    }

    public Integer getForwardReferenceTicks() { return forwardReferenceTicks; }

    public boolean limitsReady() {
        return leftLimit != null && rightLimit != null && !leftLimit.equals(rightLimit);
    }

    /** Explicit new operator request retries an aim fault; unwind faults stay latched. */
    public void retryPreAim() {
        if (fault != null && fault.startsWith("PRE-AIM FAULT")) {
            lastPreAimFault = fault;
            fault = null;
            stop();
        }
    }

    /** Hold setup to jog without limits. Normal motion unwinds at an endpoint. */
    public void manual(boolean left, boolean right, boolean setup) {
        clearAim();
        if (!hasSample) {
            command(0, "NO ENCODER SAMPLE");
            return;
        }
        if (left == right) {
            stop(); // Release or both pressed = stop.
            return;
        }
        if (setup) {
            unwindRequest = 0;
            settling = false;
            command(left ? SETUP_POWER : -SETUP_POWER, "SETUP JOG - LIMITS BYPASSED");
            return;
        }
        if (!limitsReady()) {
            command(0, "LOCKED - SET BOTH LIMITS");
            return;
        }
        // Actual D-pad motion is a manual override, including after an aim fault.
        // The no-button path above still preserves the fault for diagnosis.
        retryPreAim();
        if (fault != null) {
            command(0, fault);
            return;
        }

        // Infer encoder sign from the named endpoints, so either count direction works.
        int towardRight = Integer.compare(rightLimit, leftLimit);
        int request = left ? -1 : 1;
        if (unwindRequest != 0) {
            if (request == unwindRequest) {
                updateUnwind(towardRight);
                return;
            }
            // A changed driver direction cancels the unwind and takes control.
            unwindRequest = 0;
            settling = false;
        }
        boolean atLeft = ((long) encoderTicks - leftLimit) * towardRight <= 0;
        boolean atRight = ((long) encoderTicks - rightLimit) * towardRight >= 0;
        if ((left && atLeft) || (right && atRight)) {
            beginUnwind(request, towardRight);
            return;
        }
        long distanceToLimit = left
                ? ((long) encoderTicks - leftLimit) * towardRight
                : ((long) rightLimit - encoderTicks) * towardRight;
        double magnitude = approachPower(LEFT_POWER, distanceToLimit);
        command(left ? magnitude : -magnitude, left ? "LEFT" : "RIGHT");
    }

    public void stop() {
        clearAim();
        unwindRequest = 0;
        settling = false;
        command(0, fault == null ? "STOPPED" : fault);
    }

    /** Hold a legal equivalent of the requested angle; never wrap motor error through a limit. */
    public void aimAtDegrees(double requestedDegrees) {
        unwindRequest = 0;
        settling = false;
        if (fault != null) { stop(); return; }
        if (!hasSample || !limitsReady() || forwardReferenceTicks == null) {
            stop();
            status = "PRE-AIM BLOCKED - SET LEFT AND FORWARD REFERENCES";
            return;
        }
        if (!Double.isFinite(requestedDegrees)) {
            stop();
            status = "PRE-AIM BLOCKED - INVALID GEOMETRY";
            return;
        }
        int towardRight = Integer.compare(rightLimit, leftLimit);
        double wrappedDegrees = Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(requestedDegrees)), Math.cos(Math.toRadians(requestedDegrees))));
        double baseTicks = forwardReferenceTicks - wrappedDegrees
                * TICKS_PER_TURRET_REVOLUTION / 360.0 * towardRight;
        double low = Math.min(leftLimit, rightLimit) + AIM_LIMIT_MARGIN_TICKS;
        double high = Math.max(leftLimit, rightLimit) - AIM_LIMIT_MARGIN_TICKS;
        double minTurn = Math.ceil((low - baseTicks) / TICKS_PER_TURRET_REVOLUTION);
        double maxTurn = Math.floor((high - baseTicks) / TICKS_PER_TURRET_REVOLUTION);
        if (minTurn > maxTurn) {
            stop();
            status = "PRE-AIM UNREACHABLE - REPOSITION CHASSIS";
            return;
        }
        // Keep the selected revolution continuous as the bearing crosses +/-180.
        double anchor = aimTargetTicks == null ? encoderTicks : aimTargetTicks;
        double turn = Math.max(minTurn, Math.min(maxTurn,
                Math.rint((anchor - baseTicks) / TICKS_PER_TURRET_REVOLUTION)));
        aimTargetTicks = baseTicks + turn * TICKS_PER_TURRET_REVOLUTION;
        double tickError = aimTargetTicks - encoderTicks;
        aimErrorDeg = -tickError * towardRight * 360.0 / TICKS_PER_TURRET_REVOLUTION;
        long now = clock.getAsLong();
        if (Math.abs(aimErrorDeg) <= AIM_TOLERANCE_DEG) {
            if (aimInToleranceSince < 0) aimInToleranceSince = now;
            aimSettled = (now - aimInToleranceSince) / 1e9 >= AIM_SETTLE_SECONDS;
            aimDirection = 0;
            command(0, aimSettled ? "PRE-AIM SETTLED - CELL UNCONFIRMED" : "PRE-AIM SETTLING");
            return;
        }
        aimInToleranceSince = -1;
        aimSettled = false;
        int direction = tickError > 0 ? 1 : -1;
        if (direction != aimDirection) {
            aimDirection = direction;
            aimProgressTicks = encoderTicks;
            aimProgressTime = aimCommandStartTime = now;
        }
        long progress = ((long) encoderTicks - aimProgressTicks) * direction;
        if (progress >= 2) {
            aimProgressTicks = encoderTicks;
            aimProgressTime = now;
        }
        if ((now - aimCommandStartTime) / 1e9 > 0.30 && progress < -64) {
            fault = "PRE-AIM FAULT - WRONG ENCODER DIRECTION";
            lastPreAimFault = fault;
            stop();
            return;
        }
        if ((now - aimProgressTime) / 1e9 >= NO_PROGRESS_SECONDS) {
            fault = "PRE-AIM FAULT - NO ENCODER PROGRESS";
            lastPreAimFault = fault;
            stop();
            return;
        }
        double effort = Math.max(-AIM_MAX_POWER,
                Math.min(AIM_MAX_POWER, AIM_KP_POWER_PER_DEG * aimErrorDeg));
        if (Math.abs(effort) < AIM_MIN_MOVING_POWER) {
            effort = Math.copySign(AIM_MIN_MOVING_POWER, aimErrorDeg);
        }
        command(effort, "PRE-AIM TRACKING - CELL UNCONFIRMED");
    }

    private void clearAim() {
        aimTargetTicks = null;
        aimErrorDeg = null;
        aimInToleranceSince = -1;
        aimSettled = false;
        aimDirection = 0;
    }

    private void beginUnwind(int request, int towardRight) {
        long target = (long) encoderTicks - (long) request * towardRight * TICKS_PER_TURRET_REVOLUTION;
        long low = Math.min(leftLimit, rightLimit);
        long high = Math.max(leftLimit, rightLimit);
        if (target < low || target > high) {
            command(0, "LIMIT - NO ROOM FOR FULL TURN");
            return;
        }
        unwindRequest = request;
        unwindTarget = target;
        // Stop the old command and wait for coast motion to settle before
        // establishing the full-turn target and checking reverse progress.
        settling = true;
        settleStartTime = stationarySince = clock.getAsLong();
        settleTicks = encoderTicks;
        updateUnwind(towardRight);
    }

    private void updateUnwind(int towardRight) {
        if (settling) {
            updateSettling();
            return;
        }
        // The unwind moves opposite the direction the driver continues to hold.
        int encoderDirection = -unwindRequest * towardRight;
        long remaining = (unwindTarget - encoderTicks) * encoderDirection;
        if (remaining <= 0) {
            unwindRequest = 0;
            command(0, "UNWIND COMPLETE");
            return; // Resume the held driver direction on the next loop.
        }
        boolean atLeft = ((long) encoderTicks - leftLimit) * towardRight <= 0;
        boolean atRight = ((long) encoderTicks - rightLimit) * towardRight >= 0;
        if ((unwindRequest > 0 && atLeft) || (unwindRequest < 0 && atRight)) {
            failUnwind("UNWIND FAULT - OPPOSITE LIMIT");
            return;
        }
        long now = clock.getAsLong();
        long progress = ((long) encoderTicks - progressTicks) * encoderDirection;
        if (progress < -32) {
            failUnwind("UNWIND FAULT - WRONG ENCODER DIRECTION");
            return;
        }
        if (progress >= 2) {
            progressTicks = encoderTicks;
            progressTime = now;
        }
        if ((now - progressTime) / 1e9 >= NO_PROGRESS_SECONDS) {
            failUnwind("UNWIND FAULT - NO ENCODER PROGRESS");
            return;
        }
        if ((now - unwindStartTime) / 1e9 >= UNWIND_TIMEOUT_SECONDS) {
            failUnwind("UNWIND FAULT - TIMEOUT");
            return;
        }
        double magnitude = approachPower(UNWIND_POWER, remaining);
        command(unwindRequest > 0 ? magnitude : -magnitude,
                unwindRequest > 0 ? "UNWIND LEFT - HOLD RIGHT TO CONTINUE" : "UNWIND RIGHT - HOLD LEFT TO CONTINUE");
    }

    private void failUnwind(String reason) {
        fault = reason;
        unwindRequest = 0;
        settling = false;
        command(0, reason);
    }

    private void updateSettling() {
        command(0, "STOPPING BEFORE UNWIND");
        long now = clock.getAsLong();
        if (Math.abs((long) encoderTicks - settleTicks) >= 2) {
            settleTicks = encoderTicks;
            stationarySince = now;
        }
        if ((now - settleStartTime) / 1e9 >= SETTLE_TIMEOUT_SECONDS) {
            failUnwind("UNWIND FAULT - DID NOT SETTLE");
            return;
        }
        if ((now - settleStartTime) / 1e9 < MIN_SETTLE_SECONDS
                || (now - stationarySince) / 1e9 < STATIONARY_SECONDS) {
            return;
        }
        int towardRight = Integer.compare(rightLimit, leftLimit);
        long target = (long) encoderTicks - (long) unwindRequest * towardRight * TICKS_PER_TURRET_REVOLUTION;
        if (target < Math.min(leftLimit, rightLimit) || target > Math.max(leftLimit, rightLimit)) {
            failUnwind("UNWIND FAULT - NO ROOM AFTER SETTLING");
            return;
        }
        unwindTarget = target;
        unwindStartTime = progressTime = now;
        progressTicks = encoderTicks;
        settling = false;
        updateUnwind(towardRight);
    }

    private void command(double requestedPower, String requestedStatus) {
        power = requestedPower;
        status = requestedStatus;
        servo.setPower(power);
    }

    private double approachPower(double maximumPower, long remainingTicks) {
        // Slow before an endpoint/target to reduce overshoot at the higher speed.
        double scale = Math.max(0, Math.min(1, remainingTicks / SLOWDOWN_DISTANCE_TICKS));
        return Math.max(SETUP_POWER, maximumPower * scale);
    }

    public int getEncoderTicks() { return encoderTicks; }
    public long getDeltaTicks() { return deltaTicks; }
    public Integer getLeftLimit() { return leftLimit; }
    public Integer getRightLimit() { return rightLimit; }
    public double getPower() { return power; }
    public String getStatus() { return status; }
    public boolean isUnwinding() { return unwindRequest != 0; }
    public long getUnwindTarget() { return unwindTarget; }
    public Double getAimErrorDegrees() { return aimErrorDeg; }
    public Double getAimTargetDegrees() {
        if (aimTargetTicks == null || forwardReferenceTicks == null || !limitsReady()) return null;
        return -(aimTargetTicks - forwardReferenceTicks) * Integer.compare(rightLimit, leftLimit)
                * 360.0 / TICKS_PER_TURRET_REVOLUTION;
    }
    public boolean isPreAimSettled() { return aimSettled; }
    public String getFault() { return fault; }
    public String getLastPreAimFault() { return lastPreAimFault; }
}
