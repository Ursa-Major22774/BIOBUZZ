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
    private double power;
    private String status = "OFF";
    private int unwindRequest; // -1: driver held left; +1: driver held right.
    private long unwindTarget;
    private long unwindStartTime;
    private long progressTime;
    private int progressTicks;
    private String fault;
    private boolean settling;
    private long settleStartTime;
    private long stationarySince;
    private int settleTicks;

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
            fault = null;
            stop();
        }
    }

    public void saveRightLimit() {
        if (hasSample) {
            rightLimit = encoderTicks;
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
        fault = null;
        stop();
        status = "REFERENCED - SAVED RANGE LOADED";
    }

    public boolean limitsReady() {
        return leftLimit != null && rightLimit != null && !leftLimit.equals(rightLimit);
    }

    /** Hold setup to jog without limits. Normal motion unwinds at an endpoint. */
    public void manual(boolean left, boolean right, boolean setup) {
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
        unwindRequest = 0;
        settling = false;
        command(0, fault == null ? "STOPPED" : fault);
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
}
