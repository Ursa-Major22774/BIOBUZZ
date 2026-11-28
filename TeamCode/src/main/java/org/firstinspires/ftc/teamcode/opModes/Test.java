package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.resources.FieldTarget;

/** Shared controls for RedTeleOp and BlueTeleOp; not selectable by itself. */
public abstract class Test extends OpMode {
    protected enum Alliance { RED, BLUE }
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Follower follower;
    private Turret turret;
    private boolean previousX;
    private boolean previousY;
    private boolean previousA;
    private boolean previousB;
    private boolean previousPoseA;
    private boolean fieldReferenceSet;
    private GoBildaPinpointDriver pinpoint;
    private String preAimMessage;
    private Double requestedBearing;
    private boolean previousRightBumper;
    private String turretControl = "INIT - STOPPED";

    protected abstract Alliance getAlliance();
    protected Pose getFieldStartPose() { return null; }
    protected FieldTarget getPreAimTarget() { return null; }
    protected void updatePreAimTarget(Pose pose, boolean reset) { }
    protected String getPreAimSelectionRule() { return "Fixed field point"; }

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        shooter.stop();
        intake = new Intake(hardwareMap);
        intake.stop();
        turret = new Turret(hardwareMap);
        turret.sampleEncoder(intake.getTurretEncoderTicks());
        previousX = false;
        previousY = false;
        previousA = false;
        previousB = gamepad1.b;
        transfer = new Transfer(hardwareMap);
        transfer.stop();
        follower = Constants.create(hardwareMap);
        follower.stop();
        previousPoseA = gamepad1.a;
        previousRightBumper = gamepad1.right_bumper;
        fieldReferenceSet = false;
        if (getFieldStartPose() != null) updatePreAimTarget(getFieldStartPose(), true);
        preAimMessage = getPreAimTarget() == null ? "NO ALLIANCE TARGET CONFIGURED"
                : "INIT: at known start, tap A without LB to set field pose";
        if (getFieldStartPose() != null) {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        }
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        shooter.stop();
        turret.stop();
        turret.sampleEncoder(intake.getTurretEncoderTicks());
        captureTurretReference();
        follower.update(); // Only the follower updates the Pinpoint localizer.
        if (getFieldStartPose() != null && gamepad1.a && !previousPoseA
                && !gamepad1.left_bumper && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                follower.setPose(getFieldStartPose());
                fieldReferenceSet = true;
                updatePreAimTarget(getFieldStartPose(), true);
                preAimMessage = "FIELD POSE SET - keep chassis at start until PLAY";
            } else preAimMessage = "Pinpoint not READY; remain still, then tap A again";
        }
        previousPoseA = gamepad1.a;
        updateTargetSelection();
        showInitTelemetry();
    }

    @Override
    public void start() {
        shooter.idle();
        // Require a new button press after PLAY to capture an endpoint.
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousRightBumper = gamepad1.right_bumper;
    }

    @Override
    public void loop() {
        turret.sampleEncoder(intake.getTurretEncoderTicks());
        boolean turretLeft = gamepad1.dpad_left;
        boolean turretRight = gamepad1.dpad_right;
        boolean setup = gamepad1.left_bumper;
        captureTurretReference();
        // Capture only while stopped, on a button edge. Limits last for this run.
        boolean captureLeft = setup && !turretLeft && !turretRight
                && gamepad1.x && !previousX && !gamepad1.y && !gamepad1.a && !gamepad1.b;
        boolean captureRight = setup && !turretLeft && !turretRight
                && gamepad1.y && !previousY && !gamepad1.x && !gamepad1.a && !gamepad1.b;
        if (captureLeft) turret.saveLeftLimit();
        if (captureRight) turret.saveRightLimit();
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        boolean flushing = gamepad1.dpad_down;
        boolean shooting = gamepad1.right_trigger > 0.1 && !flushing;
        boolean collecting = gamepad1.left_trigger > 0.1;

        // Preserve original stick signs and shooting turn-speed reduction.
        double turnMultiplier = shooting ? 0.333 : 1.0;
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * turnMultiplier);
        follower.update();
        updateTargetSelection();
        requestedBearing = null;
        if (gamepad1.right_bumper && !previousRightBumper) turret.retryPreAim();
        previousRightBumper = gamepad1.right_bumper;
        if (flushing) {
            turretControl = "FLUSH - STOPPED";
            turret.stop();
        } else if (turretLeft || turretRight || setup || !gamepad1.right_bumper) {
            // D-pad and setup always override pre-aim. Release RB stops tracking.
            turretControl = setup ? "SETUP JOG" : turretLeft || turretRight
                    ? "D-PAD MANUAL OVERRIDE" : "RB RELEASED - NOT TRACKING";
            turret.manual(turretLeft, turretRight, setup);
        } else {
            turretControl = "RB HELD - PRE-AIM REQUESTED";
            updatePreAim();
        }

        // Flush takes priority; each mechanism gets one request per loop.
        if (flushing) {
            shooter.stop();
            intake.flush();
            transfer.flush();
        } else {
            if (shooting) shooter.shoot();
            else shooter.idle();

            // Preserve manual feeding: LT runs intake and transfer together.
            if (collecting) {
                intake.intake();
                transfer.transfer();
            } else {
                intake.stop();
                transfer.stop();
            }
        }

        telemetry.addData("Alliance", getAlliance());
        showPreAimTelemetry();
        showTurretTelemetry();
        telemetry.addData("Flywheel", flushing ? "OFF (flush)" : shooting ? "SHOOT" : "IDLE");
        telemetry.addData("Shoot target (ticks/s)", shooting ? Shooter.TARGET_VELOCITY : 0);
        telemetry.addData("Flywheel measured (ticks/s)", shooter.getVelocity());
        telemetry.addData("Flywheel power", shooter.getMotorPower());
        telemetry.addData("Intake / transfer", flushing ? "REVERSE" : collecting ? "FORWARD" : "OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.stop();
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (transfer != null) transfer.stop();
        if (follower != null) follower.stop();
    }

    private void showInitTelemetry() {
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Initialized - press PLAY to start flywheel idle");
        telemetry.addData("Flywheel", "OFF");
        telemetry.addData("Flywheel power", shooter.getMotorPower());
        showPreAimTelemetry();
        showTurretTelemetry();
        telemetry.addLine("LT: intake + transfer | RT: shoot | D-pad down: flush");
        telemetry.update();
    }

    private void showTurretTelemetry() {
        telemetry.addData("Turret", turret.getStatus());
        telemetry.addData("Turret encoder (ticks)", turret.getEncoderTicks());
        telemetry.addData("Turret delta (ticks/loop)", turret.getDeltaTicks());
        telemetry.addData("Turret servo power", turret.getPower());
        telemetry.addData("Turret LEFT limit", turret.getLeftLimit() == null ? "NOT SET" : turret.getLeftLimit());
        telemetry.addData("Turret RIGHT limit", turret.getRightLimit() == null ? "NOT SET" : turret.getRightLimit());
        telemetry.addData("Turret limits", turret.limitsReady() ? "ACTIVE without LB" : "SETUP REQUIRED");
        telemetry.addData("Turret full turn (ticks)", Turret.TICKS_PER_TURRET_REVOLUTION);
        telemetry.addData("Turret FORWARD count", turret.getForwardReferenceTicks() == null
                ? "NOT SET" : turret.getForwardReferenceTicks());
        Double turretAngle = turret.getAngleDegrees();
        if (turretAngle == null) telemetry.addData("Turret angle (deg)", "SET FORWARD WITH LB + B");
        else telemetry.addData("Turret angle (deg)", "%.2f", turretAngle);
        telemetry.addData("Turret unwind target", turret.isUnwinding() ? turret.getUnwindTarget() : "NONE");
        telemetry.addLine("Hold D-pad at a limit: unwind one turn; release D-pad: STOP");
        telemetry.addLine("Turret: hold LB + D-pad L/R to setup jog (bypasses limits)");
        telemetry.addLine("At physical LEFT reference, release D-pad; LB + A: load saved range");
        telemetry.addLine("Shooter straight along chassis FRONT: release D-pad; LB + B: set forward zero");
        telemetry.addLine("Release D-pad; hold LB + tap X: save LEFT | Y: save RIGHT");
    }

    private void captureTurretReference() {
        if (gamepad1.left_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right
                && !gamepad1.x && !gamepad1.y) {
            if (gamepad1.a && !previousA && !gamepad1.b) turret.referenceLeft();
            if (gamepad1.b && !previousB && !gamepad1.a) turret.referenceForward();
        }
        previousA = gamepad1.a;
        previousB = gamepad1.b;
    }

    private void updatePreAim() {
        FieldTarget target = getPreAimTarget();
        if (target == null || !fieldReferenceSet || pinpoint == null
                || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            turret.stop();
            preAimMessage = target == null ? "NO ALLIANCE TARGET CONFIGURED"
                    : !fieldReferenceSet ? "BLOCKED - set field pose with A during INIT"
                    : "BLOCKED - PINPOINT NOT READY";
            return;
        }
        Pose pose = follower.pose();
        requestedBearing = target.relativeBearingDegrees(pose.x(), pose.y(), pose.heading());
        turret.aimAtDegrees(requestedBearing);
        preAimMessage = "POSE PRE-AIM ONLY - CELL STATE UNCONFIRMED";
    }

    private void updateTargetSelection() {
        if (fieldReferenceSet && pinpoint != null
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            updatePreAimTarget(follower.pose(), false);
        }
    }

    private void showPreAimTelemetry() {
        if (getPreAimTarget() == null) return;
        Pose pose = follower.pose();
        telemetry.addData("Turret control", turretControl);
        telemetry.addData("Turret state", turret.getStatus());
        telemetry.addData("Turret power", "%.3f", turret.getPower());
        telemetry.addData("Turret fault", turret.getFault() == null ? "NONE" : turret.getFault());
        if (turret.getLastPreAimFault() != null) {
            telemetry.addData("Last pre-aim fault", turret.getLastPreAimFault());
        }
        telemetry.addData("Field reference", fieldReferenceSet ? "SET" : "NOT SET - INIT + A (no LB)");
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Robot X / Y / heading", "%.2f / %.2f / %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Selected pre-aim target", getPreAimTarget().name);
        telemetry.addData("CELL selection", getPreAimSelectionRule());
        telemetry.addData("Pre-aim", preAimMessage);
        if (requestedBearing != null) telemetry.addData("Requested turret bearing (deg)", "%.2f", requestedBearing);
        if (turret.getAimTargetDegrees() != null) {
            telemetry.addData("Turret target / actual / error", "%.2f / %.2f / %.2f deg",
                    turret.getAimTargetDegrees(), turret.getAngleDegrees(), turret.getAimErrorDegrees());
        }
        telemetry.addLine("Hold RB: red pre-aim | D-pad: manual override | release RB: stop");
        telemetry.addLine("CELL state unconfirmed; feeding remains manual");
    }
}
