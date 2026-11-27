package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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

    protected abstract Alliance getAlliance();

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
        transfer = new Transfer(hardwareMap);
        transfer.stop();
        follower = Constants.create(hardwareMap);
        follower.stop();
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        shooter.stop();
        turret.stop();
        turret.sampleEncoder(intake.getTurretEncoderTicks());
        captureTurretReference();
        showInitTelemetry();
    }

    @Override
    public void start() {
        shooter.idle();
        // Require a new button press after PLAY to capture an endpoint.
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousA = gamepad1.a;
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
                && gamepad1.x && !previousX && !gamepad1.y;
        boolean captureRight = setup && !turretLeft && !turretRight
                && gamepad1.y && !previousY && !gamepad1.x;
        if (captureLeft) turret.saveLeftLimit();
        if (captureRight) turret.saveRightLimit();
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        turret.manual(turretLeft, turretRight, setup);

        boolean flushing = gamepad1.dpad_down;
        boolean shooting = gamepad1.right_trigger > 0.1 && !flushing;
        boolean collecting = gamepad1.left_trigger > 0.1;

        // Preserve original stick signs and shooting turn-speed reduction.
        double turnMultiplier = shooting ? 0.333 : 1.0;
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * turnMultiplier);
        follower.update();

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
        telemetry.addData("Turret unwind target", turret.isUnwinding() ? turret.getUnwindTarget() : "NONE");
        telemetry.addLine("Hold D-pad at a limit: unwind one turn; release D-pad: STOP");
        telemetry.addLine("Turret: hold LB + D-pad L/R to setup jog (bypasses limits)");
        telemetry.addLine("At physical LEFT reference, release D-pad; LB + A: load saved range");
        telemetry.addLine("Release D-pad; hold LB + tap X: save LEFT | Y: save RIGHT");
    }

    private void captureTurretReference() {
        if (gamepad1.left_bumper && gamepad1.a && !previousA
                && !gamepad1.dpad_left && !gamepad1.dpad_right
                && !gamepad1.x && !gamepad1.y) {
            turret.referenceLeft();
        }
        previousA = gamepad1.a;
    }
}
