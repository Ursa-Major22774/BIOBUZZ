package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Localization-only push test: does not initialize or command motor/servo hardware. */
@TeleOp(name = "BIOBUZZ Red Pose Test", group = "Diagnostics")
public class RedPoseTest extends OpMode {
    // Corrected field-tool screenshot: front toward the RIGHT of the Pedro
    // field view (+X). Heading 0 degrees is shown in the ROBOT panel.
    private static final double START_X_IN = 30.7228246331;
    private static final double START_Y_IN = 62.7353389114;
    private static final double START_HEADING_DEG = 0;
    private final Pose startPose = PoseFactory.degrees().of(
            START_X_IN, START_Y_IN, START_HEADING_DEG);

    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private boolean referenceSet;
    private boolean previousA;
    private String message;

    @Override
    public void init() {
        referenceSet = false;
        previousA = gamepad1.a;
        // Uses the existing Pedro 3.0.1 pod configuration and IMU calibration.
        // This localizer is the only owner calling update() on Pinpoint.
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                Constants.localizerConfig.name.get());
        message = "Keep stationary during calibration; confirm placement with A in INIT";
        showTelemetry(false);
    }

    @Override
    public void init_loop() {
        localizer.update();
        if (gamepad1.a && !previousA) {
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                localizer.setPose(startPose);
                referenceSet = true;
                message = "Start pose set. Keep robot at the mark until PLAY";
            } else {
                message = "Pinpoint not READY; remain still, then tap A again";
            }
        }
        previousA = gamepad1.a;
        showTelemetry(false);
    }

    @Override
    public void start() {
        message = referenceSet ? "Push test active - all mechanism controls disabled"
                : "NO REFERENCE - STOP, INIT, confirm placement with A, then PLAY";
    }

    @Override
    public void loop() {
        localizer.update();
        // No pose resets after PLAY. Translation/rotation should now be measured.
        showTelemetry(true);
    }

    private void showTelemetry(boolean running) {
        Pose pose = localizer.pose();
        double headingDeg = (Math.toDegrees(pose.heading()) % 360 + 360) % 360;
        telemetry.addData("Mode", "RED POSE DIAGNOSTIC - NO MOTOR OUTPUT");
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Reference", referenceSet ? "SET" : "NOT SET");
        telemetry.addData("Instructions", message);
        telemetry.addData("Required start X / Y (in)", "%.3f / %.3f", START_X_IN, START_Y_IN);
        telemetry.addData("Required front direction", "RIGHT of field view (+X), 0 deg");
        telemetry.addData("X (in)", "%.3f", pose.x());
        telemetry.addData("Y (in)", "%.3f", pose.y());
        telemetry.addData("Heading (deg)", "%.2f", headingDeg);
        if (referenceSet) {
            telemetry.addData("Change X / Y (in)", "%.3f / %.3f",
                    pose.x() - START_X_IN, pose.y() - START_Y_IN);
        }
        if (!running) telemetry.addLine("Place at the start, front toward +X; tap A when READY");
        telemetry.addLine("First test: after PLAY, push BACKWARD 12 inches away from HIVE, without turning");
        telemetry.update();
    }
}
