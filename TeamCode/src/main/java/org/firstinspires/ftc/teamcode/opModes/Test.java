package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
@TeleOp(name="Testing OpMode")
public class Test extends OpMode {

    //initializations
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Arm arm;
    private Follower follower;

    double TurningMult = 1;

    //@Override
    public void init() {
        follower = Constants.create(hardwareMap); //PedroPathing follower setup

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        arm = new Arm(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //@Override
    public void start() {
        arm.closeGate();
        arm.zero();
    }
    //@Override
    public void loop() {
        double forward = -gamepad1.left_stick_y; //drive train updates each loop
        double lateral = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x*TurningMult; //adjusts turning rate based on if shooting or not
        follower.manual(forward, lateral, turn);
        follower.update();

        //limelight testing code
        double[] camResults = arm.cameraTest();
        telemetry.addData("Cam Tx", camResults[0]);
        telemetry.addData("Cam Ty", camResults[1]);
        telemetry.addData("Cam Ta", camResults[2]);
        telemetry.addData("Detects AprilTag", arm.detectsAprilTag);

        //control triggers
        //Prefered control map once finalized: LT for intake, RT for shooter, down arrow for ball flush, standard movement controls, Y for arm trim up, A for arm trim down.
        if (gamepad1.left_trigger > 0.1) { //intake trigger
            arm.zero();
            intake.intake();
            transfer.transfer();
            telemetry.addData("Intake:","Triggered");
        } else {
            intake.stop();
            transfer.stop();
            telemetry.addData("Intake:","Stopped");
        }
        if (gamepad1.right_trigger > 0.1) { //shooter
            TurningMult = 0.333;
            shooter.shoot();
            transfer.transfer();
            arm.openGate();
            telemetry.addData("Shooter:","Triggered");
        } else {
            TurningMult = 1;
            shooter.stop();
            transfer.stop();
            arm.closeGate();
            telemetry.addData("Shooter:","Stopped");
        }
        if (gamepad1.dpad_down) { //Flush balls.
            shooter.stop();
            transfer.flush();
            intake.flush();
        }

        if (gamepad1.y) { //Move arm.
            arm.raiseArm();
        } else if (gamepad1.a) {
            arm.lowerArm();
        } else if (gamepad1.b) {
            arm.zero();
        }

        //testing code; remove when no longer relevant

        //PIDF adjustment code. Keep for potential future use. Activate sibling code in arm for functionality.
        /*
        telemetry.addData("kp:", arm.kp);
        telemetry.addData("ki:", arm.ki);
        telemetry.addData("kd:", arm.kd);
        telemetry.addData("kf:", arm.kf);
        telemetry.addData("Target Variable:", arm.AdjVar);
        telemetry.addData("Target Arm Position",arm.TargetPos);
        if (gamepad1.dpad_up) {
            arm.valueAdjustmentPos();
        }
        if (gamepad1.dpad_down) {
            arm.valueAdjustmentNeg();
        }
        if (gamepad1.dpad_left) {
            arm.targetAdjustment();
        }
        telemetry.addData("Arm Power", arm.getMotorPower()); */
        telemetry.addData("Arm Position", arm.getPosition());
        telemetry.addData("Target Position", arm.TargetPos);
        telemetry.addData("Sensor?", arm.MagSenseisPressed);

        //sets arm position
        arm.setPosition(arm.TargetPos);

        //mag sensor test
        arm.sensorCheck();

        //sends all the telemetry data gathered so far
        telemetry.update();
    }
}
