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
    //@Override
    public void init() {
        follower = Constants.create(hardwareMap); //PedroPathing follower setup

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        arm = new Arm(hardwareMap);

        telemetry.addData("Does it work?","Huzzah!"); //telemetry update test
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //@Override
    public void start() {
        telemetry.addData("Going?", "I'm Goin'"); //telemetry update test
        telemetry.update();
    }
    //@Override
    public void loop() {
        double forward = -gamepad1.left_stick_y; //drive train updates each loop
        double lateral = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
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
            intake.intake();
            telemetry.addData("Intake T","Intake Triggered");
        } else {
            intake.stop();
            telemetry.addData("Intake T","Intake Stopped");
        }
        if (gamepad1.right_trigger > 0.1) { //transfer
            transfer.transfer();
            telemetry.addData("Transfer T","Transfer Triggered");
        } else {
            transfer.stop();
            telemetry.addData("Transfer T","Transfer Stopped");
        }
        if (gamepad1.left_bumper) { //shooter
            shooter.shoot();
            telemetry.addData("Shooter T","Shooter Triggered");
        } else {
            shooter.stop();
            telemetry.addData("Shooter T","Shooter Stopped");
        }
        if (gamepad1.a) { //arm
            arm.setPosition(arm.TargetPos);
            telemetry.addData("Arm T","Arm Raised");
        } else {
            arm.stop();
            telemetry.addData("Arm T","Arm Lowered");
        }
        telemetry.addData("Arm Position", arm.getPosition());

        //testing code; remove when no longer relevant
        if (gamepad1.x) { //adjust arm position manually
            arm.manualIncreasePos();
        }
        if (gamepad1.b) {
            arm.manualDecreasePos();
        }
        //telemetry.addData("kp:", arm.kp);
        //telemetry.addData("ki:", arm.ki);
        //telemetry.addData("kd:", arm.kd);
        //telemetry.addData("kf:", arm.kf);
        //telemetry.addData("Target Variable:", arm.AdjVar);
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

        //telemetry.addData("Arm Position",arm.ARM_POSITION);
        telemetry.addData("Arm Power", arm.getMotorPower());

        //sends all the telemetry data gathered so far
        telemetry.update();
    }
}
