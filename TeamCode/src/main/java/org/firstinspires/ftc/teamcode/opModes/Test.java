package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

        //control triggers
        if (gamepad1.left_trigger > 0.1) { //intake trigger
            intake.intake();
            telemetry.addData("Intake T","Intake Triggered");
        } else {
            intake.stop();
            telemetry.addData("Intake T","Intake Stopped");
        }
        if (gamepad1.right_trigger > 0.1) {
            transfer.transfer();
            telemetry.addData("Transfer T","Transfer Triggered");
        } else {
            transfer.stop();
            telemetry.addData("Transfer T","Transfer Stopped");
        }
        if (gamepad1.left_bumper) {
            shooter.shoot();
            telemetry.addData("Shooter T","Shooter Triggered");
        } else {
            shooter.stop();
            telemetry.addData("Shooter T","Shooter Stopped");
        }
        if (gamepad1.a) {
            arm.raise();
            telemetry.addData("Arm T","Arm Raised");
        } else {
            arm.lower();
            telemetry.addData("Arm T","Arm Lowered");
        }
        telemetry.update();
    }
}
