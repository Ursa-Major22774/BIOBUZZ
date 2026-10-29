package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
@Autonomous(name="Test Auto")
public class Auto_Test extends OpMode {

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

        telemetry.addData("Does it work?", "Huzzah!"); //telemetry update test
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
        for (int i = 0; i >= 50; i++) {
            arm.raiseArm();
        }
        for (int i = 0; i >= 50; i++) {
            arm.lowerArm();
        }
        arm.zero();
    }
}
