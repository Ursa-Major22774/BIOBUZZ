package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
@Autonomous(name="Test Auto")
public class Auto_Test extends OpMode {
    //Follower
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.3054, 30.0863, 180);

    //initializations
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Arm arm;

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
