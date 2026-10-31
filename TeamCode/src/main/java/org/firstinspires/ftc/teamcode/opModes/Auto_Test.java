package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
@Autonomous(name="Test Auto")
public class Auto_Test extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    //Autonomous Poses
    //Each line (pose to pose) should be assigned its own method (see below).
    private final Pose startPose = p.of(0, 0, 0);
    private final Pose testPose = p.of(0, 20, 0);
    private final Pose endPose = p.of(1, 20, 0);
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
        schedule(follow(follower, park()));
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, pathTest()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }

    //Pose methods
    private Path park() {
        return line(testPose, endPose).linear(testPose, endPose);
    }
    private Path pathTest() {
        return line(startPose, testPose).linear(startPose, testPose);
    }
}
