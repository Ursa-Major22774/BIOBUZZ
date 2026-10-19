package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transfer;

public class Test {
    private Intake intake;

    public void init() {
        telemetry.addData("Does it work?", "left (%.2f)", "Huzzah!");
        telemetry.update();

        intake = new Intake(hardwareMap);
    }
    public void loop() {
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
        } else {
            intake.stop();
        }
    }
}
