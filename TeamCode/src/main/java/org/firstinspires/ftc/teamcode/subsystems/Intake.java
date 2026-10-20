package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intakeMotor;
    public static double INTAKE_POWER = -0.5; //sets the power and direction of the intake motor.

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); //maps intakeMotor to its hardware map.
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void intake() {
        intakeMotor.setPower(INTAKE_POWER);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
}