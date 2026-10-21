package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public static int ARM_POSITION = 1;
    public static double ARM_POWER = 0.5;
    private final DcMotorEx arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(ARM_POWER);
    }
    public void raise() {
        arm.setTargetPosition(ARM_POSITION);
    }
    public void lower() {
        arm.setTargetPosition(0);
    }
}
