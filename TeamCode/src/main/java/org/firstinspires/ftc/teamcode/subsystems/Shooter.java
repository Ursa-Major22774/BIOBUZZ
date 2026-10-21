package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public static double SHOOTER_POWER = 0.5;
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void shoot() {
        shooterMotor1.setPower(-SHOOTER_POWER);
        shooterMotor2.setPower(SHOOTER_POWER);
    }
    public void stop() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
}
