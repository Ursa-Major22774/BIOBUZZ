package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.resources.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    public static double SHOOTER_POWER = 0.9;
    public static double IDLE_POWER = 0.333;
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    //private final PIDFController shooterManager;

    public Shooter(HardwareMap hardwareMap) {
        //shooterManager = new PIDFController(0.0005,0,0,0);
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void shoot() {
        shooterMotor1.setPower(-SHOOTER_POWER);
        shooterMotor2.setPower(SHOOTER_POWER);
        //shooterMotor1.setVelocity(shooterManager.calculate(shooterMotor1.getVelocity(),100));
        //shooterMotor2.setVelocity(shooterManager.calculate(shooterMotor2.getVelocity(),100));
    }
    public void stop() {
        shooterMotor1.setPower(-IDLE_POWER);
        shooterMotor2.setPower(IDLE_POWER);
    }

    public double[] getMotorPower() {
        double[] MotorVals = new double[2];
        MotorVals[0] = shooterMotor1.getPower();
        MotorVals[1] = shooterMotor2.getPower();
        return (MotorVals);
    }
}
