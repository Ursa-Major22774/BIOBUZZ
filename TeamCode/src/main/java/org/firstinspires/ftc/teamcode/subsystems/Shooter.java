package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.resources.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    public static double TARGET_VELOCITY = 1400;
    public static double IDLE_VELOCITY = 300;
    public double currentVelocity = 0;
    public double maxVelocity = 0;
    private final DcMotorEx shooterMotor1;
    private final PIDFController shooterManager;

    public Shooter(HardwareMap hardwareMap) {
        shooterManager = new PIDFController(2.24432,0.22443,0,22.44315);
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setPower(1);
        //shooterMotor1.setTargetPosition(5); //default PIDF position value reccomendation.
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void flatShoot() { //just sets the motors to run at full power.
        shooterMotor1.setPower(1);
        currentVelocity = shooterMotor1.getVelocity();
        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }
    }
    public void stop() { //despite the name, motors keep running at idle power.
        shooterMotor1.setVelocity(IDLE_VELOCITY);
    }
    public void shoot() {
        shooterMotor1.setVelocity(TARGET_VELOCITY);
        currentVelocity = shooterMotor1.getVelocity();
        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }
    }

    public double getMotorPower() {
        double MotorVal;
        MotorVal = shooterMotor1.getPower();
        return (MotorVal);
    }
}
