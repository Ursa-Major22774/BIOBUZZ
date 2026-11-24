package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.resources.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    //PIDF Adjustment Variables for tuning
    /*
    public double kp = 0;
    public double ki = 0;
    public double kd = 0;
    public double kf = 0;
    public String AdjVar = "kp";
     */

    //Testing variables
    public double currentVelocity = 0;
    public double maxVelocity = 0;

    //Shooting constants
    public static double TARGET_VELOCITY = 1400;
    public static double IDLE_POWER = 0.333;

    //Hardware
    private final DcMotorEx shooterMotor1;
    private final PIDFController shooterManager;

    public Shooter(HardwareMap hardwareMap) {
        shooterManager = new PIDFController(2.24432,0.22443,0,22.44315); //Untuned, just based on default calculations
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setPower(0.333);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void flatShoot() { //just sets the motors to run at full power.
        shooterMotor1.setPower(1);
        currentVelocity = shooterMotor1.getVelocity();
        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }
    }
    public void stop() { //despite the name, motors keep running at idle power.
        shooterMotor1.setVelocity(IDLE_POWER);
    }
    public void shoot() {
        shooterMotor1.setPower(shooterManager.calculate(shooterMotor1.getCurrentPosition(),TARGET_VELOCITY));
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
    public double getMotorCalculation() { //PIDF Testing
        return shooterManager.calculate(shooterMotor1.getCurrentPosition(),TARGET_VELOCITY);
    }

    //PIDF Tuning code. Activate sibling code in TeleOp before running, as well as above variables.
    //Originally for position-based PIDF. May require rewrites.
    /*
    public void valueAdjustmentPos() {
        if (AdjVar.equals("kp")) {
            kp += 0.00005;
        }
        if (AdjVar.equals("ki")) {
            ki += 0.00005;
        }
        if (AdjVar.equals("kd")) {
            kd += 0.00005;
        }
        if (AdjVar.equals("kf")) {
            kf += 0.00005;
        }
        if (AdjVar.equals("Pos")) {
            TargetPos += 1;
        }
        armController.setP(kp);
        armController.setI(ki);
        armController.setD(kd);
        armController.setF(kf);
    }
    public void valueAdjustmentNeg() {
        if (AdjVar.equals("kp")) {
            kp -= 0.00005;
        }
     */
}
