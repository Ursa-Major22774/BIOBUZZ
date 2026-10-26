package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm {

    public /*static*/ int ARM_POSITION = -200;
    public static double ARM_VELOCITY = 200;
    public static double ARM_POWER = 1;
    private final DcMotorEx arm;
    private final PIDFC
    //private final Limelight3A limelight;
    public static double tx;
    public static double ty;
    public static double ta;
    public static double[] camValues = new double[3];

    public Arm(HardwareMap hardwareMap) {
        //Limelight initialization
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.start();

        //Motor initialization
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //resets encoder position on initialization
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); //allows motor to hold a set position.
        arm.setPower(ARM_POWER);

    }
    /*public double[] cameraTest() {
        LLResult result = limelight.getLatestResult();
        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();
        camValues[0] = tx;
        camValues[1] = ty;
        camValues[2] = ta;
        return camValues;
    } */
    public void raise() {
        arm.setVelocity(ARM_VELOCITY);
        arm.setTargetPosition(ARM_POSITION);
    }
    public void lower() {
        arm.setTargetPosition(0);
        arm.setVelocity(0);
    }
    //testing code; remove when no longer relevant.
    public void manualDecreasePos() {
        ARM_POSITION += 1;
    }
    public void manualIncreasePos() {
        ARM_POSITION -= 1;
    }
}
