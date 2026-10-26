package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.resources.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm {

    public /*static*/ int ARM_POSITION = -200;
    public static double ARM_VELOCITY = 200;
    public static double ARM_POWER = 1;
    private final DcMotorEx arm;
    private final PIDFController armController;
    //private final Limelight3A limelight;
    public static double tx;
    public static double ty;
    public static double ta;
    public static double[] camValues = new double[3];

    public Arm(HardwareMap hardwareMap) {
        //Limelight initialization
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.start();
        armController=new PIDFController(0.0015,0,0,0.32);
        //Motor initialization
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //resets encoder position on initialization
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //allows motor to hold a set position.
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void setPosition(int targetPos){
        arm.setPower(armController.calculate(arm.getCurrentPosition(),targetPos));
    }
    public void setPower(double motorPower){
        arm.setPower(motorPower);
    }

    public void stop(){
        arm.setPower(0);
    }
    public int getPosition() {
        return arm.getCurrentPosition();
    }
    public double getMotorPower() {
        return arm.getPower();
    }
    //testing code; remove when no longer relevant.
    public void manualDecreasePos() {
        ARM_POSITION += 1;
    }
    public void manualIncreasePos() {
        ARM_POSITION -= 1;
    }
}
