package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.resources.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public /*static*/ int ARM_POSITION = -200;
    public static double ARM_VELOCITY = 200;
    public static double ARM_POWER = 1;
    private final DcMotorEx arm;
    private final PIDFController armController;
    private final Limelight3A limelight;
    public static double tx;
    public static double ty;
    public static double ta;
    public static double[] camValues = new double[3];

    // Telemetry Variables
    public boolean detectsAprilTag = false;

    //adjustment variables, remove when no longer relevant
    public double kp = 0.0034;
    public double ki = 0.0005;
    public double kd = 0.0002;
    public double kf = 0.002;
    public String AdjVar = "kp";
    public int TargetPos = 150;

    public Arm(HardwareMap hardwareMap) {
        //Limelight initialization
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.deleteSnapshots();

        //Motor initialization
        armController=new PIDFController(0.0034,0.0005,0.0002,0.002);// kp = power, ki = error, kd = break, kf = gravity offset,
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //resets encoder position on initialization
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //allows motor to hold a set position.
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public double[] cameraTest() {
        LLResult result = limelight.getLatestResult();
        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();
        camValues[0] = tx;
        camValues[1] = ty;
        camValues[2] = ta;
        return camValues;
    }

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
        if (AdjVar.equals("ki")) {
            ki -= 0.00005;
        }
        if (AdjVar.equals("kd")) {
            kd -= 0.00005;
        }
        if (AdjVar.equals("kf")) {
            kf -= 0.00005;
        }
        if (AdjVar.equals("Pos")) {
            TargetPos -= 1;
        }
        armController.setP(kp);
        armController.setI(ki);
        armController.setD(kd);
        armController.setF(kf);
    }
    public void targetAdjustment() {
        if (AdjVar.equals("kp")) {
            AdjVar = "ki";
        } else if (AdjVar.equals("ki")) {
            AdjVar = "kd";
        } else if (AdjVar.equals("kd")) {
            AdjVar = "kf";
        } else if (AdjVar.equals("kf")) {
            AdjVar = "Pos";
        } else if (AdjVar.equals("Pos")) {
            AdjVar = "kp";
        }
    }

}
