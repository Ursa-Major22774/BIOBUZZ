package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    // Original test values. Velocity is motor encoder ticks/second, not wheel RPM.
    // Motor/encoder scale and PIDF tuning come later.
    public static double TARGET_VELOCITY = 1400;
    public static double IDLE_POWER = 0.50;
    private final DcMotorEx flywheelMotor;

    public Shooter(HardwareMap hardwareMap) {
        // Preserve the exact hardware-map spelling from the saved robot code.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        flywheelMotor.setPower(0);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void idle() {
        // Power-based idle does not guarantee a specific RPM.
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setPower(IDLE_POWER);
    }

    public void shoot() {
        // Hub velocity PIDF replaces the old position-based Java calculation.
        // Use the configured hub coefficients; tuning is a later step.
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setVelocity(TARGET_VELOCITY);
    }

    public void stop() {
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setPower(0);
    }

    public double getMotorPower() {
        return flywheelMotor.getPower();
    }

    public double getVelocity() {
        return flywheelMotor.getVelocity();
    }

    private void setMode(DcMotorEx.RunMode mode) {
        if (flywheelMotor.getMode() != mode) {
            flywheelMotor.setMode(mode);
        }
    }
}
