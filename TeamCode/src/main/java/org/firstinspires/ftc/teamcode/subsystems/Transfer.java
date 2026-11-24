package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor; //considering putting cs code in here

public class Transfer {

    private DcMotorEx transferMotor = null;

    public static double TRANSFER_POWER = 1;
    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class,"transferMotor");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void transfer() {
        transferMotor.setPower(TRANSFER_POWER); //sets servo powers in opposite directions when activated.
    }
    public void flush() {
        transferMotor.setPower(-TRANSFER_POWER);
    }
    public void stop() {
        transferMotor.setPower(0); //resets to idle when stopped
    }
}
