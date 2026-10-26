package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor; //considering putting cs code in here

public class Transfer {

    private CRServo servoOne = null;
    private CRServo servoTwo = null;

    public static double TRANSFER_POWER = 1;
    public Transfer(HardwareMap hardwareMap) {
        servoOne = hardwareMap.get(CRServo.class,"transferServo1"); //maps transfer servos to its hardware map
        servoTwo = hardwareMap.get(CRServo.class, "transferServo2");
    }

    public void transfer() {
        servoOne.setPower(-TRANSFER_POWER); //sets servo powers in opposite directions when activated.
        servoTwo.setPower(TRANSFER_POWER);

    }
    public void stop() {
        servoOne.setPower(0); //resets to idle when stopped
        servoTwo.setPower(0);
    }
}
