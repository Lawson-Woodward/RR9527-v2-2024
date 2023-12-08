package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private ServoEx intakeServo;

    public Intake(HardwareMap hardwareMap)
    {
        intakeServo = (ServoEx) hardwareMap.servo.get("intakeServo");

        intakeServo.setPosition(0); // needs to be tuned
    }

    public void moveUp() { intakeServo.setPosition(100);} // arbitrary value that needs to be tuned

    public void moveDown(){intakeServo.setPosition(50);} // arbitrary value that needs to be tuned

    public double getIntakeServoPosition()
    {
        return intakeServo.getPosition();
    }


}
