package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.commands.*;

public class AnonymousServo {
    public static double AnonymousServoMinPos = 0.0, AnonymousServoMaxPos = 1.0, AnonymousServoNeutralPos = 0.5;
    private Servo servo;

    public AnonymousServo(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.servo.get(name);
        servo.setPosition(0.5);
    }

    public void setToMin() {
        servo.setPosition(AnonymousServoMinPos);
    }

    public void setToMax() {
        servo.setPosition(AnonymousServoMaxPos);
    }

    public void setToNeutral() {
        servo.setPosition(AnonymousServoNeutralPos);
    }

    public Servo getServo() {
        return servo;
    }
}
