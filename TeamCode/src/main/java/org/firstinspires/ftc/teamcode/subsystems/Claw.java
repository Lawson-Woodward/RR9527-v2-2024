package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Claw {

    private Servo claw;
    private double open = 0, close = 0;

    State state;
    private RunMode runMode;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");
        //arm.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(.0789);
    }

    public void openClaw() {
        claw.setPosition(.0784313725);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }

    public void setPosition() {
        if(claw.getPosition()>0.07) {
            closeClaw();
        }
        else if (claw.getPosition()<0.01) {
            openClaw();
        }
    }

    public Servo getClaw() {
        return claw;
    }
}
