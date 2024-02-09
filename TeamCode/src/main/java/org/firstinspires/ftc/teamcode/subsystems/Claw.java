package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Claw {

    private Servo rightClaw, leftClaw;
    private double open = 0, close = 0;



    State state;
    private RunMode runMode;

    public Claw(HardwareMap hardwareMap){
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");
        //arm.setDirection(Servo.Direction.REVERSE);   //do we need to reverse this???
        rightClaw.setPosition(0.51);
        leftClaw.setPosition(0.62);
    }

    public void openLeft() {
        leftClaw.setPosition(0.54);
    }

    public void openRight() {
        rightClaw.setPosition(0.6);
    }

    public void closeRight() {
        rightClaw.setPosition(0.51);
    }

    public void closeLeft() {
        leftClaw.setPosition(0.625);
    }
    public void looseRight() {
        rightClaw.setPosition(0.555);
    }

    public void looseLeft() {
        leftClaw.setPosition(0.58);
    }

    public Servo getRightClaw() {
        return rightClaw;
    }

    public Servo getLeftClaw() {
        return leftClaw;
    }
}
