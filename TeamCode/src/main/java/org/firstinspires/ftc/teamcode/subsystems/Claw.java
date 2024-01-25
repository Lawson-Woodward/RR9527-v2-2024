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

    public void releaseOne() {
        //SLIGHTLY open the claw, just to let the first pixel out
        //claw.setPosition(0.01);    //a little bit more than 0
    }

    public void releaseTwo() {
        //COMPLETELY open the claw, even if there are two in the outtake at once
        //claw.setPosition(0.02);    //quite a bit more than 0
    }

    public void closeClaw() {

    }

    public Servo getRightClaw() {
        return rightClaw;
    }

    public Servo getLeftClaw() {
        return leftClaw;
    }
}
