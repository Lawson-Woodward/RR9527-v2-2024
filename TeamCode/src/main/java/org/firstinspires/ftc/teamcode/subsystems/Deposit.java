package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Deposit {

    private Servo rightClamp, leftClamp, deposit;
    private double open = 0, close = 0;



    State state;
    private RunMode runMode;

    public Deposit(HardwareMap hardwareMap){
        rightClamp = hardwareMap.servo.get("rightClamp");
        leftClamp = hardwareMap.servo.get("leftClamp");
        deposit = hardwareMap.servo.get("deposit");
        //arm.setDirection(Servo.Direction.REVERSE);   //do we need to reverse this??? nahhhh probably not
        rightClamp.setPosition(0.27);
        leftClamp.setPosition(0.7);
        deposit.setPosition(0.55);

    }

    public void openLeftClamp() {
        leftClamp.setPosition(0.7);
    }

    public void openRightClamp() {
        rightClamp.setPosition(0.27);
    }

    public void closeRightClamp() {
        rightClamp.setPosition(0.6);
    }

    public void closeLeftClamp() {
        leftClamp.setPosition(0.3);
    }

    public void depositing() {
        deposit.setPosition(0.2);
    }

    public void transitioning() {
        deposit.setPosition(0.55);
    }

    public void readyToClamp() {
        deposit.setPosition(0.6);
    }

    public Servo getRightClamp() {
        return rightClamp;
    }

    public Servo getLeftClamp() {
        return leftClamp;
    }
    public Servo getDeposit() {
        return deposit;
    }
}
