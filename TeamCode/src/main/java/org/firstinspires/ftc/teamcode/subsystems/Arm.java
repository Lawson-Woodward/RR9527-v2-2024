package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm {

    private Servo arm;
    public static double armPos = 0;
    private double open = 0, close = 0;

    State state;
    private RunMode runMode;

    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.servo.get("arm");
        //arm.setDirection(Servo.Direction.REVERSE);
        arm.setPosition(0.52);
    }

    public void intake() {
        arm.setPosition(0.51); //this is the resting position
    }

    public void traveling() {
        arm.setPosition(0.49); //a tiny bit up while traveling to avoid getting stuck
    }

    public void deposit() {
        arm.setPosition(0.66); //all the way down to depositing position; should be PARALLEL TO BACKDROP
    }

    public void setIntakePosition(double num) {
        arm.setPosition(num);
    }


    public Servo getArm() {
        return arm;
    }
}
