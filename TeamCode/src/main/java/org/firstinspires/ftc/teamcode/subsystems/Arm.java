package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm {

    private Servo arm;
    private double open = 0, close = 0;

    State state;
    private RunMode runMode;

    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.servo.get("arm");
        //arm.setDirection(Servo.Direction.REVERSE);
        arm.setPosition(0);
    }

    public void intake() {
        arm.setPosition(0); //this is the resting position
    }

    public void traveling() {
        arm.setPosition(-0.02); //a tiny bit up while traveling to avoid getting stuck
    }

    public void deposit() {
        arm.setPosition(0.6); //all the way down to depositing position; should be PARALLEL TO BACKDROP
    }


    public Servo getArm() {
        return arm;
    }
}
