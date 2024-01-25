package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Wrist {

    private Servo wrist;
    private double open = 0, close = 0;



    State state;
    private RunMode runMode;

    public Wrist(HardwareMap hardwareMap){
        wrist = hardwareMap.servo.get("wrist");
        //arm.setDirection(Servo.Direction.REVERSE);   //do we need to reverse this???
        wrist.setPosition(0.98);
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
        wrist.setPosition(0);
    }

    public Servo getWrist() {
        return wrist;
    }
}
