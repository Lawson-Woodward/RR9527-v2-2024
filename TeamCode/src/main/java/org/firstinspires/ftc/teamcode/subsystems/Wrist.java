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
        wrist.setPosition(0.73);
    }

    public void intake() {
        wrist.setPosition(0.64); //this is the resting position
    }

    public void transition() {
        wrist.setPosition(0.72); //a tiny bit up while traveling to avoid getting stuck
    }

    public void shortDeposit() {
        wrist.setPosition(0.53); //all the way down to depositing position; should be PARALLEL TO BACKDROP
    }

    public Servo getWrist() {
        return wrist;
    }
}
