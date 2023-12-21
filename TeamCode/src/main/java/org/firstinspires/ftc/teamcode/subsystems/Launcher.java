package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Launcher {

    private Servo airplane;
    public static double armPos = 0;
    private double open = 0, close = 0;

    State state;
    private RunMode runMode;

    public Launcher(HardwareMap hardwareMap){
        airplane = hardwareMap.servo.get("airplane");
        //arm.setDirection(Servo.Direction.REVERSE);
        airplane.setPosition(0.0);
    }

    public void hold() {
        airplane.setPosition(0.51); //this is the resting position
    }

    public void launch() {
        airplane.setPosition(0.49); //a tiny bit up while traveling to avoid getting stuck
    }

    public Servo getPlane() {
        return airplane;
    }
}
