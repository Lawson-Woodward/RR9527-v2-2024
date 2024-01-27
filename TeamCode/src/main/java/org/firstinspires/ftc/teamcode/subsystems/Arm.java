package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm {
    private State currentArmState = State.REST;
    private ElapsedTime timer = new ElapsedTime();
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
        arm.setPosition(0.99); //this is the resting position
    }

    public void transition() {
        arm.setPosition(0.52); //a tiny bit up while traveling to avoid getting stuck
    }

    public void shortDeposit() {
        arm.setPosition(0.68); //all the way down to depositing position; should be PARALLEL TO BACKDROP
    }

    public void executeTeleOp() {
        switch (currentArmState) {
            case REST:
                timer.reset();
                transition();
            case INTAKE:
                timer.reset();
        }
    }

    public void setIntakePosition(double num) {
        arm.setPosition(num);
    }


    public Servo getArm() {
        return arm;
    }
}
