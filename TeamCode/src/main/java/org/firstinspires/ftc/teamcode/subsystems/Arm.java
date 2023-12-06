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

    public void armResting() {
        arm.setPosition(.6784313725);
    }

    public void armUp() {
        arm.setPosition(.0784313725);
    }

    public void armDepositing() {
        arm.setPosition(0.33);
    }


    public Servo getArm() {
        return arm;
    }
}
