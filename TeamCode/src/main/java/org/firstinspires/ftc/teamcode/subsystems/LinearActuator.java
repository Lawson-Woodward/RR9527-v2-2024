package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class LinearActuator {
    private DcMotorEx actuator;
    private double speed = 1;
    int ground, minExtend = 800, maxExtend = 1200, baseExtend = 1000;

    public LinearActuator(HardwareMap hardwareMap) {
        //actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator = (DcMotorEx) hardwareMap.dcMotor.get("actuator");
        //actuator.setDirection(DcMotorEx.Direction.REVERSE);
        actuator.setPower(0);
        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //actuator.setTargetPosition(ground);
    }

    /*public void setPosition(State state) {
        switch (state) {
            case GROUND:
                actuator.setTargetPosition(ground);
            case DEPOSITING:
                actuator.setTargetPosition(baseExtend);
        }
    }*/

    public void moveUp() {
        if(actuator.getCurrentPosition()>-4500) {
            actuator.setPower(-0.5);
        }
        else {
            actuator.setPower(0);
        }
    }

    public void moveDown() {
        if(actuator.getCurrentPosition()<0) {
            actuator.setPower(0.5);
        }
        else {
            actuator.setPower(0);
        }
    }

    public int getPosition() {
        return actuator.getCurrentPosition();
    }

    public void setPower(double power) {
        actuator.setPower(power);
    }

    public DcMotorEx getActuator(){
        return actuator;
    }
}
