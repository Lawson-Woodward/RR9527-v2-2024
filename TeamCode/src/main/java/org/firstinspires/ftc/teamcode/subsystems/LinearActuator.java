package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class LinearActuator {
    private boolean canGoDown;
    private DcMotorEx actuator;
    private double speed = 1;
    int ground, minExtend = 800, maxExtend = 1200, baseExtend = 1000;

    public LinearActuator(HardwareMap hardwareMap) {
        canGoDown = true;
        //actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator = (DcMotorEx) hardwareMap.dcMotor.get("actuator");
        //actuator.setDirection(DcMotorEx.Direction.REVERSE);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //actuator.setTargetPosition(ground);
        actuator.setPower(0);
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
            actuator.setPower(-1);
        }
        else {
            actuator.setPower(0);
        }
    }

    public void moveDown() {
        if(actuator.getCurrentPosition()<0 || canGoDown) {
            actuator.setPower(1);
        } else {
            actuator.setPower(0);
        }
    }


    public void canGoDown(boolean val) {
        canGoDown = val;
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
