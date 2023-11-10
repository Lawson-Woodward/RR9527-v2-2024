package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class LinearActuator {
    private DcMotorEx actuator;
    private double speed = 1;
    int target;
    int current = 0;
    int ground, minExtend = 800, maxExtend = 1200, baseExtend = 1000;

    enum Mode {POSITION, POWER}
    Mode mode = Mode.POWER;
    private int update, currentPosition;

    public LinearActuator(HardwareMap hardwareMap) {
        //actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator = (DcMotorEx) hardwareMap.dcMotor.get("actuator");
        //actuator.setDirection(DcMotorEx.Direction.REVERSE);
        //actuator.setPower(0.5);
        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setTargetPosition(ground);

        update = 20;
    }

    public void setPosition(State state) {
        switch (state) {
            case GROUND:
                actuator.setTargetPosition(ground);
            case DEPOSITING:
                actuator.setTargetPosition(baseExtend);
        }
    }

    public void moveUp() {
        if(actuator.getCurrentPosition()<1000) {
            actuator.setPower(0.5);
        }
        else {
            actuator.setPower(0);
        }
    }

    public void moveDown() {
        //current = actuator.getCurrentPosition();
        //actuator.setTargetPosition(-500);
        if(actuator.getCurrentPosition()>-1000) {
            actuator.setPower(-0.5);
        }
        else {
            actuator.setPower(0);
        }
    }

    public int getPosition() {
        return actuator.getCurrentPosition();
    }

    public DcMotorEx getActuator(){
        return actuator;
    }
}
