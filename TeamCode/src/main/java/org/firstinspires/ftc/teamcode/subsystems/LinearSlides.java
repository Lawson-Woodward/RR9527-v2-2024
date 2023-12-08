package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LinearSlides {
    private DcMotorEx spool;
    private double speed = 1;
    int ground, minExtend = 800, maxExtend = 1200, baseExtend = 1000;

    public LinearSlides(HardwareMap hardwareMap) {
        spool = (DcMotorEx) hardwareMap.dcMotor.get("spool");
        //actuator.setDirection(DcMotorEx.Direction.REVERSE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //actuator.setTargetPosition(ground);
        spool.setPower(0);
    }

    /*public void setPosition(State state) {
        switch (state) {
            case GROUND:
                actuator.setTargetPosition(ground);
            case DEPOSITING:
                actuator.setTargetPosition(baseExtend);
        }
    }*/


    public void slidesManualUp() {
        if(spool.getCurrentPosition()>-1500) {
            spool.setPower(-1);
        } else {
            spool.setPower(0);
        }
    }

    public void slidesManualDown() {
        if(spool.getCurrentPosition()<250) {
            spool.setPower(1);
        } else {
            spool.setPower(0);
        }
    }

    public int getPosition() {
        return spool.getCurrentPosition();
    }

    public void setPower(double power) {
        spool.setPower(power);
    }

    public DcMotorEx getSlides(){
        return spool;
    }
}

/*public class LinearSlides implements Subsystem   {
    //public static LinearSlides slides;
    private PIDController pidController;
    private DcMotorEx spool;
    private Mode mode;
    double targetPosition;
    double currentPosition;

    enum Mode{
        FULL_EXTEND, NO_EXTEND
    }

    public LinearSlides(HardwareMap hardwareMap)
    {
        spool = hardwareMap.get(DcMotorEx.class, "spool");
        mode = Mode.NO_EXTEND;
        currentPosition = spool.getCurrentPosition();
        targetPosition = 0;

    }

    public void setMode(Mode m)
    {
        mode = m;
    }

    public void switchMode()
    {
        if(mode == Mode.FULL_EXTEND)
        {
            mode = Mode.NO_EXTEND;
        }
        else
        {
            mode = Mode.FULL_EXTEND;
        }
    }

    public Mode getMode()
    {
        return mode;
    }

    public void drive(GamepadEx gamepad)
    {
        double targetPosition = 0;
        double currentPosition = spool.getCurrentPosition();
        switch(mode){
            case NO_EXTEND:
                targetPosition = 200; // find this value later

                break;
            case FULL_EXTEND:
                targetPosition = 1000; //find this later
                break;
        }
        spool.setPower(pidController.PIDControl(targetPosition, currentPosition));

    }

    public void slidesUp() {
        targetPosition = 1000;
        spool.setPower(pidController.PIDControl(targetPosition, currentPosition));
    }

    public void slidesDown() {
        targetPosition = 200;
        spool.setPower(pidController.PIDControl(targetPosition, currentPosition));
    }


}*/