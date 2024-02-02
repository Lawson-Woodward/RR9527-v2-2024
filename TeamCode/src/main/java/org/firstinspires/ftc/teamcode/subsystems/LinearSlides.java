package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LinearSlides {
    private DcMotorEx leftSlide, rightSlide;
    private double speed = 1;
    int ground, minExtend = 800, maxExtend = 1200, baseExtend = 1000;

    public LinearSlides(HardwareMap hardwareMap) {
        leftSlide = (DcMotorEx) hardwareMap.dcMotor.get("leftSlide");
        rightSlide = (DcMotorEx) hardwareMap.dcMotor.get("rightSlide");
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setPower(0.0);
        leftSlide.setPower(0.0);
    }

    public void slidesManualUp() {
        rightSlide.setPower(-1);
        leftSlide.setPower(-1);
    }

    public void slidesManualDown() {
        rightSlide.setPower(1);
        leftSlide.setPower(1);
    }

    public void holdSlides() {
        rightSlide.setPower(0.0);
        leftSlide.setPower(0.0);
    }

    public int getLeftPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightSlide.getCurrentPosition();
    }

    public void setRightSlidePower(double power) {
        rightSlide.setPower(power);
    }

    public void setLeftSlidePower(double power) {
        leftSlide.setPower(power);
    }

    public DcMotorEx getRightSlide() {
        return rightSlide;
    }

    public DcMotorEx getLeftSlide() {
        return leftSlide;
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