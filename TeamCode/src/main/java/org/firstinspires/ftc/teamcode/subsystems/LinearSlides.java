package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.hardware.*;



public class LinearSlides {
    private DcMotorEx actuator, spool;
    double integralSum = 0;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public LinearSlides(HardwareMap hardwareMap, double targetPosition, double currentPosition) {
        spool = hardwareMap.get(DcMotorEx.class, "spool");

        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
}
