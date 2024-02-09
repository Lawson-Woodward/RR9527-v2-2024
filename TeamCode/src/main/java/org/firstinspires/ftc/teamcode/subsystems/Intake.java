package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private DcMotorEx intake;
    private double speed = 1;
    int ground, minExtend = 800, maxExtend = 1200, baseExtend = 1000;

    public Intake(HardwareMap hardwareMap) {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);
    }

    public void intakeManualOut() {
        intake.setPower(-1.0);
    }

    public void intakeManualIn() {
        intake.setPower(1.0);
    }

    public void holdIntake() {
        intake.setPower(0.0);
    }

    public int intakePosition() {
        return intake.getCurrentPosition();
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public DcMotorEx getIntake() {
        return intake;
    }
}
