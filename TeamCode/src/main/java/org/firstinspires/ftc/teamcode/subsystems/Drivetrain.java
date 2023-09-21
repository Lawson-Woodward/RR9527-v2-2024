package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Drivetrain implements Subsystem {
    public static SampleMecanumDrive drive;
    private DcMotorEx FL, BL, FR, BR;
    double y, x, rx, FLPower, BLPower, FRPower, BRPower, heading, rotX, rotY;
    private RevIMU imu;
    private Mode mode;
    private double speed = 0.85, desiredHeading = 0.0;

    enum Mode {
        FC, RC
    }

    public Drivetrain(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        BL.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setDirection(DcMotorEx.Direction.REVERSE);

        imu = new RevIMU(hardwareMap);
        imu.init();
        mode = Mode.FC;
    }

    public void setMode(Mode m) {
        mode = m;
    }

    public void recenter() {
        imu.reset();
    }

    public void switchModes() {
        if(mode.equals(Mode.RC)) {
            mode = Mode.FC;
        }
        else {
            mode = Mode.RC;
        }
    }

    public String getMode() {
        if(mode == Mode.FC) {
            return "FIELD CENTRIC";
        }
        else {
            return "ROBOT CENTRIC";
        }
    }

    public void drive(GamepadEx gamepad) {
        y = Math.pow(gamepad.getLeftY(), 3);
        x = Math.pow(gamepad.getLeftX(), 3);
        rx = Math.pow(gamepad.getRightX(), 3);

        switch (mode) {
            case FC:
                heading = Math.toRadians(-imu.getHeading()+180);
                rotX = x * Math.cos(heading) - y * Math.sin(heading);
                rotY = x * Math.sin(heading) + y * Math.cos(heading);

                FLPower = (rotY + rotX + rx);
                BLPower = (rotY - rotX + rx)*1.1;
                FRPower = (rotY - rotX - rx);
                BRPower = (rotY + rotX - rx)*1.1;
                break;

            case RC:
                //y = -y;         //switch robot front
                //x = -x;

                FLPower = (y + x + rx);
                BLPower = (y - x + rx);
                FRPower = (y - x - rx);
                BRPower= (y + x - rx);
                break;
        }

        powerMotors(FLPower, BLPower, FRPower, BRPower);
    }

    private void powerMotors(double FLPower, double BLPower, double FRPower, double BRPower){
        FL.setPower(FLPower * speed);
        BL.setPower(BLPower * speed);
        FR.setPower(FRPower * speed);
        BR.setPower(BRPower * speed);
    }

    public double getHeading() {
        return imu.getHeading();
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}