package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode {
        public DcMotorEx BL;
        public DcMotorEx BR;
        public DcMotorEx FL;
        public DcMotorEx FR;

        /*private int leftBack;
        private int rightBack;
        private int leftFront;
        private int rightFront;*/
        @Override
        public void runOpMode() {
            BL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
            BR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
            FL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
            FR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");

            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //backLeft.setDirection(DcMotor.Direction.REVERSE);
            //backRight.setDirection(DcMotor.Direction.FORWARD);
            FL.setDirection(DcMotor.Direction.REVERSE);
            FR.setDirection(DcMotor.Direction.FORWARD);


            waitForStart();
            ElapsedTime timer = new ElapsedTime();

            int time2 = 300;
            timer = new ElapsedTime();
            while (timer.milliseconds() <= time2) {
                //backLeft.setPower(-1);
                //backRight.setPower(1);
                FL.setPower(-1);
                FR.setPower(1);
            }
            int time3 = 600;
            timer = new ElapsedTime();
            while (timer.milliseconds() <= time3) {
                //backLeft.setPower(1);
               // backRight.setPower(1);
                FL.setPower(1);
                FR.setPower(1);
            }

        }
    }