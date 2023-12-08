package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
            BL = (DcMotorEx) hardwareMap.dcMotor.get("BL");
            BR = (DcMotorEx) hardwareMap.dcMotor.get("BR");
            FL = (DcMotorEx) hardwareMap.dcMotor.get("FL");
            FR = (DcMotorEx) hardwareMap.dcMotor.get("FR");

            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            BL.setDirection(DcMotor.Direction.REVERSE);
            BR.setDirection(DcMotor.Direction.FORWARD);
            FL.setDirection(DcMotor.Direction.REVERSE);
            FR.setDirection(DcMotor.Direction.FORWARD);


            waitForStart();



            //============ THIS IS WHERE THE CAMERA VISION OBJECT DETECTION GOES =============//

            //continue with movement

            ElapsedTime timer = new ElapsedTime();
            //need camera to return a 1, 2, or 3 based on where the team prop is
            //can do something with switch cases here: case 1, 2, and 3

            int time2 = 300;
            timer = new ElapsedTime();
            while (timer.milliseconds() <= time2) {      //strafe?
                BL.setPower(-1);
                BR.setPower(1);
                FL.setPower(-1);
                FR.setPower(1);
            }
            int time3 = 600;
            timer = new ElapsedTime();
            while (timer.milliseconds() <= time3) {      //forward?
                BL.setPower(1);
                BR.setPower(1);
                FL.setPower(1);
                FR.setPower(1);
            }

        }
    }
}