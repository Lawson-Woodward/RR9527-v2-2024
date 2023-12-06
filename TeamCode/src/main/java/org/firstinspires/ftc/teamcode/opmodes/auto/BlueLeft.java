package org.firstinspires.ftc.teamcode.opmodes.auto;

public class BlueLeft {
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

    @Autonomous(name = "BlueLeft")
    public class BlueLeft extends LinearOpMode {
        public DcMotorEx backLeft;
        public DcMotorEx backRight;
        public DcMotorEx frontLeft;
        public DcMotorEx frontRight;

        /*private int leftBack;
        private int rightBack;
        private int leftFront;
        private int rightFront;*/
        @Override
        public void runOpMode() {
            backLeft = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
            backRight = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
            frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
            frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);


            waitForStart();
            ElapsedTime timer = new ElapsedTime();

            int time2 = 300;
            timer = new ElapsedTime();
            while (timer.milliseconds() <= time2) {
                backLeft.setPower(-1);
                backRight.setPower(1);
                frontLeft.setPower(-1);
                frontRight.setPower(1);
            }
            int time3 = 600;
            timer = new ElapsedTime();
            while (timer.milliseconds() <= time3) {
                backLeft.setPower(1);
                backRight.setPower(1);
                frontLeft.setPower(1);
                frontRight.setPower(1);
            }

        }
    }
}