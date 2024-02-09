package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.PropDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
public abstract class TestAuto extends LinearOpMode {

    public static int turnDegrees = 120;
    public static int forwardDistance1 = 46;
    public static int forwardDistance2 = 87;
    PropDetection propDetection;
    OpenCvCamera camera;
    String webcamName;

    AprilTagDetectionPipeline detection;
    public static int forward_milliseconds = 2900, turn_milliseconds = 1200, ms1=2500, ms2=1300, ms3=300;
    public static double power = 0.2;

    ElapsedTime timer = new ElapsedTime();

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Servo pixelHolder;




    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;
    public void runOpMode() {

        telemetry.addLine("Status: Initializing");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        propDetection = new PropDetection();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detection = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(detection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        build();

        setCameraPosition();
        initCam();

        while (!isStarted()) {
            telemetry.addData("STATUS:", "INITIALIZED");
            telemetry.addData("POSITION: ", propDetection.getPosition());
            telemetry.update();
        }
        waitForStart();



        execute(propDetection.getPosition());

        /*
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                .forward(forwardDistance1) //replace with code using camera vision(going towards the correct marker)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .forward(forwardDistance2)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .forward(forwardDistance2)
                .build();


        Trajectory left = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-10, 10), Math.toRadians(90))
                .build();
        Trajectory center = drive.trajectoryBuilder(startPose, false)
                .forward(20)
                .build();
        Trajectory right = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(10, 10), Math.toRadians(90))
                .build();
        */
        //the rest



        /*
        okay so, for the closer position(right red, left blue)
        we need to push the pixel to the correct position, then pull back to the wall and park

        for the further position(red left, blue right)
        push pixel to the correct position
        pull back
        circle around the taped area(to the back wall)
        park at the little triangle area of the back drop
         */




        //drive.followTrajectory(traj1);
        //drive.turn(Math.toRadians(turnDegrees));
        //drive.followTrajectory(traj2);

        //execute(propDetection.getPosition());

    }

    public abstract void setCameraPosition();
    public abstract void build();
    public void execute(PropDetection.TSEPosition position){  //this is where you want to put the camera action code
        timer.reset();
        switch(position) {
            case LEFT:
                telemetry.addData("TSE POSITION:", "LEFT");
                timer.reset();

                while(timer.milliseconds() < forward_milliseconds) {
                    rightRear.setPower(-power);
                    rightFront.setPower(-power);
                    leftFront.setPower(-power);
                    leftRear.setPower(-power);
                }
                timer.reset();
                while(timer.milliseconds()<500) {

                }
                timer.reset();
                while(timer.milliseconds()<turn_milliseconds) {
                    rightRear.setPower(power);
                    rightFront.setPower(power);
                    leftFront.setPower(-power);
                    leftRear.setPower(-power);
                }
                pixelHolder.setPosition(0.8);
                timer.reset();

                while(timer.milliseconds() < 1000) {

                }
                timer.reset();
                while(timer.milliseconds() < 800) {
                    rightRear.setPower(power);
                    rightFront.setPower(power);
                    leftFront.setPower(power);
                    leftRear.setPower(power);
                }


                break;
            case RIGHT:
                telemetry.addData("TSE POSITION", "RIGHT");

                timer.reset();

                while(timer.milliseconds() < ms1) {
                    rightRear.setPower(-power);
                    rightFront.setPower(-power);
                    leftFront.setPower(-power);
                    leftRear.setPower(-power);
                }
                timer.reset();
                while(timer.milliseconds()<500) {

                }
                timer.reset();
                while(timer.milliseconds()<ms2) {
                    rightRear.setPower(-power);
                    rightFront.setPower(-power);
                    leftFront.setPower(power);
                    leftRear.setPower(power);
                }
                timer.reset();
                while(timer.milliseconds() < 800) {
                    rightRear.setPower(-power);
                    rightFront.setPower(-power);
                    leftFront.setPower(-power);
                    leftRear.setPower(-power);
                }
                pixelHolder.setPosition(0.8);
                timer.reset();
                while(timer.milliseconds()<1000) {

                }

                while(timer.milliseconds() < 800) {
                    rightRear.setPower(power);
                    rightFront.setPower(power);
                    leftFront.setPower(power);
                    leftRear.setPower(power);
                }
                break;
            case MIDDLE:
                telemetry.addData("TSE POSITION", "MIDDLE");
                timer.reset();

                while(timer.milliseconds() < forward_milliseconds+0) {
                    rightRear.setPower(-power);
                    rightFront.setPower(-power);
                    leftFront.setPower(-power);

                    leftRear.setPower(-power);
                }
                pixelHolder.setPosition(0.8);
                timer.reset();
                while(timer.milliseconds()<1000) {

                }
                while(timer.milliseconds() < 800) {
                    rightRear.setPower(power);
                    rightFront.setPower(power);
                    leftFront.setPower(power);
                    leftRear.setPower(power);
                }


        }
    }

    private void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetection = new PropDetection();
        camera.setPipeline(propDetection); //prop detection

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
}
