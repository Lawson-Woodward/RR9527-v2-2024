package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.math.*;

import org.firstinspires.ftc.teamcode.commands.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.commands.State;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.PropDetection;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


@Config
public abstract class RedLeft extends LinearOpMode {

    public double armTransition = 0.52, armShortDeposit = 0.68, armIntaking = 0.99;
    public double wristShortDeposit = 0.53, wristIntaking = 0.64, wristTransition = 0.73;
    public double rightClawClose = 0.51, rightClawOpen = 0.58;
    public double leftClawClose = 0.62, leftClawOpen = 0.54;
    public double planeHoldPos = 0.0, planeReleasePos = 1.0;
    public boolean doClose = false;

    public int turnDegrees = 120;
    public int forwardDistance1 = 46;
    public int forwardDistance2 = 87;

    Robot bot;
    SampleMecanumDrive drive;
    Arm arm;
    Intake intake;
    Wrist wrist;
    Claw claw;
    PropDetection propDetection;
    OpenCvCamera camera;
    String webcamName;

    AprilTagDetectionPipeline detection;
    public static int forward_milliseconds = 2900, turn_milliseconds = 1200, ms1 = 2500, ms2 = 1300, ms3 = 300;

    //Close red starting pos
    public static double startX = -36, startY = -64.5, startAngle = 90;

    public static double spikeLeft_x = -39;
    public static double spikeLeft_y = -57.5;
    public static double spikeLeft_angle = 107; //change??

    public static double spikeRight_x = -33;
    public static double spikeRight_y = -57.5;
    public static double spikeRight_angle = 67;

    public static double spikeMiddle_x = -36;
    public static double spikeMiddle_y = -59.5;
    public static double spikeMiddle_angle = 90;

    static Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAngle));



    public static double middleToBoardSpline1_x = -36;
    public static double middleToBoardSpline1_y = -40;
    public static double middleToBoardSpline1_angle = 0;

    public static double middleToBoardSpline2_x = 41;   //maybe??
    public static double middleToBoardSpline2_y = -42;
    public static double middleToBoardSpline2_angle = -4;





    //getting to the board and/or parking


    ElapsedTime timer = new ElapsedTime();

    //private State state;



    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Status: Initializing");
        telemetry.update();

        Robot bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        propDetection = new PropDetection();


        build();

        setCameraPosition();
        initCam();

        while (!isStarted()) {
            telemetry.addData("STATUS:", "INITIALIZED");
            telemetry.addData("POSITION: ", propDetection.getPosition());
            telemetry.update();
        }
        waitForStart();


        //executeTele(START, bot);
        execute(propDetection.getPosition());


        /*
        okay so, for the closer position(right red, left blue)
        we need to push the pixel to the correct position, then pull back to the wall and park

        for the further position(red left, blue right)
        push pixel to the correct position
        pull back
        circle around the taped area(to the back wall)
        park at the little triangle area of the back drop
         */


    }
    public abstract void setCameraPosition();

    public abstract void build();

    public void execute(PropDetection.TSEPosition position) {  //this is where you want to put the camera action code

        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAngle));
        drive.setPoseEstimate(startPose);

        Trajectory spikeLeft = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(spikeLeft_x, spikeLeft_y, Math.toRadians(spikeLeft_angle)))
                .build();

        Trajectory spikeRight = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(spikeRight_x, spikeRight_y, Math.toRadians(spikeRight_angle)))
                .build();

        Trajectory spikeMiddle = drive.trajectoryBuilder(startPose, false)
                .forward(14)
                .build();

        Trajectory spikeReset = drive.trajectoryBuilder(spikeLeft.end(), false)
                .lineToSplineHeading(startPose)
                .build();

        Trajectory middleToBoard1 = drive.trajectoryBuilder(spikeReset.end(), false)
                .lineToSplineHeading(new Pose2d(middleToBoardSpline1_x, middleToBoardSpline1_y, Math.toRadians(middleToBoardSpline1_angle)))
                .build();
        Trajectory middleToBoard2 = drive.trajectoryBuilder(middleToBoard1.end(), false)
                .lineToSplineHeading(new Pose2d(middleToBoardSpline2_x, middleToBoardSpline2_y, Math.toRadians(middleToBoardSpline2_angle)))
                .build();



        timer.reset();

        switch (position) {
            case LEFT:
                telemetry.addData("TSE POSITION:", "LEFT");
                while(timer.seconds()<2) {
                    bot.setState(State.PURPLE);
                    bot.executeAuto();
                }
                drive.followTrajectory(spikeLeft);
                timer.reset();
                while (timer.seconds() < 1) {
                    bot.setState(State.REST);
                    bot.executeAuto();
                }
                drive.followTrajectory(spikeReset);
                break;


            case RIGHT:
                telemetry.addData("TSE POSITION:", "RIGHT");
                while(timer.seconds()<2) {
                    bot.setState(State.PURPLE);
                    bot.executeAuto();
                }
                drive.followTrajectory(spikeRight);
                timer.reset();
                while (timer.seconds() < 1) {
                    bot.setState(State.REST);
                    bot.executeAuto();
                }
                drive.followTrajectory(spikeReset);
                break;

            case MIDDLE:
                telemetry.addData("TSE POSITION:", "MIDDLE");
                while(timer.seconds()<2) {
                    bot.setState(State.PURPLE);
                    bot.executeAuto();
                }
                drive.followTrajectory(spikeMiddle);
                timer.reset();
                while (timer.seconds() < 1) {
                    bot.setState(State.REST);
                    bot.executeAuto();
                }
                drive.followTrajectory(spikeReset);
                drive.followTrajectory(middleToBoard1);
                drive.followTrajectory(middleToBoard2);
                timer.reset();
                while (timer.seconds() < 1) {
                    bot.setState(State.SHORT_DEPOSIT);
                    bot.executeAuto();
                }
                while(timer.seconds()<2) {
                    bot.setState(State.REST);
                    bot.executeAuto();
                }
                break;

        }
    }

    private void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetection = new PropDetection();
        camera.setPipeline(propDetection); //prop detection

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
}