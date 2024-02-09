package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.commands.State;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.PropDetection;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


@Config
public abstract class BlueLeft extends LinearOpMode {

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
    State state;

    AprilTagDetectionPipeline detection;
    public static int forward_milliseconds = 2900, turn_milliseconds = 1200, ms1 = 2500, ms2 = 1300, ms3 = 300;

    //Close red starting pos
    public static double startX = 12, startY = -64.5, startAngle = 90;

    public static double spikeLeft_x = 9;
    public static double spikeLeft_y = -57.5;
    public static double spikeLeft_angle = 110;

    public static double spikeRight_x = 15;
    public static double spikeRight_y = -57.5;
    public static double spikeRight_angle = 70;

    public static double spikeMiddle_x = 12;
    public static double spikeMiddle_y = -59.5;
    public static double spikeMiddle_angle = 90;

    static Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAngle));


    //getting to the board and/or parking

    public static double redFarsideDist = 90;
    public static double redCloseDist = 18;

    public static double blueFarsideDist = -90;
    public static double blueCloseDist = 18;

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

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAngle));

        Trajectory spikeLeft = drive.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(spikeLeft_x, spikeLeft_y, Math.toRadians(spikeLeft_angle)))
                .build();

        Trajectory spikeRight = drive.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(spikeRight_x, spikeRight_y, Math.toRadians(spikeRight_angle)))
                .build();

        Trajectory spikeMiddle = drive.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(spikeMiddle_x, spikeMiddle_y, Math.toRadians(spikeMiddle_angle)))
                .build();

        Trajectory SpikeReset = drive.trajectoryBuilder(spikeLeft.end(), false)
                .lineToSplineHeading(startPose)
                .build();

        Trajectory farBlueToBoard = drive.trajectoryBuilder(new Pose2d(), false)

                .strafeTo(new Vector2d(0, 20))
                .build();

        drive.setPoseEstimate(startPose);
        timer.reset();

        switch (position) {
            case LEFT:

                telemetry.addData("TSE POSITION:", "LEFT");

                drive.followTrajectory(spikeLeft);
                timer.reset();
                while (timer.seconds() < 4) {
                    if (timer.seconds() < 2.5) {
                        bot.setState(State.INTAKE);
                        bot.executeTele();
                    } else {
                        bot.setState(State.REST);
                        bot.executeTele();
                    }
                }
                drive.followTrajectory(SpikeReset);


                break;


            case RIGHT:
                telemetry.addData("TSE POSITION:", "RIGHT");
                break;

            case MIDDLE:
                telemetry.addData("TSE POSITION:", "MIDDLE");
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