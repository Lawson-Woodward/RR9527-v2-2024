package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.commands.State.REST;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.commands.State;
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
    public boolean doClose = true, doClamp = true;

    public int turnDegrees = 120;
    public int forwardDistance1 = 46;
    public int forwardDistance2 = 87;

    Robot bot;
    Arm arm;
    Intake intake;
    Wrist wrist;
    Claw claw;
    PropDetection propDetection;
    OpenCvCamera camera;
    String webcamName;


    AprilTagDetectionPipeline detection;
    public static int forward_milliseconds = 2900, turn_milliseconds = 1200, ms1=2500, ms2=1300, ms3=300;
    public static double power = 0.2;

    //placing pixel on marker
    public static double initialLeft_x = -2;
    public static double initialLeft_y = 6;
    public static double initialLeft_angle = 120;

    public static double initialRight_x = 3;
    public static double initialRight_y = 6;
    public static double initialRight_angle = 65;
    public static double initialMiddle = 12;

    //getting to the board and/or parking

    public static double redFarsideDist = 90;
    public static double redCloseDist = 18;

    public static double blueFarsideDist = -90;
    public static double blueCloseDist = 18;

    ElapsedTime timer = new ElapsedTime();

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Servo pixelHolder;
    private State state;



    public void runOpMode() {

        telemetry.addLine("Status: Initializing");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot bot = new Robot(hardwareMap, telemetry);

        propDetection = new PropDetection();


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


        executeTele(State.START, bot);
        execute(propDetection.getPosition(), drive, bot);








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
    public void execute(PropDetection.TSEPosition position, SampleMecanumDrive drive, Robot bot){  //this is where you want to put the camera action code

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));
        drive.setPoseEstimate(startPose);



        Trajectory left = drive.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(initialLeft_x, initialLeft_y, Math.toRadians(initialLeft_angle)))

                .build();

        Trajectory right = drive.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(initialRight_x, initialRight_y, Math.toRadians(initialRight_angle)))

                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose, false)
                .forward(initialMiddle)

                .build();



        Trajectory blueCloseLeftPark = drive.trajectoryBuilder(left.end(), false)
                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(90))

                .build();

        Trajectory blueCloseRightPark = drive.trajectoryBuilder(right.end(), false)
                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(90))

                .build();

        Trajectory blueCloseMiddlePark = drive.trajectoryBuilder(middle.end(), false)
                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(90))

                .build();

        ElapsedTime robotTimer = new ElapsedTime();





        timer.reset();
        switch(position) {
            case LEFT:
                telemetry.addData("TSE POSITION:", "LEFT");

                drive.followTrajectory(left);
                robotTimer.reset();
                executeTele(State.SHORT_DEPOSIT, bot);
                bot.claw.getLeftClaw().setPosition(leftClawOpen);
                bot.claw.getRightClaw().setPosition(rightClawOpen);

                robotTimer.reset();
                executeTele(State.RETRACT, bot);


                //drive.followTrajectory(blueCloseLeftPark);

                break;


            case RIGHT:
                telemetry.addData("TSE POSITION:", "RIGHT");

                //drive.followTrajectory(right);
                drive.followTrajectory(right);
                robotTimer.reset();
                executeTele(State.SHORT_DEPOSIT, bot);
                bot.claw.getLeftClaw().setPosition(leftClawOpen);
                bot.claw.getRightClaw().setPosition(rightClawOpen);

                robotTimer.reset();
                executeTele(State.RETRACT, bot);

                break;
            case MIDDLE:
                telemetry.addData("TSE POSITION:", "MIDDLE");
                drive.followTrajectory(middle);
                robotTimer.reset();
                executeTele(State.SHORT_DEPOSIT, bot);
                bot.claw.getLeftClaw().setPosition(leftClawOpen);
                bot.claw.getRightClaw().setPosition(rightClawOpen);

                robotTimer.reset();
                executeTele(State.RETRACT, bot);
                break;

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

    public void executeTele(State state, Robot bot) {
        ElapsedTime robotTimer = new ElapsedTime();
        robotTimer.reset();
        //driver.readButtons();
        //operator.readButtons();
        //telemetry.addData("Time: ", robotTimer.seconds());
        //telemetry.addData("B is down: ", operator.isDown(GamepadKeys.Button.B));
        /*
        if(!(lastState==state)) {
            robotTimer.reset();
            lastState = state;
        }

         */
        switch(state) {
            case START:
                break;
            case REST:
                if(robotTimer.seconds()<0.5) {
                    if (doClose) {
                        bot.claw.closeRight();
                        bot.claw.closeLeft();
                    } else {
                        bot.claw.openRight();
                        bot.claw.openLeft();
                    }
                }
                if(robotTimer.seconds()>0.5) {
                    bot.arm.transition();
                    bot.wrist.transition();
                    bot.claw.closeRight();
                    bot.claw.closeLeft();
                    if(robotTimer.seconds()>1.2 && robotTimer.seconds()<1.9) {
                        bot.intake.intakeManualIn();
                    }
                    if(robotTimer.seconds()>=1.9) {
                        bot.intake.holdIntake();
                    }
                }
                break;

            case INTAKE:
                if(robotTimer.seconds()<0.7) {
                    bot.intake.intakeManualOut();
                }
                if(robotTimer.seconds()>=0.7) {
                    bot.intake.holdIntake();
                    bot.arm.intake();
                    bot.wrist.intake();
                    bot.claw.openLeft();
                    bot.claw.openRight();
                    doClose = true;
                }
                if(robotTimer.seconds()>1.2 ) {
                    state = REST;
                }
                break;

            case SHORT_DEPOSIT:
                while(robotTimer.seconds()<1.9) {
                    bot.intake.intakeManualOut();
                }
                while(robotTimer.seconds()>=1.9) {
                    bot.intake.holdIntake();
                    bot.arm.shortDeposit();
                    bot.wrist.shortDeposit();
                    bot.claw.openLeft();
                    bot.claw.openRight();

                    doClose = false;
                }
                if(robotTimer.milliseconds()>700 ) {
                    state = REST;
                }

                break;

            case EXTEND:
                bot.claw.looseLeft();
                bot.claw.looseRight();
                if(robotTimer.seconds()>0.5 && robotTimer.seconds()<1.0) {
                    bot.deposit.readyToClamp();
                }
                if(robotTimer.seconds()>1.0 && robotTimer.seconds()<1.5) {
                    bot.deposit.closeLeftClamp();
                    bot.deposit.closeRightClamp();
                }
                if(robotTimer.seconds()>1.5 ) {
                    bot.slides.slidesManualUp();
                }
                if(robotTimer.seconds()>1.5) {
                    bot.slides.holdSlides();
                }
                break;
            case DEPOSIT:

                break;
            case RETURNING:
                bot.claw.looseLeft();
                bot.claw.looseRight();
                bot.deposit.transitioning();
                if(robotTimer.seconds()>0.5 && robotTimer.seconds()<1.0) {
                    bot.deposit.closeLeftClamp();
                    bot.deposit.closeRightClamp();
                }
                if(robotTimer.seconds()>1.0 ) {
                    bot.slides.slidesManualDown();
                }
                if(robotTimer.seconds()>1.0) {
                    bot.slides.holdSlides();
                }

                break;
            case RETRACT:
                doClamp = false;
                bot.deposit.openRightClamp();
                bot.deposit.openLeftClamp();
                bot.deposit.depositing();

        }
        telemetry.update();
    }


}

