


//THIS AUTO MODE DOES NOT DO ANYTHING TO MOVE THE BOT.
//PLEASE DO NOT AMEND THIS CODE. ONLY USE IT TO DEMONSTRATE CAMERA VISION FUNCTIONALITY
//Initialize this auto. From here you can either:
//    * Look at the telemetry to check if the TSE is being recognized correctly
//    * or click the three dots in the top right of the DS and select Camera Stream to see what is happening closer


package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.*;

import static org.firstinspires.ftc.teamcode.commands.PropDetection.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.*;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.openftc.easyopencv.*;

@Config
public abstract class CameraTest extends LinearOpMode {

    Robot bot;
    SampleMecanumDrive drive;
    PropDetection propDetection;
    OpenCvCamera camera;
    String webcamName;

    int alliance;

    TrajectorySequence ScorePreload;
    TrajectorySequence ScoreToStorage1, ScoreToStorage2, ScoreToStorage3, ScoreToStorage4, ScoreToStorage5;
    TrajectorySequence StorageToScore1, StorageToScore2, StorageToScore3, StorageToScore4, StorageToScore5;
    TrajectorySequence WaitAtScore1, WaitAtScore2, WaitAtScore3, WaitAtScore4, WaitAtScore5, WaitAtScore6, WaitAtStorage1, WaitAtStorage2, WaitAtStorage3, WaitAtStorage4, WaitAtStorage5;
    TrajectorySequence ParkLeft, ParkRight, ParkMiddle;
    Pose2d SCORING_POSITION, STORAGE_POSITION;
    int side;

    @Override
    public void runOpMode(){
        telemetry.addLine("Status: Initializing");
        telemetry.update();

        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        propDetection = new PropDetection();

        DriveConstants.MAX_VEL = 45;
        DriveConstants.MAX_ACCEL = 45;
        DriveConstants.MAX_ANG_VEL = Math.toRadians(90);
        DriveConstants.MAX_ANG_ACCEL = Math.toRadians(90);

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
    }

    public abstract void setCameraPosition();
    public abstract void build();
    public void execute(TSEPosition position){
        switch(position) {
            case LEFT:
                telemetry.addData("TSE POSITION:", "LEFT");
                break;
            case RIGHT:
                telemetry.addData("TSE POSITION", "RIGHT");
                break;
            case MIDDLE:
                telemetry.addData("TSE POSITION", "MIDDLE");
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