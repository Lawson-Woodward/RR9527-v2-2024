package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.*;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.openftc.easyopencv.*;

@Config
public abstract class BlueRight extends LinearOpMode {
    Robot bot;
    SampleMecanumDrive drive;
    //other parts of the bot

    //trajectory sequences

    @Override
    public void runOpMode() {
        telemetry.addLine("Status: Initializing");
        telemetry.update();

        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);





    }
}
