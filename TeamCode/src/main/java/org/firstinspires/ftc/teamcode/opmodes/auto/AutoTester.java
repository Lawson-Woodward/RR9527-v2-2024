package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

//
@Autonomous(name = "\uD83D\uDDFF Auto Test \uD83D\uDDFF", group = "Final")
public class AutoTester extends TestAuto {

    private double waitAtStorage = 0.2;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(-33.5, -65, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(-58 , -16, toRadians(-90));
    public static Pose2d PARK_MIDDLE = new Pose2d(-32, -16, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(-5, -16, toRadians(-90));

    public void build(){

    }

    @Override
    public void setCameraPosition(){
        webcamName = "Webcam";
    }
}