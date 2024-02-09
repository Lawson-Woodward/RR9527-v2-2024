package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTester {

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

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, 5.21, 5.21, 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(spikeLeft_x, spikeLeft_y, Math.toRadians(spikeLeft_angle)))
                                //.setReversed(true)
                                .lineToSplineHeading(startPose)
                                //.setReversed(false)
                                .lineToLinearHeading(new Pose2d(spikeRight_x, spikeRight_y, Math.toRadians(spikeRight_angle)))
                                //.setReversed(true)
                                .lineToSplineHeading(startPose)
                                //.setReversed(false)
                                .lineToLinearHeading(new Pose2d(spikeMiddle_x, spikeMiddle_y, Math.toRadians(spikeMiddle_angle)))
                                //.setReversed(true)
                                .lineToSplineHeading(startPose)
                                .build()

                );

        // Declare out second bot


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myBot)
                .start();
    }
}