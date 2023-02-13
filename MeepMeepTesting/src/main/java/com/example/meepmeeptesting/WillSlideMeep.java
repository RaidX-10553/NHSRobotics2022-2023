package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class WillSlideMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 52.48291908330528, Math.toRadians(180), Math.toRadians(180), 11)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(12, 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 64, Math.toRadians(270)))
                                .strafeLeft(23.5)
                                .forward(50)
                                //less
                                //change 1 to appropriate distance based on tuning
                                .turn(Math.toRadians(-43))
                                .forward(8)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();
    }
}