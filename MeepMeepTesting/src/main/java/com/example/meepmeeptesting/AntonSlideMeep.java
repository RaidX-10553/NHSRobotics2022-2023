package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AntonSlideMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(13, 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 64, Math.toRadians(270)))
                                .addTemporalMarker(0, () -> {
                                    //claw close
                                })
                                .strafeLeft(23.5)
                                .lineTo(new Vector2d(-11.75, 35.25))
                                .turn(Math.toRadians(-45))
                                .addDisplacementMarker(() -> {
                                    //Arm raises
                                })
                                .waitSeconds(3)
                                //change 1 to appropriate distance based on tuning
                                .forward(1)
                                .addDisplacementMarker(() -> {
                                    //Claw Opens
                                })
                                .back(1)
                                .turn(Math.toRadians(45))
                                .forward(10)
                                .splineToSplineHeading(new Pose2d(-35.25, 11.75, Math.toRadians(180)), Math.toRadians(180))
                                .turn(Math.toRadians(-90))
                                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                                .strafeLeft(23.5)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();
    }
}