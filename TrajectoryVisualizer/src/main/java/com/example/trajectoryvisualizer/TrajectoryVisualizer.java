package com.example.trajectoryvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class TrajectoryVisualizer {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
         System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
//                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -60,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-11, -60,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-11, -45,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-11, -60,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-50, -60,Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-60, -35,Math.toRadians(0)))
                                .build()
                )
                .start();
    }
}
