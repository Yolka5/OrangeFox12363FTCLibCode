package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueFarStrafe {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 60, Math.toRadians(180), Math.toRadians(180), 14.97)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(270)))
                                        //                                        .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
//                                        .strafeLeft(10)
                                        .UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
//                    System.out.println("Put BkB");

                                        })
                                        .waitSeconds(0.1)

                                        .lineToLinearHeading(new Pose2d(-39, 40, Math.toRadians(270)))
                                        .addDisplacementMarker(()->{
//                    System.out.println("put on line");
                                        })
                                        .back(10)
//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .turn(Math.toRadians(-60))
                                        .forward(15)
                                        .turn(Math.toRadians(-30))
//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, 11.5))
                                        .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))// go to bkboard
                                        .waitSeconds(1)


                                        .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-50, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, 11.5))
                                        .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(1)


                                        .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-50, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, 11.5))
                                        .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(1)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}