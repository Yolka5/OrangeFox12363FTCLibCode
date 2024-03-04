package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedBackStrafe {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 60, Math.toRadians(180), Math.toRadians(180), 14.97)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
//                                        .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
//                                        .strafeLeft(10)
                                        .UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
//                    System.out.println("Put BkB");

                                        })
                                        .waitSeconds(0.1)
//                                        .lineToLinearHeading(new Pose2d(20, -35, Math.toRadians(180)))



//                                        .turn(Math.toRadians(-90))
//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                        .setReversed(true)
//                                        .splineToLinearHeading(new Pose2d(46, -11.5, Math.toRadians(180)), Math.toRadians(180))
//                                        .setReversed(false)
//                                        .waitSeconds(1)
                                        //
//                                        .lineToConstantHeading(new Vector2d(30, -11.5))
//                                        .splineToLinearHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))

                                        .lineToLinearHeading(new Pose2d(34, -30, Math.toRadians(180)))
                                        .addDisplacementMarker(()->{
                                            System.out.println("put on prop");
                                        })
                                        .waitSeconds(1)

                                        .lineToLinearHeading(new Pose2d(45, -30, Math.toRadians(180)))
                                        .addDisplacementMarker(()->{
//                                            System.out.println("put on bkb");
                                        })
                                        .waitSeconds(1)

                                        // from now take from stacks
                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(180))// to tack
                                        .splineToLinearHeading(new Pose2d(-46, -11.5, Math.toRadians(180)), Math.toRadians(180))// to stack                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, -11.5))//back
                                        .splineToLinearHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))//back
                                        .waitSeconds(1)


                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-46, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, -11.5))
                                        .splineToLinearHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(1)

                                        //maybe another one
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

