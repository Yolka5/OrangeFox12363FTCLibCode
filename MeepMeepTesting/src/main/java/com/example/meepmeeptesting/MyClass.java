package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 60, Math.toRadians(180), Math.toRadians(180), 14.97)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-39, -60, Math.toRadians(270)))
//                                        .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
//                                        .strafeLeft(10)
                                        .UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
//                                            ang.setTargetPosition(0);
//                                            ang.setPower(0.3);
//                                            leftAngClaw.setPosition(angTake);
//                                            rightAngClaw.setPosition(angTake);
//                    rightClaw.setPosition(leftClawOpen);
                                        })
                                        .waitSeconds(0.1)

                                        .lineToLinearHeading(new Pose2d(-38, -35, Math.toRadians(180)))
                                        .addDisplacementMarker(()->{
//                    System.out.println("put on line");
//                                            leftClaw.setPosition(leftClawOpen);
                                        })
                                        .turn(Math.toRadians(-90))

//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-47, -11.5, Math.toRadians(180)), Math.toRadians(180))
//                                        .setReversed(false)
                                        .addDisplacementMarker(()->{
//                                            stack1();
                                        })
                                        .waitSeconds(1)
                                        .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->{

                                        })
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}