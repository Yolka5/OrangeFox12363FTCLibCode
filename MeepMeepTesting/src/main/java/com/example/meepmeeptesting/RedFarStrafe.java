package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFarStrafe {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 60, Math.toRadians(180), Math.toRadians(180), 14.97)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
//                                        .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
//                                        .strafeLeft(10)
                                        .addTemporalMarker(() -> {
//                                            ang.setTargetPosition(0);
//                                            ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                            ang.setPower(0.3);
//                                            leftAngClaw.setPosition(angTake);
//                                            rightAngClaw.setPosition(angTake);
////                    rightClaw.setPosition(leftClawOpen);
//                                            leftClaw.setPosition(leftClawClose);
//                                            rightClaw.setPosition(rightClawClose);
//                                            leftClaw.setPosition(leftClawClose);
//                                            rightClaw.setPosition(rightClawClose);
                                        })
                                        .waitSeconds(0.1)

                                        .lineToLinearHeading(new Pose2d(-34, -35, Math.toRadians(60)))
                                        .addTemporalMarker(()->{
//                    System.out.println("put on line");
//                                            leftClaw.setPosition(leftClawOpen);
//                                            leftClaw.setPosition(leftClawOpen);
                                        })
                                        .waitSeconds(0.2)
                                        .back(3)
                                        .addTemporalMarker(()->{
//                                            fold();
//                    telemetry.addLine("FOLDED");
                                        })
                                        .waitSeconds(0.2)
                                        .turn(Math.toRadians(90))
                                        //                .forward(0.5)

//                                        .turn(Math.toRadians(-60))
//                                        .forward(10)

//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-44, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.01)
                                        .splineToLinearHeading(new Pose2d(-56, -11.5, Math.toRadians(180)), Math.toRadians(180))
//                                        .setReversed(false)
                                        .UNSTABLE_addDisplacementMarkerOffset(-15,()->{
//                                            stack1();
//                                            leftClaw.setPosition(leftClawOpen);
                                        })
                                        .waitSeconds(0.5)
                                        .UNSTABLE_addTemporalMarkerOffset(-0.45, ()->{
//                                            leftClaw.setPosition(leftClawClose);
                                        })
                                        //
                                        .lineToConstantHeading(new Vector2d(30, -11.5))
                                        .splineToLinearHeading(new Pose2d(40, -35, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(1)
                                        .UNSTABLE_addTemporalMarkerOffset(-1.4, ()->{
//                                            putOnBkbYellow();
//                                            rightClaw.setPosition(rightClawOpen);
//                                            leftClaw.setPosition(leftClawOpen);
                                        })
                                        .addTemporalMarker(()->{
//                                            fold();
//                                            leftClaw.setPosition(leftClawClose);
//                                            rightClaw.setPosition(rightClawClose);
                                        })


                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(180))
//                .UNSTABLE_addDisplacementMarkerOffset(-18,() -> {
//                    fold();
//                })
                                        .splineToLinearHeading(new Pose2d(-43, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.05)
                                        .splineToLinearHeading(new Pose2d(-47, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, -11.5))
                                        .splineToLinearHeading(new Pose2d(42, -35, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(1)


                                        .splineToLinearHeading(new Pose2d(30, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-43, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(0.05)
                                        .splineToLinearHeading(new Pose2d(-47, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, -11.5))
                                        .splineToLinearHeading(new Pose2d(42, -35, Math.toRadians(180)), Math.toRadians(0))
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
