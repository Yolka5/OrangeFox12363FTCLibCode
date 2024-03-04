package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ClassTry {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep) .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(90)))
//                                        .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
//                                        .strafeLeft(10)
                                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
//                    ang.setTargetPosition(0);
//                    ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ang.setPower(0.3);
//                                    leftAngClaw.setPosition(angTake);
//                                    rightAngClaw.setPosition(angTake);
//                                    rightClaw.setPosition(leftClawOpen);
                                })
                                .waitSeconds(0.1)

//                                .lineToLinearHeading(new Pose2d(15, 15, Math.toRadians(90)))
//
                                .addDisplacementMarker(()->{
//                    System.out.println("put on line");
//                                    leftClaw.setPosition(leftClawOpen);
                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(30, 40, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
//                                    putOnBkbYellow();
//                                    rightClaw.setPosition(rightClawOpen);
//                                    leftClaw.setPosition(leftClawOpen);
                                })
                                .addTemporalMarker(()->{
//                                    fold();
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
