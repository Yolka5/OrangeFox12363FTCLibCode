package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueBack {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(46, 120, Math.toRadians(180), Math.toRadians(180), 14.97)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(15, 60, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(13, 45, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(6, 34.5, Math.toRadians(90)))
//                                        .splineToLinearHeading(new Pose2d(0, 31, Math.toRadians(180)), Math.toRadians(180))
                                        .lineToLinearHeading(new Pose2d(45, 43, Math.toRadians(180)))


//                                        .turn(Math.toRadians(-60))
//                                        .strafeLeft(10)
                                        .UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
                                            System.out.println("Put BkB");
                                        })
                                        .waitSeconds(0.1)
//                                        .turn(Math.toRadians(90))
                                //                                        .splineTo(new Vector2d(20, 11.5), Math.toRadians(180))
////                                        .turn(Math.toRadians(-180))
//                                        .setReversed(true)
////                                        .lineToConstantHeading(new Vector2d(20, 11.5))
//                                        .splineTo(new Vector2d(38, 35), Math.toRadians(0))
//                                        .setReversed(false)

//                                        .splineToLinearHeading(new Pose2d(23, 30, Math.toRadians(180)), Math.toRadians(180))
//                                        .addDisplacementMarker(()->{
//                                            System.out.println("put on line");
//                                        })
                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
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


                                        .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-50, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                        .waitSeconds(1)
                                        //
                                        .lineToConstantHeading(new Vector2d(30, 11.5))
                                        .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(1)




//                .setReversed(false)
//                                        .splineTo(new Vector2d(20, 11.5), Math.toRadians(180))
//                                        .splineTo(new Vector2d(-50, 11.5), Math.toRadians(180))
//                                        .waitSeconds(1)
//                                        .lineToConstantHeading(new Vector2d(20, 11.5))
//                                        .splineTo(new Vector2d(38, 35), Math.toRadians(0))
//                                        .waitSeconds(1)
//
//                                        .splineTo(new Vector2d(-32, 45), Math.toRadians(50))
//                                        .turn(Math.toRadians(50))
//                                        .splineTo(new Vector2d(-50, 11.5), Math.toRadians(180))
//                                        .lineToConstantHeading(new Vector2d(20, 11.5))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(38, 41), Math.toRadians(0))
//                                        .setReversed(false)


//
//                                        .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
//                                        .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
//                                        .waitSeconds(1)
//                                        .lineToConstantHeading(new Vector2d(20, -11.5))
//                                        .splineTo(new Vector2d(38, -35), Math.toRadians(0))
//                                        .waitSeconds(1)
////                .setReversed(false)
//                                        .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
//                                        .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
//                                        .waitSeconds(1)
//                                        .lineToConstantHeading(new Vector2d(20, -11.5))
//                                        .splineTo(new Vector2d(38, -35), Math.toRadians(0))
//                                        .waitSeconds(1)
//
//                                        .lineToLinearHeading(new Pose2d(38, -30, Math.toRadians(180)))
//                                        .lineToConstantHeading(new Vector2d(20, -11.5))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(38, -30), Math.toRadians(0))
//                                        .setReversed(false)
//
//                                        .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
//                                        .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
//                                        .waitSeconds(1)
//                                        .lineToConstantHeading(new Vector2d(20, -11.5))
//                                        .splineTo(new Vector2d(38, -35), Math.toRadians(0))
//                                        .waitSeconds(1)
//                                        .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
//                                        .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
//                                        .waitSeconds(1)
//                                        .lineToConstantHeading(new Vector2d(20, -11.5))
//                                        .splineTo(new Vector2d(38, -35), Math.toRadians(0))
//                                        .waitSeconds(1)
//
//                                        .lineToConstantHeading(new Vector2d(45, 35))

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

