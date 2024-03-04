package org.firstinspires.ftc.teamcode.opmode.auto.Auto.NOstrafe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;

//@Autonomous(group = "drive")

public class AutoEasyTestRedFarSide extends LinearOpMode {

    boolean starting=true;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(32.30, -39.38, Math.toRadians(180)));

        TrajectorySequence drive1 = drive.trajectorySequenceBuilder(new Pose2d(69.85, -39.38, Math.toRadians(180.00)))
                .splineTo(new Vector2d(32.30, -39.38), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // put on mark
                    sleep(2000);
                })
                .lineTo(new Vector2d(10.41, -39.38))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(10.41, -57.54))
                .addDisplacementMarker(()->{
                    // take stack
                    sleep(2000);
                })
                .lineTo(new Vector2d(10.41, 43.50))
                .lineTo(new Vector2d(37.33,43.50))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(35.33,43.50))
                .turn(Math.toRadians(90))
                .build();

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        TrajectorySequence drive2 = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                //prop
                .lineTo(new Vector2d(10, 0))
                .waitSeconds(1)
                .lineTo(new Vector2d(45, 0))

                //stack
                .turn(Math.toRadians(90))
                .waitSeconds(2)

//                .lineTo(new Vector2d(50, 2))

                //back board
                .lineTo(new Vector2d(50, -80))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(27, -80))
                .turn(Math.toRadians(-90)) //put
                .waitSeconds(2)


                //second round
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(50, -80))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(45, 0)) //stack
                .waitSeconds(2)

                .lineTo(new Vector2d(50, -80))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(27, -80))//backboard
                .turn(Math.toRadians(-90)) //put
                .waitSeconds(2)
                .lineTo(new Vector2d(27, -85))//backboard



//                //third round
//                .turn(Math.toRadians(-90))
//                .lineTo(new Vector2d(50, -80))
//                .turn(Math.toRadians(90))
//                .lineTo(new Vector2d(45, 0)) //stack
//
//                .lineTo(new Vector2d(50, -85))
////                .turn(Math.toRadians(90))
////                .lineTo(new Vector2d(27, -80))//backboard
////                .turn(Math.toRadians(-90)) //put

                .build();


        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        waitForStart();

        while (opModeIsActive()) {
            if (starting){
                drive.followTrajectorySequence(drive2);
                starting = false;
            }
        }
    }
}