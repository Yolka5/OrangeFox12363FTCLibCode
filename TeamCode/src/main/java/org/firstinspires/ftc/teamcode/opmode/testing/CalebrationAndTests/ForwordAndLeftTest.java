package org.firstinspires.ftc.teamcode.opmode.testing.CalebrationAndTests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;

@Autonomous
public class ForwordAndLeftTest extends LinearOpMode {
    boolean starting = true;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        TrajectorySequence seq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                //try y multiplayer = -num
                .forward(10)
                .strafeLeft(10)
                .build();

//        TrajectorySequence try1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .lineTo(new Vector2d(4,0))
////                .forward(5)
////                .lineTo(new Vector2d(10,10))
//
//                .build();
        TrajectorySequence try2 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(40 ,0))
                .lineTo(new Vector2d(40 ,40))
//                .forward(5)
//                .lineTo(new Vector2d(10,10))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            if(starting) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
//                drive.followTrajectorySequence(try1);
                telemetry.addData("pose", drive.getPoseEstimate());
                drive.followTrajectorySequence(try2);
                starting = false;
            }
            telemetry.update();
        }

    }


}
