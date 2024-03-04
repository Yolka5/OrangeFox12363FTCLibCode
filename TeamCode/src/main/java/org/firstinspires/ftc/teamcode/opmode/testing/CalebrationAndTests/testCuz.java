package org.firstinspires.ftc.teamcode.opmode.testing.CalebrationAndTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;

@Autonomous
public class testCuz extends LinearOpMode{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        boolean starting = true;
        TrajectorySequence leftPropStart = drive.trajectorySequenceBuilder(new Pose2d(15, -60, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(15, -45, Math.toRadians(90)))
                .forward(5.0)
                .build(); // to prop line
        waitForStart();
        while (opModeIsActive()){
            if (starting) {
                drive.followTrajectorySequence(leftPropStart);
                starting = false;
            }
        }
    }
}
