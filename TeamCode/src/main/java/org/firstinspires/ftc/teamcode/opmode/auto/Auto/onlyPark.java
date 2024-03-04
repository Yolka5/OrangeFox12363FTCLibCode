package org.firstinspires.ftc.teamcode.opmode.auto.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;

@Autonomous
public class onlyPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
//                .turn(-90)
//                .forward(10)
                .build();


        waitForStart();
        while (opModeIsActive()){
            drive.followTrajectorySequence(park);
        }
    }
}
