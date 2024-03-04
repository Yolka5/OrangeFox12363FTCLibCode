package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;

public class DriveTeleop {
    SampleMecanumDrive drive;

    DriveTeleop(SampleMecanumDrive drive){
        this.drive = drive;
    }
    void teleopDrive(ControllerDetect gameP) {
        if (gameP.rightBumper()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gameP.left_stick_Y() * 0.25,
                            -gameP.left_stick_X() * 0.25,
                            -gameP.right_stick_X() * 0.25
                    )
            );
        }else {
            drive.update();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gameP.left_stick_Y(),
                            -gameP.left_stick_X(),
                            -gameP.right_stick_X()
                    )
            );
        }

            drive.update();
    }

//    void teleopDrive(ControllerDetect gameP) {
//        // Get the robot's current pose on the field
//        Pose2d poseEstimate = drive.getPoseEstimate();
//
//        // Use the inverse of the robot's heading to make the drive field relative
//        double heading = poseEstimate.getHeading();
//        double cosHeading = Math.cos(heading);
//        double sinHeading = Math.sin(heading);
//
//        // Transform the joystick inputs to field relative
//        double forward = -gameP.left_stick_y * cosHeading - gameP.left_stick_x * sinHeading;
//        double strafe = -gameP.left_stick_y * sinHeading + gameP.left_stick_x * cosHeading;
//        double turn = -gameP.right_stick_x;
//
//        // Set the weighted drive power with field relative inputs
//        drive.setWeightedDrivePower(new Pose2d(forward, strafe, turn));
//
//        // Update the drive
//        drive.update();
//    }
}
