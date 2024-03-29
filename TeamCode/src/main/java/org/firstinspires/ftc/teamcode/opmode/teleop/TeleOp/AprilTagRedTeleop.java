package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagRedTeleop {
    SampleMecanumDrive drive;
    AprilTagProcessor tagProcessor;
    Telemetry telemetry;

    public AprilTagRedTeleop(SampleMecanumDrive drive, AprilTagProcessor tagProcessor, Telemetry telemetry){
        this.tagProcessor = tagProcessor;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    void aprilTagLeft(){
        if (!(tagProcessor.getDetections().size() > 0)) {return;}
        drive.setPoseEstimate(aprilStart());
        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(aprilStart())
                .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)))
                .build();

        drive.followTrajectorySequence(goToAprilTag);
    }
    void aprilTagMid(){
        if (!(tagProcessor.getDetections().size() > 0)) {return;}
        drive.setPoseEstimate(aprilStart());
        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(aprilStart())
                .lineToLinearHeading(new Pose2d(50, -41, Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(goToAprilTag);
    }
    void aprilTagRight(){
        if (!(tagProcessor.getDetections().size() > 0)) {return;}
        drive.setPoseEstimate(aprilStart());
        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(aprilStart())
                .lineToLinearHeading(new Pose2d(50, -46, Math.toRadians(0)))
                .build();

        drive.followTrajectorySequence(goToAprilTag);
    }
    Pose2d aprilStart(){
        telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        double range = tag.ftcPose.range;
        double bearing = tag.ftcPose.bearing;
        double tagYaw = tag.ftcPose.yaw;
        double yPose = Math.sin(Math.toRadians(bearing)) * range;
        double xPose = Math.cos(Math.toRadians(bearing)) * range;

        double tagFieldY = tag.metadata.fieldPosition.get(1);
        double tagFiledX = tag.metadata.fieldPosition.get(0);
        int tagId = tag.id;

        Pose2d tagPoseField = new Pose2d(tagFiledX, tagFieldY, Math.toRadians(0));
        telemetry.addData("tag pose", tagPoseField);
        telemetry.addData("tag ID", tagId);

        Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
        telemetry.addData("trigo rel pose", robotRelPose);

        double x = tagPoseField.getX() + robotRelPose.getX();
        double y = tagPoseField.getY() + robotRelPose.getY();
        double heading = tagPoseField.getHeading() - robotRelPose.getHeading();
        Pose2d absPose = new Pose2d(x, y, heading);

        telemetry.addData("trigo ABS pose", absPose);
        telemetry.addData("pose", drive.getPoseEstimate());
        telemetry.update();
        return absPose;
    }
    void aprilTag(ControllerDetect gameP, Telemetry telemetry) {
        if (tagProcessor.getDetections().size() > 0) {
            telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            double range = tag.ftcPose.range;
            double bearing = tag.ftcPose.bearing;
            double tagYaw = tag.ftcPose.yaw;
            double yPose = Math.sin(Math.toRadians(bearing)) * range;
            double xPose = Math.cos(Math.toRadians(bearing)) * range;

            double tagFieldY = tag.metadata.fieldPosition.get(1);
            double tagFiledX = tag.metadata.fieldPosition.get(0);
            int tagId = tag.id;

            Pose2d tagPoseField = new Pose2d(tagFiledX, tagFieldY, Math.toRadians(0));
            telemetry.addData("tag pose", tagPoseField);
            telemetry.addData("tag ID", tagId);

            Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
            telemetry.addData("trigo rel pose", robotRelPose);

            double x = tagPoseField.getX() + robotRelPose.getX();
            double y = tagPoseField.getY() + robotRelPose.getY();
            double heading = tagPoseField.getHeading() - robotRelPose.getHeading();
            Pose2d absPose = new Pose2d(x, y, heading);

            telemetry.addData("trigo ABS pose", absPose);
            telemetry.addData("pose", drive.getPoseEstimate());
            telemetry.update();


            // left side
            if (gameP.X()) {
                drive.setPoseEstimate(absPose);
                TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                        .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)))
                        .build();

                drive.followTrajectorySequence(goToAprilTag);
            }

            //middle
            else if (gameP.Y()) {
                drive.setPoseEstimate(absPose);
                TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                        .lineToLinearHeading(new Pose2d(50, -41, Math.toRadians(0)))
                        .build();
                drive.followTrajectorySequence(goToAprilTag);
            }

            //right side
            else if (gameP.B()) {
                drive.setPoseEstimate(absPose);
                TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                        .lineToLinearHeading(new Pose2d(50, -46, Math.toRadians(0)))
                        .build();

                drive.followTrajectorySequence(goToAprilTag);
            }
        } else {
            telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
        }
    }

//    void aprilTag(ControllerDetect gameP, Telemetry telemetry) {
//        if (tagProcessor.getDetections().size() > 0) {
//            telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
//            AprilTagDetection tag = tagProcessor.getDetections().get(0);
//            double range = tag.ftcPose.range;
//            double bearing = tag.ftcPose.bearing;
//            double tagYaw = tag.ftcPose.yaw;
//            double yPose = Math.sin(Math.toRadians(bearing)) * range;
//            double xPose = Math.cos(Math.toRadians(bearing)) * range;
//
//            double tagFieldY = tag.metadata.fieldPosition.get(1);
//            double tagFiledX = tag.metadata.fieldPosition.get(0);
//            int tagId = tag.id;
//
//            Pose2d tagPoseField = new Pose2d(tagFiledX, tagFieldY, Math.toRadians(0));
//            telemetry.addData("tag pose", tagPoseField);
//            telemetry.addData("tag ID", tagId);
//
//            Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
//            telemetry.addData("trigo rel pose", robotRelPose);
//
//            double x = tagPoseField.getX() + robotRelPose.getX();
//            double y = tagPoseField.getY() + robotRelPose.getY();
//            double heading = tagPoseField.getHeading() - robotRelPose.getHeading();
//            Pose2d absPose = new Pose2d(x, y, heading);
//
//            telemetry.addData("trigo ABS pose", absPose);
//            telemetry.addData("pose", drive.getPoseEstimate());
//            telemetry.update();
//
//
//            // left side
//            if (gameP.X()) {
//                drive.setPoseEstimate(absPose);
//                TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
//                        .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)))
//                        .build();
//
//                drive.followTrajectorySequence(goToAprilTag);
//            }
//
//            //middle
//            else if (gameP.Y()) {
//                drive.setPoseEstimate(absPose);
//                TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
//                        .lineToLinearHeading(new Pose2d(50, -41, Math.toRadians(0)))
//                        .build();
//                drive.followTrajectorySequence(goToAprilTag);
//            }
//
//            //right side
//            else if (gameP.B()) {
//                drive.setPoseEstimate(absPose);
//                TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
//                        .lineToLinearHeading(new Pose2d(50, -46, Math.toRadians(0)))
//                        .build();
//
//                drive.followTrajectorySequence(goToAprilTag);
//            }
//        } else {
//            telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
//        }
//    }
}