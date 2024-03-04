package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class April2d extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                .setLensIntrinsics(670.778, 670.778, 330.045, 206.751)


                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        telemetry.addData("work?", "yes");
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(15, TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(255);

            visionPortal.resumeLiveView();
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                double range = tag.ftcPose.range;
                double bearing = tag.ftcPose.bearing;
                double tagYaw = tag.ftcPose.yaw;
                double yPose = Math.sin(Math.toRadians(bearing)) * range;
                double xPose = Math.cos(Math.toRadians(bearing)) * range;
                double tagY = tag.ftcPose.y;
                double tagX = tag.ftcPose.x;
                double tagFieldY = tag.metadata.fieldPosition.get(1);
                double tagFiledX = tag.metadata.fieldPosition.get(0);
//                double tagFieldOrientation = tag.metadata.fieldOrientation.x;
                int tagId = tag.id;


                Pose2d tagPoseField = new Pose2d(tagFiledX, tagFieldY, Math.toRadians(0));
                Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
                telemetry.addData("tag pose", tagPoseField);

                double x = tagPoseField.getX() + robotRelPose.getX(); //* Math.cos(tagPoseField.getHeading()) - robotRelPose.getY() * Math.sin(tagPoseField.getHeading());
                double y = tagPoseField.getY() + robotRelPose.getY(); //* Math.sin(tagPoseField.getHeading()) + robotRelPose.getY() * Math.cos(tagPoseField.getHeading());
                double heading = tagPoseField.getHeading() - robotRelPose.getHeading();
                Pose2d absPose = new Pose2d(x, y, heading);
                telemetry.addData("trigo rel pose", robotRelPose);

                telemetry.addData("trigo ABS pose", absPose);

                telemetry.addData("pose" , drive.getPoseEstimate());

                telemetry.update();
                //middle side
                if (gamepad1.y) {
                    drive.setPoseEstimate(absPose);
                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .turn(Math.toRadians(tagYaw))
                            .lineToLinearHeading(new Pose2d(50, 30.418, Math.toRadians(0)))
                            .build();

                    drive.followTrajectorySequence(goToAprilTag);
                    sleep(10);
                }
                // left side
                else if (gamepad1.x) {
                    drive.setPoseEstimate(absPose);
                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .turn(Math.toRadians(tagYaw))
                            .lineToLinearHeading(new Pose2d(50, 36.4, Math.toRadians(0)))
                            .build();

                    drive.followTrajectorySequence(goToAprilTag);
                    sleep(10);
                }
                //right side
                else if (gamepad1.b) {
                    drive.setPoseEstimate(absPose);
                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .turn(Math.toRadians(tagYaw))
                            .lineToLinearHeading(new Pose2d(50, 25, Math.toRadians(0)))
                            .build();

                    drive.followTrajectorySequence(goToAprilTag);
                    sleep(10);
                }
            }
        }
    }
}