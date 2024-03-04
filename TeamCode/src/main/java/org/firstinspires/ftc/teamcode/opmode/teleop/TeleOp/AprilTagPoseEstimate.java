package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class AprilTagPoseEstimate extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
//                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) //| old, wrong?
                .setLensIntrinsics(670.778, 670.778, 330.045, 206.751)
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
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("work?", "yes");
        Trajectory toZeroZero = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            visionPortal.resumeLiveView();
            if (gamepad1.a){
                if (tagProcessor.getDetections().size() > 0) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    telemetry.addData("loaded?", "loaded");
                    double range = tag.ftcPose.range;
                    double bearing = tag.ftcPose.bearing;
                    double tagYaw = tag.ftcPose.yaw;

                    double tagY = tag.ftcPose.y;
                    double tagX = tag.ftcPose.x;
                    double tagFieldY = tag.metadata.fieldPosition.get(1);
                    double tagFiledX = tag.metadata.fieldPosition.get(0);
                    double tagFieldOrientation = tag.metadata.fieldOrientation.x;

                    Pose2d tagPoseField = new Pose2d(tag.metadata.fieldPosition.get(0), tag.metadata.fieldPosition.get(1), Math.toRadians(tag.metadata.fieldOrientation.x));

                    Pose2d robotRelPose = new Pose2d(Math.sin(Math.toRadians(bearing)) * range, Math.cos(Math.toRadians(bearing)) * range, Math.toRadians(bearing));
                    telemetry.addData("trigo pose", robotRelPose);

                    // ---- probably won't work ----
                    double x = tagPoseField.getX() + robotRelPose.getX() * Math.cos(tagPoseField.getHeading()) - robotRelPose.getY() * Math.sin(tagPoseField.getHeading());
                    double y = tagPoseField.getY() + robotRelPose.getX() * Math.sin(tagPoseField.getHeading()) + robotRelPose.getY() * Math.cos(tagPoseField.getHeading());
                    double heading = tagPoseField.getHeading() + robotRelPose.getHeading();
                    Pose2d absPose = new Pose2d(x, y, heading);
                    telemetry.addData("trigo ABS pose", absPose);
                    // -----------------------------
//                    drive.setPoseEstimate(absPose);
                    Pose2d RobotPose = new Pose2d(tagFiledX - tagX, tagFieldY - tagY, Math.toRadians(tagFieldOrientation - bearing));
//                drive.setPoseEstimate(RobotPose);
                    telemetry.addData("april tag relative pose XY", tagFiledX + " | " + tagFieldY);
                    telemetry.addData("robot position (Pose2d)", RobotPose);
                    drive.setPoseEstimate(robotRelPose);
                    drive.followTrajectory(toZeroZero);
                }
                telemetry.update();
            }
        }

    }
}
