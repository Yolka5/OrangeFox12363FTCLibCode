package org.firstinspires.ftc.teamcode.opmode.auto.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.RedBlobDetectionPipeline;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous(group = "drive")
public class AutoRedFarSide extends LinearOpMode {
    boolean starting = true;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpelnCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    public int rect = -1;
    RedBlobDetectionPipeline redBlobDetectionPipeline = new RedBlobDetectionPipeline();


    //---------------

    //---------------
    @Override
    public void runOpMode() {


        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Rect_Num:", rect);

        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-35, -70, Math.toRadians(90)));
        //---------------------------
        // --- put on prop line ---
        TrajectorySequence leftPropStart = drive.trajectorySequenceBuilder(new Pose2d(-35, -70, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-38, -45, Math.toRadians(130)))
                .build();
        TrajectorySequence rightPropStart =  drive.trajectorySequenceBuilder(new Pose2d(-35, -70, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-42, -30, Math.toRadians(-180)))
                .build();
        TrajectorySequence midPropStart = drive.trajectorySequenceBuilder(new Pose2d(-35, -70, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(90)))
                .build();
        // --- take from stack ---
        TrajectorySequence leftTakeFromStk = drive.trajectorySequenceBuilder(leftPropStart.end())
                .lineToLinearHeading(new Pose2d(-32, -11, Math.toRadians(-180)))
                .lineToConstantHeading(new Vector2d(-50, -11.5))
                .build();
        TrajectorySequence rightTakeFromStk = drive.trajectorySequenceBuilder(rightPropStart.end())
                .lineToLinearHeading(new Pose2d(-50, -11.5, Math.toRadians(180)))
                .build();
        TrajectorySequence midTakeFromStk = drive.trajectorySequenceBuilder(midPropStart.end())
                .lineToLinearHeading(new Pose2d(-50, -11.5, Math.toRadians(180)))
                .build();
        // ---to back side ---
        TrajectorySequence toBackSide = drive.trajectorySequenceBuilder(new Pose2d(-50, 11.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(38, 11.5))
                .build();
        // --- to right place on back bord ---
        TrajectorySequence leftPropToBkB = drive.trajectorySequenceBuilder(toBackSide.end())
                .lineToLinearHeading(new Pose2d(38, -30, Math.toRadians(180)))
                .build(); //to back bord
        TrajectorySequence rightPropToBkB = drive.trajectorySequenceBuilder(toBackSide.end())
                .lineToLinearHeading(new Pose2d(38, -41, Math.toRadians(180)))
                .build(); //to back bord
        TrajectorySequence midPropToBkB = drive.trajectorySequenceBuilder(toBackSide.end())
                .lineToLinearHeading(new Pose2d(38, -35, Math.toRadians(180)))
                .build(); //to back bord

        // --- take more from stack ---
        TrajectorySequence leftGetMoreFromStack = drive.trajectorySequenceBuilder(leftPropToBkB.end())
                .splineTo(new Vector2d(0, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // take from stack
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(0, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    // put on back bord
                    sleep(2000);
                })
                .splineTo(new Vector2d(0, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // take from stack
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(0, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    // put on back bord
                    sleep(2000);
                })
                .build();
        TrajectorySequence rightGetMoreFromStack = drive.trajectorySequenceBuilder(rightPropToBkB.end())
                .splineTo(new Vector2d(0, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // take from stack
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(0, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    // put on back bord
                    sleep(2000);
                })
                .splineTo(new Vector2d(0, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // take from stack
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(0, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    // put on back bord
                    sleep(2000);
                })
                .build();
        TrajectorySequence midGetMoreFromStack = drive.trajectorySequenceBuilder(midPropToBkB.end())
                .splineTo(new Vector2d(0, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // take from stack
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(0, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    // put on back bord
                    sleep(2000);
                })
                .splineTo(new Vector2d(0, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    // take from stack
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(0, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    // put on back bord
                    sleep(2000);
                })
                .build();
        // PARKING
        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(38, -35, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(90))
                .build();
        //--------------------------

        waitForStart();

        while (opModeIsActive()) {
            rect = redBlobDetectionPipeline.getPropPose();
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Rect_Num:", rect);
            telemetry.update();
            if (starting){
                switch (rect){
                    case 0:
                        telemetry.addData("rect", "0");
                        drive.followTrajectorySequence(leftPropStart); // get to line
                        // move arms (put purple pixel on the right line)
                        drive.followTrajectorySequence(leftTakeFromStk); // going to take from stack
                        // move arms (take from stack)
                        drive.followTrajectorySequence(toBackSide); // going to back side
                        drive.followTrajectorySequence(leftPropToBkB); // get to back bord
                        // move arms (put yellow pixel on the right place on back bord)
                        drive.followTrajectorySequence(leftGetMoreFromStack);// back and forth to get more from stack
                        drive.followTrajectorySequence(park); //parking
                        break;
                    case 1:
                        telemetry.addData("rect", "1");
                        drive.followTrajectorySequence(midPropStart); // get to line
                        // move arms (put purple pixel on the right line)
                        drive.followTrajectorySequence(midTakeFromStk); // going to take from stack
                        // move arms (take from stack)
                        drive.followTrajectorySequence(toBackSide); // going to back side
                        drive.followTrajectorySequence(midPropToBkB); // get to back bord
                        // move arms (put yellow pixel on the right place on back bord)
                        drive.followTrajectorySequence(midGetMoreFromStack);// back and forth to get more from stack
                        drive.followTrajectorySequence(park); //parking
                        break;
                    case 2:
                        drive.followTrajectorySequence(rightPropStart); // get to line
                        // move arms (put purple pixel on the right line)
                        drive.followTrajectorySequence(rightTakeFromStk); // going to take from stack
                        // move arms (take from stack)
                        drive.followTrajectorySequence(toBackSide); // going to back side
                        drive.followTrajectorySequence(rightPropToBkB); // get to back bord
                        // move arms (put yellow pixel on the right place on back bord)
                        drive.followTrajectorySequence(rightGetMoreFromStack);// back and forth to get more from stack
                        drive.followTrajectorySequence(park); //parking
                        break;
                    case -1:
                        telemetry.addData("rect", "didn't find prop line");
                        break;
                }
/*                if (rect == 0){
                    telemetry.addData("rect", "0");
                    drive.followTrajectorySequence(leftPropStart); // get to line
                    // move arms (put purple pixel on the right line)
                    drive.followTrajectorySequence(leftTakeFromStk); // going to take from stack
                    // move arms (take from stack)
                    drive.followTrajectorySequence(toBackSide); // going to back side
                    drive.followTrajectorySequence(leftPropToBkB); // get to back bord
                    // move arms (put yellow pixel on the right place on back bord)
                    drive.followTrajectorySequence(leftGetMoreFromStack);
                    drive.followTrajectorySequence(park); //parking
                }else if (rect == 1){
                    telemetry.addData("rect", "1");
                    drive.followTrajectorySequence(midPropStart); // get to line
                    // move arms (put purple pixel on the right line)
                    drive.followTrajectorySequence(midTakeFromStk); // going to take from stack
                    // move arms (take from stack)
                    drive.followTrajectorySequence(toBackSide); // going to back side
                    drive.followTrajectorySequence(midPropToBkB); // get to back bord
                    // move arms (put yellow pixel on the right place on back bord)
                    drive.followTrajectorySequence(midGetMoreFromStack);
                    drive.followTrajectorySequence(park); //parking
                } else if (rect == 2){
                    drive.followTrajectorySequence(rightPropStart); // get to line
                    // move arms (put purple pixel on the right line)
                    drive.followTrajectorySequence(rightTakeFromStk); // going to take from stack
                    // move arms (take from stack)
                    drive.followTrajectorySequence(toBackSide); // going to back side
                    drive.followTrajectorySequence(rightPropToBkB); // get to back bord
                    // move arms (put yellow pixel on the right place on back bord)
                    drive.followTrajectorySequence(rightGetMoreFromStack);
                    drive.followTrajectorySequence(park); //parking
                }
                else {
                    telemetry.addData("rect", "noRect");
                }*/
                telemetry.update();
                starting = false;
            }


            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }
    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(redBlobDetectionPipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
