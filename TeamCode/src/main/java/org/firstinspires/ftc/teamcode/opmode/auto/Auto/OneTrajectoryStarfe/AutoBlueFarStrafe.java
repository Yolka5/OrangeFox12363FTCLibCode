package org.firstinspires.ftc.teamcode.opmode.auto.Auto.OneTrajectoryStarfe;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.BlueBlobDetectionPipeline;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "drive")
public class AutoBlueFarStrafe extends LinearOpMode {
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

    private double leftClawClose = 0.54, rightClawClose = 0.82, leftClawOpen = 0.83, rightClawOpen = 0.53, angTake = 0.07;


    public int rect = -1;

    BlueBlobDetectionPipeline blueBlobDetectionPipeline = new BlueBlobDetectionPipeline();


    //---------------

    //---------------
    DcMotor telescope, ang;
    Servo leftAngClaw, rightAngClaw, rightClaw, leftClaw;
    @Override
    public void runOpMode() {
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightAngClaw = hardwareMap.get(Servo.class, "rightAngClaw");
        leftAngClaw = hardwareMap.get(Servo.class, "leftAngClaw");

        telescope = hardwareMap.get(DcMotor.class, "telescope");
        ang = hardwareMap.get(DcMotor.class, "ang");

        ang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telescope.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initOpenCV();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Rect_Num:", rect);

        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-35, 70, Math.toRadians(-90)));
        //---------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(270)))
                .addTemporalMarker(() -> {
                    ang.setTargetPosition(0);
                    ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ang.setPower(0.3);
                    leftAngClaw.setPosition(angTake);
                    rightAngClaw.setPosition(angTake);
//                    rightClaw.setPosition(leftClawOpen);
                    leftClaw.setPosition(leftClawClose);
                    rightClaw.setPosition(rightClawClose);
                    leftClaw.setPosition(leftClawClose);
                    rightClaw.setPosition(rightClawClose);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading(new Pose2d(-34, 35, Math.toRadians(300)))
                .addTemporalMarker(()->{
//                    System.out.println("put on line");
                    leftClaw.setPosition(leftClawOpen);
                    leftClaw.setPosition(leftClawOpen);
                })

                .back(3)
                .addTemporalMarker(()->{
                    fold();
//                    telemetry.addLine("FOLDED");
                })
                .turn(Math.toRadians(-90))
//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(-15,()->{
                    stack1();
                    leftClaw.setPosition(leftClawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.45, ()->{
                    leftClaw.setPosition(leftClawClose);
                })
                //
                .lineToConstantHeading(new Vector2d(30, 11.5))
                .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))// go to bkboard
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1.4, ()->{
                    putOnBkbYellow();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    rightClaw.setPosition(rightClawOpen);
                    leftClaw.setPosition(leftClawOpen);
                })
                .addTemporalMarker(()->{
                    fold();
                })

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
                .build();
        TrajectorySequence mid = drive.trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(270)))
                .addTemporalMarker(() -> {
                    ang.setTargetPosition(0);
                    ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ang.setPower(0.3);
                    leftAngClaw.setPosition(angTake);
                    rightAngClaw.setPosition(angTake);
//                    rightClaw.setPosition(leftClawOpen);
                    leftClaw.setPosition(leftClawClose);
                    rightClaw.setPosition(rightClawClose);
                    leftClaw.setPosition(leftClawClose);
                    rightClaw.setPosition(rightClawClose);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading(new Pose2d(-34, 35, Math.toRadians(300)))
                .addTemporalMarker(()->{
//                    System.out.println("put on line");
                    leftClaw.setPosition(leftClawOpen);
                    leftClaw.setPosition(leftClawOpen);
                })

                .back(10)
                .addTemporalMarker(()->{
                    fold();
//                    telemetry.addLine("FOLDED");
                })
                .turn(Math.toRadians(-60))
                .forward(15)
                .turn(Math.toRadians(-30))
//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(-15,()->{
                    stack1();
                    leftClaw.setPosition(leftClawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.45, ()->{
                    leftClaw.setPosition(leftClawClose);
                })
                //
                .lineToConstantHeading(new Vector2d(30, 11.5))
                .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))// go to bkboard
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1.4, ()->{
                    putOnBkbYellow();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    rightClaw.setPosition(rightClawOpen);
                    leftClaw.setPosition(leftClawOpen);
                })
                .addTemporalMarker(()->{
                    fold();
                })

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
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(270)))
                .addTemporalMarker(() -> {
                    ang.setTargetPosition(0);
                    ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ang.setPower(0.3);
                    leftAngClaw.setPosition(angTake);
                    rightAngClaw.setPosition(angTake);
//                    rightClaw.setPosition(leftClawOpen);
                    leftClaw.setPosition(leftClawClose);
                    rightClaw.setPosition(rightClawClose);
                    leftClaw.setPosition(leftClawClose);
                    rightClaw.setPosition(rightClawClose);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading(new Pose2d(-34, 35, Math.toRadians(180)))
                .addTemporalMarker(()->{
//                    System.out.println("put on line");
                    leftClaw.setPosition(leftClawOpen);
                    leftClaw.setPosition(leftClawOpen);
                })

                .back(3)
                .addTemporalMarker(()->{
                    fold();
//                    telemetry.addLine("FOLDED");
                })
                .turn(Math.toRadians(60))
//                                        .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(-15,()->{
                    stack1();
                    leftClaw.setPosition(leftClawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.45, ()->{
                    leftClaw.setPosition(leftClawClose);
                })
                //
                .lineToConstantHeading(new Vector2d(30, 11.5))
                .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(180)), Math.toRadians(0))// go to bkboard
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1.4, ()->{
                    putOnBkbYellow();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    rightClaw.setPosition(rightClawOpen);
                    leftClaw.setPosition(leftClawOpen);
                })
                .addTemporalMarker(()->{
                    fold();
                })

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
                .build();
        //--------------------------

        waitForStart();

        while (opModeIsActive()) {
            rect = blueBlobDetectionPipeline.getPropPose();
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Rect_Num:", rect);
            telemetry.update();
            if (starting){
                drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(90)));
                switch (rect){
                    case 0:
                        telemetry.addData("rect", "0");
                        drive.followTrajectorySequence(left); // get to line
                        break;
                    case 1:
                        telemetry.addData("rect", "1");
                        drive.followTrajectorySequence(mid); // get to line
                        break;
                    case 2:
                        drive.followTrajectorySequence(right);
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

        controlHubCam.setPipeline(blueBlobDetectionPipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    void putOnBkbWhite(){
        ang.setTargetPosition(-950);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telescope.setTargetPosition(0);
//        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.4);
        rightAngClaw.setPosition(0.4);
        ang.setPower(0.4);
//        telescope.setPower(0.9);
    }
    void putOnBkbYellow(){
        ang.setTargetPosition(-950);// to change
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telescope.setTargetPosition(0);
//        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.5);
        rightAngClaw.setPosition(0.5);
        ang.setPower(0.4);
//        telescope.setPower(0.9);
    }
    void fold(){
        ang.setTargetPosition(-5);// to change
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telescope.setTargetPosition(0);
//        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.5);
        rightAngClaw.setPosition(0.5);
        ang.setPower(0.42);
//        telescope.setPower(0.9);
    }
    void stack1(){
        ang.setTargetPosition(-22);// to change
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telescope.setTargetPosition(0);
//        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.05);
        rightAngClaw.setPosition(0.05);
        ang.setPower(0.8);
//        telescope.setPower(0.9);
    }


}

