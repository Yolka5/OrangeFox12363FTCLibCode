package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TelevRed extends LinearOpMode {
    DcMotor telescope, ang;
    Servo leftAngClaw, rightAngClaw, rightClaw, leftClaw, plane, climbLeft, climbRight;
    private double leftClawClose = 0.48, rightClawClose = 0.85, leftClawOpen = 0.83, rightClawOpen = 0.53, angTake = 0.07;
    private int angHuman = -6, ang1 = -950, ang2 = -800, ang3 = -800, /*ang4 = -500,*/
            fullOpen = -600, fullClose = 0, count = 0, fullFullOpen = -1500, target;
    boolean climbT = true, isToSlow = false;

    private ColorSensor ColorSright, ColorSleft;

    ControllerDetect gameP1, gameP2;
    boolean leftClawToggle, rightClawToggle, telescopeToggle, humanTrigger, clawsToggle, openToggle, testToggle;

    boolean starting = true, sensoreLeftClosed = false, sensoreRightClosed = false;

    ElapsedTime timerHP;

    boolean flagRight = false;
    boolean flagLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(667.978, 667.978, 320.47, 248.406)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        //exposure!!!
//        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}
//        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
//        exposure.setMode(ExposureControl.Mode.Manual);
//        exposure.setExposure(1, TimeUnit.MILLISECONDS);
//        GainControl gain = visionPortal.getCameraControl(GainControl.class);
//        gain.setGain(255);

        timerHP = new ElapsedTime();
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightAngClaw = hardwareMap.get(Servo.class, "rightAngClaw");
        leftAngClaw = hardwareMap.get(Servo.class, "leftAngClaw");
        plane = hardwareMap.get(Servo.class, "plane");

        telescope = hardwareMap.get(DcMotor.class, "telescope");
        ang = hardwareMap.get(DcMotorEx.class, "ang");

        climbLeft = hardwareMap.get(Servo.class, "climbLeft");
        climbRight = hardwareMap.get(Servo.class, "climbRight");

        ColorSright = hardwareMap.get(ColorSensor.class, "ColorSright");
        ColorSleft = hardwareMap.get(ColorSensor.class, "ColorSleft");

        ang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telescope.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telescope.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gameP1 = new ControllerDetect(gamepad1);
        gameP2 = new ControllerDetect(gamepad2);
//        DriveTeleop teleopDrive = new DriveTeleop(drive);
        leftClaw.setPosition(leftClawClose);
        rightClaw.setPosition(rightClawClose);
//        AprilTagRedTeleop aprilTeleop = new AprilTagRedTeleop(drive, tagProcessor, telemetry);
        drive.setExternalHeading(180);
        waitForStart();
        while (opModeIsActive()) {
            if (starting) {
                for (int i = 0; i < 30; i++) {
                    ang.setPower(0.5);
                    ang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (i == 25) {
                        ang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                }
                ang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                starting = false;
            }
            gameP1.update();
            gameP2.update();
//            telemetry.addData("ang", ang.getCurrentPosition());
//            telemetry.addData("telescope", telescope.getCurrentPosition());
//            teleopDrive.teleopDrive(gameP1);

//            ang.setTargetPosition((int) (gameP2.right_stick_Y()*0.01 + ang.getCurrentPosition()));
//            ang.setPower(1);
//            aprilTeleop.aprilTag(drive, tagProcessor, gameP1, telemetry);

//            if (gamepad1.left_bumper){
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                gamepad1.left_stick_y*0.2,
//                                gamepad1.left_stick_x*0.2,
//                                gamepad1.right_stick_x*0.2
//                        )
//                );
//            }else {
//            telescope.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            ang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (isToSlow && ang.getCurrentPosition() <= -470){
                ang.setPower(((Math.cos(ang.getCurrentPosition()*0.156521738)) * 0.2));
                telemetry.addData("power", (1-(Math.cos(ang.getCurrentPosition()*0.156521738)) * 0.1));
                isToSlow = false;
            }
            telemetry.addData("", telescope.getCurrentPosition());
            if (gameP1.dpadUpOnce()) {
                drive.setExternalHeading(270);
            }
            double heading = drive.getRawExternalHeading();
//            telemetry.addData("heading", heading);
            telemetry.addLine(String.valueOf(heading));
            double cosHeading = Math.cos(-heading);
            double sinHeading = Math.sin(-heading);

//            double forward = (gamepad1.left_stick_x) * cosHeading - -(gamepad1.left_stick_y) * sinHeading;
//            double strafe = (gamepad1.left_stick_x)* sinHeading + -(gamepad1.left_stick_y) * cosHeading;
//            double turn = gamepad1.right_stick_x;
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (gameP1.rightBumper()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -forward * 0.1,
                                -strafe * 0.1,
                                -turn * 0.1
                        )
                );
            } else if (gameP1.leftBumper()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -forward * 0.2,
                                -strafe * 0.2,
                                -turn * 0.2
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -forward * 0.8,
                                -strafe * 0.8,
                                -turn * 0.8
                        )
                );
            }
//            }
            drive.update();

            if ((tagProcessor.getDetections().size() > 0) && (gameP1.X() || gameP1.Y() || gameP1.B() || gameP1.A())) {
                AprilTagDetection tag = selectBestTag(tagProcessor);
                double range = tag.ftcPose.range;
                double bearing = tag.ftcPose.bearing;
                double tagYaw = tag.ftcPose.yaw;
                double yPose = Math.sin(Math.toRadians(bearing)) * range;
                double xPose = Math.cos(Math.toRadians(bearing)) * range;
                double tagY = tag.ftcPose.y;
                double tagX = tag.ftcPose.x;
                double tagFieldY = tag.metadata.fieldPosition.get(1);
                double tagFiledX = tag.metadata.fieldPosition.get(0);
                int tagId = tag.id;


                Pose2d tagPoseField = new Pose2d(tagFiledX, tagFieldY, Math.toRadians(0));
//                telemetry.addData("tag pose", tagPoseField);
//                telemetry.addData("tag ID", tagId);


                Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
//                telemetry.addData("trigo rel pose", robotRelPose);

                double x = tagPoseField.getX() + robotRelPose.getX(); //* Math.cos(tagPoseField.getHeading()) - robotRelPose.getY() * Math.sin(tagPoseField.getHeading());
                double y = tagPoseField.getY() + robotRelPose.getY(); //* Math.sin(tagPoseField.getHeading()) + robotRelPose.getY() * Math.cos(tagPoseField.getHeading());
                double heading1 = tagPoseField.getHeading() - robotRelPose.getHeading();
                heading1 = Math.toRadians(180) + heading1;
                Pose2d absPose = new Pose2d(x, y, heading1);
//                telemetry.addData("trigo ABS pose", absPose);
//                telemetry.addData("pose", drive.getPoseEstimate());
//                telemetry.update();
                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    // left side
                    if (gameP1.X()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(50.3, 41, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    }
                    //middle side
                    else if (gameP1.Y()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(50.3, 35, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    }
                    //right side
                    else if (gameP1.B()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(50.3, 29, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    } else if (gameP1.A()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(58, 35, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    }

                } else if (tag.id == 4 || tag.id == 5 || tag.id == 6) {
                    // left side
                    if (gameP1.X()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(50.3, -29, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    }
                    //middle side
                    else if (gameP1.Y()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(50.3, -35, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    }
                    //right side
                    else if (gameP1.B()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(50.3, -41, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    } else if (gameP1.A()) {
                        drive.setPoseEstimate(absPose);
                        TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                                .lineToLinearHeading(new Pose2d(58, -35, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(goToAprilTag);
                        drive.setExternalHeading(180); // to change value
                    }
                }

                drive.update();
            }

            if (gameP2.right_trigger > 0) {
                telescope.setTargetPosition(fullOpen);
                telescope.setPower(0.9);
                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (telescope.isBusy() && opModeIsActive()) {
                    gameP1.update();
                    forward = gameP1.left_stick_Y();
                    strafe = gameP1.left_stick_X();
                    turn = gameP1.right_stick_X();
                    if (gameP1.rightBumper()) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -forward * 0.1,
                                        -strafe * 0.1,
                                        -turn * 0.1
                                )
                        );
                    } else if (gameP1.leftBumper()) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -forward * 0.2,
                                        -strafe * 0.2,
                                        -turn * 0.2
                                )
                        );
                    } else {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -forward * 0.8,
                                        -strafe * 0.8,
                                        -turn * 0.8
                                )
                        );
                    }
                }
                ang.setTargetPosition(-15);

                ang.setPower(0.3);
//                ang.setMode(DcMotor.RunMode.);

                leftAngClaw.setPosition(0.05);
                rightAngClaw.setPosition(0.05);

                flagLeft = true;
                flagRight = true;

//                openRightClaw();
//                openLeftClaw();
            }
            if (gameP1.dpadRightOnce()) {
                plane.setPosition(1);
            }
            if (gameP2.XOnce()) {
                telescope.setTargetPosition(0);
                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telescope.setPower(0.8);
                ang.setTargetPosition(0);
                ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ang.setPower(0.2);
                leftAngClaw.setPosition(0.55);
                rightAngClaw.setPosition(0.55);
//                takeFromHuman();
//                while (ang.isBusy() || telescope.isBusy()){
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        gamepad1.left_stick_y*0.03,
//                                        gamepad1.left_stick_x*0.03,
//                                        gamepad1.right_stick_x*0.03
//                                )
//                        );
//                }
//                ang.setTargetPosition(0);
//                ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ang.setPower(0.1);
            }
            if (gameP1.dpadDownOnce() && climbT) {
                climbLeft.setPosition(-0.6);
                climbRight.setPosition(1);
                climbT = false;
            } else if (gameP1.dpadDownOnce()) {
                telescope.setTargetPosition(-2000);
                telescope.setPower(0.9);
                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
//            else if (gameP2.XOnce() && !humanTrigger) {
//                returnFromHuman();
//            }
            if ((gameP2.leftBumperOnce() && leftClawToggle) || (gameP1.rightTriggerOnce() && leftClawToggle)) {
                openLeftClaw();
            } else if (gameP2.leftBumperOnce() && !leftClawToggle) {
                closeLeftClaw();
            }

            if ((gameP2.rightBumperOnce() && rightClawToggle) || (gameP1.leftTriggerOnce() && rightClawToggle)) {
                openRightClaw();
            } else if (gameP2.rightBumperOnce() && !rightClawToggle) {
                closeRightClaw();
            }


            if (gameP2.A() /*&& testToggle*/) {
                ang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ang.setPower(0.2);
                ang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                flagRight = true;
                flagLeft = true;

                if (gamepad2.back) {
                    ang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
//                angleHuman();
//                leftAngClaw.setPosition(0.4);
//                rightAngClaw.setPosition(0.4);
////
//                closeRightClaw();
            }
            if (gameP2.YOnce() && !leftClawToggle && !rightClawToggle) {
                closeLeftClaw();
                closeRightClaw();
            } else if (gameP2.YOnce() && leftClawToggle && rightClawToggle) {
                openLeftClaw();
                openRightClaw();
            }

            double redCright = ColorSright.red();
            double greenCright = ColorSright.green();
            double blueCright = ColorSright.blue();
            double redCleft = ColorSleft.red();
            double greenCleft = ColorSleft.green();
            double blueCleft = ColorSleft.blue();

            if ((redCright >= 175 || greenCright >= 175 || blueCright >= 175) && flagRight) {
                telemetry.addLine("pixel nice close righthwoihioewhfihriohvorihvirhiw");
                telemetry.update();
                closeLeftClaw();

//                leftClaw.setPosition(leftClawClose);
//                leftClawToggle = true;

                flagRight = false;
//                sensoreLeftClosed = true;
            }
            if ((redCleft >= 175 || greenCleft >= 175 || blueCleft >= 175) && flagLeft) {
                telemetry.addLine("pixel nice close right yjwqfgyiwegvfuoegfur");
                telemetry.update();
                closeRightClaw();
//                rightClaw.setPosition(rightClawClose);
//                rightClawToggle = true;

                flagLeft = false;
//                sensoreRightClosed = true;
            }
            if (sensoreLeftClosed && sensoreRightClosed) {
                telescope.setTargetPosition(0);
                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telescope.setPower(0.8);
                ang.setTargetPosition(0);
                ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ang.setPower(0.2);
                leftAngClaw.setPosition(0.55);
                rightAngClaw.setPosition(0.55);
                sensoreRightClosed = false;
                sensoreLeftClosed = false;
            }
//            if (leftClawToggle && rightClawToggle){
//                clawsToggle = true;
//            } else if (!leftClawToggle && !rightClawToggle) {
//                clawsToggle = false;
//            }
//            telemetry.addData("claws", clawsToggle);
//            telemetry.addData("leftClaw", leftClawToggle);
//            telemetry.addData("rightClaw", rightClawToggle);
            if (gameP2.dpadDownOnce()) {
                angle1();
//                while (opModeIsActive() && ang.isBusy() && telescope.isBusy()){
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    gamepad1.left_stick_x,
//                                    gamepad1.right_stick_x
//                            )
//                    );
//                }
//                leftClaw.setPosition(leftClawOpen);
//                rightClaw.setPosition(rightClawOpen);
//                openToggle = false;
//                leftClawToggle = false;
//                rightClawToggle = false;
            }
            /*else if (gameP2.dpadRight()) {
                angle4();
//                while (opModeIsActive() && ang.isBusy() && telescope.isBusy()){
//                    if (gameP1.rightBumper()){
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        -gamepad1.left_stick_y*0.3,
//                                        gamepad1.left_stick_x*0.3,
//                                        gamepad1.right_stick_x*0.3
//                                )
//                        );
//                    }else {
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        -gamepad1.left_stick_y,
//                                        gamepad1.left_stick_x,
//                                        gamepad1.right_stick_x
//                                )
//                        );
//                    }
//                }
//                leftClaw.setPosition(leftClawOpen);
//                rightClaw.setPosition(rightClawOpen);
                openToggle = false;
                leftClawToggle = false;l
                rightClawToggle = false;
//                openToggle = false;
            }*/
            else if (gameP2.dpadUpOnce()) {
                angle3();
//                while (opModeIsActive() && ang.isBusy() && telescope.isBusy()){
//                    if (gameP1.rightBumper()){
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        -gamepad1.left_stick_y*0.3,
//                                        gamepad1.left_stick_x*0.3,
//                                        gamepad1.right_stick_x*0.3
//                                )
//                        );
//                    }else {
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        -gamepad1.left_stick_y,
//                                        gamepad1.left_stick_x,
//                                        gamepad1.right_stick_x
//                                )
//                        );
//                    }
//                }
//                leftClaw.setPosition(leftClawOpen);
//                rightClaw.setPosition(rightClawOpen);
//                openToggle = false;
//                leftClawToggle = false;
//                rightClawToggle = false;
//                openToggle = false;
            } else if (gameP2.dpadLeftOnce()) {
                angle2();
//                while (opModeIsActive() && ang.isBusy() && telescope.isBusy()){
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    gamepad1.left_stick_x,
//                                    gamepad1.right_stick_x
//                            )
//                    );
//                }
//                leftClaw.setPosition(leftClawOpen);
//                rightClaw.setPosition(rightClawOpen);
//                openToggle = false;
//                leftClawToggle = false;
//                rightClawToggle = false;
//                openToggle = false;
//            }else if (gameP2.dpadLeftOnce() || gameP2.dpadUpOnce() || gameP2.dpadRightOnce() || gameP2.dpadDownOnce()  && !openToggle) {
//                leftAngClaw.setPosition(0.4);
//                rightAngClaw.setPosition(0.4);
//                telescope.setTargetPosition(fullClose);
//                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                telescope.setPower(1);
//                ang.setTargetPosition(angHuman);
//                ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ang.setPower(1);
//                openToggle = true;
            }
            if (gameP2.dpadRightOnce()) {
                angle4();
            }
//            if (gameP2.right_stick_Y() > 0){
//                telescope.setPower(gameP2.right_stick_Y());
//                telescope.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            if (gameP2.left_stick_Y() > 0){
//                ang.setPower(gameP2.left_stick_Y());
//                ang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }




        }
    }

    void takeFromHuman() {
        leftClaw.setPosition(leftClawOpen);
        rightClaw.setPosition(rightClawOpen);
        leftAngClaw.setPosition(angTake);
        rightAngClaw.setPosition(angTake);
        ang.setTargetPosition(angHuman);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ang.setPower(0.4);
        telescope.setTargetPosition(fullOpen);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setPower(0.9);


        leftClawToggle = true;
        rightClawToggle = true;
    }

    void returnFromHuman() {
        leftClaw.setPosition(leftClawClose);
        rightClaw.setPosition(rightClawClose);
        leftAngClaw.setPosition(1);
        rightAngClaw.setPosition(1);
        telescope.setTargetPosition(0);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setPower(0.9);
    }

    void angle1() {
        ang.setTargetPosition(ang1);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setTargetPosition(0);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.5);
        rightAngClaw.setPosition(0.5);
        ang.setPower(0.4);
        telescope.setPower(0.9);
        isToSlow = true;
        target = ang1;
    }

    void angleHuman() {
        telescope.setTargetPosition(300);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setTargetPosition(angHuman);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(angTake);
        rightAngClaw.setPosition(angTake);
        ang.setPower(0.2);
        telescope.setPower(0.8);
        openRightClaw();
        openLeftClaw();

        flagRight = true;
        flagLeft = true;

    }

    void backFromHuman() {
        telescope.setTargetPosition(0);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setTargetPosition(angHuman);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.3);
        rightAngClaw.setPosition(0.3);
        ang.setPower(0.2);
        telescope.setPower(0.9);
    }

    void angle2() {
        ang.setTargetPosition(ang2);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setTargetPosition(0);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.57);
        rightAngClaw.setPosition(0.57);
        ang.setPower(0.4);
        telescope.setPower(0.9);
        isToSlow = true;
        target = ang2;
    }

    void angle3() {
        ang.setTargetPosition(-782);
        telescope.setTargetPosition(-500);
        leftAngClaw.setPosition(0.57);
        rightAngClaw.setPosition(0.57);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setPower(0.4);
        telescope.setPower(0.9);
        isToSlow = true;
        target = -782;
    }

    void angle4() {
        ang.setTargetPosition(-782);
        telescope.setTargetPosition(fullFullOpen);
        leftAngClaw.setPosition(0.57);
        rightAngClaw.setPosition(0.57);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setPower(0.4);
        telescope.setPower(0.9);
        isToSlow = true;
        target = -782;
    }
//    void angle4(){
//        ang.setTargetPosition(ang4);
//        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftAngClaw.setPosition(0.5);
//        rightAngClaw.setPosition(0.5);
//        ang.setPower(1);
//    }

    void openLeftClaw() {
        leftClaw.setPosition(leftClawOpen);
        leftClawToggle = false;
        sensoreLeftClosed = false;
    }

    void openRightClaw() {
        rightClaw.setPosition(rightClawOpen);
        rightClawToggle = false;
        sensoreRightClosed = false;
    }

    void openClaws() {
        openLeftClaw();
        openRightClaw();
    }

    void closeRightClaw() {
        rightClaw.setPosition(rightClawClose);
        rightClawToggle = true;
        sensoreRightClosed = false;
    }

    void closeLeftClaw() {
        leftClaw.setPosition(leftClawClose);
        leftClawToggle = true;
        sensoreLeftClosed = false;
    }

    void closeClaws() {
        closeRightClaw();
        closeLeftClaw();
    }

    AprilTagDetection selectBestTag(AprilTagProcessor tagProcessor) {
        double bearing = 360;
        int detectionNum = 0;
        for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
            if (tagProcessor.getDetections().get(i).ftcPose.bearing > 0) {
                if (tagProcessor.getDetections().get(i).ftcPose.bearing <= bearing) {
                    detectionNum = i;
                    bearing = tagProcessor.getDetections().get(i).ftcPose.bearing;
                }
            } else {
                if (tagProcessor.getDetections().get(i).ftcPose.bearing >= bearing) {
                    detectionNum = i;
                    bearing = tagProcessor.getDetections().get(i).ftcPose.bearing;
                }
            }
        }
        return tagProcessor.getDetections().get(detectionNum);
    }
}