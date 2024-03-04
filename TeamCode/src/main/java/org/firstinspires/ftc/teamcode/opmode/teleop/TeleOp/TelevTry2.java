package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TelevTry2 extends LinearOpMode {
    DcMotor telescope, ang;
    Servo leftAngClaw, rightAngClaw, rightClaw, leftClaw;
    private double leftClawClose = 0.56, rightClawClose = 0.8, leftClawOpen = 0.83, rightClawOpen = 0.53, angTake = 0.07;
    private int angHuman = -6, ang1 = -950, ang2 = -800, ang3 = -800, /*ang4 = -500,*/ fullOpen = -600, fullClose = 0;

    ControllerDetect gameP1, gameP2;
    boolean leftClawToggle, rightClawToggle, telescopeToggle, humanTrigger, clawsToggle, openToggle, testToggle;
    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                .setLensIntrinsics(670.778, 670.778, 330.045, 206.751)
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

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);

        exposure.setExposure(2, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightAngClaw = hardwareMap.get(Servo.class, "rightAngClaw");
        leftAngClaw = hardwareMap.get(Servo.class, "leftAngClaw");

        telescope = hardwareMap.get(DcMotor.class, "telescope");
        ang = hardwareMap.get(DcMotor.class, "ang");

        ang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telescope.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gameP1 = new ControllerDetect(gamepad1);
        gameP2 = new ControllerDetect(gamepad2);
        DriveTeleop teleopDrive = new DriveTeleop(drive);
        leftClaw.setPosition(leftClawClose);
        AprilTagRedTeleop aprilTeleop = new AprilTagRedTeleop(drive, tagProcessor, telemetry);

        waitForStart();
        while (opModeIsActive()){
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
            if (gamepad1.back) {
                drive.setExternalHeading(90);
            }
            double heading = drive.getRawExternalHeading();
            telemetry.addData("heading", heading);
            telemetry.addLine(String.valueOf(heading));
            double cosHeading = Math.cos(-heading);
            double sinHeading = Math.sin(-heading);

            double forward = gamepad1.left_stick_y * cosHeading - gamepad1.left_stick_x * sinHeading;
            double strafe = gamepad1.left_stick_y * sinHeading + gamepad1.left_stick_x * cosHeading;
            double turn = gamepad1.right_stick_x;

            if (gameP1.rightBumper()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                forward * 0.05,
                                strafe * 0.05,
                                turn * 0.05
                        )
                );
            } else if (gameP1.leftBumper()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                forward * 0.1,
                                strafe * 0.1,
                                turn * 0.1
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                forward * 0.8,
                                strafe * 0.8,
                                turn * 0.8
                        )
                );
            }
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
                telemetry.addData("tag pose", tagPoseField);
                telemetry.addData("tag ID", tagId);


                Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
                telemetry.addData("trigo rel pose", robotRelPose);

                double x = tagPoseField.getX() + robotRelPose.getX(); //* Math.cos(tagPoseField.getHeading()) - robotRelPose.getY() * Math.sin(tagPoseField.getHeading());
                double y = tagPoseField.getY() + robotRelPose.getY(); //* Math.sin(tagPoseField.getHeading()) + robotRelPose.getY() * Math.cos(tagPoseField.getHeading());
                double heading1 = tagPoseField.getHeading() - robotRelPose.getHeading();
                Pose2d absPose = new Pose2d(x, y, heading1);
                telemetry.addData("trigo ABS pose", absPose);

                telemetry.addData("pose", drive.getPoseEstimate());

                telemetry.update();


                // left side
                if (gameP1.X()) {
                    drive.setPoseEstimate(absPose);
                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                            .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)))
                            .build();
                    drive.followTrajectorySequence(goToAprilTag);


//                    sleep(500);
                }

                //middle side
                else if (gameP1.Y()) {
                    drive.setPoseEstimate(absPose);
                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                            .lineToLinearHeading(new Pose2d(50, -41, Math.toRadians(0)))
                            .build();
                    drive.followTrajectorySequence(goToAprilTag);

//                    sleep(500);
                }

                //right side
                else if (gameP1.B()) {
                    drive.setPoseEstimate(absPose);
                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
                            .lineToLinearHeading(new Pose2d(50, -46, Math.toRadians(0)))
                            .build();

                    drive.followTrajectorySequence(goToAprilTag);
//                    sleep(500);
                }
            }
//            }
            drive.update();
            if (gameP2.right_trigger > 0){
                telescope.setTargetPosition(fullOpen);
                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telescope.setPower(0.9);
                ang.setTargetPosition(0);
                ang.setPower(0.1);
                ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftAngClaw.setPosition(0.02);
                rightAngClaw.setPosition(0.02);
//                openRightClaw();
//                openLeftClaw();
            }
            if (gameP2.XOnce()){
                telescope.setTargetPosition(0);
                telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telescope.setPower(0.8);
                ang.setTargetPosition(0);
                ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ang.setPower(0.2);
                leftAngClaw.setPosition(0.6);
                rightAngClaw.setPosition(0.6);
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
//            else if (gameP2.XOnce() && !humanTrigger) {
//                returnFromHuman();
//            }

            if ((gameP2.rightBumperOnce() && leftClawToggle) || (gameP1.rightTriggerOnce() && leftClawToggle)){
                openLeftClaw();
            } else if (gameP2.rightBumperOnce() && !leftClawToggle) {
                closeLeftClaw();
            }

            if ((gameP2.leftBumperOnce() && rightClawToggle) || (gameP1.leftTriggerOnce() && rightClawToggle)){
                openRightClaw();
            }else if (gameP2.leftBumperOnce() && !rightClawToggle) {
                closeRightClaw();
            }
            if (gameP2.AOnce() /*&& testToggle*/){
                angleHuman();
                leftAngClaw.setPosition(0.4);
                rightAngClaw.setPosition(0.4);
//
                closeRightClaw();
            }
            if (gameP2.YOnce() && !leftClawToggle && !rightClawToggle){
                closeLeftClaw();
                closeRightClaw();
            }else if (gameP2.YOnce() && leftClawToggle && rightClawToggle) {
                openLeftClaw();
                openRightClaw();
            }
//            if (leftClawToggle && rightClawToggle){
//                clawsToggle = true;
//            } else if (!leftClawToggle && !rightClawToggle) {
//                clawsToggle = false;
//            }
//            telemetry.addData("claws", clawsToggle);
//            telemetry.addData("leftClaw", leftClawToggle);
//            telemetry.addData("rightClaw", rightClawToggle);
            if (gameP2.dpadDownOnce()){
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
            }else if (gameP2.dpadLeftOnce()) {
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

        }
    }
    void takeFromHuman(){
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
    void returnFromHuman(){
        leftClaw.setPosition(leftClawClose);
        rightClaw.setPosition(rightClawClose);
        sleep(20);
        leftAngClaw.setPosition(1);
        rightAngClaw.setPosition(1);
        telescope.setTargetPosition(0);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setPower(0.9);
    }
    void angle1(){
        ang.setTargetPosition(ang1);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setTargetPosition(0);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.4);
        rightAngClaw.setPosition(0.4);
        ang.setPower(0.4);
        telescope.setPower(0.9);
    }
    void angleHuman(){
        telescope.setTargetPosition(-300);
        ang.setTargetPosition(angHuman);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(angTake);
        rightAngClaw.setPosition(angTake);
        ang.setPower(0.2);
        telescope.setPower(0.8);
        openRightClaw();
        openLeftClaw();
    }
    void backFromHuman(){
        telescope.setTargetPosition(0);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setTargetPosition(angHuman);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.3);
        rightAngClaw.setPosition(0.3);
        ang.setPower(0.2);
        telescope.setPower(0.9);
    }
    void angle2(){
        ang.setTargetPosition(ang2);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telescope.setTargetPosition(0);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftAngClaw.setPosition(0.5);
        rightAngClaw.setPosition(0.5);
        ang.setPower(0.4);
        telescope.setPower(0.9);
    }
    void angle3(){
        ang.setTargetPosition(-782);
        telescope.setTargetPosition(-500);
        leftAngClaw.setPosition(0.5);
        rightAngClaw.setPosition(0.5);
        telescope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ang.setPower(0.4);
        telescope.setPower(0.9);
    }
//    void angle4(){
//        ang.setTargetPosition(ang4);
//        ang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftAngClaw.setPosition(0.5);
//        rightAngClaw.setPosition(0.5);
//        ang.setPower(1);
//    }

    void openLeftClaw(){
        leftClaw.setPosition(leftClawOpen);
        leftClawToggle = false;
    }
    void openRightClaw(){
        rightClaw.setPosition(rightClawOpen);
        rightClawToggle = false;
    }
    void openClaws(){
        openLeftClaw();
        openRightClaw();
    }
    void closeRightClaw(){
        rightClaw.setPosition(rightClawClose);
        rightClawToggle = true;
    }
    void closeLeftClaw(){
        leftClaw.setPosition(leftClawClose);
        leftClawToggle = true;
    }
    void closeClaws(){
        closeRightClaw();
        closeLeftClaw();
    }
}
