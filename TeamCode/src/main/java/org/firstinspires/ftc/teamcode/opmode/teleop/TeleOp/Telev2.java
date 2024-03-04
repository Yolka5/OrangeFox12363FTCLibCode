package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.MotorServoControl.PID;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

//@TeleOp
public class Telev2 extends LinearOpMode{
    private Servo Armservo, orangeCone, Claw;
        private DcMotor elevator;
        private boolean armServoToggle;
        private boolean clawServoToggle;
        private boolean coneToggle;
        private boolean elevatorToggle;
        private ControllerDetect gameP1, gameP2;
        private PID pidElevator;
        private AprilTagRedTeleop aprilTagRedTeleop;
        private DriveTeleop driveTeleop;
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
            //exposure!!!
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            exposure.setMode(ExposureControl.Mode.Manual);

            exposure.setExposure(2, TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(255);

            Pose2d absPose = new Pose2d(0,0,Math.toRadians(0));

            gameP1 = new ControllerDetect(gamepad1);
            gameP2 = new ControllerDetect(gamepad2);
            pidElevator = new PID(5, 1000, 5);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            aprilTagRedTeleop = new AprilTagRedTeleop(drive, tagProcessor, telemetry);

            driveTeleop = new DriveTeleop(drive);

            Armservo = hardwareMap.get(Servo.class, "arm");
            Claw = hardwareMap.get(Servo.class, "clips");
            orangeCone = hardwareMap.get(Servo.class, "orangeCone");
            elevator = hardwareMap.dcMotor.get("elevator");
            elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                gameP1.update();
                gameP2.update();
                driveTeleop.teleopDrive(gameP1);
                armTeleop();
                clawTeleop();

                if (gameP1.X()){
                    aprilTagRedTeleop.aprilTagLeft();
                }
                else if (gameP1.B()){
                    aprilTagRedTeleop.aprilTagMid();
                }
                else if (gameP1.Y()){
                    aprilTagRedTeleop.aprilTagRight();
                }
//                if (gameP1.X() || gameP1.B() || gameP1.Y()){
//                    aprilTagRedTeleop.aprilTag(drive, tagProcessor, gameP1, telemetry);
//                }

                if (gameP2.AOnce() && !elevatorToggle) {
                    elevator.setTargetPosition(-1500);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(pidElevator.PIDController(-1500, elevator.getCurrentPosition()));
                    while (opModeIsActive() && elevator.isBusy()) {
                        gameP1.update();
                        gameP2.update();
                        driveTeleop.teleopDrive(gameP1);
                        armTeleop();
                        clawTeleop();

                        telemetry.addData("elevatorPose", elevator.getCurrentPosition());
                        telemetry.update();
                    }
                    orangeCone.setPosition(1);
                    elevatorToggle = true;
                } else if (gameP2.AOnce() && elevatorToggle) {
                    // Move elevator back to starting position
                    elevator.setTargetPosition(0);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(pidElevator.PIDController(0, elevator.getCurrentPosition()));

                    while (opModeIsActive() && elevator.isBusy()) {
                        gameP1.update();
                        gameP2.update();
                        driveTeleop.teleopDrive(gameP1);
                        armTeleop();
                        clawTeleop();

                        telemetry.addData("elevatorPose", elevator.getCurrentPosition());
                        telemetry.update();
                    }

                    // Move orangeCone to position 0
                    orangeCone.setPosition(0);
                    elevatorToggle = false;
                }
                telemetry.update();

            }
            telemetry.update();

        }
        // ---------functions-----------------
//        void driveTeleop(SampleMecanumDrive drive){
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gameP1.left_stick_y,
//                            -gameP1.left_stick_x,
//                            -gameP1.right_stick_x
//                    )
//            );
//
//            drive.update(); // maybe after??
//        }
        void armTeleop(){
            if (gameP2.XOnce() && armServoToggle) {
                Armservo.setPosition(0.3);
                armServoToggle = false;
            } else if (gameP2.XOnce() && !armServoToggle) {
                Armservo.setPosition(0.9);
                armServoToggle = true;
            }
        }
        void clawTeleop(){
            if (gameP2.BOnce() && clawServoToggle) {
                Claw.setPosition(0.7);
                clawServoToggle = false;
            } else if (gameP2.BOnce() && !clawServoToggle) {
                Claw.setPosition(0.2);
                clawServoToggle = true;
            }
        }
//        void aprilTag(SampleMecanumDrive drive, AprilTagProcessor tagProcessor){
//            if (tagProcessor.getDetections().size() > 0) {
//                telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
//                AprilTagDetection tag = tagProcessor.getDetections().get(0);
//                double range = tag.ftcPose.range;
//                double bearing = tag.ftcPose.bearing;
//                double tagYaw = tag.ftcPose.yaw;
//                double yPose = Math.sin(Math.toRadians(bearing)) * range;
//                double xPose = Math.cos(Math.toRadians(bearing)) * range;
//
//                double tagFieldY = tag.metadata.fieldPosition.get(1);
//                double tagFiledX = tag.metadata.fieldPosition.get(0);
//                int tagId = tag.id;
//
//                Pose2d tagPoseField = new Pose2d(tagFiledX, tagFieldY, Math.toRadians(0));
//                telemetry.addData("tag pose", tagPoseField);
//                telemetry.addData("tag ID", tagId);
//
//                Pose2d robotRelPose = new Pose2d(-xPose, -yPose, Math.toRadians(tagYaw));
//                telemetry.addData("trigo rel pose", robotRelPose);
//
//                double x = tagPoseField.getX() + robotRelPose.getX();
//                double y = tagPoseField.getY() + robotRelPose.getY();
//                double heading = tagPoseField.getHeading() - robotRelPose.getHeading();
//                Pose2d absPose = new Pose2d(x, y, heading);
//
//                telemetry.addData("trigo ABS pose", absPose);
//                telemetry.addData("pose", drive.getPoseEstimate());
//                telemetry.update();
//
//
//                // left side
//                if (gameP1.X()) {
//                    drive.setPoseEstimate(absPose);
//                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
//                            .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)))
//                            .build();
//
//                    drive.followTrajectorySequence(goToAprilTag);
//                }
//
//                //middle
//                else if (gameP1.Y()) {
//                    drive.setPoseEstimate(absPose);
//                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
//                            .lineToLinearHeading(new Pose2d(50, -41, Math.toRadians(0)))
//                            .build();
//                    drive.followTrajectorySequence(goToAprilTag);
//                }
//
//                //right side
//                else if (gameP1.B()) {
//                    drive.setPoseEstimate(absPose);
//                    TrajectorySequence goToAprilTag = drive.trajectorySequenceBuilder(absPose)
//                            .lineToLinearHeading(new Pose2d(50, -46, Math.toRadians(0)))
//                            .build();
//
//                    drive.followTrajectorySequence(goToAprilTag);
//                }
//            }
//            else {
//                telemetry.addData("is tag detected", tagProcessor.getDetections().size() > 0);
//            }
//        }
    }

