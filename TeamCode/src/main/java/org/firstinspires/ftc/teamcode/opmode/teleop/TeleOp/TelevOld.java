package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;

@TeleOp
public class TelevOld extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ControllerDetect gameP1 = new ControllerDetect(gamepad1);
//        drive.setExternalHeading(Math.toRadians(180));
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            //MECHANISM CODE
//            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.right_stick_x;
//            double rx = gamepad1.left_stick_x;

            if (gamepad1.back) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(0)));
            }


                    // Get the robot's current pose on the field
//        Pose2d poseEstimate = drive.getPoseEstimate();

        // Use the inverse of the robot's heading to make the drive field relative
        double heading = drive.getPoseEstimate().getHeading();
            telemetry.addData("heading", heading);
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);

        // Transform the joystick inputs to field relative
        double forward = -gamepad1.left_stick_y * cosHeading - gamepad1.left_stick_x * sinHeading;
        double strafe = -gameP1.left_stick_y * sinHeading + gamepad1.left_stick_x * cosHeading;
        double turn = -gamepad1.right_stick_x;

        // Set the weighted drive power with field relative inputs
        drive.setWeightedDrivePower(new Pose2d(forward, strafe, turn));

        // Update the drive
        drive.update();


//
//            double robotYaw = drive.getRawExternalHeading();
//
//            double rotX = x * Math.cos(-robotYaw) - y * Math.sin(-robotYaw);
//            double rotY = x * Math.sin(-robotYaw) + y * Math.cos(-robotYaw);
//
//            rotX = rotX * 1.1;
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            if (gamepad2.right_bumper) {
//                frontLeftMotor.setPower(frontLeftPower * 0.3);
//                backLeftMotor.setPower(backLeftPower * 0.3);
//                frontRightMotor.setPower(frontRightPower * 0.3);
//                backRightMotor.setPower(backRightPower * 0.3);
//            }else {
//                frontLeftMotor.setPower(frontLeftPower);
//                backLeftMotor.setPower(backLeftPower);
//                frontRightMotor.setPower(frontRightPower);
//                backRightMotor.setPower(backRightPower);
//            }
//            drive.update();
        }
    }
}
