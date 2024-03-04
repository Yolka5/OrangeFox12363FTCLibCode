package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;

@TeleOp
public class FieldOrianted extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ControllerDetect gameP1 = new ControllerDetect(gamepad1);
//        drive.setExternalHeading(Math.toRadians(180));
//        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        drive.setExternalHeading(90);


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            //MECHANISM CODE
//            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.right_stick_x;
//            double rx = gamepad1.left_stick_x;

            if (gamepad1.back) {
                drive.setExternalHeading(0);
            }


            double heading = drive.getRawExternalHeading();
            telemetry.addData("heading", heading);
            telemetry.addLine(String.valueOf(heading));
            double cosHeading = Math.cos(-heading);
            double sinHeading = Math.sin(-heading);

            // Transform the joystick inputs to field relative
            double forward = gamepad1.left_stick_y * cosHeading - gamepad1.left_stick_x * sinHeading;
            double strafe = gamepad1.left_stick_y * sinHeading + gamepad1.left_stick_x * cosHeading;
            double turn = gamepad1.right_stick_x;


            // Set the weighted drive power with field relative inputs
            drive.setWeightedDrivePower(new Pose2d(-forward, -strafe, -turn));

            // Update the drive
            drive.update();
        }
    }
}

