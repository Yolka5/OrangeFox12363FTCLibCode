package org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.MotorServoControl.PID;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;

@TeleOp
public class TelevWithController extends LinearOpMode {
    private Servo Armservo, orangeCone, Claw;
    private DcMotor elevator;
    private boolean armServoToggle;
    private boolean clawServoToggle;
    private boolean coneToggle;
    private boolean elevatorToggle;
    private ControllerDetect gameP1;
    private ControllerDetect gameP2;
    private PID pidElevator;

    @Override
    public void runOpMode() throws InterruptedException {
        gameP1 = new ControllerDetect(gamepad1);
        gameP2 = new ControllerDetect(gamepad2);
        pidElevator = new PID(5, 1000, 5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DriveTeleop driveTeleop = new DriveTeleop(drive);
        Armservo = hardwareMap.get(Servo.class, "arm");
        Claw = hardwareMap.get(Servo.class, "clips");
        orangeCone = hardwareMap.get(Servo.class, "orangeCone");
        elevator = hardwareMap.dcMotor.get("elevator");
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(pidElevator.PIDController(0, elevator.getCurrentPosition()));
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gameP1.update();
            gameP2.update();
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gameP1.left_stick_y,
//                            -gameP1.left_stick_x,
//                            -gameP1.right_stick_x
//                    )
//            );
            driveTeleop.teleopDrive(gameP1);
            armTeleop();
            clawTeleop();
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gameP1.left_stick_y,
//                            -gameP1.left_stick_x,
//                            -gameP1.right_stick_x
//                    )
//            );
//
//            drive.update(); // maybe after??
//
//
//            if (gameP2.XOnce() && armServoToggle) {
//                Armservo.setPosition(0.3);
//                armServoToggle = false;
//            } else if (gameP2.XOnce() && !armServoToggle) {
//                Armservo.setPosition(0.9);
//                armServoToggle = true;
//            }
//            if (gameP2.BOnce() && clawServoToggle) {
//                Claw.setPosition(0.7);
//                clawServoToggle = false;
//            } else if (gameP2.BOnce() && !clawServoToggle) {
//                Claw.setPosition(0.2);
//                clawServoToggle = true;
//            }
//            if (gameP2.YOnce() && coneToggle){
//                orangeCone.setPosition(1);
//                coneToggle = false;
//            }else if (gameP2.YOnce() && !coneToggle){
//                orangeCone.setPosition(0);
//                coneToggle = true;
//            }

            if (gameP2.AOnce() && !elevatorToggle) {
                // Move elevator halfway
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
                // Move orangeCone to position 1
                orangeCone.setPosition(0);
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
                orangeCone.setPosition(1);
                elevatorToggle = false;
            }

//            if (gameP2.AOnce() && !elevatorToggle){
//                elevator.setTargetPosition(-1500);
//                elevator.setPower(pidController.PIDController(-1500, elevator.getCurrentPosition()));
////                elevator.setPower(pidController.PIDController(100, elevator.getCurrentPosition()));
//                telemetry.addData("elevatorPose", elevator.getCurrentPosition());
//                elevatorToggle = true;
//            }else if (gameP2.AOnce() && elevatorToggle){
//                elevator.setTargetPosition(3);
//                elevator.setPower(pidController.PIDController(3, elevator.getCurrentPosition()));
////                elevator.setPower(1);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                telemetry.addData("elevatorPose", elevator.getCurrentPosition());
//                elevatorToggle = false;
            }
            telemetry.update();

        }
//    void driveTeleop(SampleMecanumDrive drive){
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gameP1.left_stick_y,
//                        -gameP1.left_stick_x,
//                        -gameP1.right_stick_x
//                )
//        );
//
//        drive.update(); // maybe after??
//    }
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
}
