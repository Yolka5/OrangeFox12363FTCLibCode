package org.firstinspires.ftc.teamcode.opmode.testing.CalebrationAndTests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.Controller.ControllerDetect;

@TeleOp
public class ClawTest extends LinearOpMode{
    private Servo angleLeftClaw, angleRightClaw;
    private Servo leftClaw, rightClaw;
    private DistanceSensor leftClawSensor, rightClawSensor;
    private boolean clawToggle, leftClawState, rightClawState, leftSensorClosed = false, rightSensorClosed = false;
    private double leftClawClose = 0.56, rightClawClose = 0.8, leftClawOpen = 0.83, rightClawOpen = 0.53;
    private ControllerDetect gameP2, gameP1;
    boolean starting = true;

    public void runOpMode(){
//        angleLeftClaw = hardwareMap.get(Servo.class, "angleLeftClaw");
//        angleRightClaw = hardwareMap.get(Servo.class, "angleRightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        leftClawSensor = hardwareMap.get(DistanceSensor.class, "leftClawD");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClawSensor = hardwareMap.get(DistanceSensor.class, "rightClawD");
        gameP1 = new ControllerDetect(gamepad1);
        gameP2 = new ControllerDetect(gamepad2);

            waitForStart();
            while (opModeIsActive()){
                telemetry.addData("distanceLEFT", leftClawSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("distanceRight", rightClawSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
                gameP2.update();
                gameP1.update();
                if (gameP2.Y()) {
                    if (leftClawSensor.getDistance(DistanceUnit.MM) <= 33 && !leftSensorClosed && leftClaw.getPosition() != leftClawClose) {
                        closeLeftClaw();
                        leftSensorClosed = true;
                    }
                    if (rightClawSensor.getDistance(DistanceUnit.MM) <= 21 /*&& !leftSensorClosed*/ && rightClaw.getPosition() != rightClawClose) {
                        closeRightClaw();
                        rightSensorClosed = true;
                    }
                }
                if (leftClawState == rightClawState && leftClawState != clawToggle) {clawToggle = leftClawState;}
                if (gameP2.AOnce() && clawToggle){ // open
                    openClaws();
//                    rightSensorClosed = false;
//                    leftSensorClosed = false;
                }else if (gameP2.AOnce() && !clawToggle){ // close
                    closeClaws();
                }
            }

    }
    void openLeftClaw(){
        leftClaw.setPosition(leftClawOpen);
        leftClawState = false;
    }
    void openRightClaw(){
        rightClaw.setPosition(rightClawOpen);
        rightClawState = false;
    }
    void openClaws(){
        openLeftClaw();
        openRightClaw();
        clawToggle = false;
    }
    //-------------------//
    void closeRightClaw(){
        rightClaw.setPosition(rightClawClose);
        rightClawState = true;
    }
    void closeLeftClaw(){
        leftClaw.setPosition(leftClawClose);
        leftClawState = true;
    }
    void closeClaws(){
        closeRightClaw();
        closeLeftClaw();
        clawToggle = true;
    }
}
