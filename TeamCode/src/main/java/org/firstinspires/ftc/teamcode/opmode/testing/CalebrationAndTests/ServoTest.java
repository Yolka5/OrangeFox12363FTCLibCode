package org.firstinspires.ftc.teamcode.opmode.testing.CalebrationAndTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode{
    private Servo leftAngClaw, rightAngClaw;
    private double leftClawClose = 0.56, rightClawClose = 0.8, leftClawOpen = 0.83, rightClawOpen = 0.53;
    boolean starting = true;
    public void runOpMode(){
//        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
//        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftAngClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightAngClaw = hardwareMap.get(Servo.class, "rightClaw");

        waitForStart();
        while (opModeIsActive()){

//            AnalogInput angleLeftClawAnalog = hardwareMap.get(AnalogInput.class, "angleLeftClawAnalog");
//            double position = angleLeftClawAnalog.getVoltage() / 3.3 * 360;
////            double position = angleLeftClaw.getPosition();
//            telemetry.addData("pose", position);
//            telemetry.update();
            if (gamepad1.a){ // open
//                LeftClaw.setPosition(0.83);
//                rightClaw.setPosition(0.53);
                telemetry.addData("state",0);
                leftAngClaw.setPosition(leftClawClose);
                rightAngClaw.setPosition(rightClawClose);
            }
            if (gamepad1.y){ // close
                telemetry.addData("state",1);
//                LeftClaw.setPosition(0.56);
//                rightClaw.setPosition(0.8);
                leftAngClaw.setPosition(leftClawOpen);
                rightAngClaw.setPosition(rightClawOpen);
            }
        }


    }
}

