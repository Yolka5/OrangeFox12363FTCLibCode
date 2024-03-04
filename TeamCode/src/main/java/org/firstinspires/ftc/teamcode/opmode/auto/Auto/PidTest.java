package org.firstinspires.ftc.teamcode.opmode.auto.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous
public class PidTest extends LinearOpMode {
    DcMotorEx motor;
    double integralSum = 0;
    double Ki = 0;
    double Kp = 1;
    double Kd = 0.5;

    ElapsedTime time = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "elevator");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            double power = PIDController(0.9, motor.getCurrentPosition());
            motor.setPower(power);
        }

    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
//        integralSum += error * time.seconds();
        integralSum += error;
//        double derivative = (error - lastError) / time.seconds();
        double derivative = error - lastError;
        lastError = error;

        time.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}

