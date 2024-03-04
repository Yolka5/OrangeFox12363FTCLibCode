package org.firstinspires.ftc.teamcode.common.drive.opmodeOLD.MotorServoControl;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double Ki, Kp, Kd;

    public PID(double Ki, double Kp, double Kd){
        this.Ki = Ki;
        this.Kp = Kp;
        this.Kd = Kd;
    }

    double integralSum = 0;

    ElapsedTime time = new ElapsedTime();

    private double lastError = 0;

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * time.seconds();
        double derivative = (error - lastError) / time.seconds();
        lastError = error;
        time.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }
}
