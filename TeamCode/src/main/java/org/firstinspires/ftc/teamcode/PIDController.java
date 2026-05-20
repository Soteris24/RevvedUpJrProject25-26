package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer.reset();
    }

    public double update(double currentPosition) {
        double error = targetPosition - currentPosition;
        double deltaTime = timer.seconds();
        double output = 0;

        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            
            output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            
            lastError = error;
            timer.reset();
        }
        return output;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }
    
    public void changeTargetPosition(double delta) {
        this.targetPosition += delta;
    }
}
