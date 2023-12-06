package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController  {
    double integralSum = 0;
    double kP = 0.0125;
    double kI = 0;
    double kD = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    public double PIDControl(double targetPosition, double currentPosition)
    {
        double error = targetPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();
        double output = (error * kP) + (derivative * kD) + (integralSum * kI);

        return output;

    }

}