package org.firstinspires.ftc.teamcode.RobotSmecher.Util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDFController {

    private final double kp, kd, kf;
    private final ElapsedTime timer;
    private double lastError = 0;

    public PDFController(double kp, double kd, double kf) {
        this.kp = kp;
        this.kd = kd;
        this.kf = kf;

        timer = new ElapsedTime();
    }

    public double calculate(double currentPos, double targetPos) {
        double error = targetPos - currentPos;
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;

        return (kp * error) + (kd * derivative) + (kf * targetPos);
    }
}
