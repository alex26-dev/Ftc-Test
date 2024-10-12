package org.firstinspires.ftc.teamcode.LimelightDetectionTest;

public class PController {
    private final double Kp;

    public PController(double Kp) {
        this.Kp = Kp;
    }

    public double calculatePower(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;

        return error * Kp;
    }

    public double calculatePower(double error) {
        return error * Kp;
    }
}
