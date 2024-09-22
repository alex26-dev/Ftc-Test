package org.firstinspires.ftc.teamcode.PIDControllerAuto;

public class PController {
    private final double Kp;

    PController(double Kp) {
        this.Kp = Kp;
    }

    public double calculatePower(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;

        return Range.clip(error * Kp, -1, 1);
    }
}
