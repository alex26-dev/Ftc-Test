package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.ColorSensor;

public class IntakeSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private final double MOTOR_POWER = 0.75;

    private final DcMotorEx motor;
    private final ColorSensor sensor;
    private boolean isSampleCollected;
    private State currentState;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        sensor = new ColorSensor(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        isSampleCollected = false;
        currentState = State.IDLE;
    }

    @Override
    public void periodic() {
        if (currentState == State.INTAKE) {
            switch (sensor.getSampleColor()) {
                case YELLOW:
                case RED:
                    isSampleCollected = true;
                    currentState = State.IDLE;
                    break;
                case BLUE:
                    isSampleCollected = false;
                    currentState = State.OUTTAKE;
                    break;
            }
        }

        if (currentState == State.OUTTAKE && sensor.getSampleColor() == null) {
            isSampleCollected = false;
            currentState = State.IDLE;
        }

        updateServo();
    }

    private void updateServo() {
        switch (currentState) {
            case IDLE:
                motor.setPower(0);
                break;
            case INTAKE:
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setPower(MOTOR_POWER);
                break;
            case OUTTAKE:
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                motor.setPower(MOTOR_POWER);
                break;
        }
    }

    public void toggleIntake() {
        if (!isSampleCollected() && currentState == State.IDLE) currentState = State.INTAKE;
        else if (isSampleCollected() && currentState == State.IDLE) currentState = State.OUTTAKE;
    }

    public boolean isSampleCollected() {
        return isSampleCollected;
    }
}
