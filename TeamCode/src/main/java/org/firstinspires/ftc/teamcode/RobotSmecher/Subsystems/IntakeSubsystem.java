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
    private State currentState;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        sensor = new ColorSensor(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        currentState = State.IDLE;
    }

    @Override
    public void periodic() {
        if (currentState == State.INTAKE) {
            switch (sensor.getSampleColor()) {
                case YELLOW:
                case RED:
                    currentState = State.IDLE;
                    break;
                case BLUE:
                    currentState = State.OUTTAKE;
                    break;
            }
        }

        if (currentState == State.OUTTAKE && sensor.getSampleColor() == null) {
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

    public void changeState(State state) {
        if (state == currentState) currentState = State.IDLE;
        else currentState = state;
    }
}
