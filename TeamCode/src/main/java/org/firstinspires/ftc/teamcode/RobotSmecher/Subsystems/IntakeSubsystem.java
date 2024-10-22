package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.ColorSensor;

public class IntakeSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private final double SERVO_POWER = 0.75;

    private final CRServo servo;
    private final ColorSensor sensor;
    private State currentState;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "intakeServo");
        sensor = new ColorSensor(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        currentState = State.IDLE;
    }

    @Override
    public void periodic() {
        if (currentState == State.INTAKE) {
            switch (sensor.getSampleColor()) {
                case YELLOW:
                    currentState = State.IDLE;
                    break;
                case RED:
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
                servo.setPower(0);
                break;
            case INTAKE:
                servo.setDirection(DcMotorSimple.Direction.FORWARD);
                servo.setPower(SERVO_POWER);
                break;
            case OUTTAKE:
                servo.setDirection(DcMotorSimple.Direction.REVERSE);
                servo.setPower(SERVO_POWER);
                break;
        }
    }

    public void changeState(State state) {
        currentState = state;
    }
}
