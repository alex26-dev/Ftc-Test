package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        sensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        currentState = State.IDLE;
    }

    @Override
    public void periodic() {
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
