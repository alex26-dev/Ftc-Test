package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.InterpolatedServo;

import java.util.function.BooleanSupplier;

public class OuttakeSubsystem extends SubsystemBase {

    public enum ClawPos {
        COLLECT_SAMPLE,
        COLLECT_SPECIMEN,
        DEPOSIT
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public static double COLLECT_SAMPLE_POS = 0.0, COLLECT_SPECIMEN_POS = 220.0, DEPOSIT_POS = 200.0;
    public static double OPEN_CLAW = 0.0, CLOSED_CLAW = 90.0;

    private final InterpolatedServo leftServo, rightServo;
    private final ServoEx clawServo;
    private BooleanSupplier isSampleCollected, isArmRetracted;
    private ClawPos currentClawPos;
    private ClawState currentClawState;

    private final ElevatorSubsystem elevator;

    public OuttakeSubsystem(HardwareMap hardwareMap, ElevatorSubsystem elevator) {
        clawServo = hardwareMap.get(ServoEx.class, "clawServo");
        leftServo = new InterpolatedServo(hardwareMap.get(ServoEx.class, "leftOuttakeServo"));
        rightServo = new InterpolatedServo(hardwareMap.get(ServoEx.class, "rightOuttakeServo"));

        this.elevator = elevator;

        leftServo.setInverted(true);

        leftServo.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 220.0));
        rightServo.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 220.0));

        currentClawPos = ClawPos.COLLECT_SAMPLE;
        currentClawState = ClawState.OPEN;
    }

    @Override
    public void periodic() {
        if (isSampleCollected.getAsBoolean() && isArmRetracted.getAsBoolean()) {
            pickUpSample();
        }

        updateClaw();
    }

    private void updateClaw() {
        switch (currentClawPos) {
            case COLLECT_SAMPLE:
                leftServo.setToPosition(COLLECT_SAMPLE_POS);
                rightServo.setToPosition(COLLECT_SAMPLE_POS);
                break;
            case COLLECT_SPECIMEN:
                leftServo.setToPosition(COLLECT_SPECIMEN_POS);
                rightServo.setToPosition(COLLECT_SPECIMEN_POS);
                break;
            default:
                leftServo.setToPosition(DEPOSIT_POS);
                rightServo.setToPosition(DEPOSIT_POS);
                break;
        }

        switch (currentClawState) {
            case OPEN:
                clawServo.turnToAngle(OPEN_CLAW);
                break;
            case CLOSED:
                clawServo.turnToAngle(CLOSED_CLAW);
                break;
        }
    }

    public void toggleClawState() {
        switch (currentClawState) {
            case OPEN:
                clawServo.turnToAngle(CLOSED_CLAW);
                currentClawState = ClawState.CLOSED;
                break;
            case CLOSED:
                clawServo.turnToAngle(OPEN_CLAW);
                currentClawState = ClawState.OPEN;
                break;
        }
    }

    private void pickUpSample() {
        elevator.changeElevatorState(ElevatorSubsystem.ElevatorState.COLLECT);

        currentClawPos = ClawPos.COLLECT_SAMPLE;
        currentClawState = ClawState.OPEN;
        updateClaw();
        // to add: wait for claw to open

        currentClawState = ClawState.CLOSED;
        updateClaw();
        // wait for claw to close
        currentClawPos = ClawPos.DEPOSIT;
        updateClaw();
    }

    public void pickUpSpecimen() {
        elevator.changeElevatorState(ElevatorSubsystem.ElevatorState.COLLECT);

        currentClawPos = ClawPos.COLLECT_SPECIMEN;
        currentClawState = ClawState.OPEN;
        updateClaw();
    }

    public void setBooleanSupplier(BooleanSupplier sample, BooleanSupplier arm) {
        isSampleCollected = sample;
        isArmRetracted = arm;
    }
}
