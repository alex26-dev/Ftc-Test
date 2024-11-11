package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.InterpolatedServo;

public class OuttakeSubsystem extends SubsystemBase {

    public enum ClawPos {
        COLLECT,
        DEPOSIT
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public static double COLLECT_POS = 0.0, DEPOSIT_POS = 220.0;
    public static double OPEN_CLAW = 0.0, CLOSED_CLAW = 90.0;

    private final InterpolatedServo leftServo, rightServo;
    private final ServoEx clawServo;
    private ClawPos currentClawPos;
    private ClawState currentClawState;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(ServoEx.class, "clawServo");
        leftServo = new InterpolatedServo(hardwareMap.get(ServoEx.class, "leftOuttakeServo"));
        rightServo = new InterpolatedServo(hardwareMap.get(ServoEx.class, "rightOuttakeServo"));

        leftServo.setInverted(true);

        leftServo.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 220.0));
        rightServo.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 220.0));

        currentClawPos = ClawPos.COLLECT;
        currentClawState = ClawState.OPEN;
    }

    @Override
    public void periodic() {
        if (currentClawState == ClawState.OPEN && currentClawPos != ClawPos.COLLECT) {
            toggleClawPos();
        }
    }

    public void toggleClawPos() {
        switch (currentClawPos) {
            case COLLECT:
                leftServo.setToPosition(DEPOSIT_POS);
                rightServo.setToPosition(DEPOSIT_POS);

                currentClawPos = ClawPos.DEPOSIT;
                break;
            case DEPOSIT:
                leftServo.setToPosition(COLLECT_POS);
                rightServo.setToPosition(COLLECT_POS);

                currentClawPos = ClawPos.COLLECT;
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
}
