package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.InterpolatedServo;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        RETRACTED,
        EXTENDED
    }

    public static double RETRACT_ARM = 0.0, EXTEND_ARM = 120.0;

    private final InterpolatedServo leftServo, rightServo;
    private ArmState currentState;

    public ArmSubsystem(HardwareMap hardwareMap) {
        leftServo = new InterpolatedServo(hardwareMap.get(ServoEx.class, "armLeftServo"));
        rightServo = new InterpolatedServo(hardwareMap.get(ServoEx.class, "armRightServo"));

        leftServo.setInverted(true);

        leftServo.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 120.0));
        rightServo.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 120.0));

        currentState = ArmState.RETRACTED;
    }

    @Override
    public void periodic() {
        updateArmPos();
    }

    private void updateArmPos() {
        double position;

        if (currentState == ArmState.RETRACTED) {
            position = RETRACT_ARM;
        } else {
            position = EXTEND_ARM;
        }

        leftServo.setToPosition(position);
        rightServo.setToPosition(position);
    }

    public void toggleArmState() {
        if (currentState == ArmState.RETRACTED) {
            currentState = ArmState.EXTENDED;
        } else {
            currentState = ArmState.RETRACTED;
        }
    }
}
