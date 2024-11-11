package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        RETRACTED,
        EXTENDED
    }

    public static double RETRACT_ARM = 0, EXTEND_ARM = 0.7;

    private final ServoEx leftServo, rightServo;
    private ArmState currentState;

    public ArmSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(ServoEx.class, "armLeftServo");
        rightServo = hardwareMap.get(ServoEx.class, "armRightServo");

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

        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public void toggleArmState() {
        if (currentState == ArmState.RETRACTED) {
            currentState = ArmState.EXTENDED;
        } else {
            currentState = ArmState.RETRACTED;
        }
    }
}
