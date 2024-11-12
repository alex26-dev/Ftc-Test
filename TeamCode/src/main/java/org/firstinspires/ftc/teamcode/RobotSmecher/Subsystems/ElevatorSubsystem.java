package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.PDFController;

public class ElevatorSubsystem extends SubsystemBase {

    public enum ElevatorState {
        COLLECT,       // Used for collecting samples and specimens
        HIGH_BASKET,
        HIGH_CHAMBER,
        LOW_RUNG,      // Used for raising hook above low rung
        HANG,          // Used for lowering hook in order to hang in the air
        TOP,           // Max position possible
        BOTTOM,        // Min position possible
        CUSTOM         // This state is used when the driver changes positions manually
    }

    private final double SLIDER_MIN_RANGE = 0, SLIDER_MAX_RANGE = 400;
    private final double COLLECT_POS = 10, HIGH_BASKET_POS = 200, HIGH_CHAMBER_POS = 150, LOW_RUNG_POS = 100, HANG_POS = 50;

    private final DcMotorEx motor;
    private final PDFController pController;
    private ElevatorState currentState;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        pController = new PDFController(0.1, 0, 0);

        motor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentState = ElevatorState.COLLECT;
    }

    @Override
    public void periodic() {
        updatePos();
    }

    private void updatePos() {
        double position;

        switch (currentState) {
            case COLLECT:
                position = COLLECT_POS;
                break;
            case HIGH_BASKET:
                position = HIGH_BASKET_POS;
                break;
            case HIGH_CHAMBER:
                position = HIGH_CHAMBER_POS;
                break;
            case LOW_RUNG:
                position = LOW_RUNG_POS;
                break;
            case HANG:
                position = HANG_POS;
                break;
            case TOP:
                position = SLIDER_MAX_RANGE;
                break;
            case BOTTOM:
                position = SLIDER_MIN_RANGE;
                break;
            default:
                return;
        }

        double power = pController.calculate(motor.getCurrentPosition(), position);

        motor.setPower(power);

        if (!motor.isBusy() && currentState == ElevatorState.HIGH_BASKET) {
            // deposit sample on drivers input
        }

        if (!motor.isBusy() && currentState == ElevatorState.HIGH_CHAMBER) {
            // deposit specimen on drivers input
        }
    }

    public void changeElevatorState(ElevatorState state) {
        currentState = state;
    }

    // For drivers
    public void lowerElevator() {
        if (motor.getCurrentPosition() > SLIDER_MIN_RANGE + 20) {
            currentState = ElevatorState.CUSTOM;
            motor.setPower(-1);
        }
    }

    // For drivers
    public void raiseElevator() {
        if (motor.getCurrentPosition() < SLIDER_MAX_RANGE - 20) {
            currentState = ElevatorState.CUSTOM;
            motor.setPower(1);
        }
    }

    // For hanging onto low rung
    public void toggleHangState() {
        switch (currentState) {
            case LOW_RUNG:
                currentState = ElevatorState.HANG;
                break;
            case HANG:
                break;
            default:
                currentState = ElevatorState.LOW_RUNG;
                break;
        }
    }
}
