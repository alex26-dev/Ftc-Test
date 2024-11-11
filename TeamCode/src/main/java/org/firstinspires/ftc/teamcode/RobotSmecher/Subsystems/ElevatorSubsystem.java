package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.PDFController;

public class ElevatorSubsystem extends SubsystemBase {

    public enum ElevatorState {
        LOWERED,
        RAISED,
        HANGING
    }

    static public double LOWERED_POS = 0, RAISED_POS = 100, HANG_POS = 50;

    private final DcMotorEx motor;
    private final PDFController pController;
    private ElevatorState currentState;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        pController = new PDFController(0.1, 0, 0);

        motor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentState = ElevatorState.LOWERED;
    }

    @Override
    public void periodic() {
        updatePos();
    }

    private void updatePos() {
        double position;

        switch (currentState) {
            case RAISED:
                position = RAISED_POS;
                break;
            case HANGING:
                position = HANG_POS;
                break;
            default:
                position = LOWERED_POS;
                break;
        }

        double power = pController.calculate(motor.getCurrentPosition(), position);

        motor.setPower(power);
    }

    public void toggleElevatorState() {
        switch (currentState) {
            case LOWERED:
                currentState = ElevatorState.RAISED;
                break;
            case RAISED:
                currentState = ElevatorState.HANGING;
                break;
            case HANGING:
                // aici schimbam codul pentru high rung
                currentState = ElevatorState.LOWERED;
                break;
        }
    }

    public void lowerElevator() {
        currentState = ElevatorState.LOWERED;
    }
}
