package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.PDFController;

public class OuttakeSubsystem extends SubsystemBase {

    enum OuttakeState {
        LOWERED,
        RAISED,
        HANGING
    }

    static public double LOW_POS = 0, RAISED_POS = 100, HANG_POS = 50;

    private final DcMotorEx motor;
    private final PDFController pController;
    private OuttakeState currentState;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        pController = new PDFController(0.1, 0, 0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentState = OuttakeState.LOWERED;
    }

    @Override
    public void periodic() {
        updateOuttakePos();
    }

    private void updateOuttakePos() {
        double position;

        switch (currentState) {
            case RAISED:
                position = RAISED_POS;
                break;
            case HANGING:
                position = HANG_POS;
                break;
            default:
                position = LOW_POS;
                break;
        }

        double power = pController.calculate(motor.getCurrentPosition(), position);

        motor.setPower(power);
    }

    public void toggleOuttakeState() {
        switch (currentState) {
            case LOWERED:
                currentState = OuttakeState.RAISED;
                break;
            case RAISED:
                currentState = OuttakeState.HANGING;
                break;
            case HANGING:
                // aici schimbam codul pentru high rung
                currentState = OuttakeState.LOWERED;
                break;
        }
    }

    public void lowerOuttake() {
        currentState = OuttakeState.LOWERED;
    }
}
