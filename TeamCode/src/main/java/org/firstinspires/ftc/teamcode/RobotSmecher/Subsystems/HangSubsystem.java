package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.PDFController;

public class HangSubsystem extends SubsystemBase {

    enum HangerState {
        LOWERED,
        RAISED,
        HANGING
    }

    static public double LOW_POS = 0, RAISED_POS = 150, HANG_POS = 100;

    private final DcMotorEx motor;
    private final PDFController pController;
    private HangerState currentState;

    public HangSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        pController = new PDFController(0.1, 0, 0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentState = HangerState.LOWERED;
    }

    @Override
    public void periodic() {
        updateHangerPos();
    }

    private void updateHangerPos() {
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

    public void toggleHangerState() {
        switch (currentState) {
            case LOWERED:
                currentState = HangerState.RAISED;
                break;
            case RAISED:
                currentState = HangerState.HANGING;
                break;
            case HANGING:
                // aici schimbam codul pentru high rung
                currentState = HangerState.LOWERED;
                break;
        }
    }

    public void lowerHanger() {
        currentState = HangerState.LOWERED;
    }
}
