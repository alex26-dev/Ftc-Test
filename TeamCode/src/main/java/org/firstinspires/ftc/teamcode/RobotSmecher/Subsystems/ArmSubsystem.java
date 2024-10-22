package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotSmecher.Util.PDFController;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        LOWERED,
        RAISED,
        LIFT
    }

    public static double LOWER_ARM = 100, RAISE_ARM = 150, LIFT_ARM = 180;

    private final DcMotorEx armMotor;
    private final PDFController pdf;
    private ArmState currentState;

    public ArmSubsystem(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        pdf = new PDFController(0.2, 0.4, 0.5);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentState = ArmState.LOWERED;
    }

    @Override
    public void periodic() {
        if (currentState == null)
            return;

        updateArmPos();
    }

    private void updateArmPos() {
        switch (currentState) {
            case LOWERED:
                moveArm(LOWER_ARM);
                break;
            case RAISED:
                moveArm(RAISE_ARM);
                break;
            case LIFT:
                moveArm(LIFT_ARM);
                break;
        }
    }

    private void moveArm(double position) {
        double power = pdf.calculate(armMotor.getCurrentPosition(), position);

        armMotor.setPower(Range.clip(power, -0.5, 0.5));
    }

    public void changeArmState(ArmState state) {
        currentState = state;
    }

    public void toggleArmState() {
        if (currentState == ArmState.LOWERED) {
            changeArmState(ArmState.RAISED);
        } else {
            changeArmState(ArmState.LOWERED);
        }
    }
}
