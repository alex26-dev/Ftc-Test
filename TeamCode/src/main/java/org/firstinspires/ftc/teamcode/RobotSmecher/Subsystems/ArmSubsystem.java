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
        EXTENDED
    }

    public static double LOWER_ARM = 100, EXTEND_ARM = 150;

    private final DcMotorEx sliderMotor;
    private final PDFController pController;
    private ArmState currentState;

    public ArmSubsystem(HardwareMap hardwareMap) {
        pController = new PDFController(0.2, 0, 0);

        sliderMotor = hardwareMap.get(DcMotorEx.class, "sliderMotor");

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentState = ArmState.LOWERED;
    }

    @Override
    public void periodic() {
        if (currentState == ArmState.LOWERED) {
            moveArm(LOWER_ARM);
        } else {
            moveArm(EXTEND_ARM);
        }
    }

    private void moveArm(double position) {
        double power = pController.calculate(sliderMotor.getCurrentPosition(), position);

        sliderMotor.setPower(Range.clip(power, -0.5, 0.5));
    }

    public void toggleArmState() {
        if (currentState == ArmState.LOWERED) {
            currentState = ArmState.EXTENDED;
        } else {
            currentState = ArmState.LOWERED;
        }
    }
}
