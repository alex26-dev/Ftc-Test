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

    private final DcMotorEx sliderMotorL, sliderMotorR;
    private final PDFController pdf;
    private ArmState currentState;

    public ArmSubsystem(HardwareMap hardwareMap) {
        // Daca e nevoie putem pune 2 pdf controllere cu constante diferite pentru fiecare glisiera in parte
        pdf = new PDFController(0.2, 0.4, 0.5);

        sliderMotorL = hardwareMap.get(DcMotorEx.class, "sliderMotorL");
        sliderMotorR = hardwareMap.get(DcMotorEx.class, "sliderMotorR");

        sliderMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            case EXTENDED:
                moveArm(EXTEND_ARM);
                break;
        }
    }

    private void moveArm(double position) {
        double leftPower = pdf.calculate(sliderMotorL.getCurrentPosition(), position);
        double rightPower = pdf.calculate(sliderMotorR.getCurrentPosition(), position);

        sliderMotorL.setPower(Range.clip(leftPower, -0.5, 0.5));
        sliderMotorR.setPower(Range.clip(rightPower, -0.5, 0.5));
    }

    public void changeArmState(ArmState state) {
        currentState = state;
    }

    public void toggleArmState() {
        if (currentState == ArmState.LOWERED) {
            changeArmState(ArmState.EXTENDED);
        } else {
            changeArmState(ArmState.LOWERED);
        }
    }
}
