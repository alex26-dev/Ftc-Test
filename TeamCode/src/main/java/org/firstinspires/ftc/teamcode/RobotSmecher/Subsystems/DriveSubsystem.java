package org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private DoubleSupplier forward, strafe, rotation;

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftBack"),
                new Motor(hardwareMap, "rightBack")
        );
    }

    @Override
    public void periodic() {
        if (forward == null || strafe == null || rotation == null) {
            return;
        }

        updateSpeed(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble());
    }

    private void updateSpeed(double forward, double strafe, double rotation) {
        drive.driveRobotCentric(strafe, forward, rotation);
    }

    public void setAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    public void setPowerLimit(double powerLimit) {
        drive.setMaxSpeed(Range.clip(powerLimit, -1, 1));
    }
}