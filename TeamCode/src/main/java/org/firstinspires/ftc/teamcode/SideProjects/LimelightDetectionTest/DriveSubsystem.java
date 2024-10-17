package org.firstinspires.ftc.teamcode.SideProjects.LimelightDetectionTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

public class DriveSubsystem {

    private final double CIRCUMFERENCE = 100 * Math.PI;
    private final double TICKS_PER_ROTATION = 340;

    private final List<DcMotorEx> motors = new ArrayList<>();
    private final IMU imu;

    private final PController pController = new PController(0.1);

    public DriveSubsystem(HardwareMap hardwareMap, IMU imu) {
        this.imu = imu;

        motors.add(hardwareMap.get(DcMotorEx.class, "FrontLeftMotor"));
        motors.add(hardwareMap.get(DcMotorEx.class, "FrontRightMotor"));
        motors.add(hardwareMap.get(DcMotorEx.class, "BackLeftMotor"));
        motors.add(hardwareMap.get(DcMotorEx.class, "BackRightMotor"));

        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);

        motors.forEach(motor -> {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        });
    }

    public void moveForward(int distanceMm) {
        double ticks = TICKS_PER_ROTATION / CIRCUMFERENCE * distanceMm;

        motors.forEach(motor -> {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(pController.calculatePower(motors.get(0).getCurrentPosition(), motors.get(0).getTargetPosition()));
        });

        while (Math.abs(motors.get(0).getCurrentPosition() - motors.get(0).getTargetPosition()) > 50) {
            motors.forEach(motor -> motor.setPower(pController.calculatePower(motors.get(0).getCurrentPosition(), motors.get(0).getTargetPosition())));
        }
    }

    public void rotate(double angle) {
        motors.forEach(motor -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        double currentDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = (angle - currentDegrees + 180) % 360 - 180;

        while (Math.abs(error) > 2) {
            currentDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double power = pController.calculatePower(error);

            motors.get(0).setPower(power);
            motors.get(1).setPower(-power);
            motors.get(2).setPower(power);
            motors.get(3).setPower(-power);

            error = (angle - currentDegrees + 180) % 360 - 180;
        }

        motors.forEach(motor -> motor.setPower(0));
    }
}
