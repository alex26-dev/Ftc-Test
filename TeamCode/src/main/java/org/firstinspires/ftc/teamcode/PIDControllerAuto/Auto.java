package org.firstinspires.ftc.teamcode.PIDControllerAuto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "PID Test")
@Disabled
public class Auto extends LinearOpMode {

    List<DcMotorEx> motors = new ArrayList<>();
    private final double forwardThreshold = 50;

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        initMotors(motors);
        waitForStart();

        // Mers #1
        moveForward(motors, 5000, 1);

        // Rotatie de 90 de grade
        rotateRobot(imu, motors, 90);

        // Mers #2
        moveForward(motors, 5000, 1);
    }

    private void initMotors(List<DcMotorEx> motors) {
        motors.add(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class, "frontRight"));
        motors.add(hardwareMap.get(DcMotorEx.class, "backLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class, "backRight"));

        /*
            Nu am setat aici modul si target-ul motoarelor cum miai zis tu
            in tema pentru ca mi se parea mai logic sa le pun in run motors
        */
        motors.forEach(motor -> motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

    private void moveForward(List<DcMotorEx> motors, int addTicks, double power) {
        motors.forEach(motor -> {
            motor.setTargetPosition(motor.getCurrentPosition() + addTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        });

        while (Math.abs(motors.get(0).getCurrentPosition() - motors.get(0).getTargetPosition()) > forwardThreshold && !isStopRequested())
            motors.forEach(motor -> motor.setPower(power));
    }

    private void rotateRobot(IMU imu, List<DcMotorEx> motors, double targetDegrees) {
        // Dai tune cu ce valoare de Kp vrea inima ta :)
        PController pController = new PController(0.1);

        motors.forEach(motor -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        double currentDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = (targetDegrees - currentDegrees + 180) % 360 - 180;

        // Cat timp eroarea nu se incadreaza in 0.5 grade robotul se va roti
        while (Math.abs(error) > 0.5 && !isStopRequested()) {
            currentDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double power = pController.calculatePower(error);

            motors.get(0).setPower(power);
            motors.get(1).setPower(-power);
            motors.get(2).setPower(power);
            motors.get(3).setPower(-power);

            error = (targetDegrees - currentDegrees + 180) % 360 - 180;
        }

        motors.forEach(motor -> motor.setPower(0));
    }
}