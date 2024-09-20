package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Field Centric")
public class FieldCentricTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.left_stick_x; // for strafing

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // For imperfect strafing

            double frontLeftPower = rotX + rotY + rx;
            double backLeftPower = rotX - rotY + rx;
            double frontRightPower = rotX - rotY - rx;
            double backRightPower = rotX + rotY - rx;

            frontLeftMotor.setPower(Range.clip(frontLeftPower, -1, 1));
            backLeftMotor.setPower(Range.clip(backLeftPower, -1, 1));
            frontRightMotor.setPower(Range.clip(frontRightPower, -1, 1));
            backRightMotor.setPower(Range.clip(backRightPower, -1, 1));
        }
    }
}
