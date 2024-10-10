package org.firstinspires.ftc.teamcode.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp (name = "Otos Test")
public class OtosTeleOp extends LinearOpMode {

    List<DcMotorEx> motors = new ArrayList<>();
    SparkFunOTOS otos;

    @Override
    public void runOpMode() {
        configureOTOS();

        initMotors();

        waitForStart();

        while (opModeIsActive()) {
            runMotors();

            SparkFunOTOS.Pose2D pos = otos.getPosition();

            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            telemetry.update();
        }
    }

    private void initMotors() {
        motors.add(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class, "frontRight"));
        motors.add(hardwareMap.get(DcMotorEx.class, "backLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class, "backRight"));

        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);

        motors.forEach(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    private void configureOTOS() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        otos = hardwareMap.get(SparkFunOTOS.class, "OTOS");

        // Aici se modifica unitatile de masura
        otos.setLinearUnit(DistanceUnit.CM);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Daca senzorul nu e centrat cu robotul atunci se modifica offset-ul catre centrul robotului
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.update();
    }

    private void runMotors() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(strafe), 1);
        double frontLeftPower = (y + x + strafe) / denominator;
        double backLeftPower = (y - x + strafe) / denominator;
        double frontRightPower = (y - x - strafe) / denominator;
        double backRightPower = (y + x - strafe) / denominator;

        motors.get(0).setPower(frontLeftPower);
        motors.get(1).setPower(frontRightPower);
        motors.get(2).setPower(backLeftPower);
        motors.get(3).setPower(backRightPower);
    }
}
