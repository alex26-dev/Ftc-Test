package org.firstinspires.ftc.teamcode.PIDControllerAuto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "PID Test")
public class Auto extends LinearOpMode {

    // Daca nu mai stii care era toata tema am pus un comment jos de tot

    List<DcMotorEx> motors = new ArrayList<>();

    @Override
    public void runOpMode() {
        initMotors(motors);

        // Mers #1
        runMotors(motors, 5000, 1);

        // Rotatie de 90 de grade
        rotateRobot(motors, 90);

        // Mers #2
        runMotors(motors, 5000, 1);

        // Tema optionala cu tickuri in grade / cm
        final int ticks = 69420;
        final double wheelRadiusCm = 5; // Pui ce raza are roata in cm
        telemetry.addData(ticks + " tickuri", ticksToCm(ticks, wheelRadiusCm) + " cm");
        telemetry.update();

        // Daca o rotatie are 360 de tickuri nu inseamna ca 1 tick = 1 grad?
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
    }

    private void runMotors(List<DcMotorEx> motors, int ticks, double power) {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }

    private void rotateRobot(List<DcMotorEx> motors, double targetDegrees) {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        // Dai tune cu ce valoare de Kp vrea inima ta :)
        PController pController = new PController(0.1);

        motors.forEach(motor -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        double currentDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        targetDegrees += currentDegrees;
        double error = targetDegrees - currentDegrees;

        // Cat timp eroarea nu se incadreaza in 0.5 grade robotul se va roti
        while (Math.abs(error) > 0.5) {
            currentDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double power = pController.calculatePower(currentDegrees, targetDegrees);

            motors.get(0).setPower(power);
            motors.get(1).setPower(-power);
            motors.get(2).setPower(power);
            motors.get(3).setPower(-power);

            error = targetDegrees - currentDegrees;
        }

        motors.forEach(motor -> motor.setPower(0));
    }

    private double ticksToCm(int ticks, double wheelRadiusCm) {
        // Formula de la mate pe care credeam ca nu o sa o mai folosesc in viata mea dar here we are
        double wheelPerimeterCm = 2 * Math.PI * wheelRadiusCm;

        // Am citit pe net ca un encoder are 360 de tickuri per rotatie sper ca e corect
        final int ticksPerRotation = 360;

        // Cm pentru un singur tick
        double cmPerTick = wheelPerimeterCm / ticksPerRotation;

        return cmPerTick * ticks;
    }
}

/*
    5000 tickuri in fata
    cu un P controller -> rotatie 90 grade cu un gyroscope
    5000 tickuri in fata a doua oara

    *optional: transforma tickuri in:
        - grade
        - cm

    mini tema:
        - fa 2 functii: una initMotors si una runMotors.
        - ambele functii primesc ca si parametru o lista de motoare
        - functiile nu returneaza nimic, doar configureaza motoarele

    Ex:
        - dai initMotors(frontLeftMotor, backLeftMotor, ...) si iti reseteaza encoder-ul si seteaza target-ul
        - dai runMotors(frontLeftMotor, ...) si iti da RUN_TO_POSITION si puterea 1
 */