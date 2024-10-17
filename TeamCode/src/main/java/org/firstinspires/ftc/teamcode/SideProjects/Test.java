package org.firstinspires.ftc.teamcode.SideProjects;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// CR power: din nou, salveaza-ma la OCD
@Autonomous(name = "tema1")
@Disabled
public class Test extends LinearOpMode {
    /*
     CR power, mini tema:
        - fa 2 functii: una initMotors si una runMotors.
        - ambele functii primesc ca si parametru o lista de motoare
        - functiile nu returneaza nimic, doar configureaza motoarele

        Ex: 
            - dai initMotors(frontLeftMotor, backLeftMotor, ...) si iti reseteaza encoder-ul si seteaza target-ul
            - dai runMotors(frontLeftMotor, ...) si iti da RUN_TO_POSITION si puterea 1
    */

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setTargetPosition(5000);
        backLeftMotor.setTargetPosition(5000);
        frontRightMotor.setTargetPosition(5000);
        backRightMotor.setTargetPosition(5000);
        
        waitForStart();

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /*
        CR power:
            Ii dai drumul prea devreme. Din ce pare sa vad cred ca robotul ar pleca in INIT
            cu full-speed, fara sa se opreasca la target pt ca inca e pe STOP_AND_RESET_ENCODER.
            Corect ar fi sa le muti pe astea 4 dupa waitForStart si RUN_TO_POSITION.
        */

        // (Modificat sa mearga)
        frontLeftMotor.setPower(1);
        backLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backRightMotor.setPower(1);
    }
}
