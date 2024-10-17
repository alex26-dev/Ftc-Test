package org.firstinspires.ftc.teamcode.SideProjects;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// CR power: pt OCD-ul meu te rog nu mai pune spatiu intre TeleOp si paranteza :pray: (cred ca si strica codul dar nush)
@TeleOp(name = "Field Centric")
@Disabled
public class FieldCentricTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // CR power: asta e optional dar cred ca v-ar ajuta sa va puneti in config un nume mai sugestiv la motoare        
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
            /* CR Power:
                Nush ce ai facut aici sau de ce, ideea e ca in mod standard
                e bine sa ai strafe-ul si fata-spate pe acelasi joystick.
                Din ce vad aici, ai pus rotatia pe aceeasi parte cu fata-spate si strafe pe celalalt joystick?
                Poate nu inteleg eu bine, dar nu inteleg schimbarea
            */
            double y = -gamepad1.left_stick_y;

            // CR power: ???
            // Schimbat pentru ca asa doreste sefu
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x; // for strafing

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // CR power: Sunt curios daca intelegi de ce exista de fapt acest multiplier (cere explicatii daca nu), si daca da
            // cum te-ai gandi sa ii dai tuning pentru un output cat mai optim
            rotX = rotX * 1.1; // For imperfect strafing


            // CR power: bun, vad ca gm0 te-a ajutat. dar intelegi dc?
            // daca nu, este similar cu faza cu vectorii din lab. vezi imaginea cu sensul rotilor pe site la GoBilda
            double frontLeftPower = rotX + rotY + rx;
            double backLeftPower = rotX - rotY + rx;
            double frontRightPower = rotX - rotY - rx;
            double backRightPower = rotX + rotY - rx;

            /*
                CR power:
                    Ai dat clip la valori, lucru care nu este gresit, dar implementarea (cu denominator) de pe gm0 este mai buna.
                    Dar inainte sa o faci, vreau sa o analizezi si sa explici de ce este de fapt mai buna si care este
                    avantajul ei in comparatie cu metoda asta. Dupa ce o intelegi, vreau sa implementezi ceva asemanator si pe tank drive in loc sa dai clip la valori.
            */ 
            frontLeftMotor.setPower(Range.clip(frontLeftPower, -1, 1));
            backLeftMotor.setPower(Range.clip(backLeftPower, -1, 1));
            frontRightMotor.setPower(Range.clip(frontRightPower, -1, 1));
            backRightMotor.setPower(Range.clip(backRightPower, -1, 1));

            // Reminder: daca nu intelegi cv, da mesaj, nu va mananc
        }
    }
}
