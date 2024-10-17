package org.firstinspires.ftc.teamcode.SideProjects.LimelightDetectionTest;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Limelight Test Code")
public class Auto extends LinearOpMode {

    IMU imu;

    DriveSubsystem drive;
    Limelight limelight;

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        limelight = new Limelight(hardwareMap, imu);
        drive = new DriveSubsystem(hardwareMap, imu);

        waitForStart();

        while (opModeIsActive()) {
            if (limelight.isSampleDetected()) {
                telemetry.addData("Sample", "found");

                while (!limelight.isSampleAligned()) {
                    telemetry.addData("Sample", "aligning with sample");

                    // grade
                    double rotation = limelight.getSampleHorizontalAngle();

                    drive.rotate(rotation);
                }

                while (!limelight.isSampleClose()) {
                    telemetry.addData("Sample", "moving towards sample");

                    double distance = limelight.getSampleDistance();

                    drive.moveForward((int)distance);
                }

                if (limelight.isSampleClose() && limelight.isSampleAligned()) {
                    telemetry.addData("Sample", "sample can be picked up");
                    // Ridica sample-ul
                }
            }
            else {
                telemetry.addData("Sample", "not found");
            }

            telemetry.update();
        }

        limelight.shutDown();
    }
}
