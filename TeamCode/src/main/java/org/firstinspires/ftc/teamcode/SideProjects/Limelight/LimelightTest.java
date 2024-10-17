package org.firstinspires.ftc.teamcode.SideProjects.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Limelight Test")
@Disabled
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        // Updates the Driver Station telemetry faster
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0); // / A limelight may have up to 10 pipelines
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // These two lines are not necessary, but they give BETTER LOCALIZATION ACCURACY
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLStatus status = limelight.getStatus();

            telemetry.addData("Name", "%s",
                    status.getName());

            // Limelight data (probably useful for debugging)
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());

            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            LLResult result = limelight.getLatestResult();
            if (result == null) {
                telemetry.addData("Limelight", "No Targets");
            } else if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2(); // Adding MT2 (MetaTag2) at the end of getBotpose has better localization accuracy
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("MT2 Location", botpose.toString());

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double x = colorTarget.getTargetXDegrees(); // Where it is (left-right)
                    double y = colorTarget.getTargetYDegrees(); // Where it is (up-down)
                    double area = colorTarget.getTargetArea(); // Size (0%-100%)

                    telemetry.addData("Color Target", "x: %d, y: %d, area: %d% of the image",
                            x, y, area);
                }

                /*
                *   List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                *   for (LLResultTypes.FiducialResult fiducial : fiducials) {
                *       fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
                *       fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
                *       fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
                *       fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful)
                *       fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful)
                *    }
                */
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
