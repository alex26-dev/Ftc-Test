package org.firstinspires.ftc.teamcode.SideProjects.LimelightDetectionTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Limelight {
    // La distanta asta fata de sample se va opri robotul (masurata in mm)
    private final double MIN_DISTANCE = 100;

    // Unghiul minim al sample-ului orizontal fata de centrul camerei (grade)
    private final double MIN_ANGLE = 5;

    // Inaltimea de la camera la podea (in mm)
    private final double HEIGHT = 300;

    private final Limelight3A limelight;
    private final IMU imu;

    LLResult result;

    public Limelight(HardwareMap hardwareMap, IMU imu) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
        limelight.start();
    }

    public double getSampleHorizontalAngle() {
        updateCamera();

        if (!isSampleDetected() && isSampleAligned()) {
            return 0;
        }

        return result.getTx();
    }

    public double getSampleDistance() {
        updateCamera();

        if (!isSampleDetected() || !isSampleAligned()) {
            return 0;
        }

        double angle = result.getTy();

        // Distanta robot -> obiect -spatiul pe care il vom lasa intre ele (in mm)
        return HEIGHT / Math.tan(angle) - MIN_DISTANCE;
    }

    public boolean isSampleDetected() {
        updateCamera();
        return result.isValid();
    }

    public boolean isSampleAligned() {
        updateCamera();
        return (Math.abs(result.getTx()) <= MIN_ANGLE && isSampleDetected());
    }

    public boolean isSampleClose() {
        updateCamera();

        // Daca distanta robot-obiect este mai mica/egala cu distanta minima +-10 mm
        return (Math.abs(getSampleDistance()) <= 10 && isSampleDetected());
    }

    public void updateCamera() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        result = limelight.getLatestResult();
    }

    public void shutDown() {
        limelight.shutdown();
    }
}
