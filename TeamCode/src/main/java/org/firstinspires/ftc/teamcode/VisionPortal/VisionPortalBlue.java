package org.firstinspires.ftc.teamcode.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Vision Portal with Blue Detection")
public class VisionPortalBlue extends OpMode {

    VisionPortal visionPortal;
    BlueColorPipeline pipeline;

    @Override
    public void init() {
        pipeline = new BlueColorPipeline();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        telemetry.addData("Status", "Visual Portal initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Number of Blue objects", pipeline.getDetectedBlueObjects());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
