package org.firstinspires.ftc.teamcode.SideProjects.VisionPortal;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BlueColorPipeline implements VisionProcessor {

    private int detectedBlueObjects = 0;
    List<MatOfPoint> contours = new ArrayList<>();

    // Lower and Upper limits for the HSV value of blue
    private final Scalar lowerBlue = new Scalar(100, 150, 70);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    // Matrices for image modifications (Explanations for each in processFrame)
    private final Mat hsvMat = new Mat();
    private final Mat mask = new Mat();
    private final Mat blurredMat = new Mat();
    private final Mat hierarchy = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    // Method called each frame
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the input frame from RGB to HSV and store it in hsvMat
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create a mask using the HSV range for Blue
        Core.inRange(hsvMat, lowerBlue, upperBlue, mask);

        // Blur the mask to reduce noise => better contour detection
        Imgproc.GaussianBlur(mask, blurredMat, new Size(5, 5), 0);

        // List for each contour detected
        Imgproc.findContours(blurredMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedBlueObjects = 0;
        contours.forEach(contour -> {
            // Calculate the size of the contour
            Rect boundingRect = Imgproc.boundingRect(contour);

            // Filter out small objects
            if (boundingRect.area() > 500) {
                detectedBlueObjects++;
            }
        });

        // Returns the frame with added borders around blue objects
        return frame;
    }

    // Makes an OpenCv Rect into an Android Graphics Rect
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = right + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onScreenWidth, int onScreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // (Optional) Draws the bounding rectangle around the detected blue objects for visualisation

        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(2);

        contours.forEach(contour -> {
            // OpenCv Rect
            Rect boundingRect = Imgproc.boundingRect(contour);

            // Filter out small objects
            if (boundingRect.area() > 500) {
                canvas.drawRect(makeGraphicsRect(boundingRect, scaleBmpPxToCanvasPx), rectPaint);
            }
        });
    }

    public int getDetectedBlueObjects() {
        return detectedBlueObjects;
    }
}
