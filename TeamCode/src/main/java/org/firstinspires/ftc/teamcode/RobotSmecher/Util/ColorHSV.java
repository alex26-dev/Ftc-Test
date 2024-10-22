package org.firstinspires.ftc.teamcode.RobotSmecher.Util;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorHSV {

    private final float[] HSV = new float[3];

    private int toRange(double value) {
        return (int)Math.round(value * 255.0);
    }

    public ColorHSV(NormalizedRGBA color) {
        Color.RGBToHSV(
                toRange(color.red),
                toRange(color.blue),
                toRange(color.green),
                HSV
        );
    }

    public ColorHSV(float H, float S, float V) {
        this.H = H;
        this.S = S;
        this.V = V;
    }

    public boolean isValid(ColorHSV lowerLimit, ColorHSV upperLimit) {
        return (this.H >= lowerLimit.H && this.S >= lowerLimit.S && this.V == lowerLimit.V && this.H <= upperLimit.H && this.S <= upperLimit.S && this.V <= upperLimit.V);
    }

    public float H = HSV[0];
    public float S = HSV[1];
    public float V = HSV[2];
}
