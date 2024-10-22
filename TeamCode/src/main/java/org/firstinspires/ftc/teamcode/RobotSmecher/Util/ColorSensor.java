package org.firstinspires.ftc.teamcode.RobotSmecher.Util;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class ColorSensor {

    public enum Color {
        BLUE,
        RED,
        YELLOW
    }

    // Placeholders for values
    private final ColorHSV yellowLowerLimit = new ColorHSV(50, 100, 100);
    private final ColorHSV yellowUpperLimit = new ColorHSV(55, 100, 100);

    private final ColorHSV redLowerLimit = new ColorHSV(0, 100, 100);
    private final ColorHSV redUpperLimit = new ColorHSV(10, 100, 100);

    private final ColorHSV blueLowerLimit = new ColorHSV(245, 100, 100);
    private final ColorHSV blueUpperLimit = new ColorHSV(255, 100, 100);

    private final RevColorSensorV3 sensor;

    public ColorSensor(RevColorSensorV3 sensor) {
        this.sensor = sensor;
    }

    public Color getSampleColor() {
        ColorHSV color = new ColorHSV(sensor.getNormalizedColors());

        if (color.isValid(yellowLowerLimit, yellowUpperLimit)) return Color.YELLOW;
        if (color.isValid(redLowerLimit, redUpperLimit)) return Color.RED;
        if (color.isValid(blueLowerLimit, blueUpperLimit)) return Color.BLUE;

        // No sample detected => returns null
        return null;
    }
}
