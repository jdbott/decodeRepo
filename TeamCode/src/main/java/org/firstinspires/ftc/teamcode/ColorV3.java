package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorV3 {

    RevColorSensorV3 colorSensor;

    public ColorV3(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");
    }

    public String proximityAndColor() {
        double proximity = proximity();

        if (proximity <= 1) { // Adjust threshold based on testing
            return sampleColor();
        } else {
            return "Out of range";
        }
    }

    public double proximity() {
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public String sampleColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Saturation threshold to avoid false detection when colors are too dim
        int totalColor = red + green + blue;
        if (totalColor < 300) { // Adjust this threshold based on testing
            return "Unknown";
        }

        // Detect red, yellow, or blue based on RGB dominance
        if (red > green && red > blue) {
            return "Red";
        } else if (blue > red && blue > green) {
            return "Blue";
        } else if (red > 2 * blue && green > 2 * blue) {
            return "Yellow";
        } else {
            return "Unknown"; // If no clear color dominance is found
        }
    }

    public boolean isConnected() {
        return proximity() != 6;
    }

}
