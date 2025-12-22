package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorV3 {

    private final RevColorSensorV3 colorSensor;

    private static final double PROXIMITY_THRESHOLD_IN = 1.5;
    private static final int MIN_TOTAL_COLOR = 150;

    public ColorV3(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");
    }

    /**
     * Returns "Green", "Purple", or "Out of range"
     */
    public String proximityAndColor() {
        String color = sampleColor();

        if (isDominantColor(color) && proximity() < PROXIMITY_THRESHOLD_IN) {
            return color;
        } else {
            return "Out of range";
        }
    }

    public double proximity() {
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Detects only Green or Purple
     */
    public String sampleColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        int total = red + green + blue;
        if (total < MIN_TOTAL_COLOR) {
            return "Unknown";
        }

        double rRatio = red / (double) total;
        double gRatio = green / (double) total;
        double bRatio = blue / (double) total;

        // --- GREEN DETECTION ---
        // Green dominant, but allow some blue; red must stay much lower.
        if (gRatio > 0.38 && gRatio > rRatio + 0.15 && gRatio >= bRatio - 0.08) {
            return "Green";
        }
        // --- PURPLE DETECTION ---
        // Blue dominant, noticeable red, low green.
        if (bRatio > 0.45 && rRatio > 0.15 && gRatio < 0.38) {
            return "Purple";
        }

        return "Unknown";
    }

    private boolean isDominantColor(String color) {
        return color.equals("Green") || color.equals("Purple");
    }

    public boolean isConnected() {
        return proximity() != 6;
    }
}