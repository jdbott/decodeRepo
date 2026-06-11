//HELOOOOOOOOOo - UV

package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class AllianceMirror {
    public static final double FIELD_WIDTH_IN = 144.0;

    // Set this depending on whether you want turret offsets mirrored on red
    public static final boolean NEGATE_BASE_TURRET_OFFSET_ON_RED = true;
    public static final boolean NEGATE_TUNABLE_TURRET_OFFSET_ON_RED = false;

    public static boolean isRed(android.content.Context context) {
        return AllianceStore.isRed(context);
    }

    public static double mirrorX(double x, boolean isRed) {
        return isRed ? FIELD_WIDTH_IN - x : x;
    }

    // Reflect heading across the field Y axis.
    // Examples:
    // 0 -> 180
    // 180 -> 0
    // 141 -> 39
    // -90 -> -90
    public static double mirrorHeadingDeg(double headingDeg, boolean isRed) {
        if (!isRed) return normalize180(headingDeg);
        return normalize180(180.0 - headingDeg);
    }

    public static Pose mirrorPose(Pose bluePose, boolean isRed) {
        if (!isRed) return bluePose;

        return new Pose(
                mirrorX(bluePose.getX(), true),
                bluePose.getY(),
                Math.toRadians(mirrorHeadingDeg(Math.toDegrees(bluePose.getHeading()), true))
        );
    }

    public static double maybeMirrorBaseTurretOffset(double offsetDeg, boolean isRed) {
        if (isRed && NEGATE_BASE_TURRET_OFFSET_ON_RED) return -offsetDeg;
        return offsetDeg;
    }

    public static double maybeMirrorTunableTurretOffset(double offsetDeg, boolean isRed) {
        if (isRed && NEGATE_TUNABLE_TURRET_OFFSET_ON_RED) return -offsetDeg;
        return offsetDeg;
    }

    public static double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }
}