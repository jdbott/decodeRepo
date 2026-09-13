package org.firstinspires.ftc.teamcode.alliance;

import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Blue-native → red pose mirroring. Author every pose for BLUE and create it through
 * {@link #poses(boolean)}; on red the factory reflects it across the field centerline (x = 72).
 *
 * <p>Two selectable implementations ({@link #MODE} is editable live from FTC Dashboard, or pass a
 * {@link Mode} explicitly):
 * <ul>
 *   <li>{@link Mode#CUSTOM} (default) — true reflection: x → 144 − x, heading → 180° − heading.</li>
 *   <li>{@link Mode#BUILT_IN} — Pedro's {@code PoseFactory.mirrorX(72)}: same x, but heading → −heading,
 *       which points red-side robots the wrong way on DECODE's left/right mirrored field.</li>
 * </ul>
 */
@Config
public final class AllianceMirror {

    public enum Mode { CUSTOM, BUILT_IN }

    public static Mode MODE = Mode.CUSTOM;

    public static final double FIELD_WIDTH_IN = 144.0;

    // Whether turret offsets flip sign on red
    public static final boolean NEGATE_BASE_TURRET_OFFSET_ON_RED = true;
    public static final boolean NEGATE_TUNABLE_TURRET_OFFSET_ON_RED = false;

    private AllianceMirror() {}

    public static boolean isRed(Context context) {
        return AllianceStore.isRed(context);
    }

    /** Degree-heading factory for blue-native poses, mirrored on red using {@link #MODE}. */
    public static PoseFactory poses(boolean isRed) {
        return poses(isRed, MODE);
    }

    public static PoseFactory poses(boolean isRed, Mode mode) {
        return mirrored(PoseFactory.degrees(), isRed, mode);
    }

    /** Mirrors an already-built blue pose (heading in radians). */
    public static Pose mirrorPose(Pose bluePose, boolean isRed) {
        return mirrorPose(bluePose, isRed, MODE);
    }

    public static Pose mirrorPose(Pose bluePose, boolean isRed, Mode mode) {
        return mirrored(PoseFactory.radians(), isRed, mode).of(bluePose.x(), bluePose.y(), bluePose.heading());
    }

    public static double mirrorX(double x, boolean isRed) {
        return isRed ? FIELD_WIDTH_IN - x : x;
    }

    /** Mirrors a raw heading the same way {@link #MODE} mirrors poses. */
    public static double mirrorHeadingDeg(double headingDeg, boolean isRed) {
        return mirrorHeadingDeg(headingDeg, isRed, MODE);
    }

    public static double mirrorHeadingDeg(double headingDeg, boolean isRed, Mode mode) {
        if (!isRed) return normalize180(headingDeg);
        return normalize180(mode == Mode.BUILT_IN ? -headingDeg : 180.0 - headingDeg);
    }

    public static double maybeMirrorBaseTurretOffset(double offsetDeg, boolean isRed) {
        return isRed && NEGATE_BASE_TURRET_OFFSET_ON_RED ? -offsetDeg : offsetDeg;
    }

    public static double maybeMirrorTunableTurretOffset(double offsetDeg, boolean isRed) {
        return isRed && NEGATE_TUNABLE_TURRET_OFFSET_ON_RED ? -offsetDeg : offsetDeg;
    }

    public static double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    private static PoseFactory mirrored(PoseFactory factory, boolean isRed, Mode mode) {
        if (!isRed) return factory;
        return mode == Mode.BUILT_IN
                ? factory.mirrorX(FIELD_WIDTH_IN / 2)
                : factory.map(AllianceMirror::reflect);
    }

    private static Pose reflect(Pose pose) {
        return new Pose(FIELD_WIDTH_IN - pose.x(), pose.y(), Math.PI - pose.heading());
    }
}
