//IMPORTANT
//What you must configure before running:
//Limelight Web UI → AprilTag pipeline → enable "Full 3D"
//Robot-Space Pose → enter your camera's forward/right/up offsets and portrait roll (±90°)
//Field Map → upload the current season's .fmap
//TICKS_PER_METER_X / Y → calibrate for your specific odometry pods (the default 74473.0 is an example for GoBilda 35mm + REV Through Bore)
//The code handles your pod positions at (5, 136) mm and (136, 136) mm automatically (MIGHT HAVE TO CHANGE). If you need to tune fusion aggressiveness, adjust the singleTagPositionGain, multiTagPositionGain, and headingGain fields directly.

package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * All-in-one robot localizer for FTC.
 * Fuses two-wheel dead-wheel odometry with Limelight 3A MegaTag2 vision.
 *
 * HARDWARE REQUIREMENTS:
 * - Limelight 3A configured with robot-space pose and .fmap uploaded via web UI
 * - Two dead-wheel pods: X-pod at (5, 136) mm, Y-pod at (136, 136) mm from robot center
 * - Control Hub IMU (or equivalent)
 *
 * COORDINATE SYSTEM:
 * - Robot: +X forward, +Y left, +heading CCW
 * - Field: Origin at center, 0 deg = facing Red alliance wall (FTC standard)
 */
public class offLimelightLocalization {

    // Hardware
    private final Limelight3A limelight;
    private final DcMotorEx xPodEncoder;
    private final DcMotorEx yPodEncoder;
    private final IMU imu;

    // ============ CONFIGURATION CONSTANTS ============
    // Encoder resolution (ticks per meter). CALIBRATE THESE FOR YOUR PODS.
    // Example: GoBilda 35mm omni (circ ≈ 0.110 m) + REV Through Bore (8192 PPR)
    //          8192 / 0.110 ≈ 74,473 ticks/meter. ADJUST TO YOUR HARDWARE.
    public static final double TICKS_PER_METER_X = 74473.0;
    public static final double TICKS_PER_METER_Y = 74473.0;

    // Pod positions in ROBOT frame (meters).
    // User specified: X-pod at (5 mm, 136 mm), Y-pod at (136 mm, 136 mm)
    private static final double X_POD_X = 0.005;
    private static final double X_POD_Y = 0.136;
    private static final double Y_POD_X = 0.136;
    private static final double Y_POD_Y = 0.136;

    // Vision rejection thresholds
    public double maxVisionStalenessMs = 150.0;
    public double maxAngularVelocityDegPerSec = 360.0;
    public double maxPositionJumpMeters = 0.5;
    public double maxHeadingJumpDegrees = 15.0;

    // Fusion gains (0.0 = trust odometry only, 1.0 = snap fully to vision)
    public double singleTagPositionGain = 0.15;
    public double multiTagPositionGain = 0.40;
    public double headingGain = 0.10;
    // ==================================================

    // Pose state (meters, radians)
    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0; // CCW positive

    // Previous values
    private int lastXTicks = 0;
    private int lastYTicks = 0;
    private double lastRawHeadingRad = 0.0;
    private double lastImuHeadingDeg = 0.0;
    private long lastTimeMs = 0;
    private boolean firstRun = true;

    public offLimelightLocalization(HardwareMap hwMap, String limelightName,
                          String xPodName, String yPodName, String imuName) {
        limelight = hwMap.get(Limelight3A.class, limelightName);
        xPodEncoder = hwMap.get(DcMotorEx.class, xPodName);
        yPodEncoder = hwMap.get(DcMotorEx.class, yPodName);
        imu = hwMap.get(IMU.class, imuName);

        limelight.setPollRateHz(100);
        limelight.start();
    }

    /** Set initial known pose (e.g., at match start). */
    public void setPose(double xMeters, double yMeters, double headingRadians) {
        this.x = xMeters;
        this.y = yMeters;
        this.heading = headingRadians;
        normalizeHeading();
    }

    /** Main update loop. Call once per OpMode loop(). */
    public void update() {
        // 1. Read raw IMU
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double currentImuHeadingDeg = angles.getYaw(AngleUnit.DEGREES);
        double currentImuHeadingRad = angles.getYaw(AngleUnit.RADIANS);

        // 2. Compute angular velocity for motion-blur rejection
        long now = System.currentTimeMillis();
        double angularVelDegPerSec = 0.0;
        if (!firstRun && now > lastTimeMs) {
            double dt = (now - lastTimeMs) / 1000.0;
            angularVelDegPerSec = (currentImuHeadingDeg - lastImuHeadingDeg) / dt;
            while (angularVelDegPerSec > 180.0) angularVelDegPerSec -= 360.0;
            while (angularVelDegPerSec < -180.0) angularVelDegPerSec += 360.0;
        }
        lastImuHeadingDeg = currentImuHeadingDeg;
        lastTimeMs = now;

        // 3. Update dead-wheel odometry
        updateOdometry(currentImuHeadingRad);

        // 4. Send heading to Limelight for MegaTag2
        limelight.updateRobotOrientation(currentImuHeadingDeg);

        // 5. Acquire and validate vision
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;
        if (result.getStaleness() > maxVisionStalenessMs) return;
        if (Math.abs(angularVelDegPerSec) > maxAngularVelocityDegPerSec) return;

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int tagCount = (fiducials == null) ? 0 : fiducials.size();
        if (tagCount < 1) return;

        double vx = botpose.getPosition().x;
        double vy = botpose.getPosition().y;
        double vYawRad = Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES));

        // 6. Outlier rejection
        double dx = vx - x;
        double dy = vy - y;
        double posJump = Math.hypot(dx, dy);

        double headingDiff = vYawRad - heading;
        while (headingDiff > Math.PI) headingDiff -= 2 * Math.PI;
        while (headingDiff < -Math.PI) headingDiff += 2 * Math.PI;

        if (posJump > maxPositionJumpMeters || Math.abs(Math.toDegrees(headingDiff)) > maxHeadingJumpDegrees) {
            return; // Vision outlier rejected
        }

        // 7. Weighted fusion: trust multi-tag more than single-tag
        double posGain = (tagCount >= 2) ? multiTagPositionGain : singleTagPositionGain;
        x += posGain * dx;
        y += posGain * dy;
        heading += headingGain * headingDiff;
        normalizeHeading();
    }

    private void updateOdometry(double currentRawHeadingRad) {
        int currXTicks = xPodEncoder.getCurrentPosition();
        int currYTicks = yPodEncoder.getCurrentPosition();

        if (firstRun) {
            lastXTicks = currXTicks;
            lastYTicks = currYTicks;
            lastRawHeadingRad = currentRawHeadingRad;
            heading = currentRawHeadingRad;
            firstRun = false;
            return;
        }

        int dXticks = currXTicks - lastXTicks;
        int dYticks = currYTicks - lastYTicks;

        // Raw IMU delta (not fused heading) to avoid vision correction artifacts
        double dHeading = currentRawHeadingRad - lastRawHeadingRad;
        while (dHeading > Math.PI) dHeading -= 2 * Math.PI;
        while (dHeading < -Math.PI) dHeading += 2 * Math.PI;

        double dxRaw = dXticks / TICKS_PER_METER_X;
        double dyRaw = dYticks / TICKS_PER_METER_Y;

        // Compensate for pod offset due to rotation.
        // Forward wheel at (x,y) picks up +y * dHeading in its measurement
        // Strafe wheel at (x,y) picks up -x * dHeading in its measurement
        double dxRobot = dxRaw + (X_POD_Y * dHeading);
        double dyRobot = dyRaw - (Y_POD_X * dHeading);

        // Convert robot-frame deltas to field-frame deltas
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double dFieldX = dxRobot * cosH - dyRobot * sinH;
        double dFieldY = dxRobot * sinH + dyRobot * cosH;

        x += dFieldX;
        y += dFieldY;
        heading += dHeading;
        normalizeHeading();

        lastXTicks = currXTicks;
        lastYTicks = currYTicks;
        lastRawHeadingRad = currentRawHeadingRad;
    }

    private void normalizeHeading() {
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; } // radians
    public double getHeadingDegrees() { return Math.toDegrees(heading); }

    public void stop() {
        limelight.stop();
    }
}