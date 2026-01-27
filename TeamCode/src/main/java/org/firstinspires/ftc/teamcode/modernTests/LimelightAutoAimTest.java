package org.firstinspires.ftc.teamcode.modernTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import java.util.Locale;

@TeleOp(name = "Turret Aim Test (LL + Odom)", group = "TEST")
public class LimelightAutoAimTest extends LinearOpMode {

    private FlywheelASG flywheel;

    // -----------------------------
    // Field target (same as your tele)
    // -----------------------------
    private static final double TARGET_X_IN = 0.0;
    private static final double TARGET_Y_IN = 148.0;

    // Your turret coordinate convention: "backwards is 0" => forward is +180
    private static final double TURRET_OFFSET_DEG = 180.0;

    // Turret hard limits (from your Turret class defaults)
    private static final double TURRET_MIN_DEG = -160.0;
    private static final double TURRET_MAX_DEG = 160.0;

    // -----------------------------
    // Limelight tracking behavior
    // -----------------------------
    // If your tx sign is backwards, flip this to -1.
    private static final double TX_SIGN = +1.0;

    // Proportional step: degrees of turret command per degree of tx error (tx is in degrees).
    // Start conservative; increase if it feels sluggish.
    private static final double TX_KP_DEG_PER_DEG = 0.75;

    // Safety: limit how much you change turret command per loop (prevents violent oscillations)
    private static final double MAX_TURRET_STEP_PER_LOOP_DEG = 10;

    // Rumble when aim is impossible
    private static final int RUMBLE_MS = 250;
    private static final long RUMBLE_COOLDOWN_MS = 700;
    private long lastRumbleMs = 0;

    // -----------------------------
    // Hardware / subsystems
    // -----------------------------
    private Follower follower;
    private final Turret turret = new Turret();

    private IMU imu;
    private Limelight3A limelight;

    // Toggle: prefer LL tag aim when available
    private boolean llAimEnabled = false;
    private boolean dpadUpPrev = false;

    // Heading-rate estimation for turret FF
    private double lastHeadingDeg = 0.0;
    private long lastHeadingMs = 0;

    private double targetVelocityRad = 250;

    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // Follower init (same starting pose as your V2TeleBlue)
        // -----------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25, 85, Math.toRadians(180)));
        follower.updatePose();
        follower.setMaxPower(1);
        follower.startTeleOpDrive();

        // -----------------------------
        // Limelight init (match your sample)
        // -----------------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.start();

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new FlywheelASG(hardwareMap, battery);

        // -----------------------------
        // Turret init (same motor name + direction as your tele)
        // -----------------------------
        turret.init(hardwareMap, "turretMotor", com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        turret.zeroTurret();     // known reference = 0 at start (backwards)
        turret.setAngle(0);

        telemetry.addLine("Initialized. START to begin.");
        telemetry.addLine("GP1: sticks drive (robot-centric)");
        telemetry.addLine("GP1: dpad_up toggles LL AprilTag aim preference");
        telemetry.update();

        while (opModeInInit()) {
            turret.update();
            follower.updatePose();
        }

        waitForStart();
        resetRuntime();

        Pose pose = follower.getPose();
        lastHeadingDeg = Math.toDegrees(pose.getHeading());
        lastHeadingMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            targetVelocityRad += -gamepad1.right_stick_y * 5; // rad/s increment
            targetVelocityRad = Math.max(targetVelocityRad, 0); // no negative

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            follower.update();
            pose = follower.getPose();

            // -----------------------------
            // Robot-centric driving (Pedro)
            // -----------------------------
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false  // false = robot-centric in your codebase
            );

            // -----------------------------
            // Toggle LL aim preference
            // -----------------------------
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpPrev) {
                llAimEnabled = !llAimEnabled;
                gamepad1.rumble(120);
            }
            dpadUpPrev = dpadUpNow;

            // -----------------------------
            // Update Limelight orientation using current robot yaw
            // You asked for follower heading -> degrees; that’s what we do here.
            // -----------------------------
            double yawDegFromFollower = Math.toDegrees(pose.getHeading());
            limelight.updateRobotOrientation(yawDegFromFollower);

            // -----------------------------
            // Turret feedforward to oppose robot rotation (optional but helpful)
            // -----------------------------
            updateTurretFeedforwardFromHeading(pose);

            // -----------------------------
            // Decide aiming source:
            // 1) If LL toggle ON and tag is visible -> aim by TX
            // 2) Else -> aim by odometry to fixed target
            // -----------------------------
            LLResult result = limelight.getLatestResult();
            boolean llHasValid = (result != null) && result.isValid();
            boolean llShouldUse = llAimEnabled && llHasValid;

            double turretCurrentDeg = turret.getCurrentAngle();

            if (llShouldUse) {
                // Aim using tx -> drive tx toward 0
                double txDeg = result.getTx(); // degrees, 0 when perfectly aligned

                // Simple proportional correction:
                // If sign is wrong on your robot, flip TX_SIGN.
                double rawDelta = -TX_SIGN * TX_KP_DEG_PER_DEG * txDeg;
                rawDelta = Range.clip(rawDelta, -MAX_TURRET_STEP_PER_LOOP_DEG, +MAX_TURRET_STEP_PER_LOOP_DEG);

                double desired = turretCurrentDeg + rawDelta;

                if (isWithinTurretLimits(desired)) {
                    turret.setAngle(desired);
                } else {
                    rumbleIfCooldown();
                    // Don’t command out-of-range; hold current target.
                    turret.setAngle(Range.clip(desired, TURRET_MIN_DEG, TURRET_MAX_DEG));
                }

                telemetry.addData("AimSource", "LIMELIGHT (TX)");
                telemetry.addData("tx(deg)", String.format(Locale.US, "%.2f", txDeg));
                telemetry.addData("delta(deg)", String.format(Locale.US, "%.2f", rawDelta));

            } else {
                // Aim using odometry tracking to the fixed field target
                double desiredOdom = computeTurretAngleFromOdom(pose);

                // Choose equivalent within limits closest to current, if possible
                Double safe = pickClosestEquivalentInLimits(desiredOdom, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);

                if (safe != null) {
                    turret.setAngle(safe);
                } else {
                    // No valid equivalent in window: warn driver
                    rumbleIfCooldown();
                    // Hold position (do not jump into a hard stop)
                    turret.setAngle(Range.clip(desiredOdom, TURRET_MIN_DEG, TURRET_MAX_DEG));
                }

                telemetry.addData("AimSource", llAimEnabled ? "ODOM (LL LOST)" : "ODOM");
                telemetry.addData("odomDesired(deg)", String.format(Locale.US, "%.2f", desiredOdom));
            }

            turret.update();

            // -----------------------------
            // Telemetry
            // -----------------------------
            telemetry.addData("LL_Enabled", llAimEnabled);
            telemetry.addData("LL_Valid", llHasValid);
            telemetry.addData("Pose(x,y,h)", String.format(Locale.US, "%.1f, %.1f, %.1fdeg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
            telemetry.addData("TurretCur(deg)", String.format(Locale.US, "%.2f", turret.getCurrentAngle()));
            telemetry.addData("TurretTgt(deg)", String.format(Locale.US, "%.2f", turret.getTargetAngle()));
            telemetry.update();
        }
    }

    // -----------------------------
    // Odom-based turret angle solution (same logic as your tele)
    // -----------------------------
    private double computeTurretAngleFromOdom(Pose pose) {
        double botX = pose.getX();
        double botY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        double dx = TARGET_X_IN - botX;
        double dy = TARGET_Y_IN - botY;

        // Field-frame bearing to target
        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Robot-frame bearing
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);

        // Convert to turret coords (0 = backwards)
        return normalize180(angleToTargetRobotDeg + TURRET_OFFSET_DEG);
    }

    private void updateTurretFeedforwardFromHeading(Pose pose) {
        long now = System.currentTimeMillis();
        double headingDeg = Math.toDegrees(pose.getHeading());

        long dtMs = now - lastHeadingMs;
        if (dtMs > 5) {
            double dHeading = angleDeltaDeg(headingDeg, lastHeadingDeg); // signed shortest delta
            double headingRateDegPerSec = (dHeading * 1000.0) / dtMs;

            // Your turret.setFeedforward expects angularVelDegPerSec.
            // Sign: oppose robot rotation, so negative of heading rate is typically correct.
            turret.setFeedforward(-headingRateDegPerSec);

            lastHeadingDeg = headingDeg;
            lastHeadingMs = now;
        }
    }

    // -----------------------------
    // Limit + wrap helpers
    // -----------------------------
    private boolean isWithinTurretLimits(double deg) {
        return deg >= TURRET_MIN_DEG && deg <= TURRET_MAX_DEG;
    }

    /**
     * Tries desiredDeg +/- 360*k and returns the one:
     * - within [minDeg, maxDeg]
     * - closest to referenceDeg
     * Returns null if none are in-range.
     */
    private Double pickClosestEquivalentInLimits(double desiredDeg, double referenceDeg, double minDeg, double maxDeg) {
        Double best = null;
        for (int k = -2; k <= 2; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand >= minDeg && cand <= maxDeg) {
                if (best == null || Math.abs(cand - referenceDeg) < Math.abs(best - referenceDeg)) {
                    best = cand;
                }
            }
        }
        return best;
    }

    private double normalize180(double a) {
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
    }

    private double angleDeltaDeg(double aDeg, double bDeg) {
        // shortest signed delta from b -> a
        return normalize180(aDeg - bDeg);
    }

    private void rumbleIfCooldown() {
        long now = System.currentTimeMillis();
        if (now - lastRumbleMs >= RUMBLE_COOLDOWN_MS) {
            gamepad1.rumble(RUMBLE_MS);
            lastRumbleMs = now;
        }
    }
}