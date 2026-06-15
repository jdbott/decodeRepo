package org.firstinspires.ftc.teamcode.teles;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.AllianceMirror;
import org.firstinspires.ftc.teamcode.AllianceStore;
import org.firstinspires.ftc.teamcode.hardwareClasses.Feeder;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Hood;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Teleop: Yuvi")
public class TeleOpByYuvi extends LinearOpMode {

    private Hood hood;
    private Feeder feeder;
    private Flywheel flywheel;
    private Intake intake;
    private Turret turret;
    private Follower follower;

    private boolean isRedAlliance = false;

    private static final double BLUE_TARGET_X = 5.0;
    private static final double TARGET_Y = 139.0;

    private static final double TURRET_CENTER_OFFSET_IN = 1.5;
    private static final double TURRET_OFFSET_DEG = 180.0;
    private static final double BASE_TURRET_OFFSET_DEG = 1.0;
    private static final double TURRET_MIN_DEG = -175.0;
    private static final double TURRET_MAX_DEG = 175.0;

    private static final Pose DEFAULT_START_BLUE =
            new Pose(57.8, 111.2, Math.toRadians(-90.0));

    // Distance (in), Hood angle (deg), Flywheel velocity (rad/s)
    // Distances correspond to: close-goal ~37", mid-goal ~70", far-goal ~122"
    private static final double[][] SHOT_TABLE = {
            {37.0,  30.0, 267.0},   // Closest legal shot
            {43.0,  30.0, 267.0},
            {50.0,  37.0, 282.0},   // Transition to mid-range
            {57.0,  37.0, 287.0},
            {63.5,  37.0, 297.0},
            {71.0,  39.0, 312.0},   // Mid field
            {77.0,  40.0, 317.0},
            {82.0,  42.0, 332.0},
            {88.0,  43.0, 337.0},
            {93.0,  44.0, 352.0},
            {99.0,  46.0, 369.0},   // Far mid
            {104.0, 47.0, 379.0},
            {110.0, 48.0, 394.0},
            {122.0, 53.0, 414.5}    // Maximum table range
    };

    // Hardware safety limits
    private static final double HOOD_MIN_DEG = 20.0;
    private static final double HOOD_MAX_DEG = 70.0;
    private static final double FLYWHEEL_MIN_RAD = 200.0;
    private static final double FLYWHEEL_MAX_RAD = 450.0;

    private static final double DISTANCE_FILTER_ALPHA = 0.45;
    private static final double BALL_FLIGHT_TIME_OFFSET = 0.15;
    private static final double BALL_FLIGHT_TIME_PER_INCH = 1.0 / 200.0;

    private double filteredDistance = 0.0;
    private boolean distanceInitialized = false;
    private double targetVelocityRad = 275.0;
    private double hoodAngleDeg = 50.0;

    private double turretTrimDeg = 0.0;
    private static final double TURRET_TRIM_STEP = 0.5;

    // Feed sequence tuned for ~3.3 shots/sec with margin for 10-20 ms loop jitter
    private enum FeedState { IDLE, PREPARE, PUSH, RESET }
    private FeedState feedState = FeedState.IDLE;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private static final double FEED_PREPARE_TIME = 0.05;  // 50 ms: clutch engage + arm shoot
    private static final double FEED_PUSH_TIME    = 0.12;  // 120 ms: intake pushes ring
    private static final double FEED_RESET_TIME   = 0.13;  // 130 ms: clutch out + arm block + stage next

    private final ElapsedTime turretLoopTimer = new ElapsedTime();
    private double lastTurretTargetDeg = 0.0;
    private double lastTurretLoopTime = 0.0;

    // Rapid-fire logic
    private final ElapsedTime triggerHoldTimer = new ElapsedTime();
    private boolean rapidFireActive = false;
    private boolean rapidFireRumbled = false;

    // Trigger hysteresis to prevent dropout during driver fatigue
    private static final double TRIGGER_ON_THRESHOLD  = 0.55;
    private static final double TRIGGER_OFF_THRESHOLD = 0.35;
    private boolean triggerLocked = false;

    // Input debounce
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean intakeEnabled = true;

    // Deadzone to prevent stick drift
    private static final double STICK_DEADZONE = 0.05;

    // Hood deduplication: only hit the servo bus when the angle changes meaningfully
    private double lastCommandedHoodAngle = Double.NaN;
    private static final double HOOD_COMMAND_EPSILON = 0.05;

    @Override
    public void runOpMode() {
        // --- Hardware init with safety net -----------------------------------
        try {
            intake = new Intake(hardwareMap);
            hood = new Hood(hardwareMap);
            feeder = new Feeder(hardwareMap);

            VoltageSensor battery = null;
            if (hardwareMap.voltageSensor != null && hardwareMap.voltageSensor.iterator().hasNext()) {
                battery = hardwareMap.voltageSensor.iterator().next();
            }
            flywheel = new Flywheel(hardwareMap, battery);

            isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);

            follower = Constants.createFollower(hardwareMap);
            Pose startPose = AllianceMirror.mirrorPose(DEFAULT_START_BLUE, isRedAlliance);
            follower.setStartingPose(startPose);
            follower.updatePose();
            follower.setMaxPower(1.0);
            follower.startTeleOpDrive();

            turret = new Turret(hardwareMap, RobotConfig.TURRET_MOTOR, DcMotorSimple.Direction.REVERSE);

            feeder.clutchOut();
            feeder.armBlock();
            intake.setPower(0.0);

            // Prime hood to avoid a first-loop NaN deduplication skip
            hood.setAngle(hoodAngleDeg);
            lastCommandedHoodAngle = hoodAngleDeg;
        } catch (Exception e) {
            telemetry.addData("HARDWARE INIT FAILED", e.getClass().getSimpleName());
            telemetry.addData("Reason", e.getMessage());
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("PS5 Teleop Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Start Pose", formatPose(follower.getPose()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- Seed filtered distance so ballistics is valid on loop 1 -------
        Pose initialPose = follower.getPose();
        if (initialPose != null && !Double.isNaN(initialPose.getX()) && !Double.isNaN(initialPose.getY())) {
            double targetX = AllianceMirror.mirrorX(BLUE_TARGET_X, isRedAlliance);
            double dx = targetX - initialPose.getX();
            double dy = TARGET_Y - initialPose.getY();
            filteredDistance = Math.hypot(dx, dy);
            distanceInitialized = true;
        }

        intake.setPower(1.0);
        flywheel.setTargetVelocity(targetVelocityRad);
        turretLoopTimer.reset();
        lastTurretLoopTime = 0.0;

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Pose pose = follower.getPose();

            // Safety: if odometry returns NaN, skip this cycle but keep running
            if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
                telemetry.addLine("ERR: Invalid pose from follower");
                telemetry.update();
                continue;
            }

            updateDrive(pose);
            updateTurret(pose);       // Maintains filteredDistance
            updateBallistics(pose);   // Consumes filteredDistance; no redundant init here

            handleShootInput();
            handleTurretTrim();
            handleIntakeToggle();

            updateFeedSequence();
            updateIntakePower();      // Single source of truth for intake motor

            // Turret update must follow setTargetState (already done in updateTurret)
            turret.update();

            // Flywheel update
            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            // Hood update: only command servo when angle changes to prevent bus jitter / buzz
            if (Double.isNaN(lastCommandedHoodAngle) ||
                    Math.abs(hoodAngleDeg - lastCommandedHoodAngle) > HOOD_COMMAND_EPSILON) {
                hood.setAngle(hoodAngleDeg);
                lastCommandedHoodAngle = hoodAngleDeg;
            }

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Pose", formatPose(pose));
            telemetry.addData("Goal Dist", "%.1f in", filteredDistance);
            telemetry.addData("Hood", "%.1f deg", hoodAngleDeg);
            telemetry.addData("Flywheel", "%.1f rad/s", targetVelocityRad);
            telemetry.addData("Turret Trim", "%.2f deg", turretTrimDeg);
            telemetry.addData("Feed", feedState);
            telemetry.addData("Rapid Fire", rapidFireActive);
            telemetry.addData("Intake", intakeEnabled ? "ON" : "OFF");
            telemetry.update();
        }

        // --- Clean shutdown --------------------------------------------------
        shutdownHardware();
    }

    private void shutdownHardware() {
        try { if (flywheel != null) flywheel.stop(); } catch (Exception ignored) {}
        try { if (intake != null) intake.setPower(0.0); } catch (Exception ignored) {}
        try {
            if (feeder != null) {
                feeder.clutchOut();
                feeder.armBlock();
            }
        } catch (Exception ignored) {}
        try { if (follower != null) follower.startTeleOpDrive(); } catch (Exception ignored) {}
    }

    // =========================================================
    // DRIVE — Field-centric with deadzone
    // =========================================================
    private void updateDrive(Pose pose) {
        // L2 (Left Trigger) reduces power for precision shots
        double speedMod = 1.0 - (0.7 * gamepad1.left_trigger);
        speedMod = Range.clip(speedMod, 0.25, 1.0);

        // Read sticks and apply deadzone to prevent drift
        double forward = -applyDeadzone(gamepad1.left_stick_y);
        double strafe  =  applyDeadzone(gamepad1.left_stick_x);
        double turn    = -applyDeadzone(gamepad1.right_stick_x);

        // Scale by speed modifier
        forward *= speedMod;
        strafe  *= speedMod;
        turn    *= speedMod;

        // Field-centric: flip translation for red alliance
        if (isRedAlliance) {
            forward = -forward;
            strafe  = -strafe;
        }

        follower.setTeleOpDrive(forward, strafe, turn, false);
    }

    /** Zeroes small stick inputs and rescales the rest to preserve full range. */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < STICK_DEADZONE) {
            return 0.0;
        }
        return value > 0
                ? (value - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)
                : (value + STICK_DEADZONE) / (1.0 - STICK_DEADZONE);
    }

    // =========================================================
    // TURRET — Always tracking goal with shot-on-move comp
    // =========================================================
    private void updateTurret(Pose pose) {
        double targetX = AllianceMirror.mirrorX(BLUE_TARGET_X, isRedAlliance);

        // Pedro Pathing heading is continuous; use directly without redundant wrap/reconvert
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        // Turret is offset behind robot center
        double turretX = pose.getX() - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = pose.getY() - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        double dx = targetX - turretX;
        double dy = TARGET_Y - turretY;
        double rawDistance = Math.hypot(dx, dy);

        // Predict how long the ball is in the air
        double flightTime = BALL_FLIGHT_TIME_OFFSET + rawDistance * BALL_FLIGHT_TIME_PER_INCH;

        // Get robot velocity from odometry with null-safety
        double fieldVx = 0.0;
        double fieldVy = 0.0;
        double robotOmega = 0.0;

        try {
            fieldVx = follower.getVelocity().getXComponent();
            fieldVy = follower.getVelocity().getYComponent();
        } catch (Exception ignored) {}
        try {
            robotOmega = follower.getAngularVelocity();
        } catch (Exception ignored) {}

        // Velocity of turret center due to robot rotation
        double rx = turretX - pose.getX();
        double ry = turretY - pose.getY();
        double rotVx = -robotOmega * ry;
        double rotVy =  robotOmega * rx;

        double turretVx = fieldVx + rotVx;
        double turretVy = fieldVy + rotVy;

        // Compensate aim point so the ball hits the goal while moving
        double compTargetX = targetX - turretVx * flightTime;
        double compTargetY = TARGET_Y - turretVy * flightTime;

        double compDx = compTargetX - turretX;
        double compDy = compTargetY - turretY;

        double angleToGoalField = Math.toDegrees(Math.atan2(compDy, compDx));
        double angleToGoalRobot = normalize180(angleToGoalField - robotHeadingDeg);

        double tunableOffset = AllianceMirror.maybeMirrorTunableTurretOffset(turretTrimDeg, isRedAlliance);
        double baseOffset    = AllianceMirror.maybeMirrorBaseTurretOffset(BASE_TURRET_OFFSET_DEG, isRedAlliance);

        double desiredTurretDeg = normalize180(
                angleToGoalRobot + TURRET_OFFSET_DEG + tunableOffset + baseOffset
        );

        double currentTurretDeg = turret.getCurrentAngle();
        double safeTurretDeg = wrapIntoTurretWindow(desiredTurretDeg, currentTurretDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);

        // Feed-forward velocity for smooth tracking
        double now = turretLoopTimer.seconds();
        double dt = now - lastTurretLoopTime;
        double desiredTurretVel = 0.0;

        // lastTurretLoopTime == 0.0 on the first iteration: skip to avoid a velocity spike
        if (lastTurretLoopTime > 0 && dt > 1e-4 && dt < 0.5) {
            double delta = normalize180(safeTurretDeg - lastTurretTargetDeg);
            desiredTurretVel = delta / dt;
        }
        lastTurretTargetDeg = safeTurretDeg;
        lastTurretLoopTime = now;

        turret.setTargetState(safeTurretDeg, desiredTurretVel);

        // Smooth distance for ballistics (prevents hood servo jitter)
        double predictedDistance = Math.hypot(compDx, compDy);
        if (!distanceInitialized) {
            filteredDistance = predictedDistance;
            distanceInitialized = true;
        } else {
            filteredDistance = DISTANCE_FILTER_ALPHA * predictedDistance
                    + (1.0 - DISTANCE_FILTER_ALPHA) * filteredDistance;
        }
    }

    // =========================================================
    // BALLISTICS — Hood & flywheel from distance lookup
    // =========================================================
    private void updateBallistics(Pose pose) {
        // filteredDistance is owned and initialized by updateTurret()
        double distance = filteredDistance;

        // Defensive: do not overwrite good values with NaN
        if (Double.isNaN(distance)) {
            return;
        }

        // Clamp to table bounds so the loop below is guaranteed to find an interval
        double minTable = SHOT_TABLE[0][0];
        double maxTable = SHOT_TABLE[SHOT_TABLE.length - 1][0];
        distance = Range.clip(distance, minTable, maxTable);

        // Guaranteed-coverage lookup: because distance is clipped to [min, max],
        // the loop will always land in a valid interval before it runs out.
        for (int i = 0; i < SHOT_TABLE.length - 1; i++) {
            if (distance <= SHOT_TABLE[i + 1][0]) {
                double d1 = SHOT_TABLE[i][0];
                double a1 = SHOT_TABLE[i][1];
                double v1 = SHOT_TABLE[i][2];

                double d2 = SHOT_TABLE[i + 1][0];
                double a2 = SHOT_TABLE[i + 1][1];
                double v2 = SHOT_TABLE[i + 1][2];

                double t = (distance - d1) / (d2 - d1);
                hoodAngleDeg = a1 + t * (a2 - a1);
                targetVelocityRad = v1 + t * (v2 - v1);
                break;
            }
        }

        // Clamp outputs to safe hardware bounds
        hoodAngleDeg = Range.clip(hoodAngleDeg, HOOD_MIN_DEG, HOOD_MAX_DEG);
        targetVelocityRad = Range.clip(targetVelocityRad, FLYWHEEL_MIN_RAD, FLYWHEEL_MAX_RAD);
    }

    // =========================================================
    // SHOOT — R2 (Right Trigger)
    // Tap = single shot. Hold > 500 ms = rapid fire (~3.3/sec).
    // Hysteresis prevents rapid-fire dropout if trigger pressure fluctuates.
    // =========================================================
    private void handleShootInput() {
        double triggerValue = gamepad1.right_trigger;

        if (!triggerLocked && triggerValue > TRIGGER_ON_THRESHOLD) {
            triggerLocked = true;
            startShot();
            triggerHoldTimer.reset();
            rapidFireActive = false;
            rapidFireRumbled = false;
        } else if (triggerLocked) {
            if (!rapidFireActive && triggerHoldTimer.milliseconds() > 500) {
                rapidFireActive = true;
                // Rumble exactly once per activation to avoid spamming the DS thread
                if (!rapidFireRumbled) {
                    try {
                        gamepad1.rumble(150);
                    } catch (Exception ignored) {}
                    rapidFireRumbled = true;
                }
            }
            if (feedState == FeedState.IDLE && rapidFireActive) {
                startShot();
            }
            if (triggerValue < TRIGGER_OFF_THRESHOLD) {
                triggerLocked = false;
                rapidFireActive = false;
            }
        }
    }

    private void startShot() {
        if (feedState == FeedState.IDLE) {
            feedState = FeedState.PREPARE;
            feedTimer.reset();
            // WARNING: feeder helper methods must be non-blocking (no Thread.sleep)
            feeder.armShoot();
            feeder.clutchIn();
        }
    }

    private void updateFeedSequence() {
        switch (feedState) {
            case IDLE:
                break;

            case PREPARE:
                if (feedTimer.seconds() >= FEED_PREPARE_TIME) {
                    feedTimer.reset();
                    feedState = FeedState.PUSH;
                }
                break;

            case PUSH:
                if (feedTimer.seconds() >= FEED_PUSH_TIME) {
                    // WARNING: feeder helper methods must be non-blocking (no Thread.sleep)
                    feeder.clutchOut();
                    feeder.armBlock();
                    feedTimer.reset();
                    feedState = FeedState.RESET;
                }
                break;

            case RESET:
                if (feedTimer.seconds() >= FEED_RESET_TIME) {
                    feedTimer.reset();
                    feedState = FeedState.IDLE;
                    if (rapidFireActive && triggerLocked) {
                        startShot();
                    }
                }
                break;
        }
    }

    // =========================================================
    // INTAKE — Single source of truth for intake motor power
    // =========================================================
    private void updateIntakePower() {
        if (!intakeEnabled) {
            intake.setPower(0.0);
            return;
        }

        switch (feedState) {
            case IDLE:
                intake.setPower(1.0);
                break;
            case PREPARE:
                // Pause intake momentarily so the ring doesn't jam against the engaging clutch
                intake.setPower(0.0);
                break;
            case PUSH:
            case RESET:
                // Run intake to push the current ring and stage the next one
                intake.setPower(1.0);
                break;
        }
    }

    // =========================================================
    // INPUTS — Turret trim & intake toggle
    // =========================================================
    private void handleTurretTrim() {
        boolean left  = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (left && !lastDpadLeft)  turretTrimDeg += TURRET_TRIM_STEP;
        if (right && !lastDpadRight) turretTrimDeg -= TURRET_TRIM_STEP;

        lastDpadLeft = left;
        lastDpadRight = right;
    }

    private void handleIntakeToggle() {
        boolean pressed = gamepad1.x; // Cross button on PS5
        if (pressed && !lastX) {
            intakeEnabled = !intakeEnabled;
        }
        lastX = pressed;
    }

    // =========================================================
    // UTILITIES
    // =========================================================
    private double normalize180(double angle) {
        while (angle > 180.0)  angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double wrapIntoTurretWindow(double desiredDeg, double referenceDeg,
                                        double minDeg, double maxDeg) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = desiredDeg + 360.0 * k;
            if (candidate >= minDeg && candidate <= maxDeg) {
                if (Double.isNaN(best) ||
                        Math.abs(candidate - referenceDeg) < Math.abs(best - referenceDeg)) {
                    best = candidate;
                }
            }
        }
        if (Double.isNaN(best)) {
            best = Range.clip(desiredDeg, minDeg, maxDeg);
        }
        return best;
    }

    private String formatPose(Pose pose) {
        if (pose == null) return "null";
        return String.format("X:%.1f Y:%.1f H:%.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}