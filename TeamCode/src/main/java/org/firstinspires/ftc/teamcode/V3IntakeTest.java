package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@TeleOp(name = "A V3 Tele")
public class V3IntakeTest extends LinearOpMode {

    private Servo hoodServo;
    private Servo armServo;
    private Servo clutchServo;

    private Follower follower;
    private Turret turret;

    private DcMotorEx intakeMotor;

    private FlywheelASG flywheel;

    // Starts in auto shot tracking mode
    private boolean autoShotFromDistance = true;

    // Toggle this to enable rudimentary shooting-on-the-move aim compensation
    private static final boolean ENABLE_SHOT_ON_MOVE_COMP = false;

    // Fixed override shot settings
    private static final double FIXED_POWER_SHOT_VELOCITY_RAD = 420.0;
    private static final double FIXED_POWER_SHOT_HOOD_DEG = 53.0;

    // Light filtering for velocity estimate
    private static final double VELOCITY_FILTER_ALPHA = 0.25;

    // Very light smoothing for predicted shot distance
    private static final double PREDICTED_DISTANCE_ALPHA = 0.45;

    // Intake toggle
    private boolean lastX = false;
    private boolean intakeToggleOn = false;

    // Feed sequence trigger
    private boolean lastB = false;

    // Debounce for flywheel target adjustment
    private boolean lastTriangle = false;
    private boolean lastCross = false;

    // Debounce for hood adjustment
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Debounce for shot mode toggle
    private boolean lastRightBumper1 = false;
    private boolean lastRightBumper2 = false;

    // Timed shoot/feed sequence
    private final ElapsedTime sequenceTimer = new ElapsedTime();

    // Pose-based velocity estimation
    private final ElapsedTime poseVelocityTimer = new ElapsedTime();
    private boolean velocityInitialized = false;
    private double lastPoseX = 0.0;
    private double lastPoseY = 0.0;
    private double fieldVxInPerSec = 0.0;
    private double fieldVyInPerSec = 0.0;

    // Predicted distance state for hood/flywheel shot-on-move compensation
    private double predictedShotDistance = 0.0;
    private double filteredPredictedShotDistance = 0.0;
    private boolean predictedDistanceInitialized = false;
    private double radialVelocityToGoal = 0.0;

    private boolean oneDriver = false;

    private enum FeedState {
        IDLE,
        WAIT_BEFORE_INTAKE,
        RUN_INTAKE,
        DONE
    }

    private FeedState feedState = FeedState.IDLE;

    // Hood angle state
    private double hoodAngleDeg = 50.0;

    // Flywheel target velocity state
    private double targetVelocityRad = 275;

    // Goal position in field coordinates
    private static final double TARGET_X = 5;
    private static final double TARGET_Y = 139;

    // Turret center offset from robot center (inches)
    // Behind robot center by 1.5 in along robot heading direction
    private static final double TURRET_CENTER_OFFSET_IN = 1.5;

    // Turret limits
    private static final double TURRET_MIN_DEG = -175;
    private static final double TURRET_MAX_DEG = 175;

    // IMPORTANT:
    // This is the angular offset between your robot-relative target angle and what your turret class
    // considers "0 deg". In your other program, 180 worked because turret 0 was effectively backward.
    // Start with 180.0 if your turret zero is backward, and tune if needed.
    private static final double TURRET_OFFSET_DEG = 180.0;

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clutchServo = hardwareMap.get(Servo.class, "clutchServo");

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new FlywheelASG(hardwareMap, battery);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.8, 111.2, Math.toRadians(-90)));
        follower.updatePose();
        follower.setMaxPower(1);
        follower.startTeleOpDrive();

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.REVERSE);

        // Initialization state
        clutchOut();
        //setHoodAngle(hoodAngleDeg);
        armBlock();
        intakeMotor.setPower(0.0);

        telemetry.addLine("Initialized");
        telemetry.addLine("Clutch out, hood set, arm blocked, intake off");
        telemetry.addData("Auto Shot From Distance", autoShotFromDistance);
        telemetry.addData("Shot On Move Compensation", ENABLE_SHOT_ON_MOVE_COMP);
        telemetry.update();

        while (opModeInInit()) {
            turret.update();

            if (gamepad1.x) {
                oneDriver = true;
            } else if (gamepad1.b) {
                oneDriver = false;
            }

            if (gamepad1.y) {
                follower.setStartingPose(new Pose(72, 72, Math.toRadians(0)));
            }

            telemetry.addData("One Driver", oneDriver);
            telemetry.addData("Auto Shot From Distance", autoShotFromDistance);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        flywheel.setTargetVelocity(targetVelocityRad);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            fieldVxInPerSec = follower.getVelocity().getXComponent();
            fieldVyInPerSec = follower.getVelocity().getYComponent();

            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            trackGoalFromOdometry(pose);
            turret.update();

            handleShotModeToggle();

            // Automatic shot control using predicted future distance
            if (autoShotFromDistance) {
                double lookupDistance = predictedDistanceInitialized
                        ? filteredPredictedShotDistance
                        : Math.hypot(TARGET_X - pose.getX(), TARGET_Y - pose.getY());
                updateShotFromDistance(lookupDistance);
            } else {
                hoodAngleDeg = FIXED_POWER_SHOT_HOOD_DEG;
                targetVelocityRad = FIXED_POWER_SHOT_VELOCITY_RAD;
                setHoodAngle(hoodAngleDeg);
            }

            handleXToggle();
            handleBSequence();
            updateFeedSequence();

            // Continuous flywheel update
            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            telemetry.addData("Shot Mode", autoShotFromDistance ? "AUTO TRACKING" : "FIXED POWER SHOT");
            telemetry.addData("Hood Angle (deg)", hoodAngleDeg);
            telemetry.addData("Flywheel Target (rad/s)", targetVelocityRad);
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.update();
        }

        flywheel.stop();
        intakeMotor.setPower(0.0);
    }

    private void drive(double robotHeadingDeg) {
        double trigger = Range.clip(1 - gamepad1.right_trigger, 0.2, 1);

        if (!(gamepad1.left_trigger > 0.5)) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y * trigger,
                    gamepad1.left_stick_x * trigger,
                    -gamepad1.right_stick_x * trigger,
                    false
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * trigger,
                    -gamepad1.left_stick_x * trigger,
                    -gamepad1.right_stick_x * trigger,
                    true
            );
        }

        if (gamepad1.circle) {
            // follower.setPose(new Pose(24.4, 126, Math.toRadians(142)));
            gamepad1.rumble(500);
        }
    }

    private void handleShotModeToggle() {
        boolean rb1Pressed = gamepad1.right_bumper;
        boolean rb2Pressed = gamepad2.right_bumper;

        if (oneDriver) {
            if (rb1Pressed && !lastRightBumper1) {
                autoShotFromDistance = !autoShotFromDistance;
                gamepad1.rumble(400);

                if (!autoShotFromDistance) {
                    hoodAngleDeg = FIXED_POWER_SHOT_HOOD_DEG;
                    targetVelocityRad = FIXED_POWER_SHOT_VELOCITY_RAD;
                    setHoodAngle(hoodAngleDeg);
                }
            }
        } else {
            if (rb2Pressed && !lastRightBumper2) {
                autoShotFromDistance = !autoShotFromDistance;
                gamepad2.rumble(400);

                if (!autoShotFromDistance) {
                    hoodAngleDeg = FIXED_POWER_SHOT_HOOD_DEG;
                    targetVelocityRad = FIXED_POWER_SHOT_VELOCITY_RAD;
                    setHoodAngle(hoodAngleDeg);
                }
            }
        }

        lastRightBumper1 = rb1Pressed;
        lastRightBumper2 = rb2Pressed;
    }

    private void handleXToggle() {
        boolean xPressed;
        if (oneDriver) {
            xPressed = gamepad1.x;
        } else {
            xPressed = gamepad2.x;
        }

        if (xPressed && !lastX) {
            intakeToggleOn = !intakeToggleOn;

            if (feedState == FeedState.IDLE || feedState == FeedState.DONE) {
                if (intakeToggleOn) {
                    clutchOut();
                    armBlock();
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }
        }

        lastX = xPressed;
    }

    private void handleBSequence() {
        boolean bPressed;
        if (oneDriver) {
            bPressed = gamepad1.b;
        } else {
            bPressed = gamepad2.b;
        }

        if (bPressed && !lastB) {
            intakeToggleOn = false;
            intakeMotor.setPower(0.0);

            armShoot();
            clutchIn();

            sequenceTimer.reset();
            feedState = FeedState.WAIT_BEFORE_INTAKE;
        }

        lastB = bPressed;
    }

    private void updateFeedSequence() {
        switch (feedState) {
            case IDLE:
                break;

            case WAIT_BEFORE_INTAKE:
                if (sequenceTimer.seconds() >= 0.1) {
                    intakeMotor.setPower(1.0);
                    sequenceTimer.reset();
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (sequenceTimer.seconds() >= 0.9) {
                    intakeMotor.setPower(0.0);
                    feedState = FeedState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void trackGoalFromOdometry(Pose pose) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        // Compute turret center position from robot pose
        double turretX = robotX - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        // Actual distance from turret center to target
        double actualDx = TARGET_X - turretX;
        double actualDy = TARGET_Y - turretY;
        double actualDistance = Math.hypot(actualDx, actualDy);

        // Estimate shot time from distance
        double shotTimeSec = estimateShotTimeSec(actualDistance);

        // Unit vector from turret center to target
        double ux = 0.0;
        double uy = 0.0;
        if (actualDistance > 1e-6) {
            ux = actualDx / actualDistance;
            uy = actualDy / actualDistance;
        }

        // Positive radial velocity = moving toward the goal
        // Negative radial velocity = moving away from the goal
        radialVelocityToGoal = fieldVxInPerSec * ux + fieldVyInPerSec * uy;

        // Predict effective future shot distance for hood/flywheel lookup
        predictedShotDistance = actualDistance - radialVelocityToGoal * shotTimeSec;
        predictedShotDistance = Math.max(0.0, predictedShotDistance);

        if (!predictedDistanceInitialized) {
            filteredPredictedShotDistance = predictedShotDistance;
            predictedDistanceInitialized = true;
        } else {
            filteredPredictedShotDistance =
                    PREDICTED_DISTANCE_ALPHA * predictedShotDistance
                            + (1.0 - PREDICTED_DISTANCE_ALPHA) * filteredPredictedShotDistance;
        }

        // Build virtual target for shooting on the move:
        // goal_virtual = goal_actual - v_robot * t
        double compensatedTargetX = TARGET_X;
        double compensatedTargetY = TARGET_Y;

        if (ENABLE_SHOT_ON_MOVE_COMP) {
            compensatedTargetX = TARGET_X - fieldVxInPerSec * shotTimeSec;
            compensatedTargetY = TARGET_Y - fieldVyInPerSec * shotTimeSec;
        }

        // Vector from turret center to compensated target
        double dx = compensatedTargetX - turretX;
        double dy = compensatedTargetY - turretY;

        // Field angle from turret center to compensated target
        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to robot-relative angle
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);

        // Convert into turret command space
        double desiredTurretDeg = normalize180(angleToTargetRobotDeg + TURRET_OFFSET_DEG);

        // Keep command inside allowed turret window, choosing nearest equivalent
        double safeTurretDeg = wrapIntoTurretWindow(
                desiredTurretDeg,
                turret.getCurrentAngle(),
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        turret.setAngle(safeTurretDeg);

        telemetry.addData("Turret Center X", turretX);
        telemetry.addData("Turret Center Y", turretY);
        telemetry.addData("Actual Dist To Goal", actualDistance);
        telemetry.addData("Shot Time (s)", shotTimeSec);
        telemetry.addData("Radial Vel To Goal", radialVelocityToGoal);
        telemetry.addData("Predicted Shot Dist", predictedShotDistance);
        telemetry.addData("Filtered Pred Shot Dist", filteredPredictedShotDistance);
        telemetry.addData("Comp Target X", compensatedTargetX);
        telemetry.addData("Comp Target Y", compensatedTargetY);
        telemetry.addData("Angle To Goal Field", angleToTargetFieldDeg);
        telemetry.addData("Angle To Goal Robot", angleToTargetRobotDeg);
        telemetry.addData("Turret Target Deg", safeTurretDeg);
    }

    private void initVelocityEstimator(Pose pose) {
        lastPoseX = pose.getX();
        lastPoseY = pose.getY();
        fieldVxInPerSec = 0.0;
        fieldVyInPerSec = 0.0;
        velocityInitialized = true;
        poseVelocityTimer.reset();
    }

    private void updateEstimatedFieldVelocity(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();

        if (!velocityInitialized) {
            initVelocityEstimator(pose);
            return;
        }

        double dt = poseVelocityTimer.seconds();
        poseVelocityTimer.reset();

        if (dt <= 1e-4) {
            lastPoseX = x;
            lastPoseY = y;
            return;
        }

        double rawVx = (x - lastPoseX) / dt;
        double rawVy = (y - lastPoseY) / dt;

        // Light low-pass filter to reduce aim jitter
        fieldVxInPerSec = VELOCITY_FILTER_ALPHA * rawVx + (1.0 - VELOCITY_FILTER_ALPHA) * fieldVxInPerSec;
        fieldVyInPerSec = VELOCITY_FILTER_ALPHA * rawVy + (1.0 - VELOCITY_FILTER_ALPHA) * fieldVyInPerSec;

        lastPoseX = x;
        lastPoseY = y;
    }

    private double estimateShotTimeSec(double distanceInches) {
        return 0.77;
    }

    private double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    /**
     * Maps desiredDeg to an equivalent angle within [minDeg, maxDeg] by trying desired +/- 360*k.
     * If multiple equivalents are valid, chooses the one closest to referenceDeg.
     */
    private double wrapIntoTurretWindow(double desiredDeg, double referenceDeg, double minDeg, double maxDeg) {
        double best = Double.NaN;

        for (int k = -2; k <= 2; k++) {
            double candidate = desiredDeg + 360.0 * k;
            if (candidate >= minDeg && candidate <= maxDeg) {
                if (Double.isNaN(best) || Math.abs(candidate - referenceDeg) < Math.abs(best - referenceDeg)) {
                    best = candidate;
                }
            }
        }

        if (Double.isNaN(best)) {
            best = Range.clip(desiredDeg, minDeg, maxDeg);
        }

        return best;
    }

    private void handleFlywheelAdjustment() {
        boolean trianglePressed = gamepad1.triangle;
        boolean crossPressed = gamepad1.cross;

        if (trianglePressed && !lastTriangle) {
            targetVelocityRad += 5.0;
        }

        if (crossPressed && !lastCross) {
            targetVelocityRad -= 5.0;
            targetVelocityRad = Math.max(0.0, targetVelocityRad);
        }

        lastTriangle = trianglePressed;
        lastCross = crossPressed;

        telemetry.addData("Flywheel Target (rad/s)", targetVelocityRad);
    }

    private void handleHoodAdjustment() {
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;

        if (dpadUpPressed && !lastDpadUp) {
            hoodAngleDeg += 1.0;
            setHoodAngle(hoodAngleDeg);
        }

        if (dpadDownPressed && !lastDpadDown) {
            hoodAngleDeg -= 1.0;
            setHoodAngle(hoodAngleDeg);
        }

        lastDpadUp = dpadUpPressed;
        lastDpadDown = dpadDownPressed;

        telemetry.addData("Hood Angle (deg)", hoodAngleDeg);
    }

    private void updateShotFromDistance(double distance) {
        // [distance inches, hood angle deg, flywheel velocity rad/s]
        double[][] shotTable = {
                {47.0, 30.0, 275.0},
                {52.0, 30.0, 275.0},
                {59.0, 33.0, 280.0},
                {64.5, 36.0, 285.0},
                {70.0, 36.0, 300.0},
                {76.5, 36.0, 320.0},
                {81.0, 38.0, 320.0},
                {88.0, 38.0, 330.0},
                {94.0, 38.0, 340.0},
                {102.0, 40.0, 340.0},
                {115.0, 44.0, 375.0},
                {119.7, 44.0, 380.0},
                {126.0, 44.0, 385.0}
        };

        // Clamp below first point
        if (distance <= shotTable[0][0]) {
            hoodAngleDeg = shotTable[0][1];
            targetVelocityRad = shotTable[0][2];
            setHoodAngle(hoodAngleDeg);
            return;
        }

        // Clamp above last point
        int last = shotTable.length - 1;
        if (distance >= shotTable[last][0]) {
            hoodAngleDeg = shotTable[last][1];
            targetVelocityRad = shotTable[last][2];
            setHoodAngle(hoodAngleDeg);
            return;
        }

        // Interpolate between surrounding points
        for (int i = 0; i < shotTable.length - 1; i++) {
            double d1 = shotTable[i][0];
            double a1 = shotTable[i][1];
            double v1 = shotTable[i][2];

            double d2 = shotTable[i + 1][0];
            double a2 = shotTable[i + 1][1];
            double v2 = shotTable[i + 1][2];

            if (distance >= d1 && distance <= d2) {
                double t = (distance - d1) / (d2 - d1);

                hoodAngleDeg = a1 + t * (a2 - a1);
                targetVelocityRad = (v1 + t * (v2 - v1)) + 10;

                setHoodAngle(hoodAngleDeg);
                return;
            }
        }
    }

    public void clutchIn() {
        clutchServo.setPosition(0.48);
    }

    public void clutchOut() {
        clutchServo.setPosition(0.52);
    }

    public void armBlock() {
        armServo.setPosition(0.39);
    }

    public void armShoot() {
        armServo.setPosition(0.54);
    }

    public void setHoodAngle(double angleDeg) {
        final double MIN_ANGLE = 30.0;
        final double MAX_ANGLE = 60.0;

        final double MIN_POS = 0.395;
        final double MAX_POS = 0.9;

        hoodAngleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));

        double t = (hoodAngleDeg - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        double pos = MIN_POS + t * (MAX_POS - MIN_POS);

        pos = Math.max(MIN_POS, Math.min(MAX_POS, pos));

        hoodServo.setPosition(pos);
    }
}