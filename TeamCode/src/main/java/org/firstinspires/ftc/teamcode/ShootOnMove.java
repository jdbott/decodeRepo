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

@TeleOp(name = "SOTM_TeleOp")
public class ShootOnMove extends LinearOpMode {

    private Servo hoodServo, armServo, clutchServo;
    private Follower follower;
    private Turret turret;
    private DcMotorEx intakeMotor;
    private FlywheelASG flywheel;

    private static final boolean AUTO_SHOT_FROM_DISTANCE = true;
    private static final boolean ENABLE_SHOT_ON_MOVE_COMP = true;


    private static final double VELOCITY_FILTER_ALPHA = 0.25;
    private static final double PREDICTED_DISTANCE_ALPHA = 0.45;
    private static final double METERS_TO_INCHES = 39.3701;
    private static final double TIME_TUNER = 2.5;

    private boolean lastX = false;
    private boolean intakeToggleOn = false;
    private boolean lastB = false;
    private boolean lastTriangle = false;
    private boolean lastCross = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean shootOnTheMove = ENABLE_SHOT_ON_MOVE_COMP;

    private final ElapsedTime sequenceTimer = new ElapsedTime();
    private final ElapsedTime poseVelocityTimer = new ElapsedTime();

    private boolean velocityInitialized = false;
    private double lastPoseX = 0.0, lastPoseY = 0.0;
    private double fieldVxInPerSec = 0.0, fieldVyInPerSec = 0.0;

    private double predictedShotDistance = 0.0;
    private double filteredPredictedShotDistance = 0.0;
    private boolean predictedDistanceInitialized = false;
    private double radialVelocityToGoal = 0.0;

    private enum FeedState {IDLE, WAIT_BEFORE_INTAKE, RUN_INTAKE, DONE}
    private FeedState feedState = FeedState.IDLE;

    private double hoodAngleDeg = 50.0;
    private double targetVelocityRad = 250.0;

    private static final double TARGET_X = 5;
    private static final double TARGET_Y = 139;
    private static final double TURRET_CENTER_OFFSET_IN = 1.5;
    private static final double TURRET_MIN_DEG = -160.0;
    private static final double TURRET_MAX_DEG = 160.0;
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
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(0)));
        follower.updatePose();
        follower.setMaxPower(1);
        follower.startTeleOpDrive();

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.REVERSE);

        // Initialization
        clutchOut();
        setHoodAngle(hoodAngleDeg);
        armBlock();
        intakeMotor.setPower(0.0);

        telemetry.addLine("Initialized");
        telemetry.addData("Auto Shot From Distance", AUTO_SHOT_FROM_DISTANCE);
        telemetry.addData("Shot On Move Compensation", ENABLE_SHOT_ON_MOVE_COMP);
        telemetry.update();

        while (opModeInInit()) {
            turret.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        flywheel.setTargetVelocity(targetVelocityRad);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            updateEstimatedFieldVelocity(pose);

            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            trackGoalFromOdometry(pose);
            turret.update();

            if (AUTO_SHOT_FROM_DISTANCE) {
                double lookupDistance = predictedDistanceInitialized
                        ? filteredPredictedShotDistance
                        : Math.hypot(TARGET_X - pose.getX(), TARGET_Y - pose.getY());
                updateShotFromDistance(lookupDistance);
            } else {
                handleFlywheelAdjustment();
                handleHoodAdjustment();
            }

            handleXToggle();
            handleBSequence();
            updateFeedSequence();

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            telemetry.addData("Pose", pose.toString());
            telemetry.update();
        }

        flywheel.stop();
        intakeMotor.setPower(0.0);
    }

    private void drive(double robotHeadingDeg) {
        double trigger = Range.clip(1 - gamepad1.right_trigger, 0.2, 1);
        if (!(gamepad1.left_trigger > 0.5)) {
            follower.setTeleOpDrive(gamepad1.left_stick_y * trigger,
                    gamepad1.left_stick_x * trigger,
                    -gamepad1.right_stick_x * trigger,
                    false);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y * trigger,
                    -gamepad1.left_stick_x * trigger,
                    -gamepad1.right_stick_x * trigger,
                    true);
        }
    }

    private void handleXToggle() {
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            intakeToggleOn = !intakeToggleOn;
            if (feedState == FeedState.IDLE || feedState == FeedState.DONE) {
                intakeMotor.setPower(intakeToggleOn ? 1.0 : 0.0);
                if (intakeToggleOn) { clutchOut(); armBlock(); }
            }
        }
        lastX = xPressed;
    }

    private void handleBSequence() {
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) {
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
            case IDLE: break;
            case WAIT_BEFORE_INTAKE:
                if (sequenceTimer.seconds() >= 0.1) { intakeMotor.setPower(1.0); sequenceTimer.reset(); feedState = FeedState.RUN_INTAKE; }
                break;
            case RUN_INTAKE:
                if (sequenceTimer.seconds() >= 0.9) { intakeMotor.setPower(0.0); feedState = FeedState.DONE; }
                break;
            case DONE: break;
        }
    }

    private void trackGoalFromOdometry(Pose pose) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double turretX = robotX - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        double actualDx = TARGET_X - turretX;
        double actualDy = TARGET_Y - turretY;
        double actualDistance = Math.hypot(actualDx, actualDy);

        double shotTimeSec = estimateShotTimeSec(actualDistance);

        double ux = 0.0, uy = 0.0;
        if (actualDistance > 1e-6) { ux = actualDx / actualDistance; uy = actualDy / actualDistance; }

        radialVelocityToGoal = fieldVxInPerSec * ux + fieldVyInPerSec * uy;

        predictedShotDistance = Math.max(0.0, actualDistance - radialVelocityToGoal * shotTimeSec);

        if (!predictedDistanceInitialized) {
            filteredPredictedShotDistance = predictedShotDistance;
            predictedDistanceInitialized = true;
        } else {
            filteredPredictedShotDistance = PREDICTED_DISTANCE_ALPHA * predictedShotDistance
                    + (1.0 - PREDICTED_DISTANCE_ALPHA) * filteredPredictedShotDistance;
        }

        double compensatedTargetX = TARGET_X;
        double compensatedTargetY = TARGET_Y;

        if (follower.getVelocity().getMagnitude() <= 1 && follower.getAcceleration().getMagnitude() >= 4){
            shootOnTheMove = true;
        } else if (follower.getVelocity().getMagnitude() >= 2){
            shootOnTheMove = true;
        } else {
            shootOnTheMove = false;
        }

        if (shootOnTheMove) {
            compensatedTargetX = TARGET_X - fieldVxInPerSec * shotTimeSec;
            compensatedTargetY = TARGET_Y - fieldVyInPerSec * shotTimeSec;
        }

        double dx = compensatedTargetX - turretX;
        double dy = compensatedTargetY - turretY;

        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);
        double desiredTurretDeg = normalize180(angleToTargetRobotDeg + TURRET_OFFSET_DEG);
        double safeTurretDeg = wrapIntoTurretWindow(desiredTurretDeg, turret.getCurrentAngle(), TURRET_MIN_DEG, TURRET_MAX_DEG);

        turret.setAngle(safeTurretDeg);

        telemetry.addData("Shoot on the move: ", shootOnTheMove);
        telemetry.addData("Velocity: ", follower.getVelocity().getMagnitude());
        telemetry.addData("Acceleration ", follower.getAcceleration().getMagnitude());
        telemetry.addData("Actual Dist To Goal", actualDistance);
        telemetry.addData("Shot Time (s)", shotTimeSec);
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
        if (!velocityInitialized) { initVelocityEstimator(pose); return; }

        double dt = poseVelocityTimer.seconds();
        poseVelocityTimer.reset();
        if (dt <= 1e-4) { lastPoseX = x; lastPoseY = y; return; }

        double rawVx = (x - lastPoseX) / dt;
        double rawVy = (y - lastPoseY) / dt;

        fieldVxInPerSec = VELOCITY_FILTER_ALPHA * rawVx + (1 - VELOCITY_FILTER_ALPHA) * fieldVxInPerSec;
        fieldVyInPerSec = VELOCITY_FILTER_ALPHA * rawVy + (1 - VELOCITY_FILTER_ALPHA) * fieldVyInPerSec;

        lastPoseX = x;
        lastPoseY = y;
    }

    private double estimateShotTimeSec(double distanceInches) {
        double distanceMeters = distanceInches/METERS_TO_INCHES;
        double time = ShootingCalc.get2DTimeNeeded(distanceMeters, Math.toRadians(hoodAngleDeg), 0.984-0.3);
        return time * TIME_TUNER;
    }
    private double normalize180(double a) { return ((a + 180) % 360 + 360) % 360 - 180; }

    private double wrapIntoTurretWindow(double desiredDeg, double referenceDeg, double minDeg, double maxDeg) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = desiredDeg + 360.0 * k;
            if (candidate >= minDeg && candidate <= maxDeg) {
                if (Double.isNaN(best) || Math.abs(candidate - referenceDeg) < Math.abs(best - referenceDeg)) best = candidate;
            }
        }
        if (Double.isNaN(best)) best = Range.clip(desiredDeg, minDeg, maxDeg);
        return best;
    }

    private void handleFlywheelAdjustment() {
        boolean trianglePressed = gamepad1.triangle;
        boolean crossPressed = gamepad1.cross;

        if (trianglePressed && !lastTriangle) targetVelocityRad += 5.0;
        if (crossPressed && !lastCross) targetVelocityRad = Math.max(0.0, targetVelocityRad - 5.0);

        lastTriangle = trianglePressed;
        lastCross = crossPressed;
    }

    private void handleHoodAdjustment() {
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;

        if (dpadUpPressed && !lastDpadUp) setHoodAngle(hoodAngleDeg + 1.0);
        if (dpadDownPressed && !lastDpadDown) setHoodAngle(hoodAngleDeg - 1.0);

        lastDpadUp = dpadUpPressed;
        lastDpadDown = dpadDownPressed;
    }

    private void updateShotFromDistance(double distance) {
        double[][] shotTable = {
                {47.0, 30.0, 275.0},{52.0,30.0,275.0},{59.0,33.0,280.0},{64.5,36.0,285.0},
                {70.0,36.0,300.0},{76.5,36.0,320.0},{81.0,38.0,320.0},{88.0,38.0,330.0},
                {94.0,38.0,340.0},{102.0,40.0,340.0},{115.0,44.0,375.0},{119.7,44.0,380.0},
                {126.0,44.0,385.0}
        };

        if (distance <= shotTable[0][0]) { hoodAngleDeg = shotTable[0][1]; targetVelocityRad = shotTable[0][2]; setHoodAngle(hoodAngleDeg); return; }
        int last = shotTable.length - 1;
        if (distance >= shotTable[last][0]) { hoodAngleDeg = shotTable[last][1]; targetVelocityRad = shotTable[last][2]; setHoodAngle(hoodAngleDeg); return; }

        for (int i = 0; i < shotTable.length - 1; i++) {
            double d1 = shotTable[i][0], a1 = shotTable[i][1], v1 = shotTable[i][2];
            double d2 = shotTable[i+1][0], a2 = shotTable[i+1][1], v2 = shotTable[i+1][2];
            if (distance >= d1 && distance <= d2) {
                double t = (distance - d1)/(d2 - d1);
                hoodAngleDeg = a1 + t*(a2-a1);
                targetVelocityRad = (v1 + t*(v2-v1)) + 10;
                setHoodAngle(hoodAngleDeg);
                return;
            }
        }
    }

    public void clutchIn() { clutchServo.setPosition(0.48); }
    public void clutchOut() { clutchServo.setPosition(0.52); }
    public void armBlock() { armServo.setPosition(0.39); }
    public void armShoot() { armServo.setPosition(0.54); }

    public void setHoodAngle(double angleDeg) {
        final double MIN_ANGLE = 30.0, MAX_ANGLE = 60.0;
        final double MIN_POS = 0.395, MAX_POS = 0.9;
        hoodAngleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));
        double t = (hoodAngleDeg - MIN_ANGLE)/(MAX_ANGLE-MIN_ANGLE);
        double pos = MIN_POS + t*(MAX_POS-MIN_POS);
        hoodServo.setPosition(Math.max(MIN_POS, Math.min(MAX_POS, pos)));
    }
}