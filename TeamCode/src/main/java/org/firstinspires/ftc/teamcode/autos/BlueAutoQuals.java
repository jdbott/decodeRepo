package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlate;
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Blue Auto Quals (Sorted Lines)")
public class BlueAutoQuals extends OpMode {

    // ============================================================
    // Subsystems
    // ============================================================
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlate basePlate;
    private Turret turret;
    private FlywheelASG flywheelASG;

    // ============================================================
    // AprilTag via Limelight
    // ============================================================
    private Limelight3A limelight3A;
    private int lastSeenTag = 21;                 // fallback
    int tag = 21;
    private Pattern desiredPattern = Pattern.GPP; // requested shot order

    // ============================================================
    // State / timing
    // ============================================================
    private int pathState = 0;
    private Timer pathTimer;
    private boolean oneTimeDone = false;

    private MultipleTelemetry telemetryA;

    // ============================================================
    // Tunables
    // ============================================================
    private static final double SHOOTER_RAD = 309;

    // After leaving a line: keep intaking for this long, then STOP intake + gate down (lock).
    private static final double POST_INTAKE_STOP_AND_GATE_DOWN_DELAY_S = 0.75;

    // Hold at gate before returning to shoot
    private static final double GATE_HOLD_SECONDS = 0.75;

    // ============================================================
    // Turret auto-tracking
    // ============================================================
    private static final double TURRET_TARGET_X = 8;
    private static final double TURRET_TARGET_Y = 150;

    private static final double TURRET_OFFSET_DEG = 180.0;
    private static final double TURRET_MIN_DEG = -160.0;
    private static final double TURRET_MAX_DEG = 160;

    private boolean turretAutoTrackingEnabled = true;
    private boolean turretOverrideActive = false;
    private double turretOverrideAngleDeg = 0.0;

    // ============================================================
    // Sorting harness
    // ============================================================
    private static final double COLOR_CHANGE_DELAY_S = 0.75;

    private static final double GANTRY_BACK_TO_FRONT_S = 0.90;
    private static final double GANTRY_BACK_TO_MIDDLE_S = 0.45;
    private static final double GANTRY_ANY_SETTLE_S = 0.10;

    private static final double POPPER_UP_HOLD_S = 0.35;
    private static final double POPPER_DOWN_SETTLE_S = 0.25;

    private static final double ALLOW_GANTRY_BACK_BEFORE_FIRE_S = 0.50;

    private static final double RETENSION_PUSHER_MM = 127.0;
    private static final double RETENSION_HOLD_S = 0.25;

    // ------------------------------------------------------------
    // Pattern definition: BACK -> MIDDLE -> FRONT inside robot.
    // ------------------------------------------------------------
    private enum Pattern {
        GPP(new char[]{'G', 'P', 'P'}), // back->front
        PGP(new char[]{'P', 'G', 'P'}),
        PPG(new char[]{'P', 'P', 'G'});

        final char[] backToFront;

        Pattern(char[] b2f) {
            this.backToFront = b2f;
        }

        @Override
        public String toString() {
            return "" + backToFront[0] + backToFront[1] + backToFront[2];
        }
    }

    private enum Strategy {
        A_NORMAL_RAPID_FIRE,
        B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST,
        C_FRONT_POP_THEN_BALL2_RAPID,
        D_MIDDLE_POP_THEN_BALL2_RAPID,
        INVALID
    }

    private enum SortAutoPhase {
        OFF,
        PREPARE,
        EXECUTE,
        RESET,
        DONE
    }

    private SortAutoPhase sortPhase = SortAutoPhase.OFF;
    private Strategy sortStrategy = Strategy.INVALID;

    private Pattern sortCurrentPattern = Pattern.GPP;
    private Pattern sortDesiredPattern = Pattern.GPP;

    private int sortStep = 0;
    private double sortStepStartTimeS = 0.0;

    // Gate execution so PREP can happen while moving, but actual pops only at the shoot location.
    private boolean sortAllowFire = false;

    // ============================================================
    // Intake-cycle latch (new)
    // ============================================================
    private Pattern pendingIntakedPattern = Pattern.GPP;
    private boolean pendingSortBegin = false;

    // ============================================================
    // Paths
    // ============================================================
    private Path toShoot1, toMiddleLine;
    private Path toGateFromMiddle, toShoot2;
    private Path toCloseLine, toShoot3;
    private Path toFarLine, toShoot4;
    private Path toPark;

    // ============================================================
// Gantry aim compensation (NEW)
// ============================================================
    private enum GantryPos { BACK, MIDDLE, FRONT }
    private GantryPos gantryPos = GantryPos.BACK;

    // Offsets along robot-forward axis (mm)
    private static final double GANTRY_OFFSET_BACK_MM   = 0.0;
    private static final double GANTRY_OFFSET_MIDDLE_MM = 100.0;
    private static final double GANTRY_OFFSET_FRONT_MM  = 200.0;

    private static final double MM_TO_IN = 1.0 / 25.4;

    private double getGantryOffsetInches() {
        switch (gantryPos) {
            case FRONT:  return GANTRY_OFFSET_FRONT_MM  * MM_TO_IN;
            case MIDDLE: return GANTRY_OFFSET_MIDDLE_MM * MM_TO_IN;
            case BACK:
            default:     return GANTRY_OFFSET_BACK_MM   * MM_TO_IN;
        }
    }

    /** Call this instead of gantry.moveGantryToPos("...") everywhere */
    private void setGantryPos(String pos) {
        gantry.moveGantryToPos(pos);
        if ("front".equalsIgnoreCase(pos)) gantryPos = GantryPos.FRONT;
        else if ("middle".equalsIgnoreCase(pos)) gantryPos = GantryPos.MIDDLE;
        else gantryPos = GantryPos.BACK;
    }

    // ============================================================
    // Lifecycle
    // ============================================================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(30.6, 136.500, Math.toRadians(90)));
        follower.updatePose();
        follower.setMaxPower(1);

        intake = new Intake(hardwareMap);
        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlate(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setAngle(105);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheelASG = new FlywheelASG(hardwareMap, battery);

        // Baseline positions
        intake.intakeStop();
        setGantryPos("back");

        // Preload staging (you want this)
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.setPusherMm(0);
        basePlate.gateHoldBall1();
        basePlate.prepShootOnly(); // preload shot uses PREP -> startShootFromPrep

        pathTimer = new Timer();
        pathState = 0;
        oneTimeDone = false;

        // Limelight
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.setPollRateHz(100);
        limelight3A.start();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        basePlate.update();
        turret.update();
        updateAprilTagDesiredPattern();

        telemetryA.addData("AprilTag", lastSeenTag);
        telemetryA.addData("desiredPattern", desiredPattern.toString());
        telemetryA.update();
    }

    @Override
    public void start() {
        flywheelASG.setTargetVelocity(SHOOTER_RAD);

        // Keep preload staged
        basePlate.prepShootOnly();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();

        // Sort PREP can run while moving; EXECUTE gated by sortAllowFire.
        sortUpdate();

        basePlate.update();
        updateTurretAutoTracking();
        turret.update();
        flywheelASG.update();

        telemetryA.addData("State", pathState);
        telemetryA.addData("Busy", follower.isBusy());
        telemetryA.addData("t", follower.getCurrentTValue());
        telemetryA.addData("AprilTag", lastSeenTag);
        telemetryA.addData("desiredPattern", desiredPattern.toString());
        telemetryA.addData("SortPhase", sortPhase);
        telemetryA.addData("SortStrategy", sortStrategy);
        telemetryA.addData("SortAllowFire", sortAllowFire);
        telemetryA.addData("BasePlateState", basePlate.getShootState());
        telemetryA.addData("BasePlateBusy", basePlate.isShootBusy());
        telemetryA.update();
    }

    // ============================================================
    // AprilTag logic
    // ============================================================
    private void updateAprilTagDesiredPattern() {
        LLResult result = limelight3A.getLatestResult();

        if (result != null && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty()) {
            tag = result.getFiducialResults().get(0).getFiducialId();
        }

        lastSeenTag = tag;

        if (tag == 21) desiredPattern = Pattern.GPP;
        else if (tag == 22) desiredPattern = Pattern.PGP;
        else if (tag == 23) desiredPattern = Pattern.PPG;
        else desiredPattern = Pattern.GPP;
    }

    // ============================================================
    // Auto flow
    // ============================================================
    private void autoPathUpdate() {
        switch (pathState) {

            // ------------------------------------------------------
            // SHOOT 1 (preload): NO color delays
            // ------------------------------------------------------
            case 0: {
                limelight3A.pipelineSwitch(9);
                toShoot1 = new Path(new BezierLine(
                        new Pose(35.791, 135),
                        new Pose(57, 85)
                ));
                toShoot1.setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180), 0.6);
                follower.followPath(toShoot1, true);
                toShoot1.setBrakingStrength(0.6);
                // preload already prepped; don't mess with ramp logic
                turretAutoTrackingEnabled = false;
                setPathState(1);
                break;
            }

            case 1: {
                if (follower.getCurrentTValue() > 0.5) turretAutoTrackingEnabled = true;
                if (!follower.isBusy()) setPathState(111);
                break;
            }

            case 111: {
                if (pathTimer.getElapsedTimeSeconds() > 0.3) setPathState(2);
                break;
            }

            case 2: {
                oneTime(() -> {
                    basePlate.setInterShotExtraDelays(0.0, 0.0);
                    basePlate.startShootFromPrep();
                });

                if (basePlate.isDoneFiringEarly(0.25)) {
                    setPathState(3);
                }
                break;
            }

            // ------------------------------------------------------
            // MIDDLE LINE INTAKE (PGP)
            // ------------------------------------------------------
            case 3: {
                startLineIntake();
                setGantryPos("middle");
                basePlate.rampBack();
                toMiddleLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 66),
                        new Pose(48, 63),
                        new Pose(15, 62.5)
                ));
                toMiddleLine.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toMiddleLine, false);
                flywheelASG.setTargetVelocity(300);
                intake.intakeIn();
                setPathState(4);
                break;
            }

            case 4: {
                if (follower.getCurrentTValue() > 0.4) follower.setMaxPower(0.5);

                if (!follower.isBusy()) {
                    turretAutoTrackingEnabled = false;
                    turret.setAngle(0);
                    follower.setMaxPower(1);

                    // Begin leaving immediately, keep intake running for a bit
                    pendingIntakedPattern = Pattern.PGP;
                    pendingSortBegin = true;

                    toGateFromMiddle = new Path(new BezierCurve(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(35, 69),
                            new Pose(17.5, 74)
                    ));
                    toGateFromMiddle.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(toGateFromMiddle, true);

                    // still intaking + gateUp + rampBack
                    startLeavingLineStillIntaking();

                    setPathState(5);
                }
                break;
            }

            case 5: {
                // After delay: stop intake, gate DOWN (lock), then begin strategy-aware sorting PREP (no unconditional prepShootOnly)
                if (pathTimer.getElapsedTimeSeconds() >= POST_INTAKE_STOP_AND_GATE_DOWN_DELAY_S) {
                    stopIntakeAndLockForSort();
                    setPathState(6);
                }
                break;
            }

            case 6: {
                if (!follower.isBusy()) {
                    setPathState(7);
                    follower.startTeleOpDrive();
                    follower.setTeleOpDrive(0.3, 0, 0);
                }
                break;
            }

            case 7: {
                if (pathTimer.getElapsedTimeSeconds() >= GATE_HOLD_SECONDS) {
                    follower.breakFollowing();
                    setPathState(8);
                }
                break;
            }

            // Drive to shoot pose
            case 8: {
                toShoot2 = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(40, 73),
                        new Pose(57, 85)
                ));
                toShoot2.reverseHeadingInterpolation();
                follower.followPath(toShoot2, true);
                toShoot2.setBrakingStrength(0.6);
                setPathState(9);
                break;
            }

            case 9: {
                if (follower.getCurrentTValue() > 0.7) {
                    toShoot2.setConstantHeadingInterpolation(Math.toRadians(270));
                    turretAutoTrackingEnabled = true;
                }
                if (!follower.isBusy()) {
                    setPathState(91);
                }
                break;
            }

            case 91: {
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    sortAllowFire = true;
                    setPathState(10);
                }
                break;
            }

            case 10: {
                if (sortPhase == SortAutoPhase.OFF) {
                    setPathState(11);
                }
                break;
            }

            // ------------------------------------------------------
            // CLOSE LINE INTAKE (PPG)
            // ------------------------------------------------------
            case 11: {
                startLineIntake();
                setGantryPos("middle");
                follower.setMaxPower(0.6);
                basePlate.rampBack();
                toCloseLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 86.5),
                        new Pose(16.5, 86.5)
                ));
                toCloseLine.setTangentHeadingInterpolation();
                follower.followPath(toCloseLine, false);

                intake.intakeIn();
                setPathState(12);
                break;
            }

            case 12: {
                if (!follower.isBusy()) {
                    setPathState(13);
                }
            }
            break;

            case 13: {
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.setMaxPower(1);

                    pendingIntakedPattern = Pattern.PPG;
                    pendingSortBegin = true;

                    // Leave line towards shoot
                    toShoot3 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(57, 85)
                    ));
                    toShoot3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270), 0.8);
                    follower.followPath(toShoot3, true);
                    toShoot3.setBrakingStrength(0.6);
                    startLeavingLineStillIntaking();
                    setPathState(145);
                }
                break;
            }

            case 145: {
                // After delay: stop intake, gate DOWN (lock), then begin strategy-aware sorting PREP (no unconditional prepShootOnly)
                if (pathTimer.getElapsedTimeSeconds() >= 1.25) {
                    stopIntakeAndLockForSort();
                    setPathState(14);
                }
                break;
            }

            case 14: {
                if (!follower.isBusy()) {
                    setPathState(141);
                }
                break;
            }

            case 141: {
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    sortAllowFire = true;
                    setPathState(15);
                }
                break;
            }

            case 15: {
                if (sortPhase == SortAutoPhase.OFF) {
                    setPathState(16);
                }
                break;
            }

            // ------------------------------------------------------
            // FAR LINE INTAKE (GPP)
            // ------------------------------------------------------
            case 16: {
                startLineIntake();
                basePlate.rampBack();
                toFarLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84 - 48),
                        new Pose(40, 84 - 48),
                        new Pose(13, 84 - 47)
                ));
                toFarLine.setTangentHeadingInterpolation();
                follower.followPath(toFarLine, false);

                intake.intakeIn();
                setPathState(17);
                break;
            }

            case 17: {
                if (follower.getCurrentTValue() > 0.15) {
                    follower.setMaxPower(0.6);
                    toFarLine.setConstantHeadingInterpolation(Math.toRadians(180));
                }

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    pendingIntakedPattern = Pattern.GPP;
                    pendingSortBegin = true;

                    toShoot4 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(57, 85)
                    ));
                    toShoot4.setTangentHeadingInterpolation();
                    toShoot4.reverseHeadingInterpolation();
                    follower.followPath(toShoot4, true);
                    toShoot4.setBrakingStrength(0.6);
                    startLeavingLineStillIntaking();
                    setPathState(18);
                }
                break;
            }

            case 18: {
                if (pathTimer.getElapsedTimeSeconds() >= POST_INTAKE_STOP_AND_GATE_DOWN_DELAY_S) {
                    stopIntakeAndLockForSort();
                    setPathState(19);
                }
                break;
            }

            case 19: {
                if (follower.getCurrentTValue() > 0.7) {
                    toShoot4.setConstantHeadingInterpolation(Math.toRadians(270));
                }
                if (!follower.isBusy()) {
                    setPathState(191);
                }
                break;
            }

            case 191: {
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    sortAllowFire = true;
                    setPathState(20);
                }
                break;
            }

            case 20: {
                if (sortPhase == SortAutoPhase.OFF) {
                    setPathState(21);
                }
                break;
            }

            // ------------------------------------------------------
            // PARK
            // ------------------------------------------------------
            case 21: {
                intake.intakeStop();
                basePlate.rampBack();
                basePlate.gateUp();
                turretAutoTrackingEnabled = false;
                turret.setAngle(0);

                toPark = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(25, 90)
                ));
                toPark.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toPark, true);

                setPathState(22);
                break;
            }

            case 22: {
                if (!follower.isBusy()) setPathState(-1);
                break;
            }

            default:
                break;
        }
    }

    // ============================================================
    // Intake helpers (new)
    // ============================================================
    private void startLineIntake() {
        // You wanted: gate UP + ramp BACK at the start of each intake
        basePlate.rampBack();
        basePlate.gateUp();
        sortAllowFire = false; // always false during intake cycles
    }

    private void startLeavingLineStillIntaking() {
        // Keep it safe while the intake is still pulling in
        basePlate.rampBack();
        basePlate.gateUp();
    }

    private void stopIntakeAndLockForSort() {
        intake.intakeStop();

        // Lock balls immediately after the delayed "still-intake" window.
        basePlate.rampBack();
        basePlate.gateHoldBall1();

        if (pendingSortBegin) {
            sortBegin(pendingIntakedPattern, desiredPattern);
            pendingSortBegin = false;
        }
    }

    // ============================================================
    // Turret auto tracking
    // ============================================================
    private double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    private double wrapIntoTurretWindow(double desiredDeg, double referenceDeg,
                                        double minDeg, double maxDeg) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = desiredDeg + 360.0 * k;
            if (candidate >= minDeg && candidate <= maxDeg) {
                if (Double.isNaN(best) || Math.abs(candidate - referenceDeg) < Math.abs(best - referenceDeg)) {
                    best = candidate;
                }
            }
        }
        if (Double.isNaN(best)) best = Range.clip(desiredDeg, minDeg, maxDeg);
        return best;
    }

    private void updateTurretAutoTracking() {
        if (!turretAutoTrackingEnabled) return;

        // -----------------------------
        // Local tunables (self-contained)
        // -----------------------------
        final int TAG_ID_FOR_TX_AIM = 20;

        final double TX_SIGN = +1.0;           // flip if reversed
        final double TX_KP = 0.7;             // deg command per deg tx
        final double MAX_STEP_DEG = 10.0;      // clamp per loop

        // -----------------------------
        // Override wins
        // -----------------------------
        if (turretOverrideActive) {
            turret.setAngle(wrapIntoTurretWindow(
                    turretOverrideAngleDeg,
                    turret.getTargetAngle(),
                    TURRET_MIN_DEG,
                    TURRET_MAX_DEG
            ));
            return;
        }

        // -----------------------------
        // Decide: TX aim only if tag ID == 20 (TeleOp-style)
        // -----------------------------
        boolean useTxAim = false;
        LLResult result = (limelight3A != null) ? limelight3A.getLatestResult() : null;

        if (result != null && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty()) {

            int id = result.getFiducialResults().get(0).getFiducialId(); // SAME AS YOUR AUTO TAG CODE
            useTxAim = (id == TAG_ID_FOR_TX_AIM);
        }

        if (useTxAim) {

            // Update LL orientation from follower heading (same convention you used before)
            Pose pose = follower.getPose();
            limelight3A.updateRobotOrientation(Math.toDegrees(pose.getHeading()));

            // Use TX exactly like teleop
            LLResult aimRes = limelight3A.getLatestResult();
            if (aimRes != null && aimRes.isValid()) {
                double txDeg = aimRes.getTx(); // degrees

                double turretCurrentDeg = turret.getCurrentAngle();
                double delta = -TX_SIGN * TX_KP * txDeg;
                delta = Range.clip(delta, -MAX_STEP_DEG, +MAX_STEP_DEG);

                double desired = turretCurrentDeg + delta;

                double safe = wrapIntoTurretWindow(
                        desired,
                        turret.getTargetAngle(),
                        TURRET_MIN_DEG,
                        TURRET_MAX_DEG
                );

                turret.setAngle(safe);
                telemetryA.addData("ll", "ll");
                return; // TX aim wins
            }
            // If LL glitches this frame, fall through to odom aim.
        }

        // -----------------------------
        // Fallback: odometry aim (your existing math)
        // -----------------------------
        Pose pose = follower.getPose();
        double botX = pose.getX();
        double botY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        double dx = TURRET_TARGET_X - botX;
        double dy = TURRET_TARGET_Y - botY;

        double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleNeededDeg = normalize180(angleToTargetDeg - robotHeadingDeg);
        double rawAutoCmdDeg = normalize180(turretAngleNeededDeg + TURRET_OFFSET_DEG);

        double safeAutoCmdDeg = wrapIntoTurretWindow(
                rawAutoCmdDeg,
                turret.getTargetAngle(),
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        turret.setAngle(safeAutoCmdDeg);
    }

    // ============================================================
// Sorting harness (strategy-aware PREP gating of prepShootOnly)
// ============================================================

    private void sortBegin(Pattern currentInsideRobotBackToFront, Pattern desiredShotOrder) {
        sortCurrentPattern = currentInsideRobotBackToFront;
        sortDesiredPattern = desiredShotOrder;
        sortStrategy = resolveStrategy(sortCurrentPattern, sortDesiredPattern);

        sortPhase = SortAutoPhase.PREPARE;
        sortStep = 0;
        sortStepStartTimeS = getRuntime();

        // Fire is gated until you reach the shoot pose
        sortAllowFire = false;
    }

    private void sortUpdate() {
        if (sortPhase == SortAutoPhase.OFF) return;

        switch (sortPhase) {
            case PREPARE:
                if (runPrepare(sortStrategy)) {
                    sortPhase = SortAutoPhase.EXECUTE;
                    sortStep = 0;
                    sortStepStartTimeS = getRuntime();
                }
                break;

            case EXECUTE:
                if (!sortAllowFire) break; // hard gate

                if (runExecute(sortStrategy)) {
                    sortPhase = SortAutoPhase.RESET;
                    sortStep = 0;
                    sortStepStartTimeS = getRuntime();
                }
                break;

            case RESET:
                if (runReset()) sortPhase = SortAutoPhase.DONE;
                break;

            case DONE:
                sortPhase = SortAutoPhase.OFF;
                break;

            default:
                sortPhase = SortAutoPhase.OFF;
                break;
        }
    }

    private Strategy resolveStrategy(Pattern current, Pattern desired) {
        if (current == desired) return Strategy.A_NORMAL_RAPID_FIRE;

        if (current == Pattern.GPP && desired == Pattern.PPG)
            return Strategy.B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST;
        if (current == Pattern.PPG && desired == Pattern.PGP)
            return Strategy.B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST;

        if (current == Pattern.PPG && desired == Pattern.GPP)
            return Strategy.C_FRONT_POP_THEN_BALL2_RAPID;
        if (current == Pattern.PGP && desired == Pattern.PPG)
            return Strategy.C_FRONT_POP_THEN_BALL2_RAPID;

        if (current == Pattern.PGP && desired == Pattern.GPP)
            return Strategy.D_MIDDLE_POP_THEN_BALL2_RAPID;
        if (current == Pattern.GPP && desired == Pattern.PGP)
            return Strategy.D_MIDDLE_POP_THEN_BALL2_RAPID;

        return Strategy.INVALID;
    }

    private boolean runPrepare(Strategy strat) {
        switch (strat) {

            // ONLY this strategy is allowed to call prepShootOnly() during PREPARE.
            case A_NORMAL_RAPID_FIRE: {
                if (sortStep == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();

                    // CHANGED: use wrapper so gantryPos tracks
                    setGantryPos("back");

                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 1) {
                    // This is the only time we intentionally ramp forward early (prep state).
                    basePlate.prepShootOnly();
                    return true;
                }
                break;
            }

            // B/C/D: do NOT call prepShootOnly here (avoid rampForward during gantry staging)
            case B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST: {
                if (sortStep == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();

                    // CHANGED
                    setGantryPos("middle");

                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 1) {
                    if (elapsed(sortStepStartTimeS) >= (GANTRY_BACK_TO_MIDDLE_S + GANTRY_ANY_SETTLE_S)) {
                        basePlate.gateHoldBall2();
                        basePlate.rampBack();
                        return true;
                    }
                }
                break;
            }

            case C_FRONT_POP_THEN_BALL2_RAPID: {
                if (sortStep == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();

                    // CHANGED
                    setGantryPos("front");

                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 1) {
                    if (elapsed(sortStepStartTimeS) >= (GANTRY_BACK_TO_FRONT_S + GANTRY_ANY_SETTLE_S)) {
                        basePlate.gateHoldBall2();
                        basePlate.rampBack();
                        return true;
                    }
                }
                break;
            }

            case D_MIDDLE_POP_THEN_BALL2_RAPID: {
                if (sortStep == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();

                    // CHANGED
                    setGantryPos("middle");

                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 1) {
                    if (elapsed(sortStepStartTimeS) >= (GANTRY_BACK_TO_MIDDLE_S + GANTRY_ANY_SETTLE_S)) {
                        basePlate.gateHoldBall2();
                        basePlate.rampBack();
                        return true;
                    }
                }
                break;
            }

            default:
                return true;
        }
        return false;
    }

    private boolean runExecute(Strategy strat) {
        char[] desired = sortDesiredPattern.backToFront;

        switch (strat) {

            case A_NORMAL_RAPID_FIRE: {
                if (sortStep == 0) {
                    basePlate.setInterShotDelaysFromColors(desired[0], desired[1], desired[2], COLOR_CHANGE_DELAY_S);
                    sortStep++;
                } else if (sortStep == 1) {
                    basePlate.startShootFromPrep();
                    sortStep++;
                } else if (sortStep == 2) {
                    if (basePlate.isDoneFiringEarly(0.25)) return true;
                }
                break;
            }

            // KEEP your explicit rampForward calls in these strategies.
            case C_FRONT_POP_THEN_BALL2_RAPID: {
                if (sortStep == 0) {
                    basePlate.gateHoldBall2();
                    basePlate.rampBack();
                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 1) {
                    basePlate.frontPopperUp();
                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 2) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.frontPopperDown();
                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 3) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        basePlate.rampForward(); // keep

                        // CHANGED
                        setGantryPos("back");

                        double stageMm = basePlate.getShootPush1Mm() + basePlate.getShootPush2Mm();
                        basePlate.setPusherMm(stageMm);
                        basePlate.gateHoldBall3();

                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 4) {
                    if (elapsed(sortStepStartTimeS) >= ALLOW_GANTRY_BACK_BEFORE_FIRE_S) {
                        double d23 = (desired[1] == desired[2]) ? 0.0 : COLOR_CHANGE_DELAY_S;
                        basePlate.setInterShotExtraDelays(0.0, d23);

                        basePlate.startShootFromBeforeShot2();
                        sortStep++;
                    }
                } else if (sortStep == 5) {
                    if (basePlate.isDoneFiringEarly(0.25)) return true;
                }
                break;
            }

            case D_MIDDLE_POP_THEN_BALL2_RAPID: {
                if (sortStep == 0) {
                    basePlate.gateHoldBall2();
                    basePlate.rampBack();
                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 1) {
                    basePlate.middlePopperUp();
                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 2) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.middlePopperDown();
                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 3) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        basePlate.rampForward(); // keep

                        // CHANGED
                        setGantryPos("back");

                        double stageMm = basePlate.getShootPush1Mm() + basePlate.getShootPush2Mm();
                        basePlate.setPusherMm(stageMm);
                        basePlate.gateHoldBall3();

                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 4) {
                    if (elapsed(sortStepStartTimeS) >= ALLOW_GANTRY_BACK_BEFORE_FIRE_S) {
                        double d23 = (desired[1] == desired[2]) ? 0.0 : COLOR_CHANGE_DELAY_S;
                        basePlate.setInterShotExtraDelays(0.0, d23);

                        basePlate.startShootFromBeforeShot2();
                        sortStep++;
                    }
                } else if (sortStep == 5) {
                    if (basePlate.isDoneFiringEarly(0.25)) return true;
                }
                break;
            }

            case B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST: {
                if (sortStep == 0) {
                    basePlate.gateHoldBall2();
                    basePlate.rampBack();
                    sortStep++;
                } else if (sortStep == 1) {
                    basePlate.middlePopperUp();
                    sortStepStartTimeS = getRuntime();
                    sortStep++;
                } else if (sortStep == 2) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.middlePopperDown();
                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 3) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        basePlate.setPusherMm(RETENSION_PUSHER_MM);
                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 4) {
                    if (elapsed(sortStepStartTimeS) >= RETENSION_HOLD_S) {
                        basePlate.middlePopperUp();
                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 5) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.middlePopperDown();
                        sortStepStartTimeS = getRuntime();
                        sortStep++;
                    }
                } else if (sortStep == 6) {
                    if (elapsed(sortStepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        basePlate.rampForward(); // keep
                        basePlate.gateHoldBall2();
                        basePlate.setPusherMm(RETENSION_PUSHER_MM - 20);

                        // CHANGED
                        setGantryPos("back");

                        basePlate.startLastBallOnlyFromStaged();
                        sortStep++;
                    }
                } else if (sortStep == 7) {
                    if (basePlate.isDoneFiringEarly(0.25)) return true;
                }
                break;
            }

            default:
                return true;
        }

        return false;
    }

    private boolean runReset() {
        if (sortStep == 0) {
            basePlate.cancelShootAndReset();
            basePlate.rampBack();
            basePlate.gateUp();
            basePlate.setPusherMm(0.0);

            // CHANGED
            setGantryPos("back");

            sortStepStartTimeS = getRuntime();
            sortStep++;
        } else if (sortStep == 1) {
            if (elapsed(sortStepStartTimeS) >= 0.20) return true;
        }
        return false;
    }

    // ============================================================
    // Helpers
    // ============================================================
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
        oneTimeDone = false;
    }

    private void oneTime(Runnable r) {
        if (!oneTimeDone) {
            r.run();
            oneTimeDone = true;
            pathTimer.resetTimer();
        }
    }

    private double elapsed(double startS) {
        return getRuntime() - startS;
    }
}