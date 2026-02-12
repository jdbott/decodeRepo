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
import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlateFast;
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "STATES PLAYOFFS BLUE")
public class BlueQ3AutoUnsortedQuals extends OpMode {

    // Subsystems
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlateFast basePlate;
    private Turret turret;

    private FlywheelASG flywheelASG;

    private Limelight3A limelight3A;

    // State / timing
    private int pathState;
    private Timer pathTimer;
    private boolean oneTimeDone;

    // Telemetry
    private MultipleTelemetry telemetryA;

    // Re-used paths
    private Path toGateIntake, toShoot2;

    // Loop vars
    private int gateTrips = 0;

    // Tunables
    private static final double SHOOT_WAIT_S = 0.55;
    private static final double INTAKE_STOP_DELAY_INTO_SHOOT_PATH_S = 0.5;

    // Intake carry flag: set TRUE when an intake path ends; cleared after we stop intake 0.5s into the shoot path.
    private boolean intakeCarryPending = false;

    // -----------------------------
// Turret tracking (AUTO) fields
// -----------------------------
    private static final double TURRET_TARGET_X = 6;
    private static final double TURRET_TARGET_Y = 144;

    private static final double TURRET_OFFSET_DEG = 180.0;
    private static final double TURRET_MIN_DEG = -160.0;
    private static final double TURRET_MAX_DEG =  200.0;

    // Auto-tracking enable (for auto, usually always true)
    private boolean turretAutoTrackingEnabled = true;

    // Optional: state-based override (e.g., force turret to -80 during intake then resume)
    private boolean turretOverrideActive = false;
    private double turretOverrideAngleDeg = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(30.6, 136.500, Math.toRadians(-90)));
        follower.updatePose();
        follower.setMaxPower(1);

        intake = new Intake(hardwareMap);
        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlateFast(hardwareMap);
        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheelASG = new FlywheelASG(hardwareMap, battery);
        intake.intakeStop();
        gantry.moveGantryToPos("back");
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.setPusherMm(0);
        basePlate.gateHoldBall1();
        basePlate.prepShootOnly();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.setPollRateHz(100);
        limelight3A.start();

        pathTimer = new Timer();
        pathState = 0;
        oneTimeDone = false;
        intakeCarryPending = false;
        gateTrips = 0;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        basePlate.update();
        turret.update();
        turret.setAngle(0);
    }

    @Override
    public void start() {
        flywheelASG.setTargetVelocity(309 + 10);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        basePlate.update();
        updateTurretAutoTracking();
        turret.update();
        flywheelASG.update();

        telemetryA.addData("State", pathState);
        telemetryA.addData("Busy", follower.isBusy());
        telemetryA.addData("t", follower.getCurrentTValue());
        telemetryA.addData("intakeCarryPending", intakeCarryPending);
        telemetryA.update();
    }

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

    private void stopIntakeIfHalfSecondIntoShootPath() {
        if (intakeCarryPending && pathTimer.getElapsedTimeSeconds() >= INTAKE_STOP_DELAY_INTO_SHOOT_PATH_S) {
            intake.intakeStop();
            basePlate.prepShootOnly();
            intakeCarryPending = false;
        }
    }

    private void markIntakePathFinishedGateDownAndCarry() {
        basePlate.gateHoldBall1();     // "gate position to one" immediately after intake path ends
        intakeCarryPending = true;     // keep intake running into next shoot path
        // do NOT stop intake here
    }

    private double normalize180(double a) {
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
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
                double txDeg = aimRes.getTx()+3; // degrees

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

    private void autoPathUpdate() {
        switch (pathState) {

            // ------------------------------------------------------------------
            // SHOOT 1
            // ------------------------------------------------------------------
            case 0: {
                follower.setMaxPower(1);
                Path toShoot1 = new Path(new BezierLine(
                        new Pose(35.791, 135),
                        new Pose(57, 85))
                );
                toShoot1.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180), 0.8);
                toShoot1.setBrakingStrength(1);
                follower.followPath(toShoot1, true);
                setPathState(1);
                break;
            }

            case 1: {
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            }

            case 2: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(3);
                }
                break;
            }

            // ------------------------------------------------------------------
            // LINE 2 INTAKE
            // ------------------------------------------------------------------
            case 3: {
                Path toLine2 = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84 - 25),
                        new Pose(40, 84 - 23),
                        new Pose(15, 84 - 24))
                );
                toLine2.setConstantHeadingInterpolation(Math.toRadians(180));
                gantry.moveGantryToPos("middle");
                flywheelASG.setTargetVelocity(304);
                follower.followPath(toLine2, false);
                intake.intakeIn();
                setPathState(4);
                break;
            }

            case 4: {
                if (follower.getCurrentTValue() > 0.4) {
                    follower.setMaxPower(0.5);
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    markIntakePathFinishedGateDownAndCarry();
                    setPathState(5); // go to SHOOT 2 start
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT 2
            // ------------------------------------------------------------------
            case 5: {
                toShoot2 = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(40, 60),
                        new Pose(57, 85))
                );
                toShoot2.reverseHeadingInterpolation();
                toShoot2.setBrakingStrength(1);
                gantry.moveGantryToPos("back");
                follower.followPath(toShoot2, true);
                setPathState(6);
                break;
            }

            case 51: {
                toShoot2 = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(30, 50),
                        new Pose(57, 85))
                );
                toShoot2.setConstantHeadingInterpolation(135);
                gantry.moveGantryToPos("back");
                toShoot2.setBrakingStrength(1);
                follower.followPath(toShoot2, true);
                setPathState(6);
                break;
            }

            case 6: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            }

            case 7: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(8);
                }
                break;
            }

            // ------------------------------------------------------------------
            // GATE INTAKE LOOP (2 trips)
            // ------------------------------------------------------------------
            case 8: {
                toGateIntake = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(53.779, 53.946),
                        new Pose(16.2, 60))
                );
//                toGateIntake = new Path(new BezierCurve(
//                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                        new Pose(53.779, 53.946),
//                        new Pose(9, 73),
//                        new Pose(8.5, 62))
//                );
                toGateIntake.setTangentHeadingInterpolation();
                gantry.moveGantryToPos("middle");
                follower.followPath(toGateIntake, true);
                //intake.intakeIn();
                setPathState(9);
                break;
            }

            case 9: {
                if (follower.getCurrentTValue() > 0.5) {
                    toGateIntake.setConstantHeadingInterpolation(Math.toRadians(160));
                }
                if (!follower.isBusy()) {
                    follower.startTeleOpDrive();
                    follower.setTeleOpDrive(0.15, 0, 0);
                    intake.intakeIn();
                    gateTrips++;
                    setPathState(91);
                }
                break;
            }

            case 91: {
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.setMaxPower(1);
                    markIntakePathFinishedGateDownAndCarry();
                    if (gateTrips < 3) {
                        setPathState(51);   // back to SHOOT 2
                    } else {
                        setPathState(10);  // shoot after last gate
                    }
                }
            }
            break;

            // ------------------------------------------------------------------
            // SHOOT AFTER LAST GATE
            // ------------------------------------------------------------------
            case 10: {
                Path toShootAfterLastGate = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(30, 50),
                        new Pose(57, 85))
                );
                gantry.moveGantryToPos("back");
                toShootAfterLastGate.reverseHeadingInterpolation();
                toShootAfterLastGate.setBrakingStrength(1);
                follower.followPath(toShootAfterLastGate, true);
                setPathState(11);
                break;
            }

            case 11: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            }

            case 12: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(13);
                }
                break;
            }

            // ------------------------------------------------------------------
            // CLOSE LINE INTAKE
            // ------------------------------------------------------------------
            case 13: {
                follower.setMaxPower(0.75);
                Path toCloseLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84),
                        new Pose(20.5, 84))
                );
                toCloseLine.setTangentHeadingInterpolation();
                gantry.moveGantryToPos("middle");
                follower.followPath(toCloseLine, false);
                intake.intakeIn();
                setPathState(14);
                break;
            }

            case 14: {
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    markIntakePathFinishedGateDownAndCarry();
                    setPathState(15); // shoot 3 start
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT 3
            // ------------------------------------------------------------------
            case 15: {
                Path toShoot3 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot3.setTangentHeadingInterpolation();
                toShoot3.reverseHeadingInterpolation();
                toShoot3.setBrakingStrength(1);
                gantry.moveGantryToPos("back");
                follower.followPath(toShoot3, true);
                setPathState(16);
                break;
            }

            case 16: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;
            }

            case 17: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(23);
                }
                break;
            }

            // ------------------------------------------------------------------
            // BOTTOM LINE INTAKE
            // ------------------------------------------------------------------
            case 18: {
                Path toBottomLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84 - 48),
                        new Pose(21, 84 - 50),
                        new Pose(15, 84-50))
                );
                toBottomLine.setTangentHeadingInterpolation();

                follower.followPath(toBottomLine, false);
                intake.intakeIn();
                setPathState(19);
                break;
            }

            case 19: {
                if (follower.getCurrentTValue() > 0.4) {
                    follower.setMaxPower(0.5);
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    markIntakePathFinishedGateDownAndCarry();
                    setPathState(20); // shoot 4 start
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT 4
            // ------------------------------------------------------------------
            case 20: {
                Path toShoot4 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot4.setTangentHeadingInterpolation();
                toShoot4.reverseHeadingInterpolation();
                toShoot4.setBrakingStrength(1);
                follower.followPath(toShoot4, true);
                setPathState(21);
                break;
            }

            case 21: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            }

            case 22: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(23);
                }
                break;
            }

            // DONE
            case 23: {
                intake.intakeStop();
                basePlate.rampBack();
                basePlate.gateUp();
                turretAutoTrackingEnabled = false;
                turret.setAngle(0);
                setPathState(24);
                break;
            }

            case 24: {
                Path toShoot4 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(25, 85))
                );
                toShoot4.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toShoot4, true);
                setPathState(25);
                break;
            }

            case 25: {
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            }

            default:
                break;
        }
    }
}