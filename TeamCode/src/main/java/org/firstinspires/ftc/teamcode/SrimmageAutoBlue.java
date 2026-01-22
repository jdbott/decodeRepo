package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
@Disabled
@Autonomous(name = "A BLUE AUTO")
public class SrimmageAutoBlue extends OpMode {

    // Subsystems
    private Follower follower;
    private Shooter shooter;
    private Turret turret;
    private ServoController revolver;
    private IMU imu;
    private Servo gateServo, popperServo;
    private ColorV3 colorSensor;

    private Limelight3A limelight3A;

    private DcMotorEx intakeMotor;

    // FSM State variables
    private int pathState;
    private Timer pathTimer;
    private int nextPathStateAfterShooting = 3; // default old behavior

    // Telemetry
    private MultipleTelemetry telemetryA;

    Path toLowerLine, toShootAgain;

    // --- Minimal shoot FSM state kept at class scope for Java 8 ---
    private enum AutoShoot3BallsState { OFF, MOVE_TO_START, POP_UP, POP_DOWN, REVOLVE, WAIT_TO_SETTLE, RETURN_TO_START, DONE }

    public double[] ANG = {60, 180, 300};

    private static final class AutoShootCtx {
        AutoShoot3BallsState state = AutoShoot3BallsState.OFF;
        long timerMs;
        int popCount;

        final char[] chambers = new char[3];   // 'G','P',' '
        char[] pattern = new char[3];
        int[] shootIdx = new int[3];           // planned index for each shot, -1 if none
        int planPos = 0;                       // next entry in shootIdx to execute
        int currentIdx = 0;
        double currentAngle = 0;

        // timings (preserved)
        final long autoInitialMoveDelayMs = 600;
        final long postRevolveDelayMs    = 500;
        final long popUpMs               = 250;
        final long popDownMs             = 50;
    }
    private AutoShootCtx autoShootCtx = null;

    String desiredPattern = null;
    String chamberOrder = "GPP";

    @Override
    public void init() {
        // Initialize constants for Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35.791, 135.000, Math.toRadians(90)));
        follower.updatePose();
        follower.setMaxPower(1);

        // Subsystem initialization
        shooter = new Shooter();
        shooter.init(hardwareMap, "motor1", "motor2",
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(0.008);
        turret.setKF(0.003);
        turret.setAngle(0);

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        popperServo = hardwareMap.get(Servo.class, "popperServo");
        colorSensor = new ColorV3(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        revolver = new ServoController(hardwareMap);
        revolver.moveServosToPosition(60);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        imu.resetYaw();

        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight3A.pipelineSwitch(6);
        limelight3A.setPollRateHz(50);
        limelight3A.start();

        // Start configuration
        gateServo.setPosition(0.38); // gate down
        popperServo.setPosition(0.15); // popper down
        shooter.setTargetRPM(0); // flywheel off

        turret.setAngle(90);

        pathTimer = new Timer();
        pathState = 0;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {


        follower.update();
        turret.update();
        revolver.update();

        telemetry.update();
    }

    @Override
    public void start() {
        turret.setAngle(90);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        turret.update();
        shooter.update();
        revolver.update();

        telemetryA.addData("State", pathState);
        telemetryA.addData("Flywheel RPM Target", "%.0f", shooter.getMasterRPM());
        telemetryA.addData("Turret Angle", "%.1f", turret.getCurrentAngle());
        telemetryA.addData("Follower Busy", follower.isBusy());
        telemetryA.update();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void autoPathUpdate() {
        switch (pathState) {
            case 0:
                // Spin up flywheel and start moving
                shooter.setTargetRPM(3300);
                Path toShoot1 = new Path(new BezierLine(
                        new Pose(35.791, 135),
                        new Pose(57, 85))
                );
                toShoot1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180), 0.8);
                follower.followPath(toShoot1, true);
                setPathState(1);
                break;

            case 1:
                // Wait until motion completes
                if (!follower.isBusy()) {
                    setPathState(101);
                }
                if ((pathTimer.getElapsedTimeSeconds() > 0)) {
                    LLResult result = limelight3A.getLatestResult();

                    int tag;

                    if (result.isValid()) {
                        tag = result.getFiducialResults().get(0).getFiducialId();
                    } else {
                        tag = 21;
                        telemetry.addLine("no tag in sight");
                    }

                    if (tag == 21) {
                        desiredPattern = "GPP";
                    } else if (tag == 22) {
                        desiredPattern = "PGP";
                    } else if (tag == 23) {
                        desiredPattern = "PPG";
                    }
                }
                break;

            case 101:
                LLResult result = limelight3A.getLatestResult();

                int tag;

                if (result.isValid()) {
                    tag = result.getFiducialResults().get(0).getFiducialId();
                } else {
                    tag = 21;
                    telemetry.addLine("no tag in sight");
                }

                if (tag == 21) {
                    desiredPattern = "GPP";
                } else if (tag == 22) {
                    desiredPattern = "PGP";
                } else if (tag == 23) {
                    desiredPattern = "PPG";
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    turret.setAngle(46.5);
                    setPathState(2);
                }
                break;

            case 2:
                // Example: chambers-at-start 60°='G', 180°='P', 300°='P', revolver aligned to 60°
                runAutoShoot3BallsFSM(desiredPattern, chamberOrder, 0);
                break;

            case 3:
                // All done
                shooter.setTargetRPM(0);
                intakeMotor.setPower(-1);
                gateServo.setPosition(0.48);
                Path toLine1 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48.75, 84))
                );
                toLine1.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toLine1, true);
                turret.setAngle(90);
                setPathState(4);
                break;

            case 4:
                // Wait until motion completes
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                follower.setMaxPower(0.35);
                // Begin pickup sequence – REMEMBER TO TURN INTAKE ON HERE
                Path forward1 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(follower.getPose().getX() - 4.5, follower.getPose().getY())
                ));
                forward1.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(forward1, true);
                setPathState(6);
                break;

            case 6:
                // Wait 0.8s, then drop gate, wait 0.2s, revolve 120°, raise gate
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    gateServo.setPosition(0.38); // gate down
                    setPathState(61); // intermediate substate
                }
                break;

            case 61:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    revolver.moveServosByRotation(120);
                    gateServo.setPosition(0.48); // gate up
                    Path forward2 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 5, follower.getPose().getY())
                    ));
                    forward2.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(forward2, true);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                // Second ball, same timing sequence
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gateServo.setPosition(0.38);
                    pathTimer.resetTimer();
                    setPathState(71);
                }
                break;

            case 71:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    revolver.moveServosByRotation(120);
                    gateServo.setPosition(0.48);
                    Path forward3 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 9.5, follower.getPose().getY())
                    ));
                    forward3.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(forward3, true);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                // Third ball
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gateServo.setPosition(0.38);
                    pathTimer.resetTimer();
                    setPathState(81);
                }
                break;

            case 81:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    gateServo.setPosition(0.48);
                    intakeMotor.setPower(0);
                    setPathState(9); // proceed to shooting again
                }
                break;

            case 9:
                // Start moving toward the target immediately, lower gate after 0.0s
                gateServo.setPosition(0.38); // gate down
                shooter.setTargetRPM(3300);  // spin up flywheel
                follower.setMaxPower(1);
                toShootAgain = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85) // same shoot point
                ));
                toShootAgain.setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        Math.toRadians(222) // same target heading as before
                );
                follower.followPath(toShootAgain, true);
                chamberOrder = "GPP";
                revolver.moveServosToPosition(60);
                pathTimer.resetTimer();
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    setPathState(2); // reuse AutoShoot3BallsFSM to fire again
                    nextPathStateAfterShooting = 11;
                }
                break;

            case 11:
                // Drive to lower pickup line (24 in lower Y)
                toLowerLine = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(49.5, 64)
                ));
                toLowerLine.setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180), 0.8);
                follower.followPath(toLowerLine, true);
                setPathState(12);
                // revolver.moveServosToPosition(0);
                break;

            case 12:
                // Wait for arrival, then start intake
                if (!follower.isBusy()) {
                    intakeMotor.setPower(-1);
                    gateServo.setPosition(0.48);
                    follower.setMaxPower(0.35);
                    Path pick1 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 6, follower.getPose().getY())
                    ));
                    pick1.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(pick1, true);
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                // Wait 0.8s, then drop gate, wait 0.2s, revolve 120°, raise gate
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    gateServo.setPosition(0.38); // gate down
                    setPathState(14); // intermediate substate
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    revolver.moveServosToPosition(120);
                    gateServo.setPosition(0.48); // gate up
                    Path forward2 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 4.5, follower.getPose().getY())
                    ));
                    forward2.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(forward2, true);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                // Second ball, same timing sequence
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gateServo.setPosition(0.38);
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    revolver.moveServosToPosition(240);
                    gateServo.setPosition(0.48);
                    Path forward3 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 5, follower.getPose().getY())
                    ));
                    forward3.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(forward3, true);
                    pathTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                // Third ball
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gateServo.setPosition(0.38);
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    intakeMotor.setPower(0);
                    setPathState(20); // proceed to shooting again
                }
                break;

            case 20:
                // Start moving toward the target immediately, lower gate after 0.0s
                gateServo.setPosition(0.38); // gate down
                shooter.setTargetRPM(3300);  // spin up flywheel
                follower.setMaxPower(1);
                toShootAgain = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(follower.getPose().getX()+20, follower.getPose().getY()),
                        new Pose(57, 85) // same shoot point
                ));
                toShootAgain.setTangentHeadingInterpolation();
                toShootAgain.reverseHeadingInterpolation();
                revolver.moveServosToPosition(60);
                chamberOrder = "PPG";
                follower.followPath(toShootAgain, true);
                pathTimer.resetTimer();
                setPathState(21);
                break;

            case 21:
                if (follower.getCurrentTValue() > 0.7) {
                    toShootAgain.setConstantHeadingInterpolation(Math.toRadians(222));
                }
                if (!follower.isBusy()) {
                    setPathState(2); // reuse AutoShoot3BallsFSM to fire again
                    nextPathStateAfterShooting = 22;
                }
                break;

            case 22:
                Path moveToEnd = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(37, 73)
                ));
                moveToEnd.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(moveToEnd, true);
                turret.setAngle(0);
                setPathState(23);
                break;

            case 23:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }

            default:
                break;
        }
    }

    // New overload: pattern and chamber order as strings.
    // chamberOrder maps as: [0]=60°, [1]=180°, [2]=300°
    private void runAutoShoot3BallsFSM(String desiredPattern,
                                       String chamberOrder,
                                       int currentIdxHint) {
        // sanitize
        if (chamberOrder == null || chamberOrder.length() != 3) {
            // fallback: unknown/empty chambers so the FSM will finish immediately
            chamberOrder = "   ";
        }
        char c60  = chamberOrder.charAt(0);
        char c180 = chamberOrder.charAt(1);
        char c300 = chamberOrder.charAt(2);

        // delegate to your working method (unchanged)
        runAutoShoot3BallsFSM(desiredPattern, c60, c180, c300, currentIdxHint);
    }

    // Pattern-driven AutoShoot3Balls with shortest-path rotation and continuous angle tracking
    private void runAutoShoot3BallsFSM(String desiredPattern,
                                       char chamber60, char chamber180, char chamber300,
                                       int currentIdxHint) {

        if (autoShootCtx == null) autoShootCtx = new AutoShootCtx();
        AutoShootCtx ctx = autoShootCtx;

        if (ctx.state == AutoShoot3BallsState.OFF) {
            if (desiredPattern == null || desiredPattern.length() != 3) desiredPattern = "GPP";
            ctx.pattern = desiredPattern.toUpperCase().toCharArray();

            ctx.chambers[0] = normColor(chamber60);
            ctx.chambers[1] = normColor(chamber180);
            ctx.chambers[2] = normColor(chamber300);

            ctx.currentIdx = (currentIdxHint >= 0 && currentIdxHint <= 2) ? currentIdxHint : 0;
            ctx.currentAngle = ANG[ctx.currentIdx];  // track continuous angle from the start

            buildShootPlan(ctx); // fills ctx.shootIdx[0..2] deterministically

            ctx.state = AutoShoot3BallsState.MOVE_TO_START;
            ctx.popCount = 0;
            ctx.planPos  = 0;
            gateServo.setPosition(0.38); // gate down

            if (ctx.shootIdx[0] == -1) {
                ctx.state = AutoShoot3BallsState.DONE;
            } else {
                gotoIdxShortest(ctx, ctx.shootIdx[0]); // shortest-path signed delta
                ctx.timerMs = System.currentTimeMillis() + ctx.autoInitialMoveDelayMs;
            }
        }

        long now = System.currentTimeMillis();

        switch (ctx.state) {
            case MOVE_TO_START:
                if (now >= ctx.timerMs) {
                    popperServo.setPosition(0.45);
                    ctx.timerMs = now + ctx.popUpMs;
                    ctx.state = AutoShoot3BallsState.POP_UP;
                }
                break;

            case POP_UP:
                if (now >= ctx.timerMs) {
                    popperServo.setPosition(.15);
                    ctx.timerMs = now + ctx.popDownMs;
                    ctx.state = AutoShoot3BallsState.POP_DOWN;
                }
                break;

            case POP_DOWN:
                if (now >= ctx.timerMs) {
                    ctx.popCount++;
                    ctx.chambers[ctx.currentIdx] = ' '; // consumed

                    if (ctx.popCount < 3) {
                        ctx.planPos++;
                        int nextIdx = ctx.shootIdx[ctx.planPos];
                        if (nextIdx == -1) {
                            ctx.state = AutoShoot3BallsState.RETURN_TO_START;
                            gotoAngleShortest(ctx, 0.0); // go home via shortest path
                            ctx.timerMs = now + ctx.autoInitialMoveDelayMs;
                        } else {
                            ctx.state = AutoShoot3BallsState.REVOLVE;
                            gotoIdxShortest(ctx, nextIdx); // shortest-path signed delta
                            ctx.timerMs = now + ctx.postRevolveDelayMs;
                        }
                    } else {
                        ctx.state = AutoShoot3BallsState.RETURN_TO_START;
                        gotoAngleShortest(ctx, 0.0);      // go home via shortest path
                        ctx.timerMs = now + ctx.autoInitialMoveDelayMs;
                    }
                }
                break;

            case REVOLVE:
                if (now >= ctx.timerMs) {
                    ctx.state = AutoShoot3BallsState.WAIT_TO_SETTLE;
                }
                break;

            case WAIT_TO_SETTLE:
                popperServo.setPosition(0.45);
                ctx.timerMs = now + ctx.popUpMs;
                ctx.state = AutoShoot3BallsState.POP_UP;
                break;

            case RETURN_TO_START:
                if (now >= ctx.timerMs) {
                    ctx.state = AutoShoot3BallsState.DONE;
                    // no extra 60° nudge; you end exactly at 0°
                }
                break;

            case DONE:
                shooter.setTargetRPM(0);
                setPathState(nextPathStateAfterShooting);
                revolver.moveServosToPosition(0);
                autoShootCtx = null; // reset
                break;

            case OFF:
            default:
                break;
        }
    }

// ---- helpers (class scope) ----

    // Track continuous angle to avoid drift
    // ---- helpers (class scope) ----
// ABSOLUTE version: no incremental deltas; no accumulation.
// ctx.currentAngle is always set to the commanded absolute angle.

    /** Go to a specific pocket index using absolute angle. */
    private void gotoIdxShortest(AutoShootCtx ctx, int targetIdx) {
        if (targetIdx < 0 || targetIdx > 2) return;
        double target = normalize360(ANG[targetIdx]);
        revolver.moveServosToPosition(target);
        ctx.currentAngle = target;          // trust commanded absolute
        ctx.currentIdx   = targetIdx;       // exact pocket by construction
    }

    /** Go to an arbitrary absolute angle (e.g., home = 0°). */
    private void gotoAngleShortest(AutoShootCtx ctx, double targetAngle) {
        double target = normalize360(targetAngle);
        revolver.moveServosToPosition(target);
        ctx.currentAngle = target;                          // trust commanded absolute
        ctx.currentIdx   = snapToNearestPocket(target, ANG);
    }

    /** Choose nearest pocket to an absolute angle. */
    private int snapToNearestPocket(double angleDeg, double[] pockets) {
        int best = 0;
        double bestErr = 1e9;
        for (int i = 0; i < pockets.length; i++) {
            double pi = normalize360(pockets[i]);
            double err = Math.abs(wrapTo180(angleDeg - pi));
            if (err < bestErr) { bestErr = err; best = i; }
        }
        return best;
    }

    /** Map any angle to (-180, 180]. */
    private double wrapTo180(double a) {
        a = ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        if (a <= -180.0) a += 360.0;
        return a;
    }

    /** Map any angle to [0, 360). */
    private double normalize360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    private char normColor(char c) {
        char u = Character.toUpperCase(c);
        return (u == 'G' || u == 'P') ? u : ' ';
    }

    /** Build the three-shot plan in requested color order, choosing forward-nearest indices. */
    private void buildShootPlan(AutoShootCtx ctx) {
        java.util.List<Integer> gIdx = new java.util.ArrayList<>(2);
        java.util.List<Integer> pIdx = new java.util.ArrayList<>(2);
        for (int i = 0; i < 3; i++) {
            if (ctx.chambers[i] == 'G') gIdx.add(i);
            else if (ctx.chambers[i] == 'P') pIdx.add(i);
        }
        int planCursor = ctx.currentIdx;
        for (int k = 0; k < 3; k++) {
            char want = ctx.pattern[k];
            int chosen = -1;
            java.util.List<Integer> pool = (want == 'G') ? gIdx : (want == 'P') ? pIdx : null;
            if (pool != null && !pool.isEmpty()) {
                chosen = pickNearestForward(planCursor, pool);
                if (chosen != -1) planCursor = chosen;
            }
            ctx.shootIdx[k] = chosen; // -1 if unavailable
        }
    }

    /** Forward distance on a 3-pocket wheel; removes chosen candidate from the pool. */
    private int pickNearestForward(int fromIdx, java.util.List<Integer> candidates) {
        int best = -1, bestDist = 4;
        for (int idx : candidates) {
            int dist = (idx - fromIdx + 3) % 3; // 0..2 forward steps
            if (dist < bestDist) { bestDist = dist; best = idx; }
        }
        if (best != -1) {
            for (int i = 0; i < candidates.size(); i++) {
                if (candidates.get(i) == best) { candidates.remove(i); break; }
            }
        }
        return best;
    }
}