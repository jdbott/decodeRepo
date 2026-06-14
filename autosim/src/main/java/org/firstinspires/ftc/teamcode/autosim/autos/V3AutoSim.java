package org.firstinspires.ftc.teamcode.autosim.autos;

import org.firstinspires.ftc.teamcode.autoshared.V3AutoConfig;
import org.firstinspires.ftc.teamcode.autosim.geom.Bezier;
import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
import org.firstinspires.ftc.teamcode.autosim.model.ActionEvent;
import org.firstinspires.ftc.teamcode.autosim.model.Category;
import org.firstinspires.ftc.teamcode.autosim.model.Frame;
import org.firstinspires.ftc.teamcode.autosim.model.PathSpec;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;
import org.firstinspires.ftc.teamcode.autosim.sim.SimClock;
import org.firstinspires.ftc.teamcode.autosim.sim.SimFeeder;
import org.firstinspires.ftc.teamcode.autosim.sim.SimFlywheel;
import org.firstinspires.ftc.teamcode.autosim.sim.SimFollower;
import org.firstinspires.ftc.teamcode.autosim.sim.SimHood;
import org.firstinspires.ftc.teamcode.autosim.sim.SimIntake;
import org.firstinspires.ftc.teamcode.autosim.sim.SimStopwatch;
import org.firstinspires.ftc.teamcode.autosim.sim.SimTurret;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Simulatable copy of V3Auto ("V3 Auto FSM", BLUE-NATIVE). The 25-state {@code AutoState} machine,
 * the {@code FeedState} machine, path geometry, and timing are reproduced from
 * {@code V3Auto.java}; constants come from the shared {@link V3AutoConfig} so the two stay in
 * lockstep. See AUTOSIM_DESIGN.md §5/§9.
 *
 * <p>Deliberate simplifications (readouts only — path, state order, timing, intake, shot events,
 * and heading are faithful):
 * <ul>
 *   <li>Shot-on-move turret/hood compensation ({@code trackGoalFromOdometry} with
 *       predicted-distance filtering) is reduced to straight-line goal tracking + the same shot
 *       table.</li>
 * </ul>
 *
 * <p>{@code toLastLine} and {@code backToShootFromLastLine} reproduce the real auto's mid-path
 * heading-mode switches ({@link PathSpec#thenConstant}): {@code toLastLine} starts TANGENT and
 * switches to a constant 180° once {@code getCurrentTValue() > 0.2}; {@code backToShootFromLastLine}
 * starts REVERSE_TANGENT and switches to a constant -90° once {@code getCurrentTValue() > 0.8}.
 */
public final class V3AutoSim {

    private static final double DT_SEC = 0.02;
    private static final long MAX_MS = 60_000;
    private static final int CURVE_SEGMENTS = 40;

    // Shared geometry/tunables.
    private static final double START_X = V3AutoConfig.START_X;
    private static final double START_Y = V3AutoConfig.START_Y;
    private static final double START_HEADING_DEG = V3AutoConfig.START_HEADING_DEG;
    private static final double FIRST_SHOT_HOOD_DEG = V3AutoConfig.FIRST_SHOT_HOOD_DEG;
    private static final double FIRST_SHOT_FLYWHEEL_RAD = V3AutoConfig.FIRST_SHOT_FLYWHEEL_RAD;
    private static final double BLUE_TARGET_X = V3AutoConfig.BLUE_TARGET_X;
    private static final double TARGET_Y = V3AutoConfig.TARGET_Y;
    private static final double TURRET_CENTER_OFFSET_IN = V3AutoConfig.TURRET_CENTER_OFFSET_IN;
    private static final double TURRET_MIN_DEG = V3AutoConfig.TURRET_MIN_DEG;
    private static final double TURRET_MAX_DEG = V3AutoConfig.TURRET_MAX_DEG;
    private static final double TURRET_OFFSET_DEG = V3AutoConfig.TURRET_OFFSET_DEG;
    private static final double INIT_TURRET_ANGLE_DEG = V3AutoConfig.INIT_TURRET_ANGLE_DEG;
    private static final double EXTRA_GATE_INTAKE_Y_IN = V3AutoConfig.EXTRA_GATE_INTAKE_Y_IN;
    private static final double FEED_START_DELAY_SEC = V3AutoConfig.FEED_START_DELAY_SEC;
    private static final double FEED_TOTAL_TIME_SEC = V3AutoConfig.FEED_TOTAL_TIME_SEC;
    private static final double REVERSE_TIME_SEC = V3AutoConfig.REVERSE_TIME_SEC;

    // Headings
    private static final double LINE_H = V3AutoConfig.LINE_HEADING_DEG;
    private static final double GATE_INTAKE_H = V3AutoConfig.GATE_INTAKE_HEADING_DEG;
    private static final double GATE_OPEN_H_START = V3AutoConfig.GATE_OPEN_HEADING_START_DEG;
    private static final double GATE_OPEN_H_END = V3AutoConfig.GATE_OPEN_HEADING_END_DEG;
    private static final double GATE_OPEN_H_T = V3AutoConfig.GATE_OPEN_HEADING_T;
    private static final double BACK_TO_SHOOT_LAST_H = V3AutoConfig.BACK_TO_SHOOT_LAST_HEADING_DEG;

    // t-value / timer gates
    private static final double FIRST_SHOT_FEED_TVALUE = V3AutoConfig.FIRST_SHOT_FEED_TVALUE;
    private static final double LINE2_SLOW_TVALUE = V3AutoConfig.LINE2_SLOW_TVALUE;
    private static final double LINE2_SLOW_POWER = V3AutoConfig.LINE2_SLOW_POWER;
    private static final double WAIT_AT_GATE_SEC = V3AutoConfig.WAIT_AT_GATE_SEC;
    private static final double GATE_INTAKE_ON_TVALUE = V3AutoConfig.GATE_INTAKE_ON_TVALUE;
    private static final double GATE_INTAKE_SEC = V3AutoConfig.GATE_INTAKE_SEC;
    private static final double EXTRA_MOVE_TIMEOUT_SEC = V3AutoConfig.EXTRA_MOVE_TIMEOUT_SEC;
    private static final double WAIT_AT_GATE_AGAIN_SEC = V3AutoConfig.WAIT_AT_GATE_AGAIN_SEC;
    private static final double GATE_INTAKE_AGAIN_SEC = V3AutoConfig.GATE_INTAKE_AGAIN_SEC;
    private static final double LAST_LINE_SLOW_TVALUE = V3AutoConfig.LAST_LINE_SLOW_TVALUE;
    private static final double LAST_LINE_SLOW_POWER = V3AutoConfig.LAST_LINE_SLOW_POWER;
    private static final double BACK_TO_SHOOT_LAST_SWITCH_TVALUE = V3AutoConfig.BACK_TO_SHOOT_LAST_SWITCH_TVALUE;

    // Distance -> {hoodDeg, flywheelRad}
    private static final double[][] SHOT_TABLE = V3AutoConfig.SHOT_TABLE;

    private enum FeedState { IDLE, WAIT_BEFORE_INTAKE, RUN_INTAKE, DONE }
    private enum AutoState {
        DRIVE_TO_FIRST_SHOT, WAIT_FOR_FIRST_SHOT_TO_FINISH,
        DRIVE_TO_LINE2, DRIVE_BACK_TO_SHOOT, SHOOT_SECOND,
        DRIVE_TO_GATE, WAIT_AT_GATE, WAIT_FOR_GATE_INTAKE, DRIVE_EXTRA_INTAKE_AT_GATE, DRIVE_BACK_TO_SHOOT_THIRD, SHOOT_THIRD,
        DRIVE_TO_FOURTH_PICKUP, DRIVE_BACK_TO_FINAL_SHOOT, SHOOT_FOURTH,
        DRIVE_TO_GATE_AGAIN, WAIT_AT_GATE_AGAIN, WAIT_FOR_GATE_INTAKE_AGAIN, DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN, DRIVE_BACK_TO_SHOOT_FIFTH, SHOOT_FIFTH,
        DRIVE_TO_LAST_LINE, DRIVE_BACK_TO_SHOOT_LAST, SHOOT_LAST,
        DONE
    }

    private final SimTrace trace = new SimTrace();
    private final SimClock clock = new SimClock();
    private final SimFollower follower = new SimFollower(clock, trace);
    private final SimIntake intake = new SimIntake();
    private final SimFlywheel flywheel = new SimFlywheel();
    private final SimTurret turret = new SimTurret();
    private final SimHood hood = new SimHood();
    private final SimFeeder feeder = new SimFeeder();

    private final SimStopwatch feedTimer = new SimStopwatch(clock);
    private final SimStopwatch autoTimer = new SimStopwatch(clock);
    private final SimStopwatch reverseTimer = new SimStopwatch(clock);

    private boolean reversingIntake = false;
    private boolean enableDynamicShotControl = false;
    private double hoodAngleDeg = 50.0;
    private double targetVelocityRad = 0.0;

    private FeedState feedState = FeedState.IDLE;
    private AutoState autoState = AutoState.DRIVE_TO_FIRST_SHOT;

    private Pt startPt, firstShot;

    // Static paths
    private PathSpec toFirstShot, toLine2, backToShoot, toGateOpenGate, toGateIntake,
            toFourthPickup, backToFinalShoot, toLastLine, backToShootFromLastLine;
    // Dynamic paths (rebuilt from the live pose)
    private PathSpec gateExtraIntakeMove, backToShootFromGate;

    private final List<Long> shotTimes = new ArrayList<>();
    private final List<Pose2d> shotPoses = new ArrayList<>();

    public static SimTrace build() {
        return new V3AutoSim().run();
    }

    private SimTrace run() {
        trace.meta.autoName = "V3Auto";
        trace.meta.simDtMillis = (int) Math.round(DT_SEC * 1000);
        trace.field.goal = new Pt(BLUE_TARGET_X, TARGET_Y);
        trace.field.backgroundImage = "fields/decode.svg";
        trace.robot.lengthIn = 18.0;
        trace.robot.widthIn = 13.0;
        trace.robot.wheelbaseOffsetIn = 3.5;

        startPt = new Pt(START_X, START_Y);
        double hr = Math.toRadians(START_HEADING_DEG);
        firstShot = new Pt(START_X + V3AutoConfig.FIRST_SHOT_FWD_IN * Math.cos(hr), START_Y + V3AutoConfig.FIRST_SHOT_SIDE_IN * Math.sin(hr));
        trace.meta.startPose = new Pose2d(START_X, START_Y, START_HEADING_DEG);

        follower.setStartingPose(trace.meta.startPose);
        buildStaticPaths();

        intake.setPower(0.0);
        flywheel.stop();
        feeder.clutchIn();
        feeder.armBlock();
        hood.setAngle(FIRST_SHOT_HOOD_DEG);
        turret.setAngle(INIT_TURRET_ANGLE_DEG);

        hoodAngleDeg = FIRST_SHOT_HOOD_DEG;
        targetVelocityRad = FIRST_SHOT_FLYWHEEL_RAD;
        hood.setAngle(hoodAngleDeg);
        flywheel.setTargetVelocity(targetVelocityRad);
        follower.followPath(toFirstShot, false);
        autoState = AutoState.DRIVE_TO_FIRST_SHOT;

        while (true) {
            follower.update();
            updateShotControl();
            flywheel.update(DT_SEC);
            updateFeedSequence();
            updateAutoState();
            turret.update();

            trace.frames.add(sampleFrame());

            if (autoState == AutoState.DONE && !follower.isBusy()) break;
            if (clock.millis() >= MAX_MS) break;
            clock.advance(DT_SEC);
        }

        trace.meta.totalMillis = clock.millis();
        buildEvents();
        return trace;
    }

    private Frame sampleFrame() {
        Frame f = new Frame();
        f.tMs = clock.millis();
        f.pose = follower.getPose();
        f.autoState = autoState.name();
        f.feedState = feedState.name();
        f.followerBusy = follower.isBusy();
        f.flywheelTargetRadS = flywheel.getTarget();
        f.flywheelActualRadS = flywheel.getVelocityRadPerSec();
        f.turretDeg = turret.getCurrentAngle();
        f.hoodDeg = hood.angle;
        f.intakePower = intake.power;
        return f;
    }

    /** Turret goal-tracking (readout only) + hood/flywheel selection. */
    private void updateShotControl() {
        if (autoState == AutoState.DONE) {
            turret.setAngle(0.0);
            return;
        }
        Pose2d pose = follower.getPose();
        double hr = Math.toRadians(pose.headingDeg);
        double turretX = pose.x - TURRET_CENTER_OFFSET_IN * Math.cos(hr);
        double turretY = pose.y - TURRET_CENTER_OFFSET_IN * Math.sin(hr);
        double angleFieldDeg = Math.toDegrees(Math.atan2(TARGET_Y - turretY, BLUE_TARGET_X - turretX));
        double angleRobotDeg = normalize180(angleFieldDeg - pose.headingDeg);
        double desiredDeg = normalize180(angleRobotDeg + TURRET_OFFSET_DEG);
        turret.setAngle(Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, desiredDeg)));

        if (enableDynamicShotControl) {
            double dist = Math.hypot(BLUE_TARGET_X - pose.x, TARGET_Y - pose.y);
            updateShotFromDistance(dist);
        } else {
            hoodAngleDeg = FIRST_SHOT_HOOD_DEG;
            hood.setAngle(hoodAngleDeg);
            targetVelocityRad = FIRST_SHOT_FLYWHEEL_RAD;
        }
        flywheel.setTargetVelocity(targetVelocityRad);
    }

    private void updateAutoState() {
        switch (autoState) {
            case DRIVE_TO_FIRST_SHOT:
                if (feedState == FeedState.IDLE && follower.getCurrentTValue() > FIRST_SHOT_FEED_TVALUE) {
                    startFeedSequence();
                }
                if (!follower.isBusy()) {
                    if (feedState == FeedState.DONE) finishFirstShotAndStartLine2();
                    else autoState = AutoState.WAIT_FOR_FIRST_SHOT_TO_FINISH;
                }
                break;

            case WAIT_FOR_FIRST_SHOT_TO_FINISH:
                if (feedState == FeedState.DONE) finishFirstShotAndStartLine2();
                break;

            case DRIVE_TO_LINE2:
                if (follower.getCurrentTValue() > LINE2_SLOW_TVALUE) {
                    follower.setMaxPower(LINE2_SLOW_POWER);
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(backToShoot, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT;
                    enableDynamicShotControl = true;
                }
                break;

            case DRIVE_BACK_TO_SHOOT:
                follower.setMaxPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_SECOND;
                }
                break;

            case SHOOT_SECOND:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock(); feeder.clutchOut();
                    startThirdCycleToGate();
                }
                break;

            // ---- Gate cycle ----
            case DRIVE_TO_GATE:
                intake.setPower(0);
                if (!follower.isBusy()) { autoTimer.reset(); autoState = AutoState.WAIT_AT_GATE; }
                break;
            case WAIT_AT_GATE:
                intake.setPower(0.0);
                if (autoTimer.seconds() >= WAIT_AT_GATE_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, false);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE;
                }
                break;
            case WAIT_FOR_GATE_INTAKE:
                intake.setPower(follower.getCurrentTValue() >= GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= GATE_INTAKE_SEC) startExtraGateIntakeMove();
                break;
            case DRIVE_EXTRA_INTAKE_AT_GATE:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= EXTRA_MOVE_TIMEOUT_SEC) startReturnFromGateToShoot();
                break;
            case DRIVE_BACK_TO_SHOOT_THIRD:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_THIRD; }
                break;
            case SHOOT_THIRD:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock(); feeder.clutchOut();
                    startFourthPickupCycle();
                }
                break;

            // ---- Fourth pickup (no gate) ----
            case DRIVE_TO_FOURTH_PICKUP:
                if (!follower.isBusy()) startReturnToFinalShoot();
                break;
            case DRIVE_BACK_TO_FINAL_SHOOT:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_FOURTH; }
                break;
            case SHOOT_FOURTH:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock(); feeder.clutchOut();
                    startGateCycleAgain();
                }
                break;

            // ---- Gate cycle again ----
            case DRIVE_TO_GATE_AGAIN:
                intake.setPower(0);
                if (!follower.isBusy()) { autoTimer.reset(); autoState = AutoState.WAIT_AT_GATE_AGAIN; }
                break;
            case WAIT_AT_GATE_AGAIN:
                intake.setPower(0.0);
                if (autoTimer.seconds() >= WAIT_AT_GATE_AGAIN_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_AGAIN;
                }
                break;
            case WAIT_FOR_GATE_INTAKE_AGAIN:
                intake.setPower(follower.getCurrentTValue() >= GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= GATE_INTAKE_AGAIN_SEC) startExtraGateIntakeMoveAgain();
                break;
            case DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= EXTRA_MOVE_TIMEOUT_SEC) startReturnFromGateToShootAgain();
                break;
            case DRIVE_BACK_TO_SHOOT_FIFTH:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_FIFTH; }
                break;
            case SHOOT_FIFTH:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock(); feeder.clutchOut();
                    startLastLineCycle();
                }
                break;

            // ---- Last line ----
            case DRIVE_TO_LAST_LINE:
                if (follower.getCurrentTValue() > LAST_LINE_SLOW_TVALUE) {
                    follower.setMaxPower(LAST_LINE_SLOW_POWER);
                }
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    follower.setMaxPower(1.0);
                    follower.followPath(backToShootFromLastLine, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT_LAST;
                    enableDynamicShotControl = true;
                }
                break;
            case DRIVE_BACK_TO_SHOOT_LAST:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_LAST; }
                break;
            case SHOOT_LAST:
                if (feedState == FeedState.DONE) {
                    intake.setPower(0.0);
                    feeder.armBlock(); feeder.clutchOut();
                    enableDynamicShotControl = false;
                    turret.setAngle(0);
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                turret.setAngle(0);
                intake.setPower(0.0);
                break;
        }
    }

    private void finishFirstShotAndStartLine2() {
        feeder.clutchOut();
        feeder.armBlock();
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toLine2, false);
        autoState = AutoState.DRIVE_TO_LINE2;
    }

    private void startThirdCycleToGate() {
        intake.setPower(0.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE;
    }

    private void startExtraGateIntakeMove() {
        follower.setMaxPower(1.0);
        Pose2d cur = follower.getPose();
        gateExtraIntakeMove = line("gateExtra", new Pt(cur.x, cur.y),
                new Pt(cur.x, cur.y + EXTRA_GATE_INTAKE_Y_IN), GATE_INTAKE_H);
        follower.followPath(gateExtraIntakeMove, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE;
    }

    private void startReturnFromGateToShoot() {
        follower.setMaxPower(1.0);
        Pose2d cur = follower.getPose();
        backToShootFromGate = curve("backFromGate",
                Arrays.asList(new Pt(cur.x, cur.y), new Pt(V3AutoConfig.RG1_C1X, V3AutoConfig.RG1_C1Y), firstShot),
                tangentDeg(new Pt(cur.x, cur.y), firstShot)).reverseTangent();
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.followPath(backToShootFromGate, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_THIRD;
    }

    private void startFourthPickupCycle() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toFourthPickup, false);
        autoState = AutoState.DRIVE_TO_FOURTH_PICKUP;
    }

    private void startReturnToFinalShoot() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(backToFinalShoot, false);
        autoState = AutoState.DRIVE_BACK_TO_FINAL_SHOOT;
    }

    private void startGateCycleAgain() {
        intake.setPower(0.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE_AGAIN;
    }

    private void startExtraGateIntakeMoveAgain() {
        follower.setMaxPower(1.0);
        Pose2d cur = follower.getPose();
        gateExtraIntakeMove = line("gateExtraAgain", new Pt(cur.x, cur.y),
                new Pt(cur.x, cur.y + EXTRA_GATE_INTAKE_Y_IN), GATE_INTAKE_H);
        follower.followPath(gateExtraIntakeMove, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN;
    }

    private void startReturnFromGateToShootAgain() {
        follower.setMaxPower(1.0);
        Pose2d cur = follower.getPose();
        backToShootFromGate = curve("backFromGateAgain",
                Arrays.asList(new Pt(cur.x, cur.y), new Pt(V3AutoConfig.RG2_C1X, V3AutoConfig.RG2_C1Y), firstShot),
                tangentDeg(new Pt(cur.x, cur.y), firstShot)).reverseTangent();
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.followPath(backToShootFromGate, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_FIFTH;
    }

    private void startLastLineCycle() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toLastLine, false);
        autoState = AutoState.DRIVE_TO_LAST_LINE;
    }

    private void startFeedSequence() {
        feeder.armShoot();
        feedTimer.reset();
        feedState = FeedState.WAIT_BEFORE_INTAKE;
        shotTimes.add(clock.millis());
        shotPoses.add(follower.getPose());
    }

    private void updateFeedSequence() {
        if (reversingIntake) {
            if (reverseTimer.seconds() < REVERSE_TIME_SEC) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(1.0);
                reversingIntake = false;
            }
        }
        switch (feedState) {
            case IDLE:
                break;
            case WAIT_BEFORE_INTAKE:
                feeder.clutchIn();
                intake.setPower(0.0);
                if (feedTimer.seconds() >= FEED_START_DELAY_SEC) {
                    intake.setPower(1.0);
                    feedState = FeedState.RUN_INTAKE;
                }
                break;
            case RUN_INTAKE:
                if (feedTimer.seconds() >= FEED_TOTAL_TIME_SEC) {
                    reversingIntake = true;
                    reverseTimer.reset();
                    intake.setPower(0.0);
                    feedState = FeedState.DONE;
                    feeder.clutchOut();
                }
                break;
            case DONE:
                break;
        }
    }

    private void updateShotFromDistance(double distance) {
        double[][] t = SHOT_TABLE;
        if (distance <= t[0][0]) {
            hoodAngleDeg = t[0][1]; targetVelocityRad = t[0][2]; hood.setAngle(hoodAngleDeg); return;
        }
        int last = t.length - 1;
        if (distance >= t[last][0]) {
            hoodAngleDeg = t[last][1]; targetVelocityRad = t[last][2]; hood.setAngle(hoodAngleDeg); return;
        }
        for (int i = 0; i < t.length - 1; i++) {
            double d1 = t[i][0], a1 = t[i][1], v1 = t[i][2];
            double d2 = t[i + 1][0], a2 = t[i + 1][1], v2 = t[i + 1][2];
            if (distance >= d1 && distance <= d2) {
                double f = (distance - d1) / (d2 - d1);
                hoodAngleDeg = a1 + f * (a2 - a1);
                targetVelocityRad = (v1 + f * (v2 - v1)) + 15;
                hood.setAngle(hoodAngleDeg);
                return;
            }
        }
    }

    private void buildEvents() {
        long feedWindowMs = Math.round((FEED_START_DELAY_SEC + FEED_TOTAL_TIME_SEC) * 1000.0);
        for (int i = 0; i < shotTimes.size(); i++) {
            trace.events.add(new ActionEvent(shotTimes.get(i), Category.SHOOT, "shot " + (i + 1), feedWindowMs, shotPoses.get(i)));
        }
        List<Frame> fr = trace.frames;
        int i = 0;
        while (i < fr.size()) {
            if (fr.get(i).intakePower > 0.05) {
                long start = fr.get(i).tMs;
                Pose2d at = fr.get(i).pose;
                int j = i;
                while (j < fr.size() && fr.get(j).intakePower > 0.05) j++;
                long end = fr.get(j - 1).tMs;
                trace.events.add(new ActionEvent(start, Category.INTAKE, "intake", Math.max((long) (DT_SEC * 1000), end - start), at));
                i = j;
            } else {
                i++;
            }
        }
        trace.events.sort((a, b) -> Long.compare(a.tMs, b.tMs));
    }

    // ===== Path builders =====

    private void buildStaticPaths() {
        toFirstShot = line("toFirstShot", startPt, firstShot, tangentDeg(startPt, firstShot)).tangent();

        toLine2 = curve("toLine2", Arrays.asList(firstShot,
                new Pt(V3AutoConfig.LINE2_C1X, V3AutoConfig.LINE2_C1Y),
                new Pt(V3AutoConfig.LINE2_C2X, V3AutoConfig.LINE2_C2Y),
                new Pt(V3AutoConfig.LINE2_ENDX, V3AutoConfig.LINE2_ENDY)), LINE_H);

        backToShoot = curve("backToShoot", Arrays.asList(
                new Pt(V3AutoConfig.BTS_STARTX, V3AutoConfig.BTS_STARTY),
                new Pt(V3AutoConfig.BTS_C1X, V3AutoConfig.BTS_C1Y), firstShot),
                tangentDeg(new Pt(V3AutoConfig.BTS_STARTX, V3AutoConfig.BTS_STARTY), firstShot)).reverseTangent();

        toGateOpenGate = curve("toGateOpenGate", Arrays.asList(firstShot,
                new Pt(V3AutoConfig.GATE_C1X, V3AutoConfig.GATE_C1Y),
                new Pt(V3AutoConfig.GATE_ENDX, V3AutoConfig.GATE_ENDY)), GATE_OPEN_H_END)
                .linear(GATE_OPEN_H_START, GATE_OPEN_H_END, GATE_OPEN_H_T);

        toGateIntake = line("toGateIntake", new Pt(V3AutoConfig.GI_STARTX, V3AutoConfig.GI_STARTY),
                new Pt(V3AutoConfig.GI_ENDX, V3AutoConfig.GI_ENDY), GATE_INTAKE_H);

        toFourthPickup = curve("toFourthPickup", Arrays.asList(firstShot,
                new Pt(V3AutoConfig.FP_C1X, V3AutoConfig.FP_C1Y),
                new Pt(V3AutoConfig.FP_ENDX, V3AutoConfig.FP_ENDY)), LINE_H);

        Pt fourthEnd = new Pt(V3AutoConfig.FP_ENDX, V3AutoConfig.FP_ENDY);
        backToFinalShoot = line("backToFinalShoot", fourthEnd, firstShot, tangentDeg(fourthEnd, firstShot))
                .reverseTangent();

        toLastLine = curve("toLastLine", Arrays.asList(firstShot,
                new Pt(V3AutoConfig.LL_C1X, V3AutoConfig.LL_C1Y),
                new Pt(V3AutoConfig.LL_C2X, V3AutoConfig.LL_C2Y),
                new Pt(V3AutoConfig.LL_ENDX, V3AutoConfig.LL_ENDY)),
                LINE_H).tangent().thenConstant(LAST_LINE_SLOW_TVALUE, LINE_H);

        Pt lastReturnEnd = new Pt(firstShot.x + V3AutoConfig.LAST_RETURN_END_DX, firstShot.y + V3AutoConfig.LAST_RETURN_END_DY);
        backToShootFromLastLine = curve("backToShootFromLastLine", Arrays.asList(
                new Pt(V3AutoConfig.LL_ENDX, V3AutoConfig.LL_ENDY),
                new Pt(V3AutoConfig.BFLL_C1X, V3AutoConfig.BFLL_C1Y), lastReturnEnd),
                tangentDeg(new Pt(V3AutoConfig.BFLL_STARTX, V3AutoConfig.BFLL_STARTY), lastReturnEnd)).reverseTangent()
                .thenConstant(BACK_TO_SHOOT_LAST_SWITCH_TVALUE, BACK_TO_SHOOT_LAST_H);
    }

    private static double tangentDeg(Pt a, Pt b) {
        return Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    }

    private static double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    private static PathSpec line(String id, Pt a, Pt b, double headingDeg) {
        List<Pt> cp = Arrays.asList(a, b);
        return new PathSpec(id, "LINE", cp, Bezier.sample(cp, 1), headingDeg);
    }

    private static PathSpec curve(String id, List<Pt> control, double headingDeg) {
        return new PathSpec(id, "CURVE", control, Bezier.sample(control, CURVE_SEGMENTS), headingDeg);
    }
}
