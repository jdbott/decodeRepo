package org.firstinspires.ftc.teamcode.autosim.autos;

import org.firstinspires.ftc.teamcode.autoshared.V3ClosePartnerConfig;
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
 * Simulatable copy of V3ClosePartner (BLUE-NATIVE). The 25-state {@code AutoState} machine, the
 * {@code FeedState} machine, the path geometry, and the timing are reproduced from
 * {@code V3ClosePartner.java}; constants come from the shared {@link V3ClosePartnerConfig} so the
 * two stay in lockstep. See AUTOSIM_DESIGN.md §5/§9.
 *
 * <p>Deliberate simplification: the real auto's shot-on-move turret/hood compensation
 * ({@code trackGoalFromOdometry} with predicted-distance filtering) is reduced here to plain
 * straight-line goal tracking + the same {@code SHOT_TABLE} lookup. That only affects the
 * turret/hood/flywheel <i>readouts</i>; the path, state order, timing, intake, and shot events
 * are faithful. The original V3ClosePartner.java is never imported or modified.
 */
public final class V3ClosePartnerSim {

    private static final double DT_SEC = 0.02;
    private static final long MAX_MS = 60_000;
    private static final int CURVE_SEGMENTS = 40;

    // Shared geometry/tunables.
    private static final double START_X = V3ClosePartnerConfig.START_X;
    private static final double START_Y = V3ClosePartnerConfig.START_Y;
    private static final double START_HEADING_DEG = V3ClosePartnerConfig.START_HEADING_DEG;
    private static final double BLUE_TARGET_X = V3ClosePartnerConfig.BLUE_TARGET_X;
    private static final double TARGET_Y = V3ClosePartnerConfig.TARGET_Y;
    private static final double FIRST_SHOT_HOOD_DEG = V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG;
    private static final double FIRST_SHOT_FLYWHEEL_RAD = V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD;
    private static final double TURRET_CENTER_OFFSET_IN = V3ClosePartnerConfig.TURRET_CENTER_OFFSET_IN;
    private static final double TURRET_OFFSET_DEG = V3ClosePartnerConfig.TURRET_OFFSET_DEG;
    private static final double EXTRA_GATE_INTAKE_Y_IN = V3ClosePartnerConfig.EXTRA_GATE_INTAKE_Y_IN;
    private static final double LINE_H = V3ClosePartnerConfig.LINE_HEADING_DEG;
    private static final double GATE_INTAKE_H = V3ClosePartnerConfig.GATE_INTAKE_HEADING_DEG;
    private static final double GATE_OPEN_H_END = V3ClosePartnerConfig.GATE_OPEN_HEADING_END_DEG;
    private static final double FEED_START_DELAY_SEC = V3ClosePartnerConfig.FEED_START_DELAY_SEC;
    private static final double FEED_TOTAL_TIME_SEC = V3ClosePartnerConfig.FEED_TOTAL_TIME_SEC;
    private static final double REVERSE_TIME_SEC = V3ClosePartnerConfig.REVERSE_TIME_SEC;

    private enum FeedState { IDLE, WAIT_BEFORE_INTAKE, RUN_INTAKE, DONE }
    private enum AutoState {
        DRIVE_TO_FIRST_SHOT, WAIT_FOR_FIRST_SHOT_TO_FINISH,
        DRIVE_TO_LINE2, DRIVE_BACK_TO_SHOOT, SHOOT_SECOND,
        DRIVE_TO_GATE_1, WAIT_AT_GATE_1, WAIT_FOR_GATE_INTAKE_1, DRIVE_EXTRA_INTAKE_AT_GATE_1, DRIVE_BACK_TO_SHOOT_THIRD, SHOOT_THIRD,
        DRIVE_TO_GATE_2, WAIT_AT_GATE_2, WAIT_FOR_GATE_INTAKE_2, DRIVE_EXTRA_INTAKE_AT_GATE_2, DRIVE_BACK_TO_SHOOT_FOURTH, SHOOT_FOURTH,
        DRIVE_TO_GATE_3, WAIT_AT_GATE_3, WAIT_FOR_GATE_INTAKE_3, DRIVE_EXTRA_INTAKE_AT_GATE_3, DRIVE_BACK_TO_SHOOT_FIFTH, SHOOT_FIFTH,
        DRIVE_TO_FINAL_LINE, DRIVE_BACK_TO_FINAL_SHOOT, SHOOT_FINAL,
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

    // Static paths.
    private PathSpec toFirstShot, toLine2, backToShoot, toGateOpenGate, toGateIntake, toFourthPickup, backToFinalShoot;
    // Dynamic paths (rebuilt from the live pose).
    private PathSpec gateExtraIntakeMove, backToShootFromGate;

    private final List<Long> shotTimes = new ArrayList<>();
    private final List<Pose2d> shotPoses = new ArrayList<>();

    public static SimTrace build() {
        return new V3ClosePartnerSim().run();
    }

    private SimTrace run() {
        trace.meta.autoName = "V3ClosePartner";
        trace.meta.simDtMillis = (int) Math.round(DT_SEC * 1000);
        trace.field.goal = new Pt(BLUE_TARGET_X, TARGET_Y);
        trace.field.backgroundImage = "fields/decode.svg";
        trace.robot.lengthIn = 18.0;
        trace.robot.widthIn = 13.0;
        trace.robot.wheelbaseOffsetIn = 3.5;

        startPt = new Pt(START_X, START_Y);
        double hr = Math.toRadians(START_HEADING_DEG);
        firstShot = new Pt(
                START_X + V3ClosePartnerConfig.FIRST_SHOT_FWD_IN * Math.cos(hr),
                START_Y + V3ClosePartnerConfig.FIRST_SHOT_SIDE_IN * Math.sin(hr));
        trace.meta.startPose = new Pose2d(START_X, START_Y, START_HEADING_DEG);

        follower.setStartingPose(trace.meta.startPose);
        buildStaticPaths();

        // Initial mechanism state.
        intake.setPower(0.0);
        flywheel.stop();
        feeder.clutchIn();
        feeder.armBlock();
        hood.setAngle(FIRST_SHOT_HOOD_DEG);
        turret.setAngle(V3ClosePartnerConfig.INIT_TURRET_ANGLE_DEG);

        // On start.
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
        turret.setAngle(Math.max(V3ClosePartnerConfig.TURRET_MIN_DEG,
                Math.min(V3ClosePartnerConfig.TURRET_MAX_DEG, desiredDeg)));

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
                if (feedState == FeedState.IDLE
                        && follower.getCurrentTValue() > V3ClosePartnerConfig.FIRST_SHOT_FEED_TVALUE) {
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
                if (follower.getCurrentTValue() > V3ClosePartnerConfig.LINE2_SLOW_TVALUE) {
                    follower.setMaxPower(V3ClosePartnerConfig.LINE2_SLOW_POWER);
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
                    feeder.armBlock();
                    feeder.clutchOut();
                    startGateCycle(AutoState.DRIVE_TO_GATE_1);
                }
                break;

            // ---- Gate cycle 1 ----
            case DRIVE_TO_GATE_1:
                intake.setPower(0);
                if (!follower.isBusy()) { autoTimer.reset(); autoState = AutoState.WAIT_AT_GATE_1; }
                break;
            case WAIT_AT_GATE_1:
                intake.setPower(0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.WAIT_AT_GATE_1_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, false);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_1;
                }
                break;
            case WAIT_FOR_GATE_INTAKE_1:
                intake.setPower(follower.getCurrentTValue() >= V3ClosePartnerConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.GATE_INTAKE_1_SEC) startExtraGateIntakeMove(AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_1);
                break;
            case DRIVE_EXTRA_INTAKE_AT_GATE_1:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3ClosePartnerConfig.EXTRA_MOVE_TIMEOUT_SEC)
                    startReturnFromGateToShoot(V3ClosePartnerConfig.RG1_C1X, V3ClosePartnerConfig.RG1_C1Y, AutoState.DRIVE_BACK_TO_SHOOT_THIRD);
                break;
            case DRIVE_BACK_TO_SHOOT_THIRD:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_THIRD; }
                break;
            case SHOOT_THIRD:
                if (feedState == FeedState.DONE) { feeder.armBlock(); feeder.clutchOut(); startGateCycle(AutoState.DRIVE_TO_GATE_2); }
                break;

            // ---- Gate cycle 2 ----
            case DRIVE_TO_GATE_2:
                intake.setPower(0);
                if (!follower.isBusy()) { autoTimer.reset(); autoState = AutoState.WAIT_AT_GATE_2; }
                break;
            case WAIT_AT_GATE_2:
                intake.setPower(0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.WAIT_AT_GATE_2_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_2;
                }
                break;
            case WAIT_FOR_GATE_INTAKE_2:
                intake.setPower(follower.getCurrentTValue() >= V3ClosePartnerConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.GATE_INTAKE_2_SEC) startExtraGateIntakeMove(AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_2);
                break;
            case DRIVE_EXTRA_INTAKE_AT_GATE_2:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3ClosePartnerConfig.EXTRA_MOVE_TIMEOUT_SEC)
                    startReturnFromGateToShoot(V3ClosePartnerConfig.RG2_C1X, V3ClosePartnerConfig.RG2_C1Y, AutoState.DRIVE_BACK_TO_SHOOT_FOURTH);
                break;
            case DRIVE_BACK_TO_SHOOT_FOURTH:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_FOURTH; }
                break;
            case SHOOT_FOURTH:
                if (feedState == FeedState.DONE) { feeder.armBlock(); feeder.clutchOut(); startGateCycle(AutoState.DRIVE_TO_GATE_3); }
                break;

            // ---- Gate cycle 3 ----
            case DRIVE_TO_GATE_3:
                intake.setPower(0);
                if (!follower.isBusy()) { autoTimer.reset(); autoState = AutoState.WAIT_AT_GATE_3; }
                break;
            case WAIT_AT_GATE_3:
                intake.setPower(0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.WAIT_AT_GATE_3_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_3;
                }
                break;
            case WAIT_FOR_GATE_INTAKE_3:
                intake.setPower(follower.getCurrentTValue() >= V3ClosePartnerConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.GATE_INTAKE_3_SEC) startExtraGateIntakeMove(AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_3);
                break;
            case DRIVE_EXTRA_INTAKE_AT_GATE_3:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3ClosePartnerConfig.EXTRA_MOVE_TIMEOUT_SEC)
                    startReturnFromGateToShoot(V3ClosePartnerConfig.RG2_C1X, V3ClosePartnerConfig.RG2_C1Y, AutoState.DRIVE_BACK_TO_SHOOT_FIFTH);
                break;
            case DRIVE_BACK_TO_SHOOT_FIFTH:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_FIFTH; }
                break;
            case SHOOT_FIFTH:
                if (feedState == FeedState.DONE) { feeder.armBlock(); feeder.clutchOut(); startFinalLineCycle(); }
                break;

            // ---- Final higher-Y line ----
            case DRIVE_TO_FINAL_LINE:
                if (!follower.isBusy()) startReturnToFinalShoot();
                break;
            case DRIVE_BACK_TO_FINAL_SHOOT:
                intake.setPower(1.0);
                if (!follower.isBusy()) { startFeedSequence(); autoState = AutoState.SHOOT_FINAL; }
                break;
            case SHOOT_FINAL:
                if (feedState == FeedState.DONE) {
                    intake.setPower(0.0);
                    feeder.armBlock();
                    feeder.clutchOut();
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

    private void startGateCycle(AutoState next) {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = next;
    }

    private void startExtraGateIntakeMove(AutoState next) {
        follower.setMaxPower(1.0);
        Pose2d cur = follower.getPose();
        gateExtraIntakeMove = line("gateExtra", new Pt(cur.x, cur.y),
                new Pt(cur.x, cur.y + EXTRA_GATE_INTAKE_Y_IN), GATE_INTAKE_H);
        follower.followPath(gateExtraIntakeMove, true);
        autoState = next;
    }

    private void startReturnFromGateToShoot(double c1x, double c1y, AutoState next) {
        follower.setMaxPower(1.0);
        Pose2d cur = follower.getPose();
        backToShootFromGate = curve("backFromGate",
                Arrays.asList(new Pt(cur.x, cur.y), new Pt(c1x, c1y), firstShot),
                tangentDeg(new Pt(cur.x, cur.y), firstShot)).reverseTangent();
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.followPath(backToShootFromGate, true);
        autoState = next;
    }

    private void startFinalLineCycle() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(toFourthPickup, false);
        autoState = AutoState.DRIVE_TO_FINAL_LINE;
    }

    private void startReturnToFinalShoot() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;
        enableDynamicShotControl = true;
        follower.setMaxPower(1.0);
        follower.followPath(backToFinalShoot, false);
        autoState = AutoState.DRIVE_BACK_TO_FINAL_SHOOT;
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
        double[][] t = V3ClosePartnerConfig.SHOT_TABLE;
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
        // Heading modes mirror V3ClosePartner.buildPaths() exactly (tangent / constant / reversed /
        // linear). The nominal headingDeg passed to line()/curve() is only a JSON-side label;
        // SimFollower drives heading from the mode set via the chained call.
        toFirstShot = line("toFirstShot", startPt, firstShot, tangentDeg(startPt, firstShot)).tangent();

        toLine2 = curve("toLine2", Arrays.asList(firstShot,
                new Pt(V3ClosePartnerConfig.L2_C1X, V3ClosePartnerConfig.L2_C1Y),
                new Pt(V3ClosePartnerConfig.L2_C2X, V3ClosePartnerConfig.L2_C2Y),
                new Pt(V3ClosePartnerConfig.L2_ENDX, V3ClosePartnerConfig.L2_ENDY)), LINE_H);

        Pt btsStart = new Pt(V3ClosePartnerConfig.BTS_STARTX, V3ClosePartnerConfig.BTS_STARTY);
        backToShoot = curve("backToShoot", Arrays.asList(btsStart,
                new Pt(V3ClosePartnerConfig.BTS_C1X, V3ClosePartnerConfig.BTS_C1Y), firstShot),
                tangentDeg(btsStart, firstShot)).reverseTangent();

        toGateOpenGate = curve("toGateOpenGate", Arrays.asList(firstShot,
                new Pt(V3ClosePartnerConfig.GATE_C1X, V3ClosePartnerConfig.GATE_C1Y),
                new Pt(V3ClosePartnerConfig.GATE_ENDX, V3ClosePartnerConfig.GATE_ENDY)), GATE_OPEN_H_END)
                .linear(V3ClosePartnerConfig.GATE_OPEN_HEADING_START_DEG,
                        V3ClosePartnerConfig.GATE_OPEN_HEADING_END_DEG,
                        V3ClosePartnerConfig.GATE_OPEN_HEADING_T);

        toGateIntake = line("toGateIntake",
                new Pt(V3ClosePartnerConfig.GI_STARTX, V3ClosePartnerConfig.GI_STARTY),
                new Pt(V3ClosePartnerConfig.GI_ENDX, V3ClosePartnerConfig.GI_ENDY), GATE_INTAKE_H);

        toFourthPickup = curve("toFourthPickup", Arrays.asList(firstShot,
                new Pt(V3ClosePartnerConfig.FP_C1X, V3ClosePartnerConfig.FP_C1Y),
                new Pt(V3ClosePartnerConfig.FP_ENDX, V3ClosePartnerConfig.FP_ENDY)), LINE_H);

        Pt fpEnd = new Pt(V3ClosePartnerConfig.FP_ENDX, V3ClosePartnerConfig.FP_ENDY);
        backToFinalShoot = line("backToFinalShoot", fpEnd, firstShot, tangentDeg(fpEnd, firstShot))
                .reverseTangent();
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
