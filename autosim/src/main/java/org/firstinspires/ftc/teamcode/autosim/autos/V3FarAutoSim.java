package org.firstinspires.ftc.teamcode.autosim.autos;

import org.firstinspires.ftc.teamcode.autoshared.V3FarAutoConfig;
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
 * Simulatable copy of V3FarAuto (BLUE-NATIVE). The two FSMs ({@code AutoState},
 * {@code FeedState}), their constants, the path builders, and the loop body are reproduced
 * from {@code V3FarAuto.java}; the only changes are the seams: virtual clock + stopwatches
 * instead of {@code ElapsedTime}, {@link SimFollower} instead of Pedro's Follower, and
 * recording stubs instead of real hardware. See AUTOSIM_DESIGN.md §5/§9.
 *
 * <p>The original V3FarAuto.java is never imported or modified. Keep this copy in sync by
 * hand until the AUTO_BASE_DESIGN refactor lets one FSM body drive both.
 */
public final class V3FarAutoSim {

    // ===== Sim cadence (sim-only) =====
    private static final double DT_SEC = 0.02;        // 20 ms tick
    private static final long MAX_MS = 40_000;        // runaway guard
    private static final int CURVE_SEGMENTS = 40;

    // ===== Tunables / geometry — shared with the real opmode via V3FarAutoConfig =====
    private static final double FIXED_HOOD_DEG = V3FarAutoConfig.FIXED_HOOD_DEG;
    private static final double FIXED_FLYWHEEL_RAD = V3FarAutoConfig.FIXED_FLYWHEEL_RAD;

    private static final double START_X = V3FarAutoConfig.START_X,
            START_Y = V3FarAutoConfig.START_Y, START_HEADING_DEG = V3FarAutoConfig.START_HEADING_DEG;
    private static final double BLUE_TARGET_X = V3FarAutoConfig.BLUE_TARGET_X, TARGET_Y = V3FarAutoConfig.TARGET_Y;
    private static final double SHOOT2_Y_OFFSET = V3FarAutoConfig.SHOOT2_Y_OFFSET;
    private static final double LAST_LINE_X = V3FarAutoConfig.LAST_LINE_X, LAST_LINE_Y = V3FarAutoConfig.LAST_LINE_Y;
    private static final double INTAKE_1_X = V3FarAutoConfig.INTAKE_1_X, INTAKE_1_Y = V3FarAutoConfig.INTAKE_1_Y;
    private static final double BACKUP_DISTANCE = V3FarAutoConfig.BACKUP_DISTANCE,
            ALT_CYCLE_Y_OFFSET = V3FarAutoConfig.ALT_CYCLE_Y_OFFSET;
    private static final double CURVE_CTRL_X = V3FarAutoConfig.CURVE_CTRL_X, CURVE_CTRL_Y = V3FarAutoConfig.CURVE_CTRL_Y;
    private static final double RETREAT_DX = -V3FarAutoConfig.RETREAT_DX;   // blue-native (-X)
    private static final double DRIVE_HEADING_DEG = V3FarAutoConfig.DRIVE_HEADING_DEG;
    private static final double INTAKE_HEADING_DEG = V3FarAutoConfig.INTAKE_HEADING_DEG;

    private static final double INIT_TURRET_ANGLE_DEG = V3FarAutoConfig.INIT_TURRET_ANGLE_DEG;
    private static final double RUN_TURRET_ANGLE_DEG = V3FarAutoConfig.RUN_TURRET_ANGLE_DEG;

    private static final double FIRST_SHOT_DELAY_SEC = V3FarAutoConfig.FIRST_SHOT_DELAY_SEC;
    private static final double FEED_START_DELAY_SEC = V3FarAutoConfig.FEED_START_DELAY_SEC;
    private static final double FEED_TOTAL_TIME_SEC = V3FarAutoConfig.FEED_TOTAL_TIME_SEC;
    private static final double REVERSE_TIME_SEC = V3FarAutoConfig.REVERSE_TIME_SEC;

    private enum FeedState { IDLE, WAIT_BEFORE_INTAKE, RUN_INTAKE, DONE }
    private enum AutoState {
        WAIT_INITIAL_DELAY, SHOOT_FIRST,
        DRIVE_TO_LAST_LINE, DRIVE_BACK_TO_SHOOT2, SHOOT_SECOND,
        DRIVE_TO_INTAKE1, BACK_UP_FROM_INTAKE1, DRIVE_BACK_TO_SHOOT_AGAIN, SHOOT_REPEAT,
        DONE
    }

    // ===== Collaborators (sim) =====
    private final SimTrace trace = new SimTrace();
    private final SimClock clock = new SimClock();
    private final SimFollower follower = new SimFollower(clock, trace);
    private final SimIntake intake = new SimIntake();
    private final SimFlywheel flywheel = new SimFlywheel();
    private final SimTurret turret = new SimTurret();
    private final SimHood hood = new SimHood();
    private final SimFeeder feeder = new SimFeeder();

    private final SimStopwatch stateTimer = new SimStopwatch(clock);
    private final SimStopwatch feedTimer = new SimStopwatch(clock);
    private final SimStopwatch reverseTimer = new SimStopwatch(clock);

    private boolean reversingIntake = false;
    private int extraCycleCount = 0;
    private FeedState feedState = FeedState.IDLE;
    private AutoState autoState = AutoState.WAIT_INITIAL_DELAY;

    // Shot markers captured at each feed sequence; turned into SHOOT events post-run.
    private final List<Long> shotTimes = new ArrayList<>();
    private final List<Pose2d> shotPoses = new ArrayList<>();

    // ===== Poses =====
    private Pt shoot1, shoot2, lastLine;

    // Current-cycle paths (rebuilt per cycle, like buildCyclePathsForCurrentCycle)
    private PathSpec toIntake1, backupFromIntake1, backToShoot2;

    /** Entry point used by the generator. */
    public static SimTrace build() {
        return new V3FarAutoSim().run();
    }

    private SimTrace run() {
        trace.meta.autoName = "V3FarAuto";
        trace.meta.simDtMillis = (int) Math.round(DT_SEC * 1000);
        trace.meta.startPose = new Pose2d(START_X, START_Y, START_HEADING_DEG);
        trace.field.goal = new Pt(BLUE_TARGET_X, TARGET_Y);
        trace.field.backgroundImage = "fields/decode.svg";

        // Real robot footprint (edit here, or adjust live in the viewer): 18" long incl. a
        // front intake, 13" wide, wheelbase ~3.5" behind the body center.
        trace.robot.lengthIn = 18.0;
        trace.robot.widthIn = 13.0;
        trace.robot.wheelbaseOffsetIn = 3.5;

        shoot1 = new Pt(START_X, START_Y);
        shoot2 = new Pt(START_X, START_Y + SHOOT2_Y_OFFSET);
        lastLine = new Pt(LAST_LINE_X, LAST_LINE_Y);

        follower.setStartingPose(trace.meta.startPose);

        // initial mechanism state + on-start commands
        intake.setPower(0.0);
        flywheel.stop();
        feeder.clutchIn();
        feeder.armBlock();
        hood.setAngle(FIXED_HOOD_DEG);
        flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);

        stateTimer.reset();
        autoState = AutoState.WAIT_INITIAL_DELAY;

        while (true) {
            follower.update();
            turret.setAngle(autoState != AutoState.DONE ? RUN_TURRET_ANGLE_DEG : 0.0);
            turret.update();
            flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);
            flywheel.update(DT_SEC);

            updateFeedSequence();
            updateAutoState();

            trace.frames.add(sampleFrame());

            if (autoState == AutoState.DONE && !follower.isBusy()) break;
            if (clock.millis() >= MAX_MS) break;
            clock.advance(DT_SEC);
        }

        trace.meta.totalMillis = clock.millis();
        buildEvents();
        return trace;
    }

    /**
     * Derive the sparse {@code events[]} overlay from what the sim actually did. Durations come
     * from the FSM's own timing (the feed window) or from the recorded frame stream (intake
     * spans) — never a guessed constant (AUTOSIM_DESIGN.md §3/§8).
     */
    private void buildEvents() {
        long feedWindowMs = Math.round((FEED_START_DELAY_SEC + FEED_TOTAL_TIME_SEC) * 1000.0);
        for (int i = 0; i < shotTimes.size(); i++) {
            trace.events.add(new ActionEvent(
                    shotTimes.get(i), Category.SHOOT, "shot " + (i + 1), feedWindowMs, shotPoses.get(i)));
        }

        // Intake spans: contiguous frame runs where the intake is commanded forward. The true
        // powered duration falls straight out of the dense frames.
        List<Frame> fr = trace.frames;
        int i = 0;
        while (i < fr.size()) {
            if (fr.get(i).intakePower > 0.05) {
                long start = fr.get(i).tMs;
                Pose2d at = fr.get(i).pose;
                int j = i;
                while (j < fr.size() && fr.get(j).intakePower > 0.05) j++;
                long end = fr.get(j - 1).tMs;
                trace.events.add(new ActionEvent(
                        start, Category.INTAKE, "intake", Math.max((long) (DT_SEC * 1000), end - start), at));
                i = j;
            } else {
                i++;
            }
        }

        // Hood is set once at init; the turret slews to its aim angle at start. Single markers.
        trace.events.add(new ActionEvent(
                0, Category.HOOD, "hood " + FIXED_HOOD_DEG + "°", 400, trace.meta.startPose));
        trace.events.add(new ActionEvent(
                0, Category.TURRET, "turret " + (int) RUN_TURRET_ANGLE_DEG + "°", 600, trace.meta.startPose));

        trace.events.sort((a, b) -> Long.compare(a.tMs, b.tMs));
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

    private void updateAutoState() {
        switch (autoState) {
            case WAIT_INITIAL_DELAY:
                intake.setPower(0.0);
                if (stateTimer.seconds() >= FIRST_SHOT_DELAY_SEC) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FIRST;
                }
                break;

            case SHOOT_FIRST:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    intake.setPower(0.0);
                    follower.followPath(curveToLastLine(), false);
                    autoState = AutoState.DRIVE_TO_LAST_LINE;
                }
                break;

            case DRIVE_TO_LAST_LINE:
                if (!follower.isBusy()) {
                    follower.followPath(lineLastLineToShoot2(), true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT2;
                }
                break;

            case DRIVE_BACK_TO_SHOOT2:
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_SECOND;
                }
                break;

            case SHOOT_SECOND:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    buildCyclePathsForCurrentCycle();
                    follower.followPath(toIntake1, false);
                    intake.setPower(1.0);
                    autoState = AutoState.DRIVE_TO_INTAKE1;
                }
                break;

            case DRIVE_TO_INTAKE1:
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.followPath(backupFromIntake1, true);
                    autoState = AutoState.BACK_UP_FROM_INTAKE1;
                }
                break;

            case BACK_UP_FROM_INTAKE1:
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.followPath(backToShoot2, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT_AGAIN;
                }
                break;

            case DRIVE_BACK_TO_SHOOT_AGAIN:
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_REPEAT;
                }
                break;

            case SHOOT_REPEAT:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    extraCycleCount++;
                    if (extraCycleCount < 4) {
                        buildCyclePathsForCurrentCycle();
                        follower.followPath(toIntake1, false);
                        intake.setPower(1.0);
                        autoState = AutoState.DRIVE_TO_INTAKE1;
                    } else {
                        intake.setPower(0.0);
                        autoState = AutoState.DONE;
                        Pose2d cur = follower.getPose();
                        follower.followPath(
                                line("retreat", new Pt(cur.x, cur.y), new Pt(cur.x + RETREAT_DX, cur.y), 180),
                                true);
                    }
                }
                break;

            case DONE:
                intake.setPower(0.0);
                turret.setAngle(0.0);
                break;
        }
    }

    private void startFeedSequence() {
        feeder.armShoot();
        feeder.clutchIn();
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
                    feeder.clutchOut();
                    feedState = FeedState.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

    // ===== Path builders (mirror V3FarAuto) =====

    private boolean useHighIntakeCycle() {
        return (extraCycleCount % 2) == 1;
    }

    private Pt currentIntakePose() {
        double y = useHighIntakeCycle() ? ALT_CYCLE_Y_OFFSET : 0.0;
        return new Pt(INTAKE_1_X, INTAKE_1_Y + y);
    }

    private Pt currentBackedPose() {
        double y = useHighIntakeCycle() ? ALT_CYCLE_Y_OFFSET : 0.0;
        return new Pt(INTAKE_1_X + BACKUP_DISTANCE, INTAKE_1_Y + y);
    }

    private void buildCyclePathsForCurrentCycle() {
        String tag = "#" + extraCycleCount;
        Pt intakeP = currentIntakePose();
        Pt backedP = currentBackedPose();
        toIntake1 = line("toIntake1 " + tag, shoot2, intakeP, INTAKE_HEADING_DEG);
        backupFromIntake1 = line("backup " + tag, intakeP, backedP, DRIVE_HEADING_DEG);
        backToShoot2 = line("backToShoot2 " + tag, backedP, shoot2, DRIVE_HEADING_DEG);
    }

    private PathSpec curveToLastLine() {
        return curve("toLastLine",
                Arrays.asList(shoot1, new Pt(CURVE_CTRL_X, CURVE_CTRL_Y), lastLine), DRIVE_HEADING_DEG);
    }

    private PathSpec lineLastLineToShoot2() {
        return line("fromLastLineToShoot2", lastLine, shoot2, DRIVE_HEADING_DEG);
    }

    private static PathSpec line(String id, Pt a, Pt b, double headingDeg) {
        List<Pt> cp = Arrays.asList(a, b);
        return new PathSpec(id, "LINE", cp, Bezier.sample(cp, 1), headingDeg);
    }

    private static PathSpec curve(String id, List<Pt> control, double headingDeg) {
        return new PathSpec(id, "CURVE", control, Bezier.sample(control, CURVE_SEGMENTS), headingDeg);
    }
}
