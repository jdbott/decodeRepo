package org.firstinspires.ftc.teamcode.autosim.autos;

import org.firstinspires.ftc.teamcode.autosim.geom.Bezier;
import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
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

    // ===== Sim cadence =====
    private static final double DT_SEC = 0.02;        // 20 ms tick
    private static final long MAX_MS = 40_000;        // runaway guard

    // ===== Fixed shot settings =====
    private static final double FIXED_HOOD_DEG = 53.5;
    private static final double FIXED_FLYWHEEL_RAD = 445;

    // ===== Field positions (BLUE-NATIVE) =====
    private static final double START_X = 64.1, START_Y = 6.75, START_HEADING_DEG = 180.0;
    private static final double BLUE_TARGET_X = 5.0, TARGET_Y = 139.0;
    private static final double SHOOT2_Y_OFFSET = 5.5;
    private static final double LAST_LINE_X = 12.5, LAST_LINE_Y = 38.0;
    private static final double INTAKE_1_X = 15.0, INTAKE_1_Y = 9.5;
    private static final double BACKUP_DISTANCE = 10.0, ALT_CYCLE_Y_OFFSET = 19.0;
    private static final double CURVE_CTRL_X = 45.0, CURVE_CTRL_Y = 38.0;
    private static final double RETREAT_DX = -23.0;   // blue-native

    // ===== Turret / mechanism =====
    private static final double INIT_TURRET_ANGLE_DEG = 113.0;
    private static final double RUN_TURRET_ANGLE_DEG = 113.0;

    // ===== Timing =====
    private static final double FIRST_SHOT_DELAY_SEC = 3;
    private static final double FEED_START_DELAY_SEC = 0.10;
    private static final double FEED_TOTAL_TIME_SEC = 1.00;
    private static final double REVERSE_TIME_SEC = 0.25;

    private static final int CURVE_SEGMENTS = 40;

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
        toIntake1 = line("toIntake1 " + tag, shoot2, intakeP, 200);
        backupFromIntake1 = line("backup " + tag, intakeP, backedP, 180);
        backToShoot2 = line("backToShoot2 " + tag, backedP, shoot2, 180);
    }

    private PathSpec curveToLastLine() {
        return curve("toLastLine",
                Arrays.asList(shoot1, new Pt(CURVE_CTRL_X, CURVE_CTRL_Y), lastLine), 180);
    }

    private PathSpec lineLastLineToShoot2() {
        return line("fromLastLineToShoot2", lastLine, shoot2, 180);
    }

    private static PathSpec line(String id, Pt a, Pt b, double headingDeg) {
        List<Pt> cp = Arrays.asList(a, b);
        return new PathSpec(id, "LINE", cp, Bezier.sample(cp, 1), headingDeg);
    }

    private static PathSpec curve(String id, List<Pt> control, double headingDeg) {
        return new PathSpec(id, "CURVE", control, Bezier.sample(control, CURVE_SEGMENTS), headingDeg);
    }
}
