package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.api.Paths.through;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.alliance.AllianceMirror;
import org.firstinspires.ftc.teamcode.alliance.AllianceStore;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathModifiers;
import org.firstinspires.ftc.teamcode.pedro.StuckDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * Pedro 3 reference auto: pose factories, path building, heading interpolation, per-path speed
 * limits and red/blue mirroring. Path following only — no mechanisms.
 *
 * <p>Alliance comes from the Alliance Selector opmode. During init, press dpad up/down to switch
 * between our mirroring and Pedro's built-in one (see {@link AllianceMirror}).
 */
@Autonomous(name = "Example: Pedro 3 Path Following", group = "Examples")
public class ExamplePathAuto extends LinearOpMode {

    /** One leg of the route. Stop legs wait for the robot to settle; others move on at the path's end. */
    private static final class Leg {
        final String name;
        final Path path;
        final boolean stopAtEnd;

        Leg(String name, Path path, boolean stopAtEnd) {
            this.name = name;
            this.path = path;
            this.stopAtEnd = stopAtEnd;
        }
    }

    private final List<Leg> route = new ArrayList<>();
    private Follower follower;
    private Pose startPose;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean isRed = AllianceStore.isRed(hardwareMap.appContext);
        AllianceMirror.Mode mirrorMode = AllianceMirror.MODE;

        follower = Constants.create(hardwareMap);
        StuckDetector stuckDetector = new StuckDetector(follower);
        buildRoute(AllianceMirror.poses(isRed, mirrorMode));

        boolean lastDpad = false;
        while (opModeInInit()) {
            boolean dpad = gamepad1.dpad_up || gamepad1.dpad_down;
            if (dpad && !lastDpad) {
                mirrorMode = mirrorMode == AllianceMirror.Mode.CUSTOM
                        ? AllianceMirror.Mode.BUILT_IN
                        : AllianceMirror.Mode.CUSTOM;
                buildRoute(AllianceMirror.poses(isRed, mirrorMode));
            }
            lastDpad = dpad;
            follower.update();

            telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            telemetry.addData("Mirroring (dpad to switch)", mirrorMode);
            telemetry.addData("Start pose", format(startPose));
            telemetry.addData("Current pose", format(follower.pose()));
            telemetry.update();
        }
        if (isStopRequested()) return;

        int legIndex = 0;
        follower.follow(route.get(legIndex).path);

        while (opModeIsActive()) {
            follower.update();
            if (stuckDetector.isStuck()) stuckDetector.breakFollowing();

            if (legIndex < route.size() && legFinished(route.get(legIndex))) {
                legIndex++;
                if (legIndex < route.size()) follower.follow(route.get(legIndex).path);
            }

            telemetry.addData("Leg", legIndex < route.size() ? route.get(legIndex).name : "Done (holding)");
            telemetry.addData("Follower mode", follower.mode());
            telemetry.addData("Pose", format(follower.pose()));
            telemetry.addData("Leg completion", "%.2f", follower.completion());
            telemetry.addData("Remaining (in)", "%.1f", follower.remainingDistance());
            telemetry.update();
        }
    }

    /** Every pose is authored BLUE-native with degree headings; the factory mirrors it for red. */
    private void buildRoute(PoseFactory p) {
        startPose = p.of(21, 128, -40);
        Pose score = p.of(48, 96, -45);
        Pose goal = p.of(5, 139, 0);
        Pose lineStart = p.of(44, 84, 180);
        Pose lineEnd = p.of(18, 84, 180);
        Pose leave = p.of(38, 60, 90);

        route.clear();

        // Heading blends from the start heading to the score heading.
        route.add(new Leg("Start -> score (line, linear heading)",
                line(startPose, score).linear(startPose, score), true));

        // One compound path: curve onto the line, then a slow constant-heading sweep along it.
        // Interpolators take Poses rather than raw radians so headings mirror along with positions.
        route.add(new Leg("Score -> sweep line (compound, slowed sweep)", path(
                curve(score, p.of(56, 84, 0), lineStart).tangent(),
                line(lineStart, lineEnd).constant(lineEnd).with(PathModifiers.maxSpeed(0.4))
        ), false));

        // Curve back while always pointing the front of the robot at the goal.
        route.add(new Leg("Line -> score (facing goal)",
                curve(lineEnd, p.of(40, 90, 0), score).facingPoint(goal), true));

        // through() fits a curve through every pose. Follow the tangent, then turn to the final heading.
        route.add(new Leg("Score -> leave (through, piecewise heading)",
                through(score, p.of(30, 76, 0), leave).heading(Interpolator.piecewise()
                        .until(0.6, Interpolator.tangent)
                        .until(1.0, Interpolator.constant(leave))), true));

        follower.setPose(startPose);
    }

    private boolean legFinished(Leg leg) {
        return leg.stopAtEnd ? !follower.isBusy() : follower.atParametricEnd();
    }

    private static String format(Pose pose) {
        return String.format("(%.1f, %.1f, %.0f deg)", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
