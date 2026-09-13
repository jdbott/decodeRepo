package org.firstinspires.ftc.teamcode.templates;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.alliance.AllianceMirror;
import org.firstinspires.ftc.teamcode.alliance.AllianceStore;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Starting point for a new auto: Pedro 3 paths, Ivy commands and alliance mirroring. Copy into
 * autos/, rename, remove {@code @Disabled}, then replace the poses, paths and routine.
 */
@Disabled
@Autonomous(name = "Template: Auto", group = "Templates")
public class TemplateAuto extends LinearOpMode {

    private Follower follower;
    private PoseFactory p;
    private Pose startPose, scorePose, parkPose;

    @Override
    public void runOpMode() {
        boolean isRed = AllianceStore.isRed(hardwareMap.appContext);
        p = AllianceMirror.poses(isRed); // author poses BLUE-native, headings in degrees
        startPose = p.of(72, 8, 90);
        scorePose = p.of(72, 48, 90);
        parkPose = p.of(96, 24, 0);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        // Construct subsystems here.

        Scheduler.reset();
        Command routine = routine();

        while (opModeInInit()) {
            follower.update();
            telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            telemetry.addData("Pose", follower.pose());
            telemetry.update();
        }
        if (isStopRequested()) return;

        Scheduler.schedule(routine);
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            telemetry.addData("Pose", follower.pose());
            telemetry.update();
        }
        Scheduler.reset();
    }

    private Command routine() {
        return sequential(
                follow(follower, startToScore()),
                // follow() finishes at the path's end; wait here too if the robot must be settled
                waitUntil(() -> !follower.isBusy()),
                waitMs(500), // mechanism commands go between paths
                follow(follower, scoreToPark())
        );
    }

    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    private Path scoreToPark() {
        return curve(scorePose, p.of(96, 48, 0), parkPose).linear(scorePose, parkPose);
    }
}
