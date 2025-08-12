package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Custom Tuning Path")
public class CustomTuningPath extends OpMode {
    private Follower follower;
    RevBlinkinLedDriver led;
    private Timer pathTimer;
    private int pathState;
    private Telemetry telemetryA;
    private Path toCorner, curve, toStart;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.initialize();
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        follower.setMaxPower(1);
        pathTimer = new Timer();
        pathState = 0;

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        Path init = new Path(new BezierLine(
                new Point(0,0,Point.CARTESIAN),
                new Point(0,0,Point.CARTESIAN)
        ));
        follower.followPath(init, true);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        follower.telemetryDebug(telemetryA);
        autoPathUpdate();
        telemetry.update();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toCorner = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-10, 40, Point.CARTESIAN)));
                toCorner.setTangentHeadingInterpolation();
                follower.followPath(toCorner, true);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                setPathState(1);
                break;

            case 1:
                if (follower.isBusy() && follower.getCurrentTValue() > 0.7) {
                    toCorner.setConstantHeadingInterpolation(Math.toRadians(180));
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }
                if (!follower.isBusy()) {
                    setPathState(2);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    curve = new Path(new BezierCurve(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(-40, 40, Point.CARTESIAN),
                            new Point(-20, 20, Point.CARTESIAN),
                            new Point(-60, 10, Point.CARTESIAN)));
                    curve.setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90));
                    follower.followPath(curve, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (follower.getCurrentTValue() > 0.2) {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                }
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    toStart = new Path(new BezierLine(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(0, 0, Point.CARTESIAN)));
                    toStart.setConstantHeadingInterpolation(Math.toRadians(90));
                    follower.followPath(toStart, true);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    setPathState(0);
                }
                break;

            default:
                // No further action
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
}