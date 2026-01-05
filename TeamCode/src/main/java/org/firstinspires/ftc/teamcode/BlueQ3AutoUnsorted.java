package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Blue Auto Unsorted")
public class BlueQ3AutoUnsorted extends OpMode {

    // Subsystems
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlate basePlate;
    private int pathState;
    private int times;
    private Timer pathTimer;
    // Telemetry
    private MultipleTelemetry telemetryA;

    private Path toGateIntake;

    int gateTrips = 0;

    @Override
    public void init() {
        // Initialize constants for Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35.791, 135.000, Math.toRadians(90)));
        follower.updatePose();
        follower.setMaxPower(1);

        intake = new Intake(hardwareMap);
        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlate(hardwareMap);

        intake.intakeStop();
        gantry.moveGantryToPos("back");
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.setPusherMm(0);

        pathTimer = new Timer();
        pathState = 0;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
        times = 0;
    }

    private void oneTimeReset() {
        if (times != 1) {
            pathTimer.resetTimer();
            times = 1;
        }
    }

    private void autoPathUpdate() {
        switch (pathState) {
            case 0:
                Path toShoot1 = new Path(new BezierLine(
                        new Pose(35.791, 135),
                        new Pose(57, 85))
                );
                toShoot1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135), 0.8);
                follower.followPath(toShoot1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    oneTimeReset();
                    if (pathTimer.getElapsedTimeSeconds() < 0.2) {
                        basePlate.rampForward();
                        basePlate.setPusherMm(35);
                    } else if (pathTimer.getElapsedTimeSeconds() < 0.4 && pathTimer.getElapsedTimeSeconds() > 0.2) {
                        basePlate.setPusherMm(125);
                    } else if (pathTimer.getElapsedTimeSeconds() < 0.8 && pathTimer.getElapsedTimeSeconds() > 0.4) {
                        //ROTATE
                    } else if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                        basePlate.setPusherMm(0);
                        basePlate.rampBack();
                        //UNROTATE
                        setPathState(2);
                    }
                }
                break;

            case 2:
                Path toLine2 = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84-20),
                        new Pose(24, 84-22))
                );
                toLine2.setTangentHeadingInterpolation();
                follower.followPath(toLine2, false);
                intake.intakeIn();
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                Path toShoot2 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot2.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toShoot2, true);
                setPathState(41);
                break;

            case 41:
                if (follower.getCurrentTValue() > 0.3) {
                    intake.intakeStop();
                }
                if (!follower.isBusy()) {
                    setPathState(42);
                }
                break;

            case 42:
                toGateIntake = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(17.5, 64.5))
                );
                toGateIntake.setTangentHeadingInterpolation();
                follower.followPath(toGateIntake, true);
                intake.intakeIn();
                setPathState(43);
                break;

            case 43:
                if (follower.getCurrentTValue() > 0.2) {
                    toGateIntake.setConstantHeadingInterpolation(Math.toRadians(140));
                }
                if (!follower.isBusy()) {
                    gateTrips++;
                    setPathState(44);
                }
                break;

            case 44:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if (gateTrips < 2) {
                        setPathState(4);
                    } else {
                        setPathState(61);
                    }
                }
                break;

            case 61:
                Path toShootAfterLastGate = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShootAfterLastGate.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toShootAfterLastGate, true);
                setPathState(62);
                break;

            case 62:
                if (follower.getCurrentTValue() > 0.3) {
                    intake.intakeStop();
                }
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6:
                Path toCloseLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84),
                        new Pose(24, 82))
                );
                toCloseLine.setTangentHeadingInterpolation();
                follower.followPath(toCloseLine, false);
                intake.intakeIn();
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8:
                Path toShoot3 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot3.setTangentHeadingInterpolation();
                toShoot3.reverseHeadingInterpolation();
                follower.followPath(toShoot3, true);
                setPathState(9);
                break;

            case 9:
                if (follower.getCurrentTValue() > 0.3) {
                    intake.intakeStop();
                }
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                Path toBottomLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84 - 48),
                        new Pose(24, 84 - 50))
                );
                toBottomLine.setTangentHeadingInterpolation();
                follower.followPath(toBottomLine, false);
                intake.intakeIn();
                setPathState(11);
                break;

            case 11:
                if (follower.getCurrentTValue() > 0.4) {
                    follower.setMaxPower(0.7);
                }
                if (!follower.isBusy()) {
                    setPathState(12);
                    follower.setMaxPower(1);
                }
                break;

            case 12:
                Path toShoot4 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot4.setTangentHeadingInterpolation();
                toShoot4.reverseHeadingInterpolation();
                follower.followPath(toShoot4, true);
                setPathState(13);
                break;

            case 13:
                if (follower.getCurrentTValue() > 0.3) {
                    intake.intakeStop();
                }
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

//            case 6:
//                Path toLine2 = new Path(new BezierCurve(
//                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                        new Pose(48, 84-20),
//                        new Pose(24, 84-22))
//                );
//                toLine2.setTangentHeadingInterpolation();
//                follower.followPath(toLine2, false);
//                intake.intakeIn();
//                setPathState(7);
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                Path toShoot3 = new Path(new BezierLine(
//                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                        new Pose(57, 85))
//                );
//                toShoot3.setTangentHeadingInterpolation();
//                toShoot3.reverseHeadingInterpolation();
//                follower.followPath(toShoot3, true);
//                setPathState(9);
//                break;
//
//            case 9:
//                if (follower.getCurrentTValue() > 0.3) {
//                    intake.intakeStop();
//                }
//                if (!follower.isBusy()) {
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                Path toLine3 = new Path(new BezierCurve(
//                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                        new Pose(48, 84-44),
//                        new Pose(24, 84-46))
//                );
//                toLine3.setTangentHeadingInterpolation();
//                follower.followPath(toLine3, false);
//                intake.intakeIn();
//                setPathState(11);
//                break;
//
//            case 11:
//                if (follower.getCurrentTValue() > 0.4) {
//                    follower.setMaxPower(0.7);
//                }
//                if (!follower.isBusy()) {
//                    setPathState(12);
//                    follower.setMaxPower(1);
//                }
//                break;
//
//            case 12:
//                Path toShoot4 = new Path(new BezierLine(
//                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                        new Pose(57, 85))
//                );
//                toShoot4.setTangentHeadingInterpolation();
//                toShoot4.reverseHeadingInterpolation();
//                follower.followPath(toShoot4, true);
//                setPathState(13);
//                break;
//
//            case 13:
//                if (follower.getCurrentTValue() > 0.3) {
//                    intake.intakeStop();
//                }
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;

            default:
                break;
        }
    }
}