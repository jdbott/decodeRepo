package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Srimmage Auto Blue")
public class SrimmageAutoBlue extends OpMode {

    // Subsystems
    private Follower follower;
    private Shooter shooter;
    private Turret turret;
    private ServoController revolver;
    private IMU imu;
    private Servo gateServo, popperServo;
    private ColorV3 colorSensor;

    private DcMotorEx intakeMotor;

    // FSM State variables
    private int pathState;
    private Timer pathTimer;
    private int nextPathStateAfterShooting = 3; // default old behavior

    // Telemetry
    private MultipleTelemetry telemetryA;

    @Override
    public void init() {
        // Initialize constants for Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35.791, 135.000, Math.toRadians(90)));
        follower.setMaxPower(1);

        // Subsystem initialization
        shooter = new Shooter();
        shooter.init(hardwareMap, "motor1", "motor2",
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(0.008);
        turret.setKF(0.003);
        turret.setLimits(-180, 180);
        turret.setAngle(0);

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        popperServo = hardwareMap.get(Servo.class, "popperServo");
        colorSensor = new ColorV3(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        revolver = new ServoController(hardwareMap);
        revolver.zeroNow();
        revolver.update();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        imu.resetYaw();

        // Start configuration
        gateServo.setPosition(0.38); // gate down
        popperServo.setPosition(0.14); // popper down
        shooter.setTargetRPM(0); // flywheel off
        //revolver.moveServosToPosition(0); // 60° offset position

        pathTimer = new Timer();
        pathState = 0;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        //revolver.update();
    }

    @Override
    public void start() {
        revolver.zeroNow();
        revolver.update();
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
                shooter.setTargetRPM(3200);
                Path toShoot1 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(46.465, 97.116))
                );
                toShoot1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(125), 0.8);
                follower.followPath(toShoot1, true);
                setPathState(1);
                break;

            case 1:
                // Wait until motion completes
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // Execute AutoShoot3BallsFSM
                runAutoShoot3BallsFSM();
                break;

            case 3:
                // All done
                shooter.setTargetRPM(0);
                intakeMotor.setPower(1);
                gateServo.setPosition(0.48);
                Path toLine1 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(47.5, 89))
                );
                toLine1.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toLine1, true);
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
                        new Pose(follower.getPose().getX() - 5, follower.getPose().getY())
                ));
                forward1.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(forward1, true);
                setPathState(6);
                break;

            case 6:
                // Wait 0.8s, then drop gate, wait 0.2s, revolve 120°, raise gate
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gateServo.setPosition(0.38); // gate down
                    pathTimer.resetTimer();
                    setPathState(61); // intermediate substate
                }
                break;

            case 61:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    revolver.moveServosToPosition(180);
                    gateServo.setPosition(0.48); // gate up
                    Path forward2 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 13, follower.getPose().getY())
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
                    revolver.moveServosToPosition(300);
                    gateServo.setPosition(0.48);
                    Path forward3 = new Path(new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(follower.getPose().getX() - 7, follower.getPose().getY())
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
                shooter.setTargetRPM(3200);  // spin up flywheel
                follower.setMaxPower(1);
                Path toShootAgain = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(46.465, 97.116) // same shoot point
                ));
                toShootAgain.setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        Math.toRadians(125) // same target heading as before
                );
                follower.followPath(toShootAgain, true);
                pathTimer.resetTimer();
                setPathState(10);
                break;

            case 10:
                // After 0.2s, move revolver to 60°; when motion finishes, shoot again
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    //revolver.moveServosByRotation(60);
                }

                if (!follower.isBusy()) {
                    setPathState(2); // reuse AutoShoot3BallsFSM to fire again
                    nextPathStateAfterShooting = 11;
                }
                break;

            case 11:
                // Drive to lower pickup line (24 in lower Y)
                Path toLowerLine = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(47.5, 89-24)
                ));
                toLowerLine.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toLowerLine, true);
                setPathState(12);
                break;

            case 12:
                // Wait for arrival, then start intake
                if (!follower.isBusy()) {
//                    intakeMotor.setPower(1);
//                    gateServo.setPosition(0.48);
//                    Path pick1 = new Path(new BezierLine(
//                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                            new Pose(follower.getPose().getX() - 5, follower.getPose().getY())
//                    ));
//                    pick1.setConstantHeadingInterpolation(Math.toRadians(180));
//                    follower.followPath(pick1, true);
//                    pathTimer.resetTimer();
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    // ---------------- Auto Shoot FSM ----------------
    private enum AutoShoot3BallsState {
        OFF, MOVE_TO_START, POP_UP, POP_DOWN, REVOLVE, WAIT_TO_SETTLE, RETURN_TO_START, DONE
    }

    private AutoShoot3BallsState autoShootState = AutoShoot3BallsState.OFF;
    private long autoShootTimer;
    private int popCount;

    private void runAutoShoot3BallsFSM() {
        long now = System.currentTimeMillis();

        long autoInitialMoveDelayMs = 750;
        long postRevolveDelayMs = 750;
        long popUpMs = 250;
        long popDownMs = 200;
        double[] revolverPositions = {60, 180, 300}; // explicit positions
        int popRepeats = 3;

        switch (autoShootState) {
            case OFF:
                autoShootState = AutoShoot3BallsState.MOVE_TO_START;
                popCount = 0;
                gateServo.setPosition(0.38);
                revolver.moveServosToPosition(revolverPositions[0]);
                autoShootTimer = now + autoInitialMoveDelayMs;
                break;

            case MOVE_TO_START:
                if (now >= autoShootTimer) {
                    autoShootState = AutoShoot3BallsState.POP_UP;
                    popCount = 0;
                    autoShootTimer = now + popUpMs;
                    popperServo.setPosition(0.45);
                }
                break;

            case POP_UP:
                if (now >= autoShootTimer) {
                    autoShootState = AutoShoot3BallsState.POP_DOWN;
                    popperServo.setPosition(0.14);
                    autoShootTimer = now + popDownMs;
                }
                break;

            case POP_DOWN:
                if (now >= autoShootTimer) {
                    popCount++;
                    if (popCount < popRepeats) {
                        autoShootState = AutoShoot3BallsState.REVOLVE;
                        autoShootTimer = now; // no delay before revolve
                    } else {
                        autoShootState = AutoShoot3BallsState.RETURN_TO_START;
                        revolver.moveServosToPosition(0); // back to start
                        autoShootTimer = now + autoInitialMoveDelayMs;
                    }
                }
                break;

            case REVOLVE:
                if (now >= autoShootTimer) {
                    double nextPos = revolverPositions[popCount % revolverPositions.length];
                    revolver.moveServosToPosition(nextPos);
                    autoShootState = AutoShoot3BallsState.WAIT_TO_SETTLE;
                    autoShootTimer = now + postRevolveDelayMs;
                }
                break;

            case WAIT_TO_SETTLE:
                if (now >= autoShootTimer) {
                    autoShootState = AutoShoot3BallsState.POP_UP;
                    popperServo.setPosition(0.45);
                    autoShootTimer = now + popUpMs;
                }
                break;

            case RETURN_TO_START:
                if (now >= autoShootTimer) {
                    autoShootState = AutoShoot3BallsState.DONE;
                    revolver.moveServosByRotation(60);
                }
                break;

            case DONE:
                shooter.setTargetRPM(0);
                autoShootState = AutoShoot3BallsState.OFF;
                setPathState(nextPathStateAfterShooting);
                break;
        }

        revolver.update();
    }
}