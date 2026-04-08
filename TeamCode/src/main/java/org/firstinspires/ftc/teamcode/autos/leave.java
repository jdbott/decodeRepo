package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlate;
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
@Disabled
@Autonomous(name = "LEAVE")
public class leave extends OpMode {

    // Subsystems
    private Follower follower;
    private Gantry gantry;
    private BasePlate basePlate;
    private Turret turret;
    private FlywheelASG flywheelASG;
    private Intake intake;

    // State / timing
    private int pathState;
    private Timer pathTimer;
    private boolean oneTimeDone;

    // Telemetry
    private MultipleTelemetry telemetryA;

    // Tunables
    private static final double INIT_TURRET_ANGLE_DEG = -114.0;
    private static final double FLYWHEEL_SPINUP_VELOCITY = 385.0;

    private static final double SPINUP_WAIT_S = 3;
    private static final double SHOOT_ALL_WAIT_S = 2.0;   // tune if needed
    private static final double SETTLE_WAIT_S = 0.75;
    private static final double TURRET_ZERO_WAIT_S = 0.2;

    // Distances (inches, using your field coordinate system along +X)
    private static final double FORWARD_INTAKE_DISTANCE = 48;
    private static final double BACKUP_DISTANCE = 10.0;
    private static final double BUMP_FORWARD_DISTANCE = 10.0;
    private static final double LEAVE_DISTANCE = 25.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.updatePose();
        follower.setMaxPower(1);

        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlate(hardwareMap);
        intake = new Intake(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheelASG = new FlywheelASG(hardwareMap, battery);

        // Initial shoot configuration
        intake.intakeStop();

        gantry.moveGantryToPos("back");
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.setPusherMm(0);
        basePlate.gateHoldBall1();
        basePlate.prepShootOnly();

        // Fixed turret angle for both shooting cycles (until after second cycle)
        turret.setAngle(INIT_TURRET_ANGLE_DEG);

        pathTimer = new Timer();
        pathState = 0;
        oneTimeDone = false;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.addData("Turret Init Angle", INIT_TURRET_ANGLE_DEG);
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        basePlate.update();
        turret.update();
        //flywheelASG.update();
    }

    @Override
    public void start() {
        // Start flywheel spinup immediately
        flywheelASG.setTargetVelocity(FLYWHEEL_SPINUP_VELOCITY);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();

        basePlate.update();
        turret.update();
        flywheelASG.update();

        telemetryA.addData("State", pathState);
        telemetryA.addData("Busy", follower.isBusy());
        telemetryA.addData("t", follower.getCurrentTValue());
        telemetryA.addData("Pose X", follower.getPose().getX());
        telemetryA.addData("Pose Y", follower.getPose().getY());
        telemetryA.update();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
        oneTimeDone = false;
    }

    private void oneTime(Runnable r) {
        if (!oneTimeDone) {
            r.run();
            oneTimeDone = true;
            pathTimer.resetTimer();
        }
    }

    private Path straightPathTo(double targetX, double targetY) {
        Path p = new Path(
                new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(targetX, targetY)
                )
        );
        p.setConstantHeadingInterpolation(Math.toRadians(0));
        p.setBrakingStrength(1);
        return p;
    }

    private void autoPathUpdate() {
        switch (pathState) {

            // ---------------------------------------------------------
            // 0) Wait for flywheel spinup
            // ---------------------------------------------------------
            case 0: {
                if (pathTimer.getElapsedTimeSeconds() >= SPINUP_WAIT_S) {
                    setPathState(1);
                }
                break;
            }

            // ---------------------------------------------------------
            // 1) Shoot first 3 balls
            // ---------------------------------------------------------
            case 1: {
                oneTime(() -> {
                    gantry.moveGantryToPos("back");
                    basePlate.gateHoldBall1();
                    basePlate.prepShootOnly();
                    basePlate.startShootFromPrep();
                });

                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_ALL_WAIT_S) {
                    setPathState(2);
                }
                break;
            }

            // ---------------------------------------------------------
            // 2) Drive forward 40" with intake ON
            // ---------------------------------------------------------
            case 2: {
                oneTime(() -> {
                    gantry.moveGantryToPos("middle");      // intake position
                    intake.intakeIn();

                    double x = follower.getPose().getX();
                    double y = follower.getPose().getY();

                    Path forward40 = straightPathTo(x + FORWARD_INTAKE_DISTANCE, y + 5);
                    follower.followPath(forward40, false); // forward drive
                });

                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;
            }

            // ---------------------------------------------------------
            // 3) Back up 10" with intake STILL ON
            // ---------------------------------------------------------
            case 3: {
                oneTime(() -> {
                    double x = follower.getPose().getX();
                    double y = follower.getPose().getY();

                    Path back10 = straightPathTo(x - BACKUP_DISTANCE, y - 5);
                    follower.followPath(back10, true); // reverse drive (likely)
                });

                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            }

            // ---------------------------------------------------------
            // 4) Forward 10" with intake STILL ON
            // ---------------------------------------------------------
            case 4: {
                oneTime(() -> {
                    double x = follower.getPose().getX();
                    double y = follower.getPose().getY();

                    Path forward10 = straightPathTo(x + BUMP_FORWARD_DISTANCE, y);
                    follower.followPath(forward10, false); // forward drive
                });

                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            }

            // ---------------------------------------------------------
            // 5) Stop intake, gate down/hold, prep for shoot
            // ---------------------------------------------------------
            case 5: {
                oneTime(() -> {
                    intake.intakeStop();

                    // "Gate down" / staged for shooting (same prep idea as before)
                    basePlate.gateHoldBall1();
                    basePlate.prepShootOnly();

                    // Keep same turret angle and flywheel speed
                    turret.setAngle(INIT_TURRET_ANGLE_DEG);
                    flywheelASG.setTargetVelocity(FLYWHEEL_SPINUP_VELOCITY);

                    gantry.moveGantryToPos("back");
                });

                // no extra wait required here unless you want one
                setPathState(6);
                break;
            }

            // ---------------------------------------------------------
            // 6) Return to starting position (0,0), keep turret angle/speed
            // ---------------------------------------------------------
            case 6: {
                oneTime(() -> {
                    turret.setAngle(INIT_TURRET_ANGLE_DEG - 1);
                    flywheelASG.setTargetVelocity(FLYWHEEL_SPINUP_VELOCITY);

                    Path returnToStart = straightPathTo(0, 0);
                    follower.followPath(returnToStart, true); // reverse back to shooting spot
                });

                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            }

            // ---------------------------------------------------------
            // 7) Settle for 1 second
            // ---------------------------------------------------------
            case 7: {
                if (pathTimer.getElapsedTimeSeconds() >= SETTLE_WAIT_S) {
                    setPathState(8);
                }
                break;
            }

            // ---------------------------------------------------------
            // 8) Shoot second set of 3 balls
            // ---------------------------------------------------------
            case 8: {
                oneTime(() -> {
                    // Reassert everything before shooting
                    turret.setAngle(INIT_TURRET_ANGLE_DEG);
                    flywheelASG.setTargetVelocity(FLYWHEEL_SPINUP_VELOCITY);
                    gantry.moveGantryToPos("back");

                    basePlate.gateHoldBall1();
                    basePlate.prepShootOnly();
                    basePlate.startShootFromPrep();
                });

                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_ALL_WAIT_S) {
                    setPathState(9);
                }
                break;
            }

            // ---------------------------------------------------------
            // 9) Move turret to 0 degrees
            // ---------------------------------------------------------
            case 9: {

                if (pathTimer.getElapsedTimeSeconds() >= TURRET_ZERO_WAIT_S) {
                    setPathState(2);
                }
                break;
            }

            // ---------------------------------------------------------
            // 10) Leave line / park
            // ---------------------------------------------------------
            case 10: {
                oneTime(() -> {
                    double y = follower.getPose().getY();
                    Path leaveLine = straightPathTo(LEAVE_DISTANCE, y);
                    follower.followPath(leaveLine, false);
                });

                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            }

            // ---------------------------------------------------------
            // 11) Finish / safe stop
            // ---------------------------------------------------------
            case 11: {
                intake.intakeStop();
                flywheelASG.setTargetVelocity(0);
                setPathState(-1);
                break;
            }

            default:
                break;
        }
    }
}