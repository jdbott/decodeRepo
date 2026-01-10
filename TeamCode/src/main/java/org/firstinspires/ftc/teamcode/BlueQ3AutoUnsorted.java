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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Blue Auto Unsorted (Refactored Shoot)")
public class BlueQ3AutoUnsorted extends OpMode {

    private DcMotorEx shootMotor2; // mechanically linked
    private DcMotorEx shootMotor1; // mechanically linked

    // Subsystems
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlate basePlate;
    private Turret turret;

    // State / timing
    private int pathState;
    private Timer pathTimer;
    private boolean oneTimeDone;

    // Telemetry
    private MultipleTelemetry telemetryA;

    // Re-used paths
    private Path toGateIntake;

    // Loop vars
    private int gateTrips = 0;

    // Tunables
    private static final double SHOOT_WAIT_S = 1.5;
    private static final double INTAKE_STOP_DELAY_INTO_SHOOT_PATH_S = 0.5;

    // Intake carry flag: set TRUE when an intake path ends; cleared after we stop intake 0.5s into the shoot path.
    private boolean intakeCarryPending = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35.791, 135.000, Math.toRadians(90)));
        follower.updatePose();
        follower.setMaxPower(1);

        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new Intake(hardwareMap);
        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlate(hardwareMap);
        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);

        intake.intakeStop();
        gantry.moveGantryToPos("back");
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.setPusherMm(0);
        basePlate.gateHoldBall1();
        basePlate.prepShootOnly();

        pathTimer = new Timer();
        pathState = 0;
        oneTimeDone = false;
        intakeCarryPending = false;
        gateTrips = 0;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        basePlate.update();
        turret.update();
        turret.setAngle(180);
    }

    @Override
    public void start() {
        setPathState(0);
        shootMotor2.setPower(0.8);
        shootMotor1.setPower(0.8);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        basePlate.update();
        turret.update();

        telemetryA.addData("State", pathState);
        telemetryA.addData("Busy", follower.isBusy());
        telemetryA.addData("t", follower.getCurrentTValue());
        telemetryA.addData("intakeCarryPending", intakeCarryPending);
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

    private void stopIntakeIfHalfSecondIntoShootPath() {
        if (intakeCarryPending && pathTimer.getElapsedTimeSeconds() >= INTAKE_STOP_DELAY_INTO_SHOOT_PATH_S) {
            intake.intakeStop();
            intakeCarryPending = false;
        }
    }

    private void markIntakePathFinishedGateDownAndCarry() {
        basePlate.gateHoldBall1();     // "gate position to one" immediately after intake path ends
        intakeCarryPending = true;     // keep intake running into next shoot path
        // do NOT stop intake here
    }

    private void autoPathUpdate() {
        switch (pathState) {

            // ------------------------------------------------------------------
            // SHOOT 1
            // ------------------------------------------------------------------
            case 0: {
                Path toShoot1 = new Path(new BezierLine(
                        new Pose(35.791, 135),
                        new Pose(57, 85))
                );
                toShoot1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135), 0.8);

                basePlate.prepShootOnly();
                follower.followPath(toShoot1, true);
                setPathState(1);
                break;
            }

            case 1: {
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            }

            case 2: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(3);
                }
                break;
            }

            // ------------------------------------------------------------------
            // LINE 2 INTAKE
            // ------------------------------------------------------------------
            case 3: {
                Path toLine2 = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84 - 20),
                        new Pose(24, 84 - 22))
                );
                toLine2.setTangentHeadingInterpolation();
                turret.setAngle(-80);
                follower.followPath(toLine2, false);
                intake.intakeIn();
                setPathState(4);
                break;
            }

            case 4: {
                if (!follower.isBusy()) {
                    markIntakePathFinishedGateDownAndCarry();
                    setPathState(5); // go to SHOOT 2 start
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT 2
            // ------------------------------------------------------------------
            case 5: {
                Path toShoot2 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot2.setConstantHeadingInterpolation(Math.toRadians(180));

                basePlate.prepShootOnly();
                follower.followPath(toShoot2, true);
                setPathState(6);
                break;
            }

            case 6: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            }

            case 7: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(8);
                }
                break;
            }

            // ------------------------------------------------------------------
            // GATE INTAKE LOOP (2 trips)
            // ------------------------------------------------------------------
            case 8: {
                toGateIntake = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(17.5, 64.5))
                );
                toGateIntake.setTangentHeadingInterpolation();

                follower.followPath(toGateIntake, true);
                intake.intakeIn();
                setPathState(9);
                break;
            }

            case 9: {
                if (follower.getCurrentTValue() > 0.2) {
                    toGateIntake.setConstantHeadingInterpolation(Math.toRadians(140));
                }
                if (!follower.isBusy()) {
                    gateTrips++;
                    markIntakePathFinishedGateDownAndCarry();
                    if (gateTrips < 2) {
                        setPathState(5);   // back to SHOOT 2
                    } else {
                        setPathState(10);  // shoot after last gate
                    }
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT AFTER LAST GATE
            // ------------------------------------------------------------------
            case 10: {
                Path toShootAfterLastGate = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShootAfterLastGate.setConstantHeadingInterpolation(Math.toRadians(180));

                basePlate.prepShootOnly();
                follower.followPath(toShootAfterLastGate, true);
                setPathState(11);
                break;
            }

            case 11: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            }

            case 12: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(13);
                }
                break;
            }

            // ------------------------------------------------------------------
            // CLOSE LINE INTAKE
            // ------------------------------------------------------------------
            case 13: {
                Path toCloseLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84),
                        new Pose(24, 82))
                );
                toCloseLine.setTangentHeadingInterpolation();

                follower.followPath(toCloseLine, false);
                intake.intakeIn();
                setPathState(14);
                break;
            }

            case 14: {
                if (!follower.isBusy()) {
                    markIntakePathFinishedGateDownAndCarry();
                    setPathState(15); // shoot 3 start
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT 3
            // ------------------------------------------------------------------
            case 15: {
                Path toShoot3 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot3.setTangentHeadingInterpolation();
                toShoot3.reverseHeadingInterpolation();

                basePlate.prepShootOnly();
                follower.followPath(toShoot3, true);
                setPathState(16);
                break;
            }

            case 16: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;
            }

            case 17: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(18);
                }
                break;
            }

            // ------------------------------------------------------------------
            // BOTTOM LINE INTAKE
            // ------------------------------------------------------------------
            case 18: {
                Path toBottomLine = new Path(new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(48, 84 - 48),
                        new Pose(24, 84 - 50))
                );
                toBottomLine.setTangentHeadingInterpolation();

                follower.followPath(toBottomLine, false);
                intake.intakeIn();
                setPathState(19);
                break;
            }

            case 19: {
                if (follower.getCurrentTValue() > 0.4) {
                    follower.setMaxPower(0.7);
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    markIntakePathFinishedGateDownAndCarry();
                    setPathState(20); // shoot 4 start
                }
                break;
            }

            // ------------------------------------------------------------------
            // SHOOT 4
            // ------------------------------------------------------------------
            case 20: {
                Path toShoot4 = new Path(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(57, 85))
                );
                toShoot4.setTangentHeadingInterpolation();
                toShoot4.reverseHeadingInterpolation();

                basePlate.prepShootOnly();
                follower.followPath(toShoot4, true);
                setPathState(21);
                break;
            }

            case 21: {
                stopIntakeIfHalfSecondIntoShootPath();
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            }

            case 22: {
                oneTime(() -> basePlate.startShootFromPrep());
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_WAIT_S) {
                    setPathState(23);
                }
                break;
            }

            // DONE
            case 23: {
                intake.intakeStop();
                basePlate.rampBack();
                basePlate.gateUp();
                break;
            }

            default:
                break;
        }
    }
}