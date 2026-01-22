package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TEST InterShot Spacing (Selectable FSM)", group="TEST")
public class TestSorting extends LinearOpMode {

    private BasePlate basePlate;
    private Gantry gantry;

    // ============================================================
    // Selectable MODE (FSM1 = your existing test, FSM2 = new sequence)
    // ============================================================
    private enum Mode { FSM1, FSM2 }
    private Mode mode = Mode.FSM2;

    // ============================================================
    // Ball order (used for FSM1; and for FSM2 we only care about shots 2->3)
    // ============================================================
    // 'P' = purple, 'G' = green
    private final char[] order = new char[]{'P','P','G'};

    // Delay applied ONLY when a color changes between consecutive shots (FSM1)
    private static final double COLOR_CHANGE_DELAY_S = 2.0;

    // ============================================================
    // FSM2 tunables (all in seconds)
    // ============================================================
    private static final double FSM2_DELAY_BEFORE_FRONT_POPPER_UP_S = 0.9;  // after gate-down+ramp-back+gantry-front start
    private static final double FSM2_FRONT_POPPER_UP_HOLD_S         = 0.35;  // how long to keep it "up"
    private static final double FSM2_FRONT_POPPER_DOWN_SETTLE_S     = 0.25;  // allow time to come down / settle
    private static final double FSM2_DELAY_ALLOW_GANTRY_BACK_S      = 0.5;  // after commanding gantry back + ramp forward + pusher/gate staging

    // ============================================================
    // FSM1 states (your existing flow)
    // ============================================================
    private enum StateFSM1 {
        INIT,
        PREP,
        START_SHOOT,
        RUN,
        DONE
    }
    private StateFSM1 state1 = StateFSM1.INIT;

    // ============================================================
    // FSM2 states (new flow)
    // ============================================================
    private enum StateFSM2 {
        INIT,

        // Step 1: gate down (hold), ramp BACK, gantry FRONT (all at once)
        STEP1_COMMAND_CONCURRENT,
        STEP1_WAIT_BEFORE_POPPER,

        // Step 2: front popper UP then DOWN with configurable timings
        POPPER_UP,
        POPPER_UP_WAIT,
        POPPER_DOWN,
        POPPER_DOWN_WAIT,

        // Step 3: ramp FORWARD, gantry BACK, stage gate+pusher to "about to fire ball 2" (all at once)
        STEP3_STAGE_BEFORE_SHOT2,
        STEP3_WAIT_ALLOW_GANTRY_BACK,

        // Step 4: start BasePlate FSM beginning at ball 2, then run until complete
        START_BASEPLATE_FROM_BALL2,
        RUN_BASEPLATE,
        DONE
    }
    private StateFSM2 state2 = StateFSM2.INIT;

    private final ElapsedTime fsm2Timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        basePlate = new BasePlate(hardwareMap);
        gantry = new Gantry(hardwareMap);

        // Shooter motor spin (as in your test)
        DcMotor motor = hardwareMap.get(DcMotorEx.class, "shooter1");
        motor.setPower(-0.5);

        // ============================================================
        // Mode select loop (before start)
        //   X -> FSM1
        //   Y -> FSM2
        // ============================================================
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) mode = Mode.FSM1;
            if (gamepad1.y) mode = Mode.FSM2;

            telemetry.addLine("Select mode:");
            telemetry.addLine("  X = FSM1 (inter-shot spacing test)");
            telemetry.addLine("  Y = FSM2 (front-pop + reposition + start from ball 2)");
            telemetry.addData("mode", mode);

            telemetry.addData("order", "" + order[0] + order[1] + order[2]);
            telemetry.addData("colorChangeDelayS (FSM1)", COLOR_CHANGE_DELAY_S);

            telemetry.addLine();
            telemetry.addLine("FSM2 tunables:");
            telemetry.addData("delayBeforePopperUpS", FSM2_DELAY_BEFORE_FRONT_POPPER_UP_S);
            telemetry.addData("popperUpHoldS", FSM2_FRONT_POPPER_UP_HOLD_S);
            telemetry.addData("popperDownSettleS", FSM2_FRONT_POPPER_DOWN_SETTLE_S);
            telemetry.addData("delayAllowGantryBackS", FSM2_DELAY_ALLOW_GANTRY_BACK_S);

            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Optional: put things in a predictable starting state right after start
        // (safe defaults; adjust if you prefer)
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();

        while (opModeIsActive()) {

            // Always update BasePlate FSM so it can run when commanded
            basePlate.update();

            if (mode == Mode.FSM1) {
                runFSM1();
            } else {
                runFSM2();
            }

            telemetry.addData("mode", mode);
            telemetry.addData("order", "" + order[0] + order[1] + order[2]);

            telemetry.addData("baseplateBusy", basePlate.isShootBusy());
            telemetry.addData("shootState", String.valueOf(basePlate.getShootState()));
            telemetry.addData("holdingForSpacing", basePlate.isHoldingForInterShotSpacing());
            telemetry.addData("pusherMmCmd", basePlate.getLastPusherMmCmd());

            if (mode == Mode.FSM1) telemetry.addData("fsm1State", state1);
            if (mode == Mode.FSM2) telemetry.addData("fsm2State", state2);

            telemetry.update();
        }
    }

    // ============================================================
    // FSM1 (your existing test, unchanged behavior)
    // ============================================================
    private void runFSM1() {
        switch (state1) {
            case INIT: {
                basePlate.setInterShotDelaysFromColors(order[0], order[1], order[2], COLOR_CHANGE_DELAY_S);
                state1 = StateFSM1.PREP;
                break;
            }

            case PREP: {
                basePlate.prepShootOnly();
                gantry.moveGantryToPos("back");
                state1 = StateFSM1.START_SHOOT;
                break;
            }

            case START_SHOOT: {
                basePlate.startShootFromPrep();
                state1 = StateFSM1.RUN;
                break;
            }

            case RUN: {
                if (!basePlate.isShootBusy()) state1 = StateFSM1.DONE;
                break;
            }

            case DONE:
            default:
                // idle
                break;
        }
    }

    // ============================================================
    // FSM2 (new sequence you described)
    // ============================================================
    private void runFSM2() {
        switch (state2) {

            case INIT: {
                // Reset + prepare. We do NOT call prepShootOnly because you explicitly want ramp BACK first.
                // Put gantry somewhere deterministic if you want; not required.
                state2 = StateFSM2.STEP1_COMMAND_CONCURRENT;
                break;
            }

            case STEP1_COMMAND_CONCURRENT: {
                // "Gate down to hold balls (like rapid fire), but ramp BACK instead of forward"
                basePlate.gateHoldBall2();
                basePlate.rampBack();

                // Concurrent: gantry to FRONT
                gantry.moveGantryToPos("front");

                fsm2Timer.reset();
                state2 = StateFSM2.STEP1_WAIT_BEFORE_POPPER;
                break;
            }

            case STEP1_WAIT_BEFORE_POPPER: {
                if (fsm2Timer.seconds() >= FSM2_DELAY_BEFORE_FRONT_POPPER_UP_S) {
                    state2 = StateFSM2.POPPER_UP;
                }
                break;
            }

            case POPPER_UP: {
                basePlate.frontPopperUp();
                fsm2Timer.reset();
                state2 = StateFSM2.POPPER_UP_WAIT;
                break;
            }

            case POPPER_UP_WAIT: {
                if (fsm2Timer.seconds() >= FSM2_FRONT_POPPER_UP_HOLD_S) {
                    state2 = StateFSM2.POPPER_DOWN;
                }
                break;
            }

            case POPPER_DOWN: {
                basePlate.frontPopperDown();
                fsm2Timer.reset();
                state2 = StateFSM2.POPPER_DOWN_WAIT;
                break;
            }

            case POPPER_DOWN_WAIT: {
                if (fsm2Timer.seconds() >= FSM2_FRONT_POPPER_DOWN_SETTLE_S) {
                    state2 = StateFSM2.STEP3_STAGE_BEFORE_SHOT2;
                }
                break;
            }

            case STEP3_STAGE_BEFORE_SHOT2: {
                // "Once popper back down, back ramp forward, gantry back,
                //  and gate+pusher to the point where about to fire the second ball" all concurrently.

                basePlate.rampForward();
                gantry.moveGantryToPos("back");

                // Stage to *right before shot #2 moment* (i.e., at entry to PUSH_1_WAIT_MID):
                // pusher at push1+push2, gate at holdBall3.
                double stageMm = basePlate.getShootPush1Mm() + basePlate.getShootPush2Mm();
                basePlate.setPusherMm(stageMm);
                basePlate.gateHoldBall3();

                fsm2Timer.reset();
                state2 = StateFSM2.STEP3_WAIT_ALLOW_GANTRY_BACK;
                break;
            }

            case STEP3_WAIT_ALLOW_GANTRY_BACK: {
                // "After a delay has passed (e.g., 1s) to allow gantry to go all the way back"
                if (fsm2Timer.seconds() >= FSM2_DELAY_ALLOW_GANTRY_BACK_S) {
                    state2 = StateFSM2.START_BASEPLATE_FROM_BALL2;
                }
                break;
            }

            case START_BASEPLATE_FROM_BALL2: {
                // Key point:
                // We want to start the BasePlate FSM effectively at "before shot 2", then let it rapid-fire shot2+shot3.
                //
                // We MUST ensure the shot1->shot2 spacing doesn’t hold us (since we’re starting at ball2).
                // So we set delay12=0, and set delay23 based on colors between ball2 and ball3.
                basePlate.setInterShotExtraDelays(0.0, 0);

                // Start from the internal "before shot2" point.
                // This method is NEW and must be added to BasePlate (see code below).
                basePlate.startShootFromBeforeShot2();

                state2 = StateFSM2.RUN_BASEPLATE;
                break;
            }

            case RUN_BASEPLATE: {
                if (!basePlate.isShootBusy()) state2 = StateFSM2.DONE;
                break;
            }

            case DONE:
            default:
                // idle
                break;
        }
    }
}