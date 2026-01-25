package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlate;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;

@TeleOp(name="TEST Sort Harness (Current->Desired, Prep+Execute)", group="TEST")
public class TestSorting extends LinearOpMode {

    // ============================================================
    // Adjustable variables (tune here)
    // ============================================================

    // --- Shooter motor spin (optional for testing) ---
    private static final double SHOOTER_POWER = -0.50; // negative for your config

    // --- Inter-shot spacing for COLOR changes (applies ONLY inside BasePlate rapid-fire) ---
    // This is the "total spacing between shot events" target; BasePlate counts time spent on motions too.
    private static final double COLOR_CHANGE_DELAY_S = 1.00;

    // --- Gantry move timing assumptions (used as timed waits inside prep/execute) ---
    // Make these conservative at first; you can shorten after testing.
    private static final double GANTRY_BACK_TO_FRONT_S  = 0.90;
    private static final double GANTRY_BACK_TO_MIDDLE_S = 0.45; // about half of back->front
    private static final double GANTRY_ANY_SETTLE_S      = 0.10; // extra settle time after a move

    // --- Popper timing (up/down holds) ---
    private static final double POPPER_UP_HOLD_S     = 0.35; // how long to hold popper "up"
    private static final double POPPER_DOWN_SETTLE_S = 0.25; // settle after popper goes down

    // --- FSM "ball2 rapid fire" staging waits ---
    // After staging gantry back + ramp forward + pusher/gate staged, wait before starting BasePlate
    private static final double ALLOW_GANTRY_BACK_BEFORE_FIRE_S = 0.50;

    // --- Case B specific: "retension" pusher move ---
    // After first middle-pop, move pusher to retension remaining balls.
    // This should be a position that pulls balls into consistent seating.
    private static final double RETENSION_PUSHER_MM = 127.0;
    private static final double RETENSION_HOLD_S    = 0.25;

    // --- End-of-case reset behavior ---
    private static final boolean RESET_SHOOTER_MOTOR_AT_END = false;

    // ============================================================
    // Subsystems
    // ============================================================
    private BasePlate basePlate;
    private Gantry gantry;

    private DcMotor shooterMotor;

    // ============================================================
    // Patterns (always 2P + 1G)
    // back->front (inside robot): [shot1, shot2, shot3]
    // ============================================================
    private enum Pattern {
        GPP(new char[]{'G','P','P'}),
        PGP(new char[]{'P','G','P'}),
        PPG(new char[]{'P','P','G'});

        final char[] backToFront;
        Pattern(char[] b2f) { this.backToFront = b2f; }

        @Override public String toString() {
            return "" + backToFront[0] + backToFront[1] + backToFront[2];
        }
    }

    // User-selectable during init
    private Pattern currentPattern = Pattern.GPP;  // balls currently inside robot (back->front)
    private Pattern desiredPattern = Pattern.GPP;  // desired out order (shot1->shot3)

    // ============================================================
    // Strategy buckets (your 4 “unique actions”)
    // ============================================================
    private enum Strategy {
        // Case A: current == desired -> normal rapid fire (3 shots) with BasePlate spacing
        A_NORMAL_RAPID_FIRE,

        // Case B: middle pop, retension, middle pop, then shoot LAST ball only (via BasePlate)
        // Applies to:
        //   current=GPP desired=PPG
        //   current=PPG desired=PGP
        B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST,

        // Case C: go FRONT, front pop, then BasePlate from ball2 for remaining 2
        // Applies to:
        //   current=PPG desired=GPP
        //   current=PGP desired=PPG
        C_FRONT_POP_THEN_BALL2_RAPID,

        // Case D: go MIDDLE, middle pop, then BasePlate from ball2 for remaining 2
        // Applies to:
        //   current=PGP desired=GPP
        //   current=GPP desired=PGP
        D_MIDDLE_POP_THEN_BALL2_RAPID,

        INVALID
    }

    // ============================================================
    // Harness FSM (high-level)
    // ============================================================
    private enum HarnessState {
        INIT_HOLDING,    // just after start: set safe holding state
        IDLE,            // waiting for LB (prepare)
        PREPARING,       // running prepare sequence for selected case
        ARMED,           // prepared; waiting for RB (execute)
        EXECUTING,       // executing selected case
        RESETTING        // end-of-case reset
    }
    private HarnessState harnessState = HarnessState.INIT_HOLDING;

    // Sub-state for preparing and executing (keeps logic clean)
    private int step = 0;
    private double stepStartTimeS = 0.0;

    // Edge-detect buttons so you don’t retrigger while holding
    private boolean prevLB = false;
    private boolean prevRB = false;
    private boolean prevDpadUp = false, prevDpadDown = false, prevDpadLeft = false, prevDpadRight = false;

    // ============================================================
    // Main
    // ============================================================
    @Override
    public void runOpMode() {
        basePlate = new BasePlate(hardwareMap);
        gantry = new Gantry(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor.setPower(0);

        // ----------------------------
        // INIT LOOP: choose patterns
        // ----------------------------
        while (!isStarted() && !isStopRequested()) {

            // D-pad controls:
            //   Left/Right = cycle CURRENT pattern
            //   Up/Down    = cycle DESIRED pattern
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;

            if (dl && !prevDpadLeft)  currentPattern = prevPattern(currentPattern);
            if (dr && !prevDpadRight) currentPattern = nextPattern(currentPattern);

            if (du && !prevDpadUp)    desiredPattern = prevPattern(desiredPattern);
            if (dd && !prevDpadDown)  desiredPattern = nextPattern(desiredPattern);

            prevDpadUp = du; prevDpadDown = dd; prevDpadLeft = dl; prevDpadRight = dr;

            // Force safe hold during init (as you requested)
            basePlate.rampBack();
            basePlate.gateHoldBall1(); // “first hold” position to keep balls from moving
            basePlate.frontPopperDown();
            basePlate.middlePopperDown();
            gantry.moveGantryToPos("back");

            Strategy strat = resolveStrategy(currentPattern, desiredPattern);

            telemetry.addLine("=== Sort Harness Init ===");
            telemetry.addLine("D-pad LEFT/RIGHT: CURRENT (back->front)");
            telemetry.addLine("D-pad UP/DOWN:    DESIRED (shot1->shot3)");
            telemetry.addData("CURRENT", currentPattern.toString());
            telemetry.addData("DESIRED", desiredPattern.toString());
            telemetry.addData("Strategy", strat);
            telemetry.addLine();

            telemetry.addData("COLOR_CHANGE_DELAY_S", COLOR_CHANGE_DELAY_S);
            telemetry.addLine("After Start:");
            telemetry.addLine("  LB = PREPARE (stage gantry/ramp/gate)");
            telemetry.addLine("  RB = EXECUTE (run shots)");
            telemetry.addLine();

            telemetry.addLine("Init enforcement (always): rampBack + gateHoldBall1");
            telemetry.update();

            idle();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Start shooter motor (optional)
        shooterMotor.setPower(SHOOTER_POWER);

        // Immediately put in safe holding state at start as well (you requested)
        basePlate.cancelShootAndReset();
        basePlate.rampBack();
        basePlate.gateHoldBall1();
        gantry.moveGantryToPos("back");

        harnessState = HarnessState.IDLE;

        // ----------------------------
        // MAIN LOOP
        // ----------------------------
        while (opModeIsActive()) {

            // Always update BasePlate FSM so it can run once we start it
            basePlate.update();

            // Button edges
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean lbPressed = lb && !prevLB;
            boolean rbPressed = rb && !prevRB;
            prevLB = lb;
            prevRB = rb;

            Strategy strat = resolveStrategy(currentPattern, desiredPattern);

            switch (harnessState) {

                case IDLE: {
                    if (lbPressed) {
                        // Begin PREPARE
                        step = 0;
                        stepStartTimeS = getRuntime();
                        harnessState = HarnessState.PREPARING;
                    }
                    break;
                }

                case PREPARING: {
                    if (runPrepare(strat)) {
                        harnessState = HarnessState.ARMED;
                    }
                    break;
                }

                case ARMED: {
                    if (rbPressed) {
                        // Begin EXECUTE
                        step = 0;
                        stepStartTimeS = getRuntime();
                        harnessState = HarnessState.EXECUTING;
                    }
                    break;
                }

                case EXECUTING: {
                    if (runExecute(strat)) {
                        // Begin RESET
                        step = 0;
                        stepStartTimeS = getRuntime();
                        harnessState = HarnessState.RESETTING;
                    }
                    break;
                }

                case RESETTING: {
                    if (runReset()) {
                        harnessState = HarnessState.IDLE;
                    }
                    break;
                }

                default:
                    harnessState = HarnessState.IDLE;
                    break;
            }

            // ----------------------------
            // Telemetry
            // ----------------------------
            telemetry.addLine("=== Sort Harness Runtime ===");
            telemetry.addData("CURRENT (back->front)", currentPattern.toString());
            telemetry.addData("DESIRED (shot1->shot3)", desiredPattern.toString());
            telemetry.addData("Strategy", strat);
            telemetry.addData("HarnessState", harnessState);
            telemetry.addData("step", step);

            telemetry.addLine();
            telemetry.addData("BasePlateState", basePlate.getShootState());
            telemetry.addData("BasePlateBusy", basePlate.isShootBusy());
            telemetry.addData("HoldingForSpacing", basePlate.isHoldingForInterShotSpacing());
            telemetry.addData("LastPusherMmCmd", basePlate.getLastPusherMmCmd());

            telemetry.update();
        }
    }

    // ============================================================
    // Prepare phase: stage gantry/ramp/gate for the chosen case
    // LB triggers this; it runs until complete, then ARMED.
    // ============================================================
    private boolean runPrepare(Strategy strat) {

        switch (strat) {

            case A_NORMAL_RAPID_FIRE: {
                // Prepare for a full 3-shot BasePlate rapid fire.
                // We want ramp forward and gate staged (prepShootOnly).
                if (step == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();
                    gantry.moveGantryToPos("back"); // “gantry stays”; choose back as default
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 1) {
                    // Stage rapid-fire “prep”
                    basePlate.prepShootOnly(); // rampForward + gateHoldBall2, then PREPPED
                    return true;
                }
                break;
            }

            case B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST: {
                // Prepare: gantry to middle, ramp back, gate holding (so balls don’t drift)
                if (step == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();
                    gantry.moveGantryToPos("middle");
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 1) {
                    if (elapsed(stepStartTimeS) >= (GANTRY_BACK_TO_MIDDLE_S + GANTRY_ANY_SETTLE_S)) {
                        // Hold balls like you requested (gate down holding)
                        basePlate.gateHoldBall2(); // “hold like rapid fire” style
                        basePlate.rampBack();
                        return true;
                    }
                }
                break;
            }

            case C_FRONT_POP_THEN_BALL2_RAPID: {
                // Prepare: gantry to front; ramp back; gate holding
                if (step == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();
                    gantry.moveGantryToPos("front");
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 1) {
                    if (elapsed(stepStartTimeS) >= (GANTRY_BACK_TO_FRONT_S + GANTRY_ANY_SETTLE_S)) {
                        basePlate.gateHoldBall2();
                        basePlate.rampBack();
                        return true;
                    }
                }
                break;
            }

            case D_MIDDLE_POP_THEN_BALL2_RAPID: {
                // Prepare: gantry to middle; ramp back; gate holding
                if (step == 0) {
                    basePlate.cancelShootAndReset();
                    basePlate.rampBack();
                    basePlate.gateHoldBall1();
                    gantry.moveGantryToPos("middle");
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 1) {
                    if (elapsed(stepStartTimeS) >= (GANTRY_BACK_TO_MIDDLE_S + GANTRY_ANY_SETTLE_S)) {
                        basePlate.gateHoldBall2();
                        basePlate.rampBack();
                        return true;
                    }
                }
                break;
            }

            default:
                // Invalid mapping; do nothing
                return true;
        }

        return false;
    }

    // ============================================================
    // Execute phase: run the actual action sequence for the case
    // RB triggers this; it runs until complete, then RESETTING.
    // ============================================================
    private boolean runExecute(Strategy strat) {

        // Desired order (shot1->shot3) as chars
        char[] desired = desiredPattern.backToFront;

        switch (strat) {

            case A_NORMAL_RAPID_FIRE: {
                // Configure inter-shot delays from desired colors (shot1->shot2 and shot2->shot3)
                if (step == 0) {
                    basePlate.setInterShotDelaysFromColors(desired[0], desired[1], desired[2], COLOR_CHANGE_DELAY_S);
                    step++;
                }
                else if (step == 1) {
                    // Kick off BasePlate from PREPPED
                    basePlate.startShootFromPrep();
                    step++;
                }
                else if (step == 2) {
                    // Wait until BasePlate finishes
                    if (!basePlate.isShootBusy()) return true;
                }
                break;
            }

            case C_FRONT_POP_THEN_BALL2_RAPID: {
                // Front popper fires the first ball, then BasePlate starts at ball2 to fire remaining two.
                //
                // Color spacing is only enforced within BasePlate for shots 2->3 (i.e., desired[1] vs desired[2]).
                if (step == 0) {
                    // Ensure gate/ramp are in your “hold” staging
                    basePlate.gateHoldBall2();
                    basePlate.rampBack();
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 1) {
                    // Front popper UP
                    basePlate.frontPopperUp();
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 2) {
                    if (elapsed(stepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.frontPopperDown();
                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 3) {
                    if (elapsed(stepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        // Stage for “before shot2”: ramp forward, gantry back, pusher staged, gate staged
                        basePlate.rampForward();
                        gantry.moveGantryToPos("back");

                        double stageMm = basePlate.getShootPush1Mm() + basePlate.getShootPush2Mm();
                        basePlate.setPusherMm(stageMm);
                        basePlate.gateHoldBall3();

                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 4) {
                    if (elapsed(stepStartTimeS) >= ALLOW_GANTRY_BACK_BEFORE_FIRE_S) {
                        // Configure only shot2->shot3 spacing based on desired colors
                        double d23 = (desired[1] == desired[2]) ? 0.0 : COLOR_CHANGE_DELAY_S;
                        basePlate.setInterShotExtraDelays(0.0, d23);

                        basePlate.startShootFromBeforeShot2();
                        step++;
                    }
                }
                else if (step == 5) {
                    if (!basePlate.isShootBusy()) return true;
                }
                break;
            }

            case D_MIDDLE_POP_THEN_BALL2_RAPID: {
                // Middle popper fires first ball, then BasePlate from ball2 for remaining two.
                if (step == 0) {
                    basePlate.gateHoldBall2();
                    basePlate.rampBack();
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 1) {
                    // Middle popper UP
                    basePlate.middlePopperUp();
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 2) {
                    if (elapsed(stepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.middlePopperDown();
                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 3) {
                    if (elapsed(stepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        // Stage for “before shot2”
                        basePlate.rampForward();
                        gantry.moveGantryToPos("back");

                        double stageMm = basePlate.getShootPush1Mm() + basePlate.getShootPush2Mm();
                        basePlate.setPusherMm(stageMm);
                        basePlate.gateHoldBall3();

                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 4) {
                    if (elapsed(stepStartTimeS) >= ALLOW_GANTRY_BACK_BEFORE_FIRE_S) {
                        double d23 = (desired[1] == desired[2]) ? 0.0 : COLOR_CHANGE_DELAY_S;
                        basePlate.setInterShotExtraDelays(0.0, d23);

                        basePlate.startShootFromBeforeShot2();
                        step++;
                    }
                }
                else if (step == 5) {
                    if (!basePlate.isShootBusy()) return true;
                }
                break;
            }

            case B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST: {
                // Two middle pops (with retension between), then “shoot last ball only”.
                // You said we can ignore color spacing here for now because this sequence is slow enough.
                if (step == 0) {
                    basePlate.gateHoldBall2();
                    basePlate.rampBack();
                    step++;
                }
                else if (step == 1) {
                    // Middle pop #1
                    basePlate.middlePopperUp();
                    stepStartTimeS = getRuntime();
                    step++;
                }
                else if (step == 2) {
                    if (elapsed(stepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.middlePopperDown();
                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 3) {
                    if (elapsed(stepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        // Retension: move pusher back by RETENSION_PUSHER_MM
                        basePlate.setPusherMm(RETENSION_PUSHER_MM);
                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 4) {
                    if (elapsed(stepStartTimeS) >= RETENSION_HOLD_S) {
                        // Middle pop #2
                        basePlate.middlePopperUp();
                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 5) {
                    if (elapsed(stepStartTimeS) >= POPPER_UP_HOLD_S) {
                        basePlate.middlePopperDown();
                        stepStartTimeS = getRuntime();
                        step++;
                    }
                }
                else if (step == 6) {
                    if (elapsed(stepStartTimeS) >= POPPER_DOWN_SETTLE_S) {
                        // Now shoot last ball only.
                        // Stage conditions for BasePlate last-ball-only entry:
                        // - ramp forward (ready to shoot)
                        // - gate staged for the last ball (we choose gateHoldBall2 as the “ready” hold)
                        // - pusher staged where your last-ball behavior expects (BasePlate handles internal timing)
                        basePlate.rampForward();
                        basePlate.gateHoldBall2();
                        basePlate.setPusherMm(RETENSION_PUSHER_MM - 20);
                        gantry.moveGantryToPos("back");

                        // Use the new BasePlate entry that fires only the final ball then performs reset.
                        basePlate.startLastBallOnlyFromStaged();

                        step++;
                    }
                }
                else if (step == 7) {
                    if (!basePlate.isShootBusy()) return true;
                }
                break;
            }

            default:
                return true;
        }

        return false;
    }

    // ============================================================
    // Reset phase: return everything to a known baseline
    // ============================================================
    private boolean runReset() {
        if (step == 0) {
            // Stop any BasePlate FSM and put hardware in baseline
            basePlate.cancelShootAndReset();

            // Your desired reset positions:
            basePlate.rampBack();
            basePlate.gateUp();
            basePlate.setPusherMm(0.0);
            gantry.moveGantryToPos("back");

            if (RESET_SHOOTER_MOTOR_AT_END && shooterMotor != null) {
                shooterMotor.setPower(0.0);
            }

            stepStartTimeS = getRuntime();
            step++;
        }
        else if (step == 1) {
            // Give a moment to settle
            if (elapsed(stepStartTimeS) >= 0.20) {
                return true;
            }
        }
        return false;
    }

    // ============================================================
    // Strategy resolver (the “4 cases cover all 9” mapping)
    // ============================================================
    private Strategy resolveStrategy(Pattern current, Pattern desired) {

        // Case A: current == desired
        if (current == desired) return Strategy.A_NORMAL_RAPID_FIRE;

        // Case B:
        //   current=GPP -> desired=PPG
        //   current=PPG -> desired=PGP
        if (current == Pattern.GPP && desired == Pattern.PPG) return Strategy.B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST;
        if (current == Pattern.PPG && desired == Pattern.PGP) return Strategy.B_MIDDLE_POP_RETENSION_MIDDLE_POP_THEN_LAST;

        // Case C (front pop then ball2 rapid):
        //   current=PPG -> desired=GPP
        //   current=PGP -> desired=PPG
        if (current == Pattern.PPG && desired == Pattern.GPP) return Strategy.C_FRONT_POP_THEN_BALL2_RAPID;
        if (current == Pattern.PGP && desired == Pattern.PPG) return Strategy.C_FRONT_POP_THEN_BALL2_RAPID;

        // Case D (middle pop then ball2 rapid):
        //   current=PGP -> desired=GPP
        //   current=GPP -> desired=PGP
        if (current == Pattern.PGP && desired == Pattern.GPP) return Strategy.D_MIDDLE_POP_THEN_BALL2_RAPID;
        if (current == Pattern.GPP && desired == Pattern.PGP) return Strategy.D_MIDDLE_POP_THEN_BALL2_RAPID;

        return Strategy.INVALID;
    }

    // ============================================================
    // Helpers
    // ============================================================
    private Pattern nextPattern(Pattern p) {
        Pattern[] vals = Pattern.values();
        int i = p.ordinal() + 1;
        if (i >= 3) i = 0; // only first 3 are real patterns; enum has exactly 3
        return vals[i];
    }

    private Pattern prevPattern(Pattern p) {
        Pattern[] vals = Pattern.values();
        int i = p.ordinal() - 1;
        if (i < 0) i = 2;
        return vals[i];
    }

    private double elapsed(double startS) {
        return getRuntime() - startS;
    }
}