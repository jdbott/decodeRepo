package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Three-ball pattern shooter with a 9-case plan table and generic executor.
 *
 * <p>This class coordinates BasePlate (ramp, poppers, pusher) and Gantry (shooter carriage)
 * to achieve a desired color shoot order from a known 3-ball chamber state.</p>
 */
public class ThreeBallPatternShooter {
    public enum MacroStep {
        POP_MID,
        POP_FRONT,
        PUSH_ONE,
        ROTATE_LAST,
        RAPID_TWO_LEFT,
        RAPID_TWO_FIRST
    }

    private enum RampMode { BACK, FORWARD }
    private enum ShooterPos { BACK, MID, FRONT }

    private enum StepPhase {
        START,
        WAIT_POSITION,
        WAIT_GATE,
        POPPER_UP_WAIT,
        POPPER_DOWN_WAIT,
        RETENSION_WAIT,
        PUSH_STROKE_WAIT,
        PUSH_RETURN_WAIT,
        ROTATE_BACK_WAIT,
        ROTATE_SHOVE_WAIT,
        ROTATE_RESET_WAIT,
        COMPLETE
    }

    private static final long COLOR_DELAY_MS = 500;
    private static final long PUSH_SPACING_MS = 200;

    private static final long RAMP_SETTLE_MS = 200;
    private static final long SHOOTER_SETTLE_MS = 200;
    private static final long POPPER_UP_MS = 120;
    private static final long POPPER_DOWN_MS = 120;
    private static final long SHOT_CLEAR_MS = 120;
    private static final long PUSH_STROKE_MS = 200;
    private static final long PUSH_RETURN_MS = 160;
    private static final long ROTATE_BACK_MS = 160;
    private static final long ROTATE_SHOVE_MS = 200;
    private static final long ROTATE_RESET_MS = 160;
    private static final long RETENSION_MS = 150;

    private static final double PUSH_HOME_MM = 0.0;
    private static final double PUSH_ONE_MM = 60.0;
    private static final double ROTATE_BACK_MM = 40.0;
    private static final double ROTATE_SHOVE_MM = 80.0;

    private static final double TENSION_BACK_MM = 0.0;
    private static final double RAMP_TRAVEL_MM = 70.0;
    private static final double TENSION_TUNE_OFFSET_MM = 0.0;

    private static final Map<String, Map<String, MacroStep[]>> PLAN_TABLE = buildPlanTable();

    private final BasePlate basePlate;
    private final Gantry gantry;

    private List<MacroStep> plan;
    private char[] desiredShots;
    private int planIndex;
    private int shotsFired;
    private int ballsRemaining;

    private MacroStep currentStep;
    private StepPhase stepPhase;
    private int rapidStep;

    private RampMode rampMode = RampMode.BACK;
    private ShooterPos shooterPos = ShooterPos.BACK;
    private long rampCommandMs = 0;
    private long shooterCommandMs = 0;
    private long waitUntilMs = 0;

    private long lastReleaseMs = 0;
    private long lastPushReleaseMs = 0;
    private Character lastShotColor = null;

    public ThreeBallPatternShooter(BasePlate basePlate, Gantry gantry) {
        this.basePlate = basePlate;
        this.gantry = gantry;
    }

    public void startShootThree(String currentPattern, String desiredPattern) {
        if (!isValidPattern(currentPattern) || !isValidPattern(desiredPattern)) {
            reset();
            return;
        }

        Map<String, MacroStep[]> planRow = PLAN_TABLE.get(currentPattern);
        if (planRow == null || !planRow.containsKey(desiredPattern)) {
            reset();
            return;
        }

        plan = new ArrayList<>();
        for (MacroStep step : planRow.get(desiredPattern)) {
            plan.add(step);
        }

        desiredShots = desiredPattern.toCharArray();
        planIndex = 0;
        shotsFired = 0;
        ballsRemaining = 3;
        lastReleaseMs = 0;
        lastPushReleaseMs = 0;
        lastShotColor = null;

        currentStep = plan.get(0);
        stepPhase = StepPhase.START;
        rapidStep = 0;
    }

    public boolean isBusy() {
        return plan != null && planIndex < plan.size();
    }

    public void update() {
        if (!isBusy()) {
            return;
        }

        long now = System.currentTimeMillis();
        switch (currentStep) {
            case POP_MID:
                handlePopperStep(true, now);
                break;

            case POP_FRONT:
                handlePopperStep(false, now);
                break;

            case PUSH_ONE:
                handlePushOne(now, true);
                break;

            case ROTATE_LAST:
                handleRotateShot(true, now, true, true);
                break;

            case RAPID_TWO_LEFT:
                handleRapidTwoLeft(now);
                break;

            case RAPID_TWO_FIRST:
                handleRapidTwoFirst(now);
                break;

            default:
                advancePlan();
                break;
        }
    }

    public void reset() {
        plan = null;
        desiredShots = null;
        planIndex = 0;
        shotsFired = 0;
        ballsRemaining = 0;
        currentStep = null;
        stepPhase = StepPhase.COMPLETE;
        rapidStep = 0;
        waitUntilMs = 0;
        lastReleaseMs = 0;
        lastPushReleaseMs = 0;
        lastShotColor = null;
    }

    private void handlePopperStep(boolean midPopper, long now) {
        if (stepPhase == StepPhase.START) {
            waitUntilMs = now;
            setRampMode(RampMode.BACK, now);
            setShooterPos(midPopper ? ShooterPos.MID : ShooterPos.FRONT, now);
            stepPhase = StepPhase.WAIT_POSITION;
        }

        if (stepPhase == StepPhase.WAIT_POSITION && now >= waitUntilMs) {
            if (!gateRelease(false, now)) {
                stepPhase = StepPhase.WAIT_GATE;
                return;
            }
            if (midPopper) {
                basePlate.middlePopperUp();
            } else {
                basePlate.frontPopperUp();
            }
            waitUntilMs = now + POPPER_UP_MS;
            stepPhase = StepPhase.POPPER_UP_WAIT;
        }

        if (stepPhase == StepPhase.WAIT_GATE && now >= waitUntilMs) {
            stepPhase = StepPhase.WAIT_POSITION;
            return;
        }

        if (stepPhase == StepPhase.POPPER_UP_WAIT && now >= waitUntilMs) {
            recordRelease(false, now);
            if (midPopper) {
                basePlate.middlePopperDown();
            } else {
                basePlate.frontPopperDown();
            }
            waitUntilMs = Math.max(waitUntilMs, now + POPPER_DOWN_MS);
            stepPhase = StepPhase.POPPER_DOWN_WAIT;
        }

        if (stepPhase == StepPhase.POPPER_DOWN_WAIT && now >= waitUntilMs) {
            if (ballsRemaining >= 2) {
                basePlate.setPusherMm(TENSION_BACK_MM);
                waitUntilMs = now + RETENSION_MS;
                stepPhase = StepPhase.RETENSION_WAIT;
            } else {
                advancePlan();
            }
        }

        if (stepPhase == StepPhase.RETENSION_WAIT && now >= waitUntilMs) {
            advancePlan();
        }
    }

    private boolean handlePushOne(long now, boolean advanceOnComplete) {
        if (stepPhase == StepPhase.START) {
            waitUntilMs = now;
            setRampMode(RampMode.FORWARD, now);
            setShooterPos(ShooterPos.BACK, now);
            stepPhase = StepPhase.WAIT_POSITION;
        }

        if (stepPhase == StepPhase.WAIT_POSITION && now >= waitUntilMs) {
            if (!gateRelease(true, now)) {
                stepPhase = StepPhase.WAIT_GATE;
                return false;
            }
            basePlate.setPusherMm(PUSH_ONE_MM);
            waitUntilMs = now + PUSH_STROKE_MS;
            stepPhase = StepPhase.PUSH_STROKE_WAIT;
        }

        if (stepPhase == StepPhase.WAIT_GATE && now >= waitUntilMs) {
            stepPhase = StepPhase.WAIT_POSITION;
            return false;
        }

        if (stepPhase == StepPhase.PUSH_STROKE_WAIT && now >= waitUntilMs) {
            recordRelease(true, now);
            basePlate.setPusherMm(PUSH_HOME_MM);
            waitUntilMs = Math.max(waitUntilMs, now + PUSH_RETURN_MS);
            stepPhase = StepPhase.PUSH_RETURN_WAIT;
        }

        if (stepPhase == StepPhase.PUSH_RETURN_WAIT && now >= waitUntilMs) {
            if (advanceOnComplete) {
                advancePlan();
            } else {
                stepPhase = StepPhase.START;
            }
            return true;
        }
        return false;
    }

    private boolean handleRotateShot(boolean rotateBackFirst,
                                     long now,
                                     boolean advanceOnComplete,
                                     boolean resetToHome) {
        if (stepPhase == StepPhase.START) {
            waitUntilMs = now;
            setRampMode(RampMode.FORWARD, now);
            setShooterPos(ShooterPos.BACK, now);
            stepPhase = StepPhase.WAIT_POSITION;
        }

        if (stepPhase == StepPhase.WAIT_POSITION && now >= waitUntilMs) {
            if (!gateRelease(true, now)) {
                stepPhase = StepPhase.WAIT_GATE;
                return false;
            }
            if (rotateBackFirst) {
                basePlate.setPusherMm(ROTATE_BACK_MM);
                waitUntilMs = now + ROTATE_BACK_MS;
                stepPhase = StepPhase.ROTATE_BACK_WAIT;
            } else {
                basePlate.setPusherMm(ROTATE_SHOVE_MM);
                waitUntilMs = now + ROTATE_SHOVE_MS;
                stepPhase = StepPhase.ROTATE_SHOVE_WAIT;
            }
        }

        if (stepPhase == StepPhase.WAIT_GATE && now >= waitUntilMs) {
            stepPhase = StepPhase.WAIT_POSITION;
            return false;
        }

        if (stepPhase == StepPhase.ROTATE_BACK_WAIT && now >= waitUntilMs) {
            basePlate.setPusherMm(ROTATE_SHOVE_MM);
            waitUntilMs = now + ROTATE_SHOVE_MS;
            stepPhase = StepPhase.ROTATE_SHOVE_WAIT;
        }

        if (stepPhase == StepPhase.ROTATE_SHOVE_WAIT && now >= waitUntilMs) {
            recordRelease(true, now);
            if (resetToHome) {
                basePlate.setPusherMm(PUSH_HOME_MM);
                waitUntilMs = Math.max(waitUntilMs, now + ROTATE_RESET_MS);
            }
            stepPhase = StepPhase.ROTATE_RESET_WAIT;
        }

        if (stepPhase == StepPhase.ROTATE_RESET_WAIT && now >= waitUntilMs) {
            if (advanceOnComplete) {
                advancePlan();
            } else {
                stepPhase = StepPhase.START;
            }
            return true;
        }
        return false;
    }

    private void handleRapidTwoLeft(long now) {
        if (stepPhase == StepPhase.START && rapidStep == 0) {
            waitUntilMs = now;
        }

        if (rapidStep == 0) {
            if (handlePushOne(now, false)) {
                rapidStep = 1;
            }
            return;
        }

        if (rapidStep == 1) {
            if (handleRotateShot(true, now, false, true)) {
                rapidStep = 0;
                advancePlan();
            }
        }
    }

    private void handleRapidTwoFirst(long now) {
        if (stepPhase == StepPhase.START && rapidStep == 0) {
            waitUntilMs = now;
        }

        if (rapidStep == 0) {
            if (handlePushOne(now, false)) {
                rapidStep = 1;
            }
            return;
        }

        if (rapidStep == 1) {
            if (handleRotateShot(false, now, false, false)) {
                rapidStep = 0;
                advancePlan();
            }
        }
    }

    private void advancePlan() {
        planIndex++;
        if (planIndex >= plan.size()) {
            reset();
            return;
        }
        currentStep = plan.get(planIndex);
        stepPhase = StepPhase.START;
        rapidStep = 0;
    }

    private void setRampMode(RampMode target, long now) {
        if (rampMode != target) {
            rampMode = target;
            if (target == RampMode.BACK) {
                basePlate.rampBack();
                basePlate.setPusherMm(TENSION_BACK_MM);
            } else {
                basePlate.rampForward();
                basePlate.setPusherMm(tensionForwardMm());
            }
            rampCommandMs = now;
        }
        waitUntilMs = Math.max(waitUntilMs, rampCommandMs + RAMP_SETTLE_MS);
    }

    private void setShooterPos(ShooterPos target, long now) {
        if (shooterPos != target) {
            shooterPos = target;
            switch (target) {
                case MID:
                    gantry.moveGantryToPos("middle");
                    break;
                case FRONT:
                    gantry.moveGantryToPos("front");
                    break;
                case BACK:
                default:
                    gantry.moveGantryToPos("back");
                    break;
            }
            shooterCommandMs = now;
        }
        waitUntilMs = Math.max(waitUntilMs, shooterCommandMs + SHOOTER_SETTLE_MS);
    }

    private boolean gateRelease(boolean pushBased, long now) {
        long earliest = now;
        char nextColor = nextShotColor();
        if (lastShotColor != null && nextColor != lastShotColor) {
            earliest = Math.max(earliest, lastReleaseMs + COLOR_DELAY_MS);
        }
        if (pushBased) {
            earliest = Math.max(earliest, lastPushReleaseMs + PUSH_SPACING_MS);
        }
        if (now < earliest) {
            waitUntilMs = earliest;
            return false;
        }
        return true;
    }

    private void recordRelease(boolean pushBased, long now) {
        lastReleaseMs = now;
        if (pushBased) {
            lastPushReleaseMs = now;
        }
        lastShotColor = nextShotColor();
        shotsFired++;
        ballsRemaining = Math.max(0, ballsRemaining - 1);
        waitUntilMs = Math.max(waitUntilMs, now + SHOT_CLEAR_MS);
    }

    private char nextShotColor() {
        if (desiredShots == null || shotsFired >= desiredShots.length) {
            return ' ';
        }
        return desiredShots[shotsFired];
    }

    private double tensionForwardMm() {
        return TENSION_BACK_MM - RAMP_TRAVEL_MM + TENSION_TUNE_OFFSET_MM;
    }

    private static boolean isValidPattern(String pattern) {
        return pattern != null
                && pattern.length() == 3
                && (pattern.equals("GPP") || pattern.equals("PGP") || pattern.equals("PPG"));
    }

    private static Map<String, Map<String, MacroStep[]>> buildPlanTable() {
        Map<String, Map<String, MacroStep[]>> table = new HashMap<>();

        table.put("GPP", new HashMap<>());
        table.get("GPP").put("GPP", new MacroStep[]{MacroStep.PUSH_ONE, MacroStep.RAPID_TWO_LEFT});
        table.get("GPP").put("PGP", new MacroStep[]{MacroStep.POP_MID, MacroStep.PUSH_ONE, MacroStep.ROTATE_LAST});
        table.get("GPP").put("PPG", new MacroStep[]{MacroStep.POP_MID, MacroStep.POP_MID, MacroStep.ROTATE_LAST});

        table.put("PGP", new HashMap<>());
        table.get("PGP").put("GPP", new MacroStep[]{MacroStep.POP_MID, MacroStep.RAPID_TWO_LEFT});
        table.get("PGP").put("PGP", new MacroStep[]{MacroStep.PUSH_ONE, MacroStep.PUSH_ONE, MacroStep.ROTATE_LAST});
        table.get("PGP").put("PPG", new MacroStep[]{MacroStep.PUSH_ONE, MacroStep.POP_MID, MacroStep.ROTATE_LAST});

        table.put("PPG", new HashMap<>());
        table.get("PPG").put("GPP", new MacroStep[]{MacroStep.POP_FRONT, MacroStep.RAPID_TWO_LEFT});
        table.get("PPG").put("PGP", new MacroStep[]{MacroStep.PUSH_ONE, MacroStep.POP_MID, MacroStep.ROTATE_LAST});
        table.get("PPG").put("PPG", new MacroStep[]{MacroStep.RAPID_TWO_FIRST, MacroStep.ROTATE_LAST});

        return table;
    }
}
