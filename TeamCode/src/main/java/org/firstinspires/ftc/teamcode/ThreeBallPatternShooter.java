package org.firstinspires.ftc.teamcode;

/**
 * Three-ball pattern shooter with explicit micro-step sequences for all 9 cases.
 */
public class ThreeBallPatternShooter {
    private enum CaseId {
        GPP_TO_GPP,
        GPP_TO_PGP,
        GPP_TO_PPG,
        PGP_TO_GPP,
        PGP_TO_PGP,
        PGP_TO_PPG,
        PPG_TO_GPP,
        PPG_TO_PGP,
        PPG_TO_PPG
    }

    private final BasePlate basePlate;
    private final Gantry gantry;

    private CaseId activeCase;
    private int stepIndex;
    private boolean stepStarted;
    private long stepStartMs;

    private int ballsRemaining;
    private int shotsFired;
    private char[] desiredShots;

    private long lastReleaseMs;
    private long lastPushReleaseMs;
    private Character lastShotColor;

    private static final long COLOR_DELAY_MS = 500;
    private static final long PUSH_SPACING_MS = 200;

    private static final long RAMP_SETTLE_MS = 200;
    private static final long SHOOTER_SETTLE_MS = 200;
    private static final long POPPER_UP_MS = 120;
    private static final long POPPER_DOWN_MS = 120;
    private static final long SHOT_CLEAR_MS = 120;
    private static final long RETENSION_MS = 150;

    private static final double TENSION_BACK_MM = 0.0;
    private static final double RAMP_TRAVEL_MM = 70.0;
    private static final double TENSION_TUNE_OFFSET_MM = 0.0;

    public ThreeBallPatternShooter(BasePlate basePlate, Gantry gantry) {
        this.basePlate = basePlate;
        this.gantry = gantry;
    }

    public void startShootThree(String currentPattern, String desiredPattern) {
        activeCase = resolveCase(currentPattern, desiredPattern);
        if (activeCase == null) {
            reset();
            return;
        }
        stepIndex = 0;
        stepStarted = false;
        ballsRemaining = 3;
        shotsFired = 0;
        desiredShots = desiredPattern.toCharArray();
        lastReleaseMs = 0;
        lastPushReleaseMs = 0;
        lastShotColor = null;
    }

    public boolean isBusy() {
        return activeCase != null;
    }

    public void reset() {
        activeCase = null;
        stepIndex = 0;
        stepStarted = false;
        ballsRemaining = 0;
        shotsFired = 0;
        desiredShots = null;
        lastReleaseMs = 0;
        lastPushReleaseMs = 0;
        lastShotColor = null;
    }

    public void update() {
        if (!isBusy()) {
            return;
        }

        long now = System.currentTimeMillis();
        switch (activeCase) {
            case GPP_TO_GPP:
                runGppToGpp(now);
                break;
            case GPP_TO_PGP:
                runGppToPgp(now);
                break;
            case GPP_TO_PPG:
                runGppToPpg(now);
                break;
            case PGP_TO_GPP:
                runPgpToGpp(now);
                break;
            case PGP_TO_PGP:
                runPgpToPgp(now);
                break;
            case PGP_TO_PPG:
                runPgpToPpg(now);
                break;
            case PPG_TO_GPP:
                runPpgToGpp(now);
                break;
            case PPG_TO_PGP:
                runPpgToPgp(now);
                break;
            case PPG_TO_PPG:
                runPpgToPpg(now);
                break;
            default:
                reset();
                break;
        }
    }

    // ---------------- Case logic ----------------

    private void runGppToGpp(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 3:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.setPusherMm(push1);
                }, basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 8:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 9:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 12:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 13:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 14:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 15:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runGppToPgp(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampBackWithSafeTension();
                    gantry.moveGantryToPos("middle");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.middlePopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, basePlate::middlePopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 3:
                if (ballsRemaining >= 2) {
                    if (runTimedStep(now, () -> basePlate.setPusherMm(TENSION_BACK_MM), RETENSION_MS)) {
                        nextStep();
                    }
                } else {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 8:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 9:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 12:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 13:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runGppToPpg(long now) {
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampBackWithSafeTension();
                    gantry.moveGantryToPos("middle");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.middlePopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, basePlate::middlePopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 3:
                if (runTimedStep(now, () -> basePlate.setPusherMm(TENSION_BACK_MM), RETENSION_MS)) {
                    nextStep();
                }
                break;
            case 4:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.middlePopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 5:
                if (runTimedStep(now, basePlate::middlePopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 7:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 8:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 9:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runPgpToGpp(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampBackWithSafeTension();
                    gantry.moveGantryToPos("middle");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.middlePopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, basePlate::middlePopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 3:
                if (ballsRemaining >= 2) {
                    if (runTimedStep(now, () -> basePlate.setPusherMm(TENSION_BACK_MM), RETENSION_MS)) {
                        nextStep();
                    }
                } else {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 8:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 9:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 12:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 13:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runPgpToPgp(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 3:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.setPusherMm(push1);
                }, basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 8:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 9:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 12:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 13:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 14:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 15:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runPgpToPpg(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 3:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (runTimedStep(now, () -> {
                    setRampBackWithSafeTension();
                    gantry.moveGantryToPos("middle");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 8:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.middlePopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 9:
                if (runTimedStep(now, basePlate::middlePopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 12:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 13:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runPpgToGpp(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampBackWithSafeTension();
                    gantry.moveGantryToPos("front");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.frontPopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, basePlate::frontPopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 3:
                if (ballsRemaining >= 2) {
                    if (runTimedStep(now, () -> basePlate.setPusherMm(TENSION_BACK_MM), RETENSION_MS)) {
                        nextStep();
                    }
                } else {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 8:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 9:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 12:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 13:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runPpgToPgp(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 3:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(0.0);
                    basePlate.gateUp();
                }, basePlate.getDelayResetGateUpMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (runTimedStep(now, () -> {
                    setRampBackWithSafeTension();
                    gantry.moveGantryToPos("middle");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 8:
                if (!gateRelease(now, false)) break;
                if (runTimedStep(now, () -> {
                    basePlate.middlePopperUp();
                    recordRelease(false, now);
                }, POPPER_UP_MS)) {
                    nextStep();
                }
                break;
            case 9:
                if (runTimedStep(now, basePlate::middlePopperDown, POPPER_DOWN_MS)) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 11:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 12:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 13:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    private void runPpgToPpg(long now) {
        double push1 = basePlate.getShootPush1Mm();
        double push2 = basePlate.getShootPush2Mm();
        switch (stepIndex) {
            case 0:
                if (runTimedStep(now, () -> {
                    setRampForwardWithSafeTension();
                    gantry.moveGantryToPos("back");
                    basePlate.gateRotateBack();
                }, Math.max(RAMP_SETTLE_MS, SHOOTER_SETTLE_MS))) {
                    nextStep();
                }
                break;
            case 1:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> basePlate.setPusherMm(push1), basePlate.getDelayPush1Ms())) {
                    nextStep();
                }
                break;
            case 2:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2);
                    basePlate.gateRotateStage2();
                }, basePlate.getDelayPush1MidMs())) {
                    nextStep();
                }
                break;
            case 3:
                if (runTimedStep(now, basePlate::gateRotateStage3, basePlate.getDelayPush1EndMs())) {
                    nextStep();
                }
                break;
            case 4:
                if (runTimedStep(now, () -> {
                    basePlate.setPusherMm(push1 + push2 - 40.0);
                    basePlate.gateRotateBack();
                }, basePlate.getDelayGateRotateMs())) {
                    nextStep();
                }
                break;
            case 5:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 6:
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateBackMs())) {
                    nextStep();
                }
                break;
            case 7:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 8:
                if (runTimedStep(now, basePlate::gateRotateBack, basePlate.getDelayRotateResetMs())) {
                    nextStep();
                }
                break;
            case 9:
                if (!gateRelease(now, true)) break;
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateShoot();
                    recordRelease(true, now);
                }, SHOT_CLEAR_MS)) {
                    nextStep();
                }
                break;
            case 10:
                if (runTimedStep(now, () -> {
                    basePlate.gateRotateBack();
                    basePlate.gateUp();
                    basePlate.setPusherMm(0.0);
                }, basePlate.getDelayRotateResetMs())) {
                    finishCase();
                }
                break;
            default:
                reset();
                break;
        }
    }

    // ---------------- Helpers ----------------

    private boolean runTimedStep(long now, Runnable action, long durationMs) {
        if (!stepStarted) {
            action.run();
            stepStarted = true;
            stepStartMs = now;
        }
        if (now - stepStartMs >= durationMs) {
            stepStarted = false;
            return true;
        }
        return false;
    }

    private void nextStep() {
        stepIndex++;
        stepStarted = false;
    }

    private void finishCase() {
        reset();
    }

    private boolean gateRelease(long now, boolean pushBased) {
        long earliest = now;
        char nextColor = nextShotColor();
        if (lastShotColor != null && nextColor != lastShotColor) {
            earliest = Math.max(earliest, lastReleaseMs + COLOR_DELAY_MS);
        }
        if (pushBased) {
            earliest = Math.max(earliest, lastPushReleaseMs + PUSH_SPACING_MS);
        }
        return now >= earliest;
    }

    private void recordRelease(boolean pushBased, long now) {
        lastReleaseMs = now;
        if (pushBased) {
            lastPushReleaseMs = now;
        }
        lastShotColor = nextShotColor();
        shotsFired++;
        ballsRemaining = Math.max(0, ballsRemaining - 1);
    }

    private char nextShotColor() {
        if (desiredShots == null || shotsFired >= desiredShots.length) {
            return ' ';
        }
        return desiredShots[shotsFired];
    }

    private void setRampBackWithSafeTension() {
        basePlate.rampBack();
        basePlate.setPusherMm(TENSION_BACK_MM);
    }

    private void setRampForwardWithSafeTension() {
        basePlate.rampForward();
        basePlate.setPusherMm(tensionForwardMm());
    }

    private double tensionForwardMm() {
        return TENSION_BACK_MM - RAMP_TRAVEL_MM + TENSION_TUNE_OFFSET_MM;
    }

    private static CaseId resolveCase(String currentPattern, String desiredPattern) {
        if (currentPattern == null || desiredPattern == null
                || currentPattern.length() != 3 || desiredPattern.length() != 3) {
            return null;
        }
        String key = currentPattern.toUpperCase() + ":" + desiredPattern.toUpperCase();
        switch (key) {
            case "GPP:GPP":
                return CaseId.GPP_TO_GPP;
            case "GPP:PGP":
                return CaseId.GPP_TO_PGP;
            case "GPP:PPG":
                return CaseId.GPP_TO_PPG;
            case "PGP:GPP":
                return CaseId.PGP_TO_GPP;
            case "PGP:PGP":
                return CaseId.PGP_TO_PGP;
            case "PGP:PPG":
                return CaseId.PGP_TO_PPG;
            case "PPG:GPP":
                return CaseId.PPG_TO_GPP;
            case "PPG:PGP":
                return CaseId.PPG_TO_PGP;
            case "PPG:PPG":
                return CaseId.PPG_TO_PPG;
            default:
                return null;
        }
    }
}
