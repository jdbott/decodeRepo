package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TEST InterShot Spacing (Selectable Order)", group="TEST")
public class TestSorting extends LinearOpMode {

    private BasePlate basePlate;

    // Configure these 3 chars before running:
    // 'P' = purple, 'G' = green
    // Examples:
    //   new char[]{'P','G','P'}
    //   new char[]{'P','P','G'}
    //   new char[]{'G','P','P'}
    private final char[] order = new char[]{'P','G','P'};

    // Delay applied ONLY when a color changes between consecutive shots
    private static final double COLOR_CHANGE_DELAY_S = 0.5;

    private enum State {
        INIT,
        PREP,
        START_SHOOT,
        RUN,
        DONE
    }

    private State state = State.INIT;

    @Override
    public void runOpMode() {
        basePlate = new BasePlate(hardwareMap);

        telemetry.addLine("TEST: Inter-shot spacing based on colors");
        telemetry.addData("order", "" + order[0] + order[1] + order[2]);
        telemetry.addData("colorChangeDelayS", COLOR_CHANGE_DELAY_S);
        telemetry.addLine("Edit 'order' in code to test different sequences.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            basePlate.update();

            switch (state) {
                case INIT: {
                    // Configure the extra delays based on the color order.
                    basePlate.setInterShotDelaysFromColors(order[0], order[1], order[2], COLOR_CHANGE_DELAY_S);
                    state = State.PREP;
                    break;
                }

                case PREP: {
                    basePlate.prepShootOnly();
                    state = State.START_SHOOT;
                    break;
                }

                case START_SHOOT: {
                    // Starts shot 1 immediately; spacing enforcement happens automatically later
                    basePlate.startShootFromPrep();
                    state = State.RUN;
                    break;
                }

                case RUN: {
                    if (!basePlate.isShootBusy()) {
                        state = State.DONE;
                    }
                    break;
                }

                case DONE:
                default:
                    // idle
                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("order", "" + order[0] + order[1] + order[2]);
            telemetry.addData("busy", basePlate.isShootBusy());
            telemetry.addData("shootState", String.valueOf(basePlate.getShootState()));
            telemetry.addData("holdingForSpacing", basePlate.isHoldingForInterShotSpacing());
            telemetry.addData("pusherMmCmd", basePlate.getLastPusherMmCmd());
            telemetry.update();
        }
    }
}