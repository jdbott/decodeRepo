package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro AutoTune procedures. Deploy, join the robot's Wi-Fi and open http://192.168.43.1:10158.
 * Tune in order — Mecanum, Pinpoint, Foresight — pasting each generated config into {@link Constants},
 * then verify with Tests. The procedure classes are copied unchanged from the Pedro Quickstart.
 */
public final class Tuning {

    private Tuning() {}

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::foresight);
    }
}
