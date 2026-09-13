package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;

/**
 * Pedro Pathing 3 robot configuration. Paste AutoTune output (http://192.168.43.1:10158, see
 * {@link Tuning}) over the matching block below.
 *
 * <p>Drivetrain and Pinpoint values carry over from the Pedro 2 constants. The Foresight block is
 * <b>not yet tuned</b>: the velocity/deceleration/braking numbers are Pedro 2 measurements of the same
 * physical quantities, and the controllers are the Pedro Quickstart's example values. Run the
 * Foresight AutoTune before relying on autos.
 */
public final class Constants {

    private Constants() {}

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConfig.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.0);   // Pedro 2 forwardPodY
        c.yPodOffset.set(-5.46);  // Pedro 2 strafePodX
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Pedro 2 measurements (xVelocity / yVelocity, zero-power acceleration, predictive braking)
        c.maxAchievableForwardVelocity.set(73.8);   // in/s
        c.maxAchievableStrafeVelocity.set(60.8);    // in/s
        c.naturalForwardDeceleration.set(33.7);     // in/s^2
        c.naturalStrafeDeceleration.set(60.9);      // in/s^2
        // Stopping distance = linear * v + quadratic * v|v|. Pedro 2 only measured forward; strafe assumed equal.
        c.linearBrakeCoefficients.set(Matrix.diag(0.146551085, 0.146551085));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.000622545779, 0.000622545779));

        // PLACEHOLDERS (Pedro Quickstart example robot) until Foresight AutoTune is run
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        c.forwardTranslational.set(
                Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.3)));
        c.strafeTranslational.set(
                Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.3)));
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), foresight());
    }

    public static PinpointLocalizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Mecanum drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Foresight foresight() {
        return new Foresight(foresightConfig);
    }
}
