package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;
@Disabled

@TeleOp(name = "Flywheel Feedforward Tuner", group = "Flywheel Tuning")
public class FeedForwardTuner extends LinearOpMode {

    private static final double[] POWER_STEPS = {
            0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4,
            0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8,
            0.85, 0.9, 0.95, 1.0
    };
    private static final double WAIT_TIME = 2.0; // seconds

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx flywheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Feedforward Tuner Ready");
        telemetry.addLine("Ensure flywheel can spin safely");
        telemetry.update();

        waitForStart();

        double tunedVoltage = battery.getVoltage();
        List<Double> powers = new ArrayList<>();
        List<Double> velocitiesRad = new ArrayList<>();
        List<Double> velocitiesRPM = new ArrayList<>();

        // Sweep through power stepse
        for (double power : POWER_STEPS) {
            flywheel.setPower(-power);
            flywheel2.setPower(power);
            sleep((long)(WAIT_TIME * 1000));

            double velTicks = -flywheel2.getVelocity();
            double velRad = -velTicks * 2 * Math.PI / 28.0;
            double velRPM = -velRad * 60 / (2 * Math.PI);

            powers.add(power);
            velocitiesRad.add(velRad);
            velocitiesRPM.add(velRPM);

            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Velocity (rad/s)", "%.2f", velRad);
            telemetry.addData("Velocity (RPM)", "%.2f", velRPM);
            telemetry.addData("Battery", "%.2f", battery.getVoltage());
            telemetry.update();
        }

        flywheel.setPower(0);
        flywheel2.setPower(0);
        // Linear regression: power = kV*ω + kS
        double meanVel = velocitiesRad.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        double meanPower = powers.stream().mapToDouble(Double::doubleValue).average().orElse(0);

        double num = 0, den = 0;
        for (int i = 0; i < powers.size(); i++) {
            num += (velocitiesRad.get(i) - meanVel) * (powers.get(i) - meanPower);
            den += Math.pow(velocitiesRad.get(i) - meanVel, 2);
        }
        double kV = num / den;
        double kS = meanPower - kV * meanVel;

        // Clear telemetry and show regression results
        telemetry.clearAll();
        telemetry.addLine("Feedforward Regression Complete");
        telemetry.addData("kV", kV);
        telemetry.addData("kS", kS);
        telemetry.addData("V_tuned", tunedVoltage);
        telemetry.addLine("Use in Flywheel class: feedforwardPower = (kV * ω + kS) * V_tuned / V_battery");
        telemetry.addLine("Velocity for each power step:");

        // Display each power step with velocity
        for (int i = 0; i < powers.size(); i++) {
            telemetry.addData("P=", powers.get(i));
            telemetry.addData("Rad/s=", velocitiesRad.get(i));
        }
        telemetry.update();

        // Keep opmode running so you can copy down values
        while (opModeIsActive()) idle();
    }
}