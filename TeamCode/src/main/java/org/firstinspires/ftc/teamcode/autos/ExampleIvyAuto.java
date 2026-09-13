package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.hardwareClasses.Prism;

/**
 * Ivy command-based demo for the offseason robot: intake and Prism LED actions composed with
 * sequential / parallel / repeat / race groups. No drivetrain — to mix in driving, add
 * {@code PedroCommands.follow(follower, path)} steps to the sequence (see templates/TemplateAuto).
 */
@Autonomous(name = "Example: Ivy Intake + LEDs (Offseason)", group = "Examples")
public class ExampleIvyAuto extends LinearOpMode {

    private static final float LED_SPEED = 0.5f; // 0 (slow) to 1 (fast)

    private DcMotor intake;
    private Prism leds;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, RobotConfig.OFFSEASON_INTAKE_MOTOR);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leds = new Prism(hardwareMap);

        Scheduler.reset();
        Command routine = routine();

        telemetry.addLine("Ready: intake + LED demo");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        Scheduler.schedule(routine);
        while (opModeIsActive() && routine.isScheduled()) {
            Scheduler.execute();
            telemetry.addData("Intake power", intake.getPower());
            telemetry.update();
        }

        Scheduler.reset();
        intake.setPower(0);
        leds.turnOff();
    }

    private Command routine() {
        return sequential(
                leds(Prism.Pattern.PULSE, Color.BLUE),
                waitMs(1000),
                // Intake for 2 s while the LEDs chase green
                parallel(leds(Prism.Pattern.CHASE, Color.GREEN), runIntake(1.0, 2000)),
                // Unjam: reverse then forward pulses, three times
                repeat(sequential(runIntake(-1.0, 250), runIntake(1.0, 250)), 3),
                // Blink orange until the driver presses A or 3 s pass, whichever comes first
                leds(Prism.Pattern.BLINK, Color.ORANGE),
                race(waitUntil(() -> gamepad1.a), waitMs(3000)),
                leds(Prism.Pattern.RAINBOW, Color.WHITE),
                waitMs(1500),
                instant(leds::turnOff)
        );
    }

    /** Runs the intake at {@code power} for {@code ms}, then stops it. */
    private Command runIntake(double power, long ms) {
        return sequential(
                instant(() -> intake.setPower(power)),
                waitMs(ms),
                instant(() -> intake.setPower(0))
        ).requiring(intake);
    }

    private Command leds(Prism.Pattern pattern, Color color) {
        return instant(() -> leds.start(pattern, color, LED_SPEED));
    }
}
