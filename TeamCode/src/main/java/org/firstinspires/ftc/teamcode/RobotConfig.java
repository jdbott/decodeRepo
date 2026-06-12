package org.firstinspires.ftc.teamcode;

/**
 * Central registry of hardware device names. Every string here must match the
 * device name in the Control Hub robot configuration exactly.
 *
 * Reference these constants instead of inlining string literals so a hardware
 * rename is a one-file change and the compiler catches typos.
 */
public final class RobotConfig {

    private RobotConfig() {}

    // Shooter flywheels (velocity-controlled, rad/s)
    public static final String FLYWHEEL_TOP = "shootTop";
    public static final String FLYWHEEL_BOTTOM = "shootBottom";

    // Turret aim motor
    public static final String TURRET_MOTOR = "turretMotor";

    // Intake
    public static final String INTAKE_MOTOR = "intake_motor";

    // Hood (launch-angle servo)
    public static final String HOOD_SERVO = "hoodServo";

    // Feeder (arm + clutch servos)
    public static final String FEEDER_ARM_SERVO = "armServo";
    public static final String FEEDER_CLUTCH_SERVO = "clutchServo";

    // Drivetrain
    public static final String DRIVE_LEFT_FRONT = "leftFront";
    public static final String DRIVE_LEFT_BACK = "leftBack";
    public static final String DRIVE_RIGHT_FRONT = "rightFront";
    public static final String DRIVE_RIGHT_BACK = "rightBack";

    // Localizer
    public static final String PINPOINT = "pinpoint";
}
