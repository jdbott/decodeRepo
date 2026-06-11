# CLAUDE.md

Guidance for Claude Code when working in this repository. This is an **FTC (FIRST Tech Challenge)** robot codebase for the **DECODE** season, built on **FTC SDK 11.1.0** with **Pedro Pathing** for autonomous path following.

## Where the code lives

- **All real code is in `TeamCode/`** — specifically `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- **Do not touch `FtcRobotController/`** unless explicitly asked. It is the stock FTC SDK app module — not our code.
- `build.dependencies.gradle` holds dependencies (FTC SDK, `com.pedropathing:ftc`, bylazar panels). `pedroPathing/constants/` holds Pedro's `Constants`/`Tuning`.

## Package layout (`.../teamcode/`)

| Package | Contents |
|---|---|
| (root) | Primary teleops (`V3Tele`), alliance/start selectors & stores (`AllianceStore`, `AllianceMirror`, `AutoStartStore`), shared calc (`ShootingCalc`, `ShootOnMove`) |
| `hardwareClasses/` | Subsystem/hardware wrapper classes (see glossary) |
| `autos/` | Autonomous opmodes (`V3Auto`, States autos); `autos/legacy/` older ones |
| `teles/` | Additional teleops / scrimmage variants |
| `tuners/` | Gain/feedforward tuning opmodes |
| `modernTests/`, `randomTests/` | Test/bring-up opmodes |
| `vision/limelight/`, `vision/opencv/` | Vision (Limelight, OpenCV ball detection) |

Nothing here is treated as off-limits "scratch" — any file may be live. If you're unsure whether something is current, ask rather than assuming.

## Conventions

**OpModes**
- Extend `LinearOpMode` (current norm) — iterative `OpMode` is acceptable if it fits better.
- Name opmodes **descriptively** in the `@TeleOp(name=...)` / `@Autonomous(name=...)` string. Don't worry about Driver Station ordering prefixes (the `"A "` prefixes in existing names are just a manual sort hack) — Jason manages ordering.

**Hardware classes (`hardwareClasses/`)**
- For **new** hardware classes, use **constructor injection**: take `HardwareMap` (and any deps like `VoltageSensor`) in the constructor so the object is ready to use after construction — follow `FlywheelASG`, not `Turret`'s separate `init()` method.
- **Device names are string literals** (e.g. `"shootTop"`, `"intakeMotor"`, `"hoodServo"`) passed to `hardwareMap.get(...)`. There is **no central config file** — reuse the exact names already used in existing opmodes (grep `V3Tele`/`V3Auto`). **Never invent a new device name**; if you need one that doesn't exist yet, ask.

**Units & control gains (enforce these)**
- Flywheel/angular velocity: **rad/s**
- Angles (turret, hood, headings): **degrees**
- Field distances/coordinates: **inches**
- Control gains: **`kP`, `kV`, `kS`, `kD`, `kF`** naming. Label units in a comment when a variable's unit isn't obvious.

**Autonomous / alliance**
- Pedro Pathing (`Follower`, `Path`, `BezierLine`/`BezierCurve`, `Pose`) is the standard for movement.
- Autos store poses **blue-native** and use `AllianceMirror` + `AllianceStore` to flip for red. Understand this system when editing autos, but it's not a hard rule — separate red/blue autos are fine too. Use your judgment per-auto.

**Code style**
- Prefer **clean, concise code with fewer inline comments** than some existing files (which are heavily annotated). Comment the non-obvious (a tunable's unit, why a threshold exists), not the obvious.

## Subsystem glossary

> The robot shoots game "artifacts" into a goal: an intake feeds a flywheel shooter aimed by a turret + adjustable hood. This list will go stale as new mechanisms/classes are added — update it when that happens.

- **`FlywheelASG` / `Shooter` / `ShooterV2`** — shooter flywheel(s). `FlywheelASG` is the current velocity-controlled flywheel (rad/s, kP + kV/kS feedforward, bang-bang spin-up). `shootTop`/`shootBottom` motors.
- **`Turret`** — rotates the shooter to aim at the goal (degrees, ±180).
- **`HoodKinematics`** — adjustable hood angle (launch angle) via servo; maps hood angle → servo position.
- **`Intake`** — intake motor that pulls in artifacts.
- **`BasePlate` / `BasePlateFast`** — popper servos (`popperFront`/`popperMiddle`) that feed artifacts up into the shooter.
- **`Gantry`** — preset-position mechanism (`gantry1`/`gantry2`).
- **`Pivot`** — angled-position mechanism with position correction.
- **`LinearSlide`** — extending slides (zeroing, max-extension in inches).
- **`Diffy`** — differential (two-servo) mechanism mixing pitch/roll.
- **`Linkage`** — linkage mechanism.
- **`ServoController`** — closed-loop (kP/kD) controller for a servo with zeroing.
- **`ColorV3`** — REV V3 color/proximity sensor (artifact detection/sorting).

## Build & git

- **Don't run gradle builds by default.** Jason builds and deploys to the robot through Android Studio. Only run `./gradlew ...` to verify compilation if explicitly asked.
- **Never run `git add`/`commit`/`push` unless explicitly asked.** Edit files and leave committing to Jason.
