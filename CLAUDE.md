# CLAUDE.md

Guidance for Claude Code when working in this repository. This is an **FTC (FIRST Tech Challenge)** robot codebase for the **DECODE** season, built on **FTC SDK 11.1.0** with **Pedro Pathing** for autonomous path following.

## Where the code lives

- **All real code is in `TeamCode/`** — specifically `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- **Do not touch `FtcRobotController/`** unless explicitly asked. It is the stock FTC SDK app module — not our code.
- `build.dependencies.gradle` holds dependencies (FTC SDK, `com.pedropathing:ftc`, bylazar panels). `pedroPathing/` holds Pedro's `Constants` and `Tuning` directly (no subpackages).

## Package layout (`.../teamcode/`)

| Package | Contents |
|---|---|
| (root) | Alliance/start selectors & stores (`AllianceStore`, `AllianceMirror`, `AutoStartStore`, `AllianceSelectorTeleOp`, `CloseFarSelectorTeleOp`); `RobotConfig` (central device-name constants) |
| `hardwareClasses/` | Subsystem/hardware wrapper classes (see glossary) |
| `autos/` | Autonomous opmodes (`V3Auto`, `V3FarAuto`, `V3ClosePartner`) |
| `teles/` | Primary teleop (`V3Tele`) |
| `tuners/` | Gain/feedforward tuning opmodes (`FeedForwardTuner`, `FlywheelKpTuner`, `ManualControl`) |
| `modernTests/` | Bring-up/test opmodes (`FlywheelTest`) |
| `pedroPathing/` | Pedro Pathing `Constants` and `Tuning` |

The codebase was swept clean of prior-season and superseded files (old V2 teleops, legacy autos, OpenCV/older vision pipelines, duplicate subsystem classes, dead tuners/tests). A full snapshot of everything removed is preserved at git tag `season-2025-decode-archive` if old code is ever needed for reference. Nothing in the current tree is off-limits "scratch" — any file may be live. If you're unsure whether something is current, ask rather than assuming.

## Conventions

**OpModes**
- Extend `LinearOpMode` (current norm) — iterative `OpMode` is acceptable if it fits better.
- Name opmodes **descriptively** in the `@TeleOp(name=...)` / `@Autonomous(name=...)` string. Don't worry about Driver Station ordering prefixes (the `"A "` prefixes in existing names are just a manual sort hack) — Jason manages ordering.

**Hardware classes (`hardwareClasses/`)**
- For **new** hardware classes, use **constructor injection**: take `HardwareMap` (and any deps like `VoltageSensor`) in the constructor so the object is ready to use after construction — every hardware class (`Flywheel`, `Turret`, `Hood`, `Feeder`, `Intake`) now follows this pattern.
- **Device names live in `RobotConfig`** (root `teamcode/` package) as `public static final String` constants — reference these (e.g. `RobotConfig.FLYWHEEL_TOP`, `RobotConfig.INTAKE_MOTOR`, `RobotConfig.HOOD_SERVO`) instead of inlining string literals. The string **values** must match the Control Hub robot configuration exactly, so don't change a value without re-configuring the hub. **Never invent a new device name**; if you need one that doesn't exist yet, ask.
- **Naming new hardware classes**: use plain, simple subsystem names (`Hood`, `Feeder`, `Intake`) — don't append suffixes like `ASG` unless asked. If the name would collide with an existing class, ask Jason how he wants to disambiguate rather than picking a suffix yourself.

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

- **`Flywheel`** — shooter flywheel(s); velocity-controlled (rad/s, kP + kV/kS feedforward, bang-bang spin-up). `flywheelTop`/`flywheelBottom` motors (config names `shootTop`/`shootBottom`).
- **`Turret`** — rotates the shooter to aim at the goal (degrees, ±180).
- **`Hood`** — adjustable hood angle (launch angle) via servo; maps hood angle → servo position (`hoodServo`).
- **`Intake`** — intake motor that pulls in artifacts (`intake_motor`).
- **`Feeder`** — arm/clutch mechanism (`armServo`/`clutchServo`) that feeds artifacts toward the shooter.

## Build & git

- **Don't run gradle builds by default.** Jason builds and deploys to the robot through Android Studio. Only run `./gradlew ...` to verify compilation if explicitly asked.
- **Never run `git add`/`commit`/`push` unless explicitly asked.** Edit files and leave committing to Jason.
