# AGENTS.md

Guidance for Codex when working in this repository. This is an **FTC (FIRST Tech Challenge)** robot codebase, built on **FTC SDK 11.2.1** with **Pedro Pathing 3** for path following, **Ivy** for command-based autos, and **Sloth** (hot reload) + **Slothboard** (FTC Dashboard) for fast deploys and live tuning. The DECODE (2025–26) competition code is archived; the DECODE main teleop (`V3Tele`) and hardware classes are still live.

## Where the code lives

- **All real code is in `TeamCode/`** — specifically `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- **Do not touch `FtcRobotController/`** unless explicitly asked. It is the stock FTC SDK app module — not our code.
- `build.dependencies.gradle` holds shared dependencies (FTC SDK, `com.pedropathing:revhub`, `com.pedropathing:tuning`, Ivy). `TeamCode/build.gradle` holds Sloth, the Sloth `Load` plugin and Slothboard.
- `archive/` holds retired code. It is **not compiled**; see `archive/README.md`.
- `autosim/` is a standalone plain-Java tool that renders DECODE autos to HTML. It does not affect the robot build.

## Package layout (`.../teamcode/`)

| Package | Contents |
|---|---|
| (root) | `RobotConfig` (central device-name constants) |
| `alliance/` | Alliance/start-side selection + mirroring: `AllianceStore`, `AllianceSelectorTeleOp`, `AutoStartStore`, `CloseFarSelectorTeleOp`, `AllianceMirror` |
| `pedro/` | Pedro 3 `Constants` (robot config + `create()`), `Tuning` (AutoTune procedures), `PathModifiers`, `StuckDetector`; `procedures/` = AutoTune tuners copied unchanged from the Pedro Quickstart |
| `hardwareClasses/` | Subsystem/hardware wrapper classes (see glossary) |
| `autos/` | `ExamplePathAuto` (Pedro 3 path-following reference), `ExampleIvyAuto` (Ivy actions on the offseason robot), plus offseason-robot test autos |
| `teles/` | Main teleop `V3Tele`; `biobuzzTele` (standalone tank-style field-centric drive) |
| `templates/` | `@Disabled` starting points for new autos/teleops (`TemplateAuto`, `TemplateTeleOp`) |
| `tuners/` | Flywheel gain tuners, `ManualControl` (Dashboard hardware poking) |
| `modernTests/` | Bring-up/test opmodes |
| `Prism/` | goBILDA Prism LED driver, animations, and examples (vendor code + our LED opmodes) |

**Offseason-robot files — keep as-is; don't modify, move, or archive them without asking:** `modernTests/IntakeTest`, `IntakeHopperTest`, `LiftTest`; `autos/offIntake`, `offOuttake`, `linearSlides`, `offLimelightLocalization`, `simple_offseason_dt_test`; `hardwareClasses/off*`. They use inline device names by design.

If you're unsure whether something is current, ask rather than assuming.

## Conventions

**OpModes**
- Extend `LinearOpMode` (current norm) — iterative `OpMode` is acceptable if it fits better.
- Name opmodes **descriptively** in the `@TeleOp(name=...)` / `@Autonomous(name=...)` string. Don't worry about Driver Station ordering prefixes (the `"A "` prefixes in existing names are just a manual sort hack) — Jason manages ordering.
- New autos/teleops: start from `templates/`.

**Hardware classes (`hardwareClasses/`)**
- For **new** hardware classes, use **constructor injection**: take `HardwareMap` (and any deps like `VoltageSensor`) in the constructor so the object is ready to use after construction — every hardware class (`Flywheel`, `Turret`, `Hood`, `Feeder`, `Intake`, `Prism`) follows this pattern.
- **Device names live in `RobotConfig`** (root `teamcode/` package) as `public static final String` constants — reference these instead of inlining string literals. The string **values** must match the Control Hub robot configuration exactly, so don't change a value without re-configuring the hub. **Never invent a new device name**; if you need one that doesn't exist yet, ask.
- **Naming new hardware classes**: use plain, simple subsystem names (`Hood`, `Feeder`, `Intake`) — don't append suffixes like `ASG` unless asked. If the name would collide with an existing class, ask Jason how he wants to disambiguate rather than picking a suffix yourself.
- **Live-tunable gains**: annotate the class `@Config` (`com.acmerobotics.dashboard.config.Config`) and make the gains `public static` (see `Flywheel`, `Turret`). They show up in Dashboard's Configuration tab.

**Units & control gains (enforce these)**
- Flywheel/angular velocity: **rad/s**
- Angles (turret, hood, headings): **degrees** in our code. Pedro `Pose.heading()` is **radians**, so convert at the boundary (`PoseFactory.degrees()` does this for pose creation).
- Field distances/coordinates: **inches** (Pedro frame: origin at the bottom-left corner, +x right, +y up, 0 rad = facing +x, CCW positive)
- Control gains: **`kP`, `kV`, `kS`, `kD`, `kF`** naming. Label units in a comment when a variable's unit isn't obvious.

**Pedro Pathing 3 (autos & driving)**
- `Follower follower = Constants.create(hardwareMap)`; `follower.setPose(start)`; call `follower.update()` every loop.
- Poses: author them **blue-native with degree headings** through `AllianceMirror.poses(isRed)`, which returns a `PoseFactory` that mirrors onto red. `AllianceMirror.MODE` selects our reflection (`CUSTOM`, default: x → 144−x, heading → 180°−h) or Pedro's `mirrorX` (`BUILT_IN`, heading → −h). The toggle is editable from Dashboard. Pass Poses (not raw radians) to heading interpolators so headings mirror too.
- Paths: `line(a, b)`, `curve(a, ctrl…, b)`, `through(a, …, b)`, `path(p1, p2, …)` (static imports from `com.pedropathing.api.Paths`). Heading: `.linear(a, b)`, `.tangent()` (default), `.reverseTangent()`, `.constant(pose)`, `.facingPoint(pose)`, `.heading(Interpolator.piecewise()…)`. Prefer methods that return paths over stored fields.
- Following: `follower.follow(path)`. `!follower.isBusy()` = settled at the end; `follower.atParametricEnd()` = reached the end (use it for pass-through points; it's faster).
- Per-path speed/braking: `path.with(PathModifiers.maxSpeed(0.4))`, `softBraking(0.5)`, `brakeAggression(x)`, `coastThrough()`. These replace Pedro 2's `setMaxPower` / `setBrakingStrength`. To slow down partway, split the route into two paths.
- Stuck recovery: `StuckDetector` replaces Pedro 2's `isRobotStuck()` / `breakFollowing()`.
- Teleop driving: `follower.manual(forward, strafe, turn)` (+strafe = left, +turn = CCW) or `follower.manual(ManualDrive.fieldCentric(f, s, t, heading))`.
- Velocity: `follower.velocity()` → field-frame `vx`, `vy` (in/s), `omega` (rad/s).
- Tuning: AutoTune web page at `http://192.168.43.1:10158` (procedures registered in `pedro/Tuning`). Paste generated configs into `pedro/Constants`. **The Foresight block in `Constants` is still placeholder/Pedro-2-derived until AutoTune is run.**
- Ivy: `Scheduler.reset()` in init, `Scheduler.schedule(cmd)` at start, `Scheduler.execute()` every loop. `PedroCommands.follow(follower, path)` ends at the parametric end.

**Pedro 2 → 3 call mapping** (for porting archived code)

| Pedro 2 | Pedro 3 |
|---|---|
| `com.pedropathing.geometry.Pose`, `getX/getY/getHeading` | `com.pedropathing.math.Pose`, `x()/y()/heading()` |
| `Constants.createFollower(hw)` | `Constants.create(hw)` |
| `setStartingPose` + `updatePose` | `setPose` |
| `followPath(p, holdEnd)` | `follow(p)` (hold at end is the default) |
| `getCurrentTValue()` | `parametricCompletion()` / `completion()` |
| `new Path(new BezierLine(a, b))` + `setLinearHeadingInterpolation` | `line(a, b).linear(a, b)` |
| `reverseHeadingInterpolation()` | `.reverseTangent()` |
| `pathBuilder().addPath(..).build()` | `path(p1, p2, ..)` |
| `startTeleOpDrive` / `setTeleOpDrive(f, s, t, false)` | `manual(ManualDrive.fieldCentric(f, s, t, heading))` |
| `getVelocity().getXComponent()`, `getAngularVelocity()` | `velocity().vx`, `velocity().omega` |
| `setMaxPower`, `setBrakingStrength` | `PathModifiers.maxSpeed`, `PathModifiers.softBraking` |
| `isRobotStuck`, `breakFollowing` | `StuckDetector` |

**Code style**
- Prefer **clean, concise code with fewer inline comments** than some existing files (which are heavily annotated). Comment the non-obvious (a tunable's unit, why a threshold exists), not the obvious.

## Subsystem glossary

> DECODE robot: an intake feeds a flywheel shooter aimed by a turret + adjustable hood. This list will go stale as new mechanisms/classes are added — update it when that happens.

- **`Flywheel`** — shooter flywheel(s); velocity-controlled (rad/s, kP + kV/kS feedforward, bang-bang spin-up). `flywheelTop`/`flywheelBottom` motors (config names `shootTop`/`shootBottom`). Gains are Dashboard-tunable.
- **`Turret`** — rotates the shooter to aim at the goal (degrees, ±180). Gains are Dashboard-tunable.
- **`Hood`** — adjustable hood angle (launch angle) via servo; maps hood angle → servo position (`hoodServo`).
- **`Intake`** — intake motor that pulls in artifacts (`intake_motor`).
- **`Feeder`** — arm/clutch mechanism (`armServo`/`clutchServo`) that feeds artifacts toward the shooter.
- **`Prism`** — goBILDA Prism RGB LED driver wrapper (`prism`); patterns CHASE / RAINBOW / PULSE / BLINK.
- Offseason robot: `offSeasonIntake`, `offSeasonDeposit`, `offLinearSlides`, `offMasterRobot` (placeholder device names); the tests use `intake` (`RobotConfig.OFFSEASON_INTAKE_MOTOR`), `hoper`, `lift1`.

## Dashboard & deploying

- FTC Dashboard (Slothboard fork): `http://192.168.43.1:8080/dash` while on the robot's Wi-Fi. Opmodes send telemetry there with `telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())`.
- Panels is removed for now: its Sloth build only exists for Sloth 0.2.4, and Pedro AutoTune requires Sloth 0.3.0. Re-add `com.bylazar.sloth:fullpanels:0.3.0+<version>` in `TeamCode/build.gradle` once it's published.
- Sloth, the `Load` plugin and Slothboard versions must move together.
- Load 0.3.0 is compiled for Java 25, so the build needs **Gradle 9.1+** (wrapper) running on **JDK 25** (`gradle/gradle-daemon-jvm.properties`). In Android Studio set Settings → Build Tools → Gradle → Gradle JDK to a JDK 25 (Temurin 25 is at `~/Library/Java/JavaVirtualMachines/jdk-25.0.4.1+1`); the bundled JBR 21 won't sync.
- `deploySloth` hot-reloads **only** TeamCode. After any gradle/library change, do one normal full install (Run 'TeamCode') first.

## Starting a new season

1. Archive last season's competition autos/teleops under `archive/<season>/` (same relative paths) and list them in `archive/README.md`.
2. Update `RobotConfig` for the new robot's device names.
3. Run AutoTune (Mecanum → Pinpoint/localizer → Foresight → Tests) and paste the results into `pedro/Constants`.
4. Copy `templates/TemplateAuto` / `TemplateTeleOp` into `autos/` / `teles/` and build from there. Use `ExamplePathAuto` and `ExampleIvyAuto` as references.
5. Update the glossary above.

## Build & git

- **Don't run gradle builds by default.** Jason builds and deploys to the robot through Android Studio. Only run `./gradlew ...` to verify compilation if explicitly asked.
- **Never run `git add`/`commit`/`push` unless explicitly asked.** Edit files and leave committing to Jason.
- **Prefer working directly on `master`** for simple, low-risk changes — new opmodes, behavior tweaks, button-mapping changes, tuning constants, etc. Push directly to `master` rather than opening a PR for these.
- **Only create a feature branch/PR** for major refactors (e.g. renaming/restructuring shared classes across many files) or when Jason explicitly asks for one.
- **If unsure which to do**, ask before pushing — don't default to a branch+PR "just in case."
