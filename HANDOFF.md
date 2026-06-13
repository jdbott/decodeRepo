# Codebase Improvement Handoff

This document captures the findings of a full architectural review of `TeamCode/` performed prior to a major cleanup pass, plus a follow-up session that added a new auto and centralized hardware config. The original cleanup (deleting dead V2/legacy/test files, fixing the `Intake`/`IntakeASG` device-name conflict, restructuring `pedroPathing/` into a flat package, updating `CLAUDE.md`) is **done**. A follow-up pass (this session) centralized device names into `RobotConfig`, converted all hardware classes to constructor injection, renamed `FlywheelASG` → `Flywheel`, and integrated a new auto (`V3FarAutoByYuvi`) contributed via PR. This file covers the **remaining** open items, ranked by priority, so a new session can pick up directly.

A full snapshot of the pre-cleanup codebase is preserved at git tag `season-2025-decode-archive` if any deleted file is ever needed for reference.

## What was accomplished this session

- **Centralized device names** into a new `RobotConfig.java` (root `teamcode/` package) — `public static final String` constants for every motor/servo name (`FLYWHEEL_TOP`, `FLYWHEEL_BOTTOM`, `TURRET_MOTOR`, `INTAKE_MOTOR`, `HOOD_SERVO`, `FEEDER_ARM_SERVO`, `FEEDER_CLUTCH_SERVO`, drivetrain motors, `PINPOINT`, etc.). This was open item #3 from the previous handoff — **it is now done**.
- **Renamed `FlywheelASG.java` → `Flywheel.java`** (and `modernTests/FlywheelASGTest.java` → `FlywheelTest.java`), and converted **all hardware classes** (`Flywheel`, `Turret`, `Hood`, `Feeder`, `Intake`) to **constructor injection** — they take `HardwareMap` (+ deps like `VoltageSensor`) in the constructor and are ready to use immediately, reading device names from `RobotConfig` instead of inline string literals.
- Updated every caller of those classes (`V3Auto`, `V3ClosePartner`, `V3FarAuto`, `V3Tele`, `FeedForwardTuner`, `FlywheelKpTuner`, `pedroPathing/Constants.java`) to the new constructors and `RobotConfig` constants.
- **Added `autos/V3FarAutoByYuvi.java`** (a new far-side auto contributed via PR #14). It originally had 5 compile errors (referenced not-yet-existent `RobotConfig`/`Flywheel`, a no-arg `Turret()` + `.init(...)` that doesn't exist, and a wrong `pedroPathing.constants.Constants` import). All fixed: now uses `Feeder`/`Hood`/`Intake`/`Flywheel`/`Turret` via constructor injection and `RobotConfig` constants, matching the other autos. Unused helper methods (`clutchIn()`, `clutchOut()`, `armBlock()`, `armShoot()`, `setHoodAngle()` — now provided by `Feeder`/`Hood`) were removed.
- Both the refactor and the `V3FarAutoByYuvi` fixes were merged via **PR #15** (`fix-v3-far-auto-yuvi` → `master`, commit `9074533`).
- A small follow-up tweak from Yuvi's branch (PR #16's underlying intent — seg8's heading interpolation changed from `.setLinearHeadingInterpolation(h(0.0), h(0.0))` to `.setTangentHeadingInterpolation().setReversed()`) was ported directly onto `master` and pushed as commit `77ca7de` (per explicit instruction — too small to need a PR).
- Verified `./gradlew :TeamCode:compileDebugJavaWithJavac` compiles cleanly on `master` (commit `77ca7de`) — only pre-existing `source/target value 8 is obsolete` deprecation warnings remain, 0 errors.

## Current state of the codebase (post-refactor)

```
teamcode/
├── AllianceMirror.java, AllianceStore.java, AutoStartStore.java
├── AllianceSelectorTeleOp.java, CloseFarSelectorTeleOp.java
├── RobotConfig.java     — central device-name constants (NEW)
├── hardwareClasses/
│   ├── Feeder.java       — armServo/clutchServo feed mechanism (constructor injection)
│   ├── Flywheel.java     — shooter flywheel(s), shootTop/shootBottom, rad/s (renamed from FlywheelASG)
│   ├── Hood.java         — hoodServo, linear angle mapping (constructor injection)
│   ├── Intake.java       — intake_motor (constructor injection)
│   └── Turret.java        — turret aim, degrees ±180 (constructor injection)
├── autos/
│   ├── V3Auto.java          (954 LOC)
│   ├── V3ClosePartner.java  (1025 LOC)
│   ├── V3FarAuto.java       (525 LOC)
│   └── V3FarAutoByYuvi.java — new far-side auto (NEW, see "untested" note below)
├── teles/
│   └── V3Tele.java        (1010 LOC) — primary teleop
├── tuners/
│   ├── FeedForwardTuner.java
│   ├── FlywheelKpTuner.java
│   └── ManualControl.java
├── modernTests/
│   └── FlywheelTest.java  (renamed from FlywheelASGTest.java)
└── pedroPathing/
    ├── Constants.java     — Pedro Pathing follower/localizer config
    └── Tuning.java        — Pedro Pathing tuning menu (multi-class file, see note below)
```

All vision (`vision/limelight/`, `vision/opencv/`) was removed in the original purge — there is currently **no active vision/Limelight code** on `master` (there is a separate `Limelight` branch, currently 19 commits behind `master`, not part of this handoff). If vision (AprilTag/Limelight aiming) is still part of the plan for this season, that's effectively a "rebuild from archive/branch or scratch" item — flag to Jason if it's needed.

---

## Open Improvements, Ranked

### 1. Extract a shared autonomous FSM/base to de-duplicate `V3Auto` / `V3ClosePartner` / `V3FarAuto` (/ `V3FarAutoByYuvi`)
**Why it matters**: These autos (954 / 1025 / 525 LOC, plus the new `V3FarAutoByYuvi`) share ~80% of their structure — hardware init, pose tracking, FSM skeleton, turret-aim/shoot sequence. Every tuning fix (e.g. shot-timing tweak) currently has to be copied into up to 4 files, and they will drift apart over time.

**Files affected**: [V3Auto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3Auto.java), [V3ClosePartner.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3ClosePartner.java), [V3FarAuto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3FarAuto.java), [V3FarAutoByYuvi.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3FarAutoByYuvi.java).

**Risk**: Medium-high — autonomous code is highest-stakes (a bad refactor = non-functional auto at competition). Requires field testing after the change.

**Timing**: Do this between competitions, with field-test time budgeted afterward. Highest payoff item on this list, but not a "quick fix."

---

### 2. Pull repeated pose-velocity-estimation logic into a shared utility
**Why it matters**: `V3Tele` and `V3Auto` both independently implement "track last pose → compute field velocity → exponential filter" for shot-on-move compensation, with different/inconsistent filter alpha values across files. This is exactly the kind of math divergence that produces "works in auto but shoots wrong in teleop" bugs that are hard to trace.

**Files affected**: [V3Tele.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java), [V3Auto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3Auto.java), and likely `V3ClosePartner.java`/`V3FarAuto.java` too. Note: `ShootOnMove.java` (the natural home for this) was deleted in the original purge — check the archive tag if its logic should be revived as the shared utility, or write a fresh small class.

**Risk**: Medium — affects shoot-on-move accuracy in both auto and teleop; needs field validation after extraction.

**Timing**: Bundle with item #1 since they touch the same code paths (pose tracking lives inside the FSM).

---

### 3. Standardize control-gain naming conventions
**Why it matters**: CLAUDE.md mandates `kP`/`kV`/`kS`/`kD`/`kF` naming with units commented. This is mostly followed in the current hardware classes, but worth a final consistency check now that the class list is small (`Flywheel`, `Turret`, `Hood`, `Intake`, `Feeder`) — confirm each tunable gain follows the convention and has a units comment where the unit isn't obvious from context.

**Files affected**: [Flywheel.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/Flywheel.java), [Turret.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/Turret.java), [Hood.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/Hood.java).

**Risk**: Very low — naming/comment-only changes.

**Timing**: Quick win, can be done any time — low priority but cheap.

---

### 4. Re-evaluate the `teles/` and `modernTests/`/`tuners/` folder names now that they're small
**Why it matters**: `teles/` contains only the single primary teleop (`V3Tele`), `modernTests/` contains only `FlywheelTest.java`, and `tuners/` has 3 files. The original "teles" (plural, implying multiple teleop variants) and "modernTests" (implying there's an "old tests" counterpart, which no longer exists) names are now slightly misleading for a clean tree. Not urgent, but worth considering during the next organizational pass:
- `teles/` → could merge `V3Tele` back to root, or rename folder to something like `teleop/`.
- `modernTests/` + `tuners/` → could merge into a single `dev/` or `tools/` folder (this was suggested in the original review as well).

**Files affected**: [V3Tele.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java), `modernTests/FlywheelTest.java`, `tuners/*`.

**Risk**: Low — pure file moves + package renames, same caution as the `pedroPathing` move (check for sibling-class static imports or multi-class files before moving; `FlywheelTest` and tuner files are single-class so likely fine).

**Timing**: Low priority, cosmetic. Could be folded into whatever the next organizational pass is.

---

### 5. Field-test `V3FarAutoByYuvi`
**Why it matters**: This auto was added and made to compile this session, but its Bezier path segments (especially seg1, seg7, and the recently-tweaked seg8) have **not been field-tested**. Seg8's heading interpolation was changed from linear to tangent+reversed based on Yuvi's branch, but that change itself is also untested on the field as of this handoff.

**Files affected**: [V3FarAutoByYuvi.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3FarAutoByYuvi.java).

**Risk**: High in the sense that "compiles" ≠ "drives correctly" — paths/headings need on-field verification before relying on this auto in a match.

**Timing**: Before this auto is used in competition.

---

## Future Task Ideas (not yet ranked into the main list)

These were raised in a review session and are tracked here for later triage:

- **XML/Android-Studio-defined hardware configuration** — explore whether robot configuration (motor/servo names, ports) can be defined ahead of time in Android Studio/source instead of relying entirely on manual Driver Hub configuration. Likely low payoff given `RobotConfig.java` already centralizes device-name constants and the FTC SDK doesn't have a clean way to push a config to the Driver Hub from source — but worth a quick feasibility look if config drift becomes a recurring pain point.
- **Reusable boilerplate/base structure for future TeleOp and autonomous programs** — this is effectively the same as open item #1 above (shared autonomous FSM/base) for autos. If a similar base/template is wanted for *future TeleOps* too (beyond just `V3Tele`), that would be a separate, smaller follow-up once #1 lands and the pattern is established.
- **Broader naming-convention review** — beyond the control-gain naming already covered in item #3 and the folder-naming questions in item #4, do a pass over variable/method/class/package/file names across the whole `teamcode/` tree for consistency (e.g. capitalization, abbreviations, plural vs singular package names). Low risk, low urgency — good candidate for a quiet/cleanup session.

---

## Notes for the next session

- The codebase currently **compiles cleanly** (`./gradlew :TeamCode:compileDebugJavaWithJavac` → BUILD SUCCESSFUL on `master` @ `77ca7de`) — only pre-existing `source/target value 8 is obsolete` deprecation warnings remain.
- **Build environment gotchas** (only relevant if explicitly asked to run a build — see CLAUDE.md, builds aren't run by default):
  - There's no system Java; use Android Studio's bundled JBR: `export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"`.
  - Gradle needs `local.properties` with `sdk.dir=/Users/jasonbottino/Library/Android/sdk` — copy it into any worktree you create, it's gitignored.
- **Git worktree workflow**: when a branch needs to build against `origin/master` in isolation from a dirty working tree, `git worktree add <path> <branch>` works well — just remember to copy `local.properties` in.
- **Lesson learned this session — multi-class files**: a single `.java` file can contain many top-level classes sharing state via a self-referencing `import static ThisClass.*;` (e.g. `pedroPathing/Tuning.java`) — don't remove such imports without grepping the whole file for unqualified usages of the imported class's static members.
- **Git branch naming gotcha**: there's a remote branch literally named `origin/making-new-auto-by-yuvi`, which combined with a local branch of the same name produces a confusing doubled ref `origin/origin/making-new-auto-by-yuvi` in some contexts. Watch for this if working with Yuvi's branch again.
- **Local branch hygiene**: as of this handoff, the local repo has some stale/leftover branches that could be cleaned up (not urgent, just noise):
  - local `master` is stale (15 commits behind `origin/master`) — the active work happened on `add-claude-md`, which is at the same commit as `origin/master`.
  - `yuvi-seg8-tmp` and `fix-v3-far-auto-yuvi` are leftover temporary/merged branches from this session's work and can be deleted locally.
- Don't run `git add`/`commit`/`push` unless explicitly asked (per CLAUDE.md).
- Don't run gradle builds by default — only when explicitly asked to verify compilation (per CLAUDE.md).
