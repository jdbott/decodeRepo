# Codebase Improvement Handoff

This document captures the findings of a full architectural review of `TeamCode/` performed prior to a major cleanup pass. The cleanup (deleting dead V2/legacy/test files, fixing the `Intake`/`IntakeASG` device-name conflict, restructuring `pedroPathing/` into a flat package, updating `CLAUDE.md`) is **already done** — this file covers the **remaining** open items, ranked by priority, so a new session can pick up directly.

A full snapshot of the pre-cleanup codebase is preserved at git tag `season-2025-decode-archive` if any deleted file is ever needed for reference.

## Current state of the codebase (post-cleanup)

```
teamcode/
├── AllianceMirror.java, AllianceStore.java, AutoStartStore.java
├── AllianceSelectorTeleOp.java, CloseFarSelectorTeleOp.java
├── hardwareClasses/
│   ├── Feeder.java       — armServo/clutchServo feed mechanism
│   ├── FlywheelASG.java  — shooter flywheel(s), shootTop/shootBottom, rad/s
│   ├── Hood.java         — hoodServo, linear angle mapping
│   ├── Intake.java       — intake_motor (formerly IntakeASG; old Intake deleted)
│   └── Turret.java       — turret aim, degrees ±180
├── autos/
│   ├── V3Auto.java        (954 LOC)
│   ├── V3ClosePartner.java (1025 LOC)
│   └── V3FarAuto.java      (525 LOC)
├── teles/
│   └── V3Tele.java        (1010 LOC) — primary teleop
├── tuners/
│   ├── FeedForwardTuner.java
│   ├── FlywheelKpTuner.java
│   └── ManualControl.java
├── modernTests/
│   └── FlywheelASGTest.java
└── pedroPathing/
    ├── Constants.java     — Pedro Pathing follower/localizer config
    └── Tuning.java        — Pedro Pathing tuning menu (multi-class file, see note below)
```

All vision (`vision/limelight/`, `vision/opencv/`) was removed in the purge — there is currently **no active vision/Limelight code** in the tree. If vision (AprilTag/Limelight aiming) is still part of the plan for this season, that's effectively a "rebuild from archive or scratch" item, not listed below since it wasn't part of the original improvement list — flag to Jason if it's needed.

---

## Open Improvements, Ranked

### 1. Extract a shared autonomous FSM/base to de-duplicate `V3Auto` / `V3ClosePartner` / `V3FarAuto`
**Why it matters**: These three autos (954 / 1025 / 525 LOC) share ~80% of their structure — hardware init, pose tracking, FSM skeleton, turret-aim/shoot sequence. Every tuning fix (e.g. shot-timing tweak) currently has to be copied into up to 3 files, and they will drift apart over time.

**Files affected**: [V3Auto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3Auto.java), [V3ClosePartner.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3ClosePartner.java), [V3FarAuto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3FarAuto.java).

**Risk**: Medium-high — autonomous code is highest-stakes (a bad refactor = non-functional auto at competition). Requires field testing after the change.

**Timing**: Do this between competitions, with field-test time budgeted afterward. Highest payoff item on this list, but not a "quick fix."

---

### 2. Pull repeated pose-velocity-estimation logic into a shared utility
**Why it matters**: `V3Tele` and `V3Auto` both independently implement "track last pose → compute field velocity → exponential filter" for shot-on-move compensation, with different/inconsistent filter alpha values across files. This is exactly the kind of math divergence that produces "works in auto but shoots wrong in teleop" bugs that are hard to trace.

**Files affected**: [V3Tele.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java), [V3Auto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3Auto.java), and likely `V3ClosePartner.java`/`V3FarAuto.java` too. Note: `ShootOnMove.java` (the natural home for this) was deleted in the purge — check the archive tag if its logic should be revived as the shared utility, or write a fresh small class.

**Risk**: Medium — affects shoot-on-move accuracy in both auto and teleop; needs field validation after extraction.

**Timing**: Bundle with item #1 since they touch the same code paths (pose tracking lives inside the FSM).

---

### 3. Centralize device-name strings into one constants file
**Why it matters**: Device name strings (`"shootTop"`, `"hoodServo"`, `"intake_motor"`, `"armServo"`, `"turretMotor"`, drivetrain motor names, `"imu"`, etc.) are scattered as literals across `hardwareClasses/` and all opmodes. CLAUDE.md currently documents this as "no central config — reuse exact names by grepping V3Tele/V3Auto," which is a workaround for a missing abstraction. A single `RobotConfig`/`DeviceNames` class with `public static final String` constants would let the compiler catch typos and make hardware renames a one-file change.

**Files affected**: All of `hardwareClasses/`, `V3Tele.java`, `V3Auto.java`, `V3FarAuto.java`, `V3ClosePartner.java`.

**Risk**: Medium — touches nearly every file, but mechanically (string literal → constant reference), so low-risk if done incrementally with a compile check after each file.

**Timing**: Good off-season/between-competitions task. High value for long-term maintainability and for making future agent-driven edits safer (less chance of a typo'd device name).

---

### 4. Standardize control-gain naming conventions
**Why it matters**: CLAUDE.md mandates `kP`/`kV`/`kS`/`kD`/`kF` naming with units commented. This is mostly followed in the current (post-purge) hardware classes, but worth a final consistency check now that the class list is small (`FlywheelASG`, `Turret`, `Hood`, `Intake`, `Feeder`) — confirm each tunable gain follows the convention and has a units comment where the unit isn't obvious from context.

**Files affected**: [FlywheelASG.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/FlywheelASG.java), [Turret.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/Turret.java), [Hood.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/Hood.java).

**Risk**: Very low — naming/comment-only changes.

**Timing**: Quick win, can be done any time — low priority but cheap.

---

### 5. Re-evaluate the `teles/` and `modernTests/`/`tuners/` folder names now that they're small
**Why it matters**: Post-purge, `teles/` contains only the single primary teleop (`V3Tele`), `modernTests/` contains only `FlywheelASGTest.java`, and `tuners/` has 3 files. The original "teles" (plural, implying multiple teleop variants) and "modernTests" (implying there's an "old tests" counterpart, which no longer exists) names are now slightly misleading for a clean tree. Not urgent, but worth considering during the next organizational pass:
- `teles/` → could merge `V3Tele` back to root, or rename folder to something like `teleop/`.
- `modernTests/` + `tuners/` → could merge into a single `dev/` or `tools/` folder (this was suggested in the original review as well).

**Files affected**: [V3Tele.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java), `modernTests/FlywheelASGTest.java`, `tuners/*`.

**Risk**: Low — pure file moves + package renames, same caution as the `pedroPathing` move (check for sibling-class static imports or multi-class files before moving; `FlywheelASGTest` and tuner files are single-class so likely fine).

**Timing**: Low priority, cosmetic. Could be folded into whatever the next organizational pass is.

---

## Notes for the next session
- The codebase currently **compiles cleanly** (`./gradlew :TeamCode:compileDebugJavaWithJavac` → BUILD SUCCESSFUL) as of this handoff.
- **Lesson learned this session** (relevant if touching multi-class files like `pedroPathing/Tuning.java` again): a single `.java` file can contain many top-level classes sharing state via a self-referencing `import static ThisClass.*;` — don't remove such imports without grepping the whole file for unqualified usages of the imported class's static members.
- Don't run `git add`/`commit`/`push` unless explicitly asked (per CLAUDE.md).
- Don't run gradle builds by default — only when explicitly asked to verify compilation (per CLAUDE.md).
