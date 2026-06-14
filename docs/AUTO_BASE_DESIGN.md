# Design: Shared Autonomous FSM / Base Structure

**Status:** Planning output (Prompt 1). No code written. This is a design to review before any
implementation prompt (Prompt 2) is drafted.

**Scope:** De-duplicate the four autonomous opmodes in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/` —
`V3Auto.java` (955 LOC), `V3ClosePartner.java` (1026), `V3FarAuto.java` (526),
`V3FarAutoByYuvi.java` (461) — without baking in this season's specific hardware,
action set, or path-following style.

---

## 1. What the four files actually share (evidence)

I read all four end-to-end. The duplication is concrete and, in several places, byte-for-byte.

### Identical or near-identical code (copy-paste duplication)

| Concern | V3Auto | V3ClosePartner | V3FarAuto | Yuvi | Notes |
|---|---|---|---|---|---|
| `FeedState` enum (IDLE→WAIT_BEFORE_INTAKE→RUN_INTAKE→DONE) | 113-118 | 113-118 | 103-108 | 81-86 | **Identical in all four.** |
| `updateFeedSequence()` (incl. reverse-after-feed) | 771-810 | 810-848 | 441-477 | 400-434 | Same shape; only timing constants and where `clutchIn()` is called differ. |
| `startFeedSequence()` | 765-769 | 804-808 | 434-439 | 393-398 | Far/Yuvi add `clutchIn()` here; close autos do it inside `WAIT_BEFORE_INTAKE`. |
| `normalize180()` | 882-884 | 920-922 | 505-507 | — | **Identical.** |
| `wrapIntoTurretWindow()` | 886-903 | 924-941 | 509-526 | — | **Identical.** |
| `p()`, `p(x,y,h)`, `h()` alliance helpers | 295-308 | 305-318 | 232-245 | 451-461 | **Identical** (Pose/heading mirroring). |
| `getTargetX()` | 291-293 | 301-303 | 228-230 | — | **Identical.** |
| `mirrorTurretCommand()` | — | — | 247-249 | 447-449 | Identical in the two that use it. |
| `trackGoalFromOdometry()` (dynamic aim + shot-on-move + predicted-distance filter) | 812-876 | 850-914 | 479-503¹ | — | V3Auto and ClosePartner are **byte-identical**. |
| `updateShotFromDistance()` + 14-row lookup table | 905-955 | 943-993 | — | — | V3Auto and ClosePartner **byte-identical**, including the `+5`/`+15` fudge constants. |
| Hardware init (intake, hood, feeder, flywheel-w/-VoltageSensor, follower, turret) | 169-182 | 177-192 | 142-155 | 104-118 | Same sequence; ClosePartner uses raw `Servo`/`DcMotorEx` instead of wrappers (see §7). |
| Outer run-loop shape (`follower.update()` → actuator updates → feed → FSM → turret → telemetry) | 235-285 | 245-295 | 194-222 | 156-184 | Same *set* of per-loop calls; **ordering differs** (see §4, "ordering is load-bearing"). |
| Telemetry block (alliance / state / feed / pose / follower-busy / flywheel) | 272-284 | 282-294 | 211-221 | 175-183 | Same lines, minor per-auto extras. |
| `DONE` behavior (intake 0, turret→0, flywheel stop) | 610-613 | 611-614 | 427-430 | 361-365 | Same intent. |
| End-of-run cleanup (`flywheel.stop(); intake 0`) | 287-288 | 297-298 | 224-225 | 362-363 | Same. |

¹ `V3FarAuto.trackGoalFromOdometry()` is a simpler (non-predictive) variant and is **currently
dead code** — the call site at line 198 is commented out; Far auto uses a fixed turret angle.

### What genuinely varies between them

- **`AutoState` enum + `updateAutoState()` switch.** Completely different per auto: V3Auto 26
  states, ClosePartner 30 (three gate cycles), FarAuto 9 (+ a `extraCycleCount` loop), Yuvi 18
  (`DRIVE_SEGn`/`PAUSE_SEGn`). This is the "program" — see §3.
- **Path strategy.** V3Auto/ClosePartner/FarAuto build individual `Path` objects (some up-front in
  `buildPaths()`, some rebuilt mid-run from the live pose, e.g. `startExtraGateIntakeMove`,
  `startReturnFromGateToShoot`). Yuvi uses `PathChain` segments via `follower.pathBuilder()`
  (`seg1..seg8`, lines 190-246).
- **Aiming.** Dynamic odometry aim + lookup table (V3Auto, ClosePartner) vs. fixed turret angle +
  fixed hood/flywheel (FarAuto, Yuvi).
- **Base class.** `LinearOpMode` (three) vs. `OpMode` (Yuvi). See §5.
- **Hardware access.** Wrapper classes (`Hood`/`Feeder`/`Intake`) vs. raw `Servo`/`DcMotorEx`
  (ClosePartner). See §7.
- **Constants/config:** start poses, cycle counts, fixed shot values, feed timings,
  `AutoStartStore.setClose()` vs `setFar()`.

**Takeaway:** ~80% of the duplicated *volume* lives in reusable, hardware-touching helpers
(feed FSM, aim controller, lookup table, alliance math). The remaining variation is almost
entirely the FSM program and path data, which *should* stay per-auto. This shapes the design: the
big win is **composition of small helpers**, not a deep inheritance hierarchy.

---

## 2. Design goals (from the prompt) and how the design meets them

1. **Generalize beyond this robot/season** — the base must not assume `Flywheel`/`Turret`/`Hood`/
   `Feeder`/`Intake`, the feed FSM, or even shooting at all.
2. **Generalize the path-following layer** — don't hard-code Pedro `Path`/`PathChain`/`isBusy()`
   into the state-machine core where it can reasonably be abstracted.
3. **Separate truly-common from truly-auto-specific.**
4. **Concrete class/interface structure** with tradeoffs.
5. **Resolve `OpMode` vs `LinearOpMode`.**
6. **Migration path.**

The recommended structure is a **thin lifecycle base class + a toolkit of composable helpers**.
The base owns *only* the season-agnostic loop scaffold and knows about **no** subsystem. Everything
season-specific (subsystems, aiming, feeding, shot tables) is a helper the auto composes, or stays
in the per-auto FSM.

---

## 3. Recommended architecture

### 3.1 Thin lifecycle base — `AutoMode extends LinearOpMode`

New package: `autos/framework/` (name TBD — see §8).

The base captures the parts of `runOpMode()` that are identical in every auto and **references no
hardware**. It exposes the four lifecycle phases as template methods and runs the
init→init-loop→start→loop→stop skeleton plus the stop guards:

```java
public abstract class AutoMode extends LinearOpMode {

    protected abstract void onInit();         // build hardware, paths, helpers
    protected void onInitLoop() {}            // optional: re-zero, hold turret, telemetry
    protected abstract void onStart();        // spin up, set first state
    protected abstract void onLoopTick();     // ONE iteration: the auto owns its exact ordering
    protected void onStop() {}                // safe shutdown

    @Override
    public final void runOpMode() {
        onInit();
        while (opModeInInit()) { onInitLoop(); }
        waitForStart();
        if (isStopRequested()) return;
        onStart();
        while (opModeIsActive()) { onLoopTick(); }
        onStop();
    }
}
```

**Why `onLoopTick()` is a single hook rather than a registry of `Updatable`s:** the per-loop call
*ordering* in the existing autos is load-bearing and field-tuned. For example V3Auto
(`235-285`) computes the aim, then `flywheel.update()`, then feed, then the FSM, then **finally**
`turret.update()` — because the FSM (e.g. `DONE`) may itself call `turret.setAngle(0)` and must win.
FarAuto orders it differently. A fixed-order "update everything" list cannot express "turret updates
*after* the FSM but flywheel updates *before* it" without contortions, and getting it subtly wrong in
high-stakes auto code is a real regression risk. Keeping `onLoopTick()` per-auto preserves each
auto's exact, tested ordering while still removing the lifecycle boilerplate and giving the helpers a
home. This is a deliberate "don't over-abstract the hot loop" call.

The base deliberately knows nothing about turrets or flywheels — a future no-shooter auto implements
`onLoopTick()` with whatever its subsystems are.

### 3.2 Composable helpers (where the real dedup lives)

All in `autos/framework/`. Each is independently unit-testable and individually skippable.

**`AutoMath`** (static utility) — `normalize180()`, `wrapIntoTurretWindow()`. Pure functions,
zero dependencies, trivially testable. Replaces 4 copies.

**`AllianceGeometry`** — constructed with `(boolean isRed, double blueTargetX, double targetY)`,
provides `p(x,y)`, `p(x,y,headingDeg)`, `h(deg)`, `targetX()`, `mirrorTurretCommand(deg)`. Wraps the
existing static `AllianceMirror`. Replaces the `p`/`h`/`getTargetX`/`mirrorTurretCommand` quartet
copied into all four files. Season-agnostic (mirroring is a field-geometry concept, not a hardware
one).

**`FeedSequence`** — owns the `FeedState` enum, the feed/reverse timers, and the
`start()/update()/isIdle()/isDone()/reset()` API. **Decoupled from hardware** via a small interface
so it survives a season where "feeding" means something else:

```java
public interface FeedActuator {
    void feedDrive(double power);  // today: intake.setPower
    void gateOpen();               // today: feeder.armShoot
    void gateClosed();             // today: feeder.armBlock
    void clutchIn();
    void clutchOut();
}
```

`FeedSequence` takes a `FeedActuator` + a `FeedTimings` value object (start delay, total time,
reverse time) so each auto keeps its own tested constants. The current `Intake`+`Feeder` wrappers
get a tiny adapter implementing `FeedActuator`. Replaces 4 copies of `updateFeedSequence()` /
`startFeedSequence()`.

**`OdometryAimController`** (optional) — encapsulates `trackGoalFromOdometry()`: turret-offset
geometry, the predicted-distance EMA filter, and shot-on-move compensation. Constructed with
`AllianceGeometry`, turret offset/limits, and a `Turret` handle (or an aim-output interface, see
§8). Returns the desired turret angle and the filtered predicted distance. Used only by close autos;
far autos simply never instantiate it.

**`ShotLookupTable`** (optional) — the 14-row hood/flywheel interpolation. Constructed with the
table; `solve(distance) → (hoodDeg, flywheelRad)`. Pure data + interpolation, easily unit-tested,
easily re-tuned in one place. Today it is copy-pasted (with magic `+5`/`+15` offsets) into two files;
those offsets become explicit table values or named params.

**Pose re-zero helper** (optional small util) — the `gamepad1.cross` debounced re-zero used in
V3Auto/ClosePartner init loops (`207-222`). Minor, but removes a fiddly duplicated pattern.

### 3.3 What a migrated auto looks like

A migrated close auto becomes roughly: `extends AutoMode`; fields for its subsystems + an
`AllianceGeometry`, a `FeedSequence`, an `OdometryAimController`, a `ShotLookupTable`; `onInit()`
builds those + the paths; `onLoopTick()` is the existing loop body verbatim but calling
`feed.update()`, `aim.update(pose)`, `shotTable.solve(...)` instead of inline copies; `onLoopTick()`
still contains the auto's exact ordering; the `AutoState` enum and `updateAutoState()` switch stay
as-is. Net: the ~400 lines of helpers collapse to construction + delegation; the FSM (the part that
*is* this auto) is untouched.

---

## 4. Path-following: where Pedro stays vs. where it's hidden

The prompt asks to abstract path-following "where reasonable" and to call out the boundary
explicitly. My recommendation:

- **Keep Pedro Pathing as a hard dependency at the auto + helper level, not in the base.** `Pose`,
  `Follower`, `Path`, `PathChain`, `BezierLine/Curve` are pervasive and deeply intertwined with the
  field-geometry math (`AllianceGeometry` produces `Pose`s; the aim controller reads
  `follower.getVelocity()`). Building a full drivetrain abstraction layer to hide Pedro would be
  over-engineering for an FTC codebase and would obscure the very API the team tunes against.

- **The state-machine *core* (the `AutoMode` base) already contains zero Pedro references** — it
  does not even call `follower.update()`. The auto's `onLoopTick()` owns every follower interaction
  (`update()`, `followPath()`, `isBusy()`, `getCurrentTValue()`, `isRobotStuck()`,
  `breakFollowing()`). So the *core* is path-library-agnostic by construction; if the team ever
  leaves Pedro, only the autos and the geometry/aim helpers change, not the base.

- **The auto-specific seam is "drive a segment, then know it's done."** Today that's expressed two
  ways — `!follower.isBusy()` after `followPath(individualPath)` vs. the same after
  `followPath(pathChainSegment)`. Both are just "advance the FSM when the current move completes,"
  and both already live inside `updateAutoState()`. No abstraction is needed to support both styles
  simultaneously; the base imposes nothing on how paths are defined or triggered. Individual-`Path`
  autos and `PathChain` autos coexist with the same base.

- **Optional future seam (do *not* build now):** if path-following ever needs to be swapped, the
  natural interface is a `PathRunner { void update(); boolean isBusy(); double progress(); Pose
  pose(); ... }` that `Follower` trivially satisfies. Flagged as a future option, explicitly
  deferred (YAGNI) — there is no second path library today and speculative abstraction here costs
  readability against the API the team actually tunes.

---

## 5. `OpMode` vs `LinearOpMode`

**Recommendation: standardize the base on `LinearOpMode`; migrate Yuvi off `OpMode`.**

- It's the documented team norm (CLAUDE.md) and 3 of 4 autos already use it.
- Yuvi's `OpMode` `init()/init_loop()/start()/loop()` lifecycle maps *cleanly* onto the
  `onInit()/onInitLoop()/onStart()/onLoopTick()` template methods in §3.1 — the structured phases
  Yuvi gets from `OpMode` are reproduced inside `AutoMode`'s `runOpMode()`, so nothing is lost in the
  conversion. Yuvi uses no `OpMode`-only capability; it was simply authored in a different style.
- Supporting *both* base classes would require either a duplicated base or a shared mixin pulled in
  two ways — added surface area for zero functional gain.

So: one base, `LinearOpMode`-rooted, with the iterative phases expressed internally. Yuvi's
conversion is part of its migration (§6) and is mechanical.

---

## 6. Migration path

Autonomous is high-stakes; each migrated auto must be **field-tested before the next**. Order chosen
to de-risk the base first, bank the biggest dedup win second, and absorb the one base-class change
last.

1. **`V3FarAuto` first (lowest risk, clearest validation).** Smallest (526 LOC), fewest states (9),
   already uses the hardware wrappers, fixed aim (no odometry controller or lookup table to port).
   Migrating it exercises and proves `AutoMode`, `FeedSequence`, `AllianceGeometry`, and `AutoMath`
   in isolation, with the simplest possible FSM. Delete its dead `trackGoalFromOdometry()` (§1) as
   part of this.

2. **`V3Auto`, then `V3ClosePartner` (biggest payoff).** These two carry the byte-identical
   `trackGoalFromOdometry()` + 14-row table + aim math, and they are exactly where "every tuning fix
   has to be hand-copied" hurts most. Migrate `V3Auto` first to prove `OdometryAimController` +
   `ShotLookupTable` against a known-good auto, then `V3ClosePartner` — whose migration **also**
   folds in the raw-servo→wrapper cleanup (§7), since once it shares the helpers there's no reason
   for it to keep its private `setHoodAngle`/`armBlock`/`clutchIn` copies.

3. **`V3FarAutoByYuvi` last.** It is the only auto needing a base-class change (`OpMode` →
   `LinearOpMode`) and the only one using `PathChain` segments. Doing it after the base and helpers
   are proven on the other three means the riskiest single structural change happens against a stable
   foundation, and it confirms the base handles segmented paths with no core changes.

---

## 7. The `V3ClosePartner` raw-servo inconsistency

`V3ClosePartner` uniquely accesses hood/arm/clutch as raw `Servo`s with its own `setHoodAngle()`
(1011-1026), `armBlock/armShoot` (1003-1009), `clutchIn/clutchOut` (995-1001). Comparing constants:
its hood mapping (`30–60°` → `0.42–0.95`) is **identical** to `Hood` (`Hood.java:10-13`), and its
arm/clutch positions (`0.28/0.42/0.48/0.52`) are **identical** to `Feeder` (`Feeder.java:10-13`).

So this is a pure, behavior-preserving swap to the `Hood`/`Feeder` wrappers — no tuning changes.
**Recommendation: fold it into V3ClosePartner's migration (step 2), not a separate effort.** Once it
adopts `FeedSequence` (which talks to a `FeedActuator` backed by `Feeder`) and `ShotLookupTable`
(which calls `Hood.setAngle`), the raw servos disappear naturally. Confirm with Jason (§8) — it's
low-risk but it does touch a tested auto.

---

## 8. Tradeoffs & alternatives considered

**Chosen: thin base + composition.**
- *New-auto boilerplate:* low — a new auto writes its FSM, paths, and `onLoopTick()` ordering, and
  constructs whichever helpers it needs. Roughly the lifecycle scaffold (~40 lines) is saved and
  ~300–400 lines of helper duplication are eliminated.
- *Skipping/overriding shared behavior:* trivial — skip a piece by not constructing it (a fixed-aim
  auto just never makes an `OdometryAimController`). No inheritance gymnastics, no
  `super.someHook()` to suppress.
- *Testability:* strong — `AutoMath`, `ShotLookupTable`, `AllianceGeometry`, and `FeedSequence`
  (with a fake `FeedActuator`) are pure/injectable and unit-testable off-robot. The current monoliths
  can't be tested at all.
- *Readability:* the FSM (the interesting part) stays front-and-center in each auto; the noise moves
  to named, single-responsibility helpers.

**Rejected: deep abstract base with template methods for feed/aim/shoot baked in.** DRYer for
*today's* autos, but it hard-codes this season's hardware and action set into the base — directly
violating goal #1. A no-shooter or re-mechanism'd auto next season would fight the base. Also harder
to test and to selectively skip behavior.

**Rejected: pure composition, no base class (each auto is a bare `LinearOpMode` wiring helpers).**
Maximum flexibility, but every auto re-copies the init→loop→stop lifecycle and stop-guard
boilerplate — the one piece that genuinely *is* identical and worth a base class. The thin base
costs almost nothing and removes that last duplication.

**Rejected (deferred): generic state-graph engine** (states-as-objects, transition tables). Real
frameworks do this, but the switch-based FSMs here are readable and the team edits them directly
on-site; replacing them with a graph DSL adds indirection exactly where clarity matters most under
competition pressure. The FSM stays a per-auto `enum` + `switch`.

---

## 9. Open questions for Jason (confirm before Prompt 2)

1. **Package & naming.** New package `autos/framework/`? Base class name — `AutoMode`,
   `StateMachineAuto`, or `AutoOpMode`? Helper names (`FeedSequence`, `OdometryAimController`,
   `ShotLookupTable`, `AllianceGeometry`, `AutoMath`) OK, or prefer different ones?
2. **`FeedSequence` coupling.** Decouple via a `FeedActuator` interface (looser, season-proof, a
   little more wiring) — recommended — vs. just hand it the concrete `Intake`+`Feeder` (less code,
   but re-couples to this season). Confirm the interface approach.
3. **Preserve per-auto feed timings exactly?** The close autos and far autos differ in feed
   constants and in where `clutchIn()` happens. Plan is a `FeedTimings` config per auto so migration
   is behavior-preserving rather than "unify to one timing." Confirm.
4. **V3ClosePartner raw-servo cleanup** folded into its migration (recommended, §7) vs. separate.
5. **Dynamic-shot toggles** (`enableDynamicShotControl` / `enableShotOnMoveComp`) — keep as per-auto
   fields driving the optional `OdometryAimController`, or move the flags inside the controller?
6. **Telemetry** — a standard base telemetry header + a per-auto `addTelemetry()` hook, or leave
   telemetry fully per-auto? (Leaning: standard header + hook.)
7. **Scope of Prompt 2** — migrate only `V3FarAuto` first (build base + helpers, prove on the
   simplest auto, field-test, then continue), or attempt more in one pass? Recommend the former given
   the high-stakes nature of auto.

---

## 10. Summary

The four autos duplicate a feed FSM, alliance/turret math, an odometry aim controller, and a 14-row
shot table — several of them byte-for-byte — while genuinely differing only in their FSM program,
path data, aim mode, and base class. The recommended design is a **thin, hardware-agnostic
`AutoMode` lifecycle base** (no subsystem knowledge, no Pedro references in the core) plus a
**toolkit of small, individually-skippable, unit-testable helpers** that own the duplicated
behavior. Pedro Pathing stays a hard dependency at the auto/helper layer (abstracting it is
over-engineering) but is absent from the base by construction. Standardize on `LinearOpMode`. Migrate
**V3FarAuto → V3Auto → V3ClosePartner (with the raw-servo cleanup) → V3FarAutoByYuvi**, field-testing
between each. Confirm the §9 questions before implementation.
