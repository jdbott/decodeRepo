# AI Opportunities Audit

A ranked list of high-leverage opportunities where an advanced AI model could improve workflow,
code quality, autonomous performance, or team capability for Team Tesseract (21689), DECODE season.

Based on a full read of the `TeamCode/` tree (all hardware classes, all 4 autos, the teleop, the
alliance/selector/store helpers, all tuners, the Pedro `Constants`/`Tuning` files, the CI workflow,
and both design docs). Companion to [AUTO_BASE_DESIGN.md](AUTO_BASE_DESIGN.md).

**Prioritized below:** the two flagged favorites (#1 and #10) are written out in full. Everything
else is a one-paragraph stub to revisit later — promote any of them to full detail on request.

---

## The structural truth everything hangs on

This codebase has **zero off-robot executable verification** — no unit tests, no simulation, no
replay. The CI ([.github/workflows/android.yml](../.github/workflows/android.yml)) compiles and
stops (and its last step downloads a just-produced artifact via a nonexistent
`actions/download-artifact@v7.0.0`, so it's cargo-culted). Every behavioral change — a path tweak, a
shot-table value, an FSM edit — can only be validated by deploying to a physical robot on a physical
field with a charged battery. For a 9-person team with limited field time, that is the binding
constraint on everything. The two favorites below both attack it (one directly, one for continuity).

---

## ⭐ Favorite #1 — `AutoSim`: headless off-robot simulation + replay harness

**One-sentence:** A pure-Java kinematic simulator that runs an entire autonomous opmode (FSM + Pedro
follower model) to completion in milliseconds on a laptop or in CI, asserting end-state invariants.

**Problem it solves / capability unlocked:** Today an auto iteration costs ~5 minutes of field time;
in sim it costs milliseconds, enabling hundreds of iterations and *regression testing* of auto logic.
You already have the physical parameters to seed a credible model —
[Constants.java:21-50](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java)
has `mass(10.46)`, `forwardZeroPowerAcceleration(-33.7)`, `xVelocity(73.8)`, `yVelocity(60.8)`. The
FSMs are deterministic switch statements driven by `follower.isBusy()` / `getCurrentTValue()` and
timers — all mockable. A test like "V3Auto fires 6 times and ends within 4″ of the shoot pose"
becomes a CI gate.

**Impact: High.** It converts the codebase's central liability (untestable monoliths) into a tested
system and is the substrate for the shot-solver, config validation, and next-season bring-up. The
only thing it can't catch is real-world physics divergence — but it catches *logic* regressions (a
broken state transition, a path that overshoots, a feed that fires in the wrong state), which is
where most auto failures actually live.

**Feasibility: Medium.** The honest hard part is modeling Pedro's follower fidelity. The pragmatic
move is a *coarse* model (point-mass with the measured accel/velocity caps, treat `followPath` as
"arrive after t = path_length / v_avg") — enough to exercise FSM logic, not to predict exact poses.
AI is well-suited to scaffold both the sim and the assertion DSL. Student skill level is fine for
*reading/extending* the tests once the harness exists.

**Tooling:** Lives entirely in-stack — a new `src/test/java` source set, JUnit (already available via
the Android Gradle plugin). No external service.

**Why this is the keystone:** The auto-base refactor in
[AUTO_BASE_DESIGN.md](AUTO_BASE_DESIGN.md) is high-value but high-risk ("a bad refactor =
non-functional auto at competition," requiring field re-test of each migrated auto). `AutoSim` lets
you prove each migration preserves behavior in milliseconds instead of burning field time — turning a
risky refactor into a safe one. Build this first, and the shot-solver, config externalization, and
next-season portability items all get dramatically cheaper and safer to land.

---

## ⭐ Favorite #10 — AI-maintained living docs + a code-grounded onboarding tutor

**One-sentence:** Auto-regenerate the subsystem glossary / architecture overview from the actual
source on each merge, and provide a code-grounded Q&A onboarding guide for new students.

**Problem it solves / capability unlocked:** This is a **9-person team with high between-season
turnover** — institutional memory is the scarce resource. `CLAUDE.md` itself admits the subsystem
glossary "will go stale as new mechanisms are added," and `HANDOFF.md` is a careful but *manually*
maintained artifact. The stock 1,699-line [README.md](../README.md) is the generic FTC SDK readme —
there is **no team-specific onboarding doc**. A pipeline that keeps a "how our robot's code actually
works" doc synced to the code, plus an AI tutor a rookie can ask ("how does the turret know where the
goal is?"), directly attacks turnover risk and shortens the ramp for every new student.

**Impact: Medium-High.** Low for any single match, high across seasons. Every September the team
otherwise re-pays the cost of lost knowledge; this caps that cost.

**Feasibility: High.** This is squarely what AI is good at; the only cost is wiring it into the
workflow (e.g. a regeneration step on merge, plus a checked-in onboarding doc the tutor reads from).
No special skill required of students to *consume* it.

**Tooling:** AI-assisted generation; the docs themselves live in-repo. Optionally hook regeneration
into CI so the architecture doc can never drift from the code.

---

## Other opportunities (brief — revisit later)

Each of these was analyzed in full during the audit; promote any to full detail on request.

### Tier 1 — high-leverage

- **#2 `ShotSolver` (physics model + data fit).** Replace the hand-tuned 14-row shot table — which is
  copy-pasted into three files with *divergent* fudge factors (`+15` in autos vs. `+8` in teleop, so
  the same distance commands a different shot in auto vs. teleop) — with one canonical table generated
  from logged `(distance, hood, flywheel, made/missed)` data. Also fixes `estimateShotTimeSec()` being
  stubbed to a constant `0.77` (distance-independent flight time, physically wrong). Impact: High.

- **#3 Match-log capture + AI post-match analysis.** The robot already *computes* rich telemetry
  (turret target vs. actual, flywheel target vs. actual, predicted distance) and throws it away.
  Persist it to a file per match and post-process with AI to turn "I think it shot low that time" into
  data. Nearly free; compounds every match. Impact: High relative to cost.

- **#4 AprilTag relocalization.** The entire aim chain runs on drift-prone odometry with no absolute
  reference — which is *why* the team built manual drift hacks (`gamepad1.circle` pose-reset, turret
  zeroing mode). FTC Vision is already a dependency and DECODE goals carry AprilTags; relocalizing off
  them fixes the root cause instead of patching symptoms. Impact: High.

### Tier 2 — strong, near-term

- **#5 Auto-base refactor (already designed).** Execute [AUTO_BASE_DESIGN.md](AUTO_BASE_DESIGN.md) to
  collapse ~80% duplication across the four autos. Known item; gate it on #1 (AutoSim) to de-risk.

- **#6 Flywheel control upgrade + universal feed-when-ready gate.** Replace bang-bang + EMA with
  Take-Back-Half (or proper PIDF) for faster post-shot recovery, and gate *every* feed on
  flywheel-at-speed (only Yuvi's auto does this today; the others fire on a timer into a possibly
  sagging wheel). Impact: High on shot consistency.

- **#7 Wire the turret tracking controller into the autos.** The velocity-feedforward `setTargetState`
  controller exists and is used only in teleop; all four autos use the legacy position mode, so the
  turret lags during shoot-on-the-move — the exact scenario autos depend on. It's a wiring change.

- **#8 Externalize tuning constants into a hot-reloadable config.** Poses, gains, the shot table, and
  feed timings are buried as literals across every opmode. The team already depends on the Panels /
  FTC-Dashboard `@Configurable` infra but barely uses it for its own subsystems. Extend the
  `RobotConfig` centralization pattern to tunables.

### Tier 3 — workflow, ops, continuity

- **#9 Codebase-specific CI guardrails.** Replace the no-op compile-only CI with small AI-authored
  checks tuned to this repo's real failure mode — silently divergent duplicated logic (e.g. "shot
  table appears in >1 file," "feed runs with no flywheel-ready guard," "`normalize180` copies have
  drifted"), plus turn on Android Lint and fix the broken `download-artifact` step. Once #1 exists, CI
  also runs the sim.

- **#11 Next-season portability layer ("digital twin").** Combine #1 (sim) + #8 (config) + #5
  (season-agnostic base) so a *future* robot's auto can be authored and validated in simulation before
  the hardware exists. Emergent payoff of the substrate items, not a standalone build.

- **#12 Closed-loop path authoring.** An AI-assisted tool that round-trips between waypoints/
  constraints and the Java Bezier definitions (the spurious-precision coordinates like
  `GATE_X = 136.5559649236386` are hand-pasted from an external visualizer today). Quality-of-life;
  fewer copy errors.

---

## Concrete bugs/smells found while reading (not being fixed yet — logged for later)

1. **Shot-table velocity divergence** — auto adds `+15`, teleop `+8`, clamp branches `0`; same
   distance → different shot in auto vs. teleop, with edge discontinuities.
   ([V3Auto.java:950](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3Auto.java),
   [V3Tele.java:1008](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java))
2. **Dead velocity estimator in teleop** — `updateEstimatedFieldVelocity()` / `initVelocityEstimator()`
   ([V3Tele.java:829-864](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java))
   are never called; the loop uses `follower.getVelocity()` directly.
3. **`estimateShotTimeSec()` is a constant `0.77`** in all three opmodes — shot-on-move lead is
   distance-independent (physically wrong).
4. **Copy-paste button bug** — `getAutoParkTogglePressed()` returns `gamepad1.triangle ||
   gamepad1.triangle`
   ([V3Tele.java:357](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teles/V3Tele.java)).
5. **Stuck-robot mishandled as "arrived"** — `breakFollowing()` on `isRobotStuck()`
   ([V3Auto.java:240-242](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autos/V3Auto.java))
   but the FSM advances on `!follower.isBusy()`, so a stuck-then-broken path is treated as a
   successful arrival → it shoots from the wrong spot, no recovery.
6. **Feed without flywheel-ready gate** in all opmodes except Yuvi's (see #6 above).
7. **Flywheel encoder ambiguity** — regulates both motors but reads velocity from `flywheelTop` while
   comments imply `flywheelBottom` is the encoder motor
   ([Flywheel.java:13-14, 164](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardwareClasses/Flywheel.java)).
8. **CI dead step** — `actions/download-artifact@v7.0.0` (nonexistent version) downloading the
   artifact it just produced ([android.yml](../.github/workflows/android.yml)).
9. **`V3ClosePartner` raw-servo duplication** — reimplements `Hood`/`Feeder` with identical magic
   constants instead of using the wrappers (also noted in AUTO_BASE_DESIGN.md §7).

---

## Two seasons from now, if AI were first-class from day one

- The four 500–1,000-line auto monoliths are gone, replaced by a thin season-agnostic `AutoMode` base
  plus small, unit-tested helpers.
- There is a `src/test` source set that actually runs; whole autos run headless in CI with end-state
  assertions. A red build means a broken auto *before* anyone drives to the field.
- Tunables live in one hot-reloadable config; the shot table is *generated* from logged data, and
  there is exactly one of it.
- The robot writes a match log every match, with a standing AI post-match report.
- Aim is vision-corrected, so the manual pose-reset and turret-zeroing hacks are gone.
- A new student is productive in days, because the architecture doc regenerates from the code and an
  AI tutor answers grounded questions.

**What's done by hand that fundamentally shouldn't be:** validating auto logic on a physical field;
hand-filling and hand-fudging the shot table (in three divergent copies); hand-copying shared logic
across four autos; hand-correcting odometry drift mid-match; hand-maintaining architecture/handoff
docs and onboarding; pasting raw Bezier coordinates from an external visualizer.

The throughline: this is a strong, control-theory-literate codebase throttled almost entirely by one
missing capability — **the ability to run and check the robot's logic without the robot.** Build #1,
and most of the rest gets cheaper and safer.
