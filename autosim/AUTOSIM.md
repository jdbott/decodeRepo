# AutoSim — Maintainer & Extension Guide

This is the **operator's manual** for the `:autosim` module: what it is, how it's wired
together, and — most importantly — **how to change it** without breaking the existing
autos or drifting out of sync with the real robot code. It's written for whoever (human
or AI agent) picks this up next with no prior context.

For the original design rationale and a phase-by-phase build log, see
[`docs/AUTOSIM_DESIGN.md`](../docs/AUTOSIM_DESIGN.md). This document is the practical
companion: task-oriented "how do I…" recipes plus the non-obvious things that were
learned the hard way while building it.

---

## 1. What this thing is, in one paragraph

AutoSim is a **plain-Java, Pedro/Android-free re-implementation** of each autonomous
opmode's FSM. Each `...Sim.java` class runs its FSM against a fake clock and a coarse
kinematic follower instead of real hardware, recording every frame (pose, mechanism
states, follower status) into a `SimTrace`. The generator (`AutoSimGenerator`) turns the
registered traces into one **self-contained HTML file** (`dist/autosim.html`) with a
field view, scrub bar, live readouts, and action overlays (shoot rings, intake chevrons,
turret/hood gauges). Open it in any browser — no server needed.

This is a **dev/visualization tool only**. It never runs on the robot and never imports
Pedro or Android classes. It exists to let you "play back" an auto's logic — timing,
state transitions, path shapes, heading behavior — to sanity-check changes before
deploying to a robot.

---

## 2. Quick start

```bash
# Regenerate dist/autosim.html from the current source:
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :autosim:autosim

# (No java on PATH in this environment — must use the Android Studio JBR. See
#  ~/.claude memory "Headless JDK" for why.)
```

This prints a one-line summary per registered auto (path count, frame count, total
duration, event count, bounding box, in-bounds check) and writes
`autosim/dist/autosim.html`. Open that file in a browser, or — if working via the Claude
Preview MCP — it's served at the `autosim` preview server (port 8099); reload the page
to pick up a fresh build.

Optional: `./gradlew :autosim:autosim -Pauto=V3ClosePartner` makes a different auto the
*default* selection in the dropdown (all autos are always embedded regardless).

---

## 3. Module layout

```
autosim/
  build.gradle                 plain-java module; pulls in autoshared/ from TeamCode (see §5)
  AUTOSIM.md                   this file
  fields/decode.svg            field background image (embedded as base64 into the HTML)
  viewer/autosim.template.html the entire viewer UI — HTML/CSS/JS in one file, with
                                __TRACES_JSON__ / __DEFAULT_AUTO__ / __FIELD_IMAGE_DATAURI__ /
                                __GENERATED_AT__ placeholders
  dist/autosim.html            GENERATED — committed so reviewers can open it without building
  src/main/java/.../autosim/
    AutoRegistry.java           the dropdown's source of truth (see §6)
    AutoSimGenerator.java       entry point: builds every registered trace, stamps it into
                                 the template, writes dist/autosim.html
    autos/                      one *Sim.java per simulated auto (V3FarAutoSim,
                                 V3ClosePartnerSim, V3AutoSim)
    geom/                        Pt, Pose2d, Bezier (De Casteljau curve sampler)
    model/                       SimTrace and its sub-records (the JSON schema — see §8)
    sim/                         SimClock, SimStopwatch, SimFollower, and mechanism stubs
                                 (SimFlywheel, SimTurret, SimHood, SimIntake, SimFeeder)
    json/TraceWriter.java        dependency-free JSON serializer for SimTrace

TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoshared/
  V3FarAutoConfig.java          shared blue-native constants for V3FarAuto (Tier 1, §5)
  V3ClosePartnerConfig.java     same, for V3ClosePartner
  (V3Auto has no config yet — its Sim copy is fully self-contained, see §9.3)
```

---

## 4. Data flow (how a click becomes a pixel)

```
 ...Sim.java (run())
   │  drives a hand-copied FSM against SimClock/SimFollower/Sim* mechanism stubs,
   │  appending one Frame per 20ms tick + building sparse ActionEvents at the end
   ▼
 SimTrace  (meta, field, robot, paths[], frames[], events[])
   │
   ▼ TraceWriter.toJson(...)   — dependency-free, hand-rolled JSON
 AutoRegistry.AUTOS  { "V3FarAuto": ..., "V3ClosePartner": ..., "V3Auto": ... }
   │
   ▼ AutoSimGenerator
 viewer/autosim.template.html
   __TRACES_JSON__          → { "V3FarAuto": {...}, ... }   (all autos, always)
   __DEFAULT_AUTO__         → which key is selected on load
   __FIELD_IMAGE_DATAURI__  → base64 fields/decode.svg
   __GENERATED_AT__         → ISO timestamp
   │
   ▼
 dist/autosim.html   — fully standalone, open anywhere
```

The viewer is **pure client-side JS** operating on the embedded `TRACES` object. Nothing
is computed in the browser except interpolation/rendering — all simulation happens in
Java at generate-time.

---

## 5. The "Tier 1 shared config" pattern — keeping sim and reality in sync

The single biggest risk in this project is **drift**: the `...Sim.java` FSM is a
**hand-copied duplicate** of the real opmode's FSM (`TeamCode/.../autos/V3FarAuto.java`,
`V3ClosePartner.java`, `V3Auto.java`). Nothing enforces that they stay identical.

To reduce (not eliminate) this risk, **numeric tunables and field geometry** for
V3FarAuto and V3ClosePartner live in a shared, plain-Java config class under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoshared/`:

- `V3FarAutoConfig.java`
- `V3ClosePartnerConfig.java`

**Why this works:** `autosim/build.gradle` adds a second source directory —
`../TeamCode/src/main/java`, filtered to **only** `autosim/**` and `autoshared/**`. So
these config classes get compiled *both* into the Android/TeamCode build *and* into the
plain-Java `:autosim` module, from the same file. The real opmode and the `...Sim.java`
copy both read `V3FarAutoConfig.START_X`, `V3FarAutoConfig.FIXED_HOOD_DEG`, etc. — one
edit updates both.

**Hard constraint:** anything in `autoshared/` **must be plain Java** — no Pedro
(`Pose`, `BezierCurve`, `Follower`), no Android, no FTC SDK types. If you need a `Pose`
on the real-auto side, construct it from the config's raw doubles
(`new Pose(V3FarAutoConfig.START_X, V3FarAutoConfig.START_Y, ...)`).

**What's still duplicated and must be kept in sync by hand:**
- The FSM *structure* itself — state enum, transition logic, loop body. This is NOT
  shared. If you add/remove/reorder states in the real auto, you must mirror that in the
  `...Sim.java` by hand.
- Path *shapes* (control points) — these live in the `...Sim.java`'s path builders,
  mirroring the real auto's `BezierLine`/`BezierCurve` calls. Only get pulled into
  `autoshared` if they're simple named constants (like `LAST_LINE_X/Y`); ad-hoc inline
  control points are duplicated.
- **V3Auto has no `autoshared` config at all** — see §9.3 for why and what that means.

When asked to "keep the sim in sync" after a real-auto edit, the procedure is:
1. Diff the real auto's state enum/transitions/path builders against the `...Sim.java`.
2. If a changed constant exists in `autoshared/`, update it there (one edit, both sides
   pick it up on next build).
3. If a changed constant is *not* in `autoshared/` (i.e. it's inline in both files),
   update both copies by hand.
4. If the FSM *shape* changed (new state, new path, new heading mode), update
   `...Sim.java`'s enum/switch/path builders to match — see §9.

---

## 6. Adding a brand-new simulated auto (step by step)

This is the most common future task. Concretely, for a hypothetical `V4Auto`:

1. **Read the real opmode** (`TeamCode/.../autos/V4Auto.java`) end to end. Extract:
   - the state enum(s) (auto FSM, feed FSM, anything else)
   - start pose, all path control points, and **for each path, its heading-interpolation
     call** (`setConstantHeadingInterpolation`, `setTangentHeadingInterpolation`,
     `setReverseHeadingInterpolation`/`reverseHeadingInterpolation`, or
     `setLinearHeadingInterpolation`)
   - every timer/t-value threshold that gates a transition
   - the shot table / hood-flywheel targets if it has shot-on-the-move logic
   - feed-sequence timings (usually shared constants like `FEED_START_DELAY_SEC`)

2. **Decide on shared config.** If V4Auto reuses an existing alliance's geometry (e.g.
   it's a variant of V3ClosePartner), consider extending `V3ClosePartnerConfig` rather
   than duplicating. If it's wholly new, you can either create
   `autoshared/V4AutoConfig.java` (preferred — keeps future syncs cheap) **or** write a
   fully self-contained `...Sim.java` with literal constants, as was done for `V3AutoSim`
   when the priority was an additive, low-risk change (see §9.3 for that tradeoff). Use
   your judgment; ask if unsure.

3. **Write `autos/V4AutoSim.java`**, following the shape of `V3AutoSim.java` /
   `V3ClosePartnerSim.java`:
   - `private enum FeedState { IDLE, WAIT_BEFORE_INTAKE, RUN_INTAKE, DONE }` (copy as-is —
     this pattern is identical across all current autos)
   - `private enum AutoState { ... }` — **must match the real opmode's states 1:1**, in
     the same order, same names (makes future diffing trivial)
   - Collaborators block: `SimTrace`, `SimClock`, `SimFollower`, `SimIntake`,
     `SimFlywheel`, `SimTurret`, `SimHood`, `SimFeeder`, plus `SimStopwatch`s for any
     timers the real FSM uses
   - `public static SimTrace build() { return new V4AutoSim().run(); }`
   - `run()`: set `trace.meta.autoName`, `trace.field.goal`, robot footprint
     (`trace.robot.lengthIn/widthIn/wheelbaseOffsetIn` — match the real robot's physical
     dimensions, not a placeholder), initial mechanism state, kick off the first
     `follower.followPath(...)`, then the main loop:
     ```java
     while (true) {
         follower.update();
         updateShotControl();      // if shot-on-move logic exists
         flywheel.update(DT_SEC);
         updateFeedSequence();
         updateAutoState();
         turret.update();
         trace.frames.add(sampleFrame());
         if (autoState == AutoState.DONE && !follower.isBusy()) break;
         if (clock.millis() >= MAX_MS) break;
         clock.advance(DT_SEC);
     }
     trace.meta.totalMillis = clock.millis();
     buildEvents();
     return trace;
     ```
   - `updateAutoState()`: one `case` per `AutoState`, faithfully mirroring the real
     opmode's transition conditions (`follower.getCurrentTValue() >= X`,
     `!follower.isBusy()`, `stateTimer.seconds() >= Y`, etc.)
   - Path builders: for **every** path, call the appropriate heading-mode chained method
     (see §7) — don't silently default to `CONSTANT` unless the real path actually uses
     constant heading.
   - `buildEvents()`: copy verbatim from an existing `...Sim.java` — it derives SHOOT
     events from `shotTimes`/`shotPoses` and INTAKE spans from the frame stream. This is
     season-generic.
   - Static helpers `line(...)`, `curve(...)`, `tangentDeg(...)`, `normalize180(...)`:
     copy verbatim.

4. **Register it** in `AutoRegistry.java`:
   ```java
   import org.firstinspires.ftc.teamcode.autosim.autos.V4AutoSim;
   ...
   AUTOS.put("V4Auto", V4AutoSim::build);
   ```
   That's the entire integration point with the generator/viewer — insertion order =
   dropdown order, first entry = default.

5. **Regenerate and verify** (§10).

6. **Document deliberate simplifications** in the new file's class-level Javadoc (see
   `V3AutoSim`'s doc comment for the pattern) — anything the sim does NOT model exactly
   (e.g. a dynamic mid-path heading override) should be called out explicitly so a future
   reader doesn't mistake sim output for ground truth.

---

## 7. Heading interpolation — `HeadingMode` / `PathSpec`

`autosim/.../model/HeadingMode.java` is a 4-value enum mirroring Pedro's heading
interpolation modes:

| `HeadingMode` | Pedro equivalent | `PathSpec` chained setter |
|---|---|---|
| `CONSTANT` (default) | `setConstantHeadingInterpolation(deg)` | *(none — set `headingDeg` via constructor)* |
| `TANGENT` | `setTangentHeadingInterpolation()` | `.tangent()` |
| `REVERSE_TANGENT` | `reverseHeadingInterpolation()` / `setReverseHeadingInterpolation()` | `.reverseTangent()` |
| `LINEAR` | `setLinearHeadingInterpolation(startDeg, endDeg)` (with an end-t cutoff) | `.linear(startDeg, endDeg, endT)` |

**When writing a new path in a `...Sim.java`**, look at what the real auto's
`Path`/`PathChain` calls after building the `BezierLine`/`BezierCurve`, and chain the
matching setter onto the returned `PathSpec`:

```java
private PathSpec toFirstShot() {
    return line("toFirstShot", start, firstShot, /*unused for TANGENT*/ 0).tangent();
}

private PathSpec toGateOpenGate() {
    return curve("toGateOpenGate", Arrays.asList(p1, ctrl, p2), 0)
            .linear(GATE_OPEN_H_START, GATE_OPEN_H_END, GATE_OPEN_H_T);
}
```

For `CONSTANT` paths, `headingDeg` (passed to the `line()`/`curve()` constructor) is the
target heading the robot snaps toward via the in-place pivot (§7.1) — pass the real
value, not `0`, unless the mode is non-constant (in which case the constructor's
`headingDeg` argument is ignored and `0` is conventional).

### 7.1 "Turn, then move" and why durations grew

`SimFollower.followPath()` computes, at path-start, the heading the path will command at
arc-length 0 (`headingAt(spec, 0.0)`). If that differs from the robot's current heading,
the follower **pivots in place first** at `TURN_RATE_DEG_S = 270.0°/s`
(`SimFollower.java`), THEN begins translating. This pivot **consumes real simulated
time** — it's added to `tEndMs` and gates `isBusy()`. This is why every auto's total
duration grew when this was introduced (documented in
`AUTOSIM_DESIGN.md` §16): it's modeling Pedro's actual turn-then-drive behavior, not a
bug.

`getCurrentTValue()` returns `0` during the pivot phase and only starts climbing once
translation begins — so any FSM transition gated on a t-value (`getCurrentTValue() >=
0.9`, etc.) fires on real path progress, matching the real robot.

**Lesson learned the hard way:** `SimFollower.getPose()` must update `lastPose`
**unconditionally on every call** — including during the pivot phase and on every
mid-translation frame — not just when a path completes. The original implementation only
refreshed `lastPose` at move-completion, so an FSM that chains `followPath()` calls
back-to-back with no idle frame in between would compute the next pivot's `turnFromDeg`
from a *stale* heading, producing a visible single-frame heading snap. If you ever see a
heading "pop" at a path boundary in a new auto, this is the first thing to check — but
it should already be fixed for all autos since `SimFollower` is shared.

### 7.2 Adding a 5th heading mode (if a future season needs one)

1. Add the enum value to `HeadingMode.java`.
2. Add a chained setter to `PathSpec.java` (follow `.linear()`'s pattern for any extra
   parameters).
3. Add a `case` to `SimFollower.headingAt(PathSpec s, double d)`.
4. No viewer changes needed — heading is baked into per-frame poses before
   serialization; the viewer just draws the robot rotated to `frame.pose.headingDeg`.

---

## 8. The trace schema (`SimTrace` and friends)

This is the JSON contract between Java and the viewer. All classes are in
`model/`, all fields are public, `TraceWriter` serializes them by reflection-free hand
code (don't add a field without also handling it in `TraceWriter` if it needs to reach
the JSON — check `TraceWriter.java` when adding new `SimTrace` fields).

- **`Meta`**: `autoName`, `generatedAt`, `simDtMillis`, `startPose`, `totalMillis`,
  `effectProfile` (selects an `EFFECT_PROFILES` key in the viewer; defaults to
  `"decode"` when absent/unset).
- **`FieldSpec`**: `widthIn`/`heightIn` (144×144), `goal` (`Pt`), `backgroundImage`
  (path under `fields/`, embedded as base64 by the generator).
- **`RobotSpec`**: `lengthIn`, `widthIn`, `wheelbaseOffsetIn` — the physical footprint
  drawn around the tracked point. **Set these to the real robot's dimensions** in each
  `...Sim.java`'s `run()` — they're also live-editable in the viewer sidebar for
  what-if exploration, but the generated default should be accurate.
- **`PathSpec`**: `id`, `kind` (`"LINE"`/`"CURVE"`), `controlPoints`, `polyline`
  (pre-sampled, `CURVE_SEGMENTS=40` points for curves), `headingDeg` + `headingMode` +
  `headingStartDeg/headingEndDeg/headingEndT` (§7), `tStartMs`/`tEndMs` (stamped by
  `SimFollower.followPath()`).
- **`Frame`**: one per 20ms tick — `tMs`, `pose` (`Pose2d` = x/y/headingDeg),
  `autoState`, `feedState` (both as `.name()` strings, shown verbatim in the viewer's
  "NOW" panel), `followerBusy`, `flywheelTargetRadS`/`flywheelActualRadS`, `turretDeg`,
  `hoodDeg`, `intakePower`.
- **`ActionEvent`**: sparse overlay markers — `tMs`, `category` (`Category` enum),
  `label`, `durationMs`, optional `pose`. Built post-hoc in `buildEvents()` from the
  frame stream + recorded shot times (never a guessed constant — see §11.1).

If you add a new `Frame` or `PathSpec` field that the viewer needs to read, you must:
1. Add the field to the Java model class.
2. Add it to `TraceWriter.toJson(...)` (or the relevant sub-writer).
3. Read it in the viewer JS (it'll appear in `TRACE.frames[i].yourField`).

---

## 9. Per-auto notes

### 9.1 V3FarAutoSim
Fully Tier-1: all geometry/timing in `V3FarAutoConfig`. 10-state FSM with a repeating
4x intake/shoot cycle (alternating high/low intake Y via `useHighIntakeCycle()`).
Constant heading throughout (`DRIVE_HEADING_DEG=180`, `INTAKE_HEADING_DEG=200`).

### 9.2 V3ClosePartnerSim
Tier-1 via `V3ClosePartnerConfig`. 17-state-ish FSM with a gate-intake sub-cycle. Uses
all four heading modes: `.tangent()` on the opening drive, `.reverseTangent()` on every
"drive back to shoot" path, `.linear(GATE_OPEN_HEADING_START_DEG,
GATE_OPEN_HEADING_END_DEG, GATE_OPEN_HEADING_T)` on the gate-opening path.

### 9.3 V3AutoSim — the self-contained one
**No `autoshared` config** — all 25+ constants (start pose, shot table, timing
thresholds, path control points) are literals inside `V3AutoSim.java`, copied directly
from `TeamCode/.../autos/V3Auto.java`. This was a deliberate choice when the auto was
added (see `AUTOSIM_DESIGN.md` history): creating a shared config or touching the real
`V3Auto.java` was judged higher-risk than an additive, fully-isolated sim copy.

**Consequence:** if `V3Auto.java`'s constants change, `V3AutoSim.java` will NOT pick
them up automatically — someone has to re-diff and hand-update it. If you're doing
substantial future work on V3Auto, consider promoting its constants to
`autoshared/V3AutoConfig.java` (following `V3FarAutoConfig`'s pattern) as a first step —
that converts this from "hand-sync forever" to "edit once."

**Known, documented simplifications** (in the file's class Javadoc — re-state here so
they're discoverable without opening the file):
1. Shot-on-the-move compensation is simplified to straight-line goal tracking + the
   literal shot table (same simplification as `V3ClosePartnerSim`).
2. `toLastLine` is modeled as `TANGENT` for its entire length; the real auto switches to
   `CONSTANT 180°` after `t > 0.2`. Not modeled.
3. `backToShootFromLastLine` is modeled as `REVERSE_TANGENT` for its entire length; the
   real auto switches to `CONSTANT -90°` after `t > 0.8`. Not modeled.

These cause small heading discrepancies near the end of the last-line cycle only — not
worth fixing unless someone's specifically debugging that cycle's heading behavior. If
you do fix it, that's exactly the kind of thing `PathSpec`/`HeadingMode` would need a
"mode switch partway through a path" feature for — currently each `PathSpec` has exactly
one mode for its whole length.

---

## 10. Verifying changes

1. **Build**: `JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
   ./gradlew :autosim:autosim`. Check the per-auto summary line:
   - `inBounds=true` — every polyline point must be within `[0, 144]` on both axes.
   - Sanity-check `total=...ms` against the previous build if you changed timing.
   - `events=...` should be non-zero and roughly match the number of shots + intake
     spans you'd expect.
2. **Visual check** (via Claude Preview MCP, if available — server `autosim` on port
   8099):
   ```js
   // reload to pick up the new dist/autosim.html, then:
   selectAuto('YourAuto'); play.t = 0;
   poseAt(0);              // sanity-check start pose
   ```
   Useful invariant checks:
   ```js
   // max per-frame heading step should be <= TURN_RATE_DEG_S * (simDtMillis/1000)
   // = 270 * 0.02 = 5.4 deg/frame for the current follower
   let maxStep=0, prev=poseAt(0).headingDeg;
   for(let t=20;t<=TOTAL;t+=20){
     const h=poseAt(t).headingDeg;
     let d=Math.abs(h-prev); if(d>180)d=360-d;
     maxStep=Math.max(maxStep,d); prev=h;
   }
   maxStep; // should be ~5.4, not e.g. 20
   ```
   Then `preview_screenshot` at a few `play.t` values spanning each cycle to eyeball
   path shapes, robot orientation during pivots, and shot/intake overlays.
3. **Send the result**: `dist/autosim.html` is a single file — `SendUserFile` it after
   regenerating so the user can open it directly.

---

## 11. Modifying or adding action overlays (the viewer's visual language)

All overlay rendering lives in `viewer/autosim.template.html`. The model side
(`Category`, `ActionEvent`) is season-agnostic by design — the viewer's
`EFFECT_PROFILES` table is where a category becomes a specific visual.

### 11.1 Adding events from a `...Sim.java`
`buildEvents()` is the only place `ActionEvent`s are created, and the rule is: **every
duration must come from something the sim actually measured** — either the FSM's own
timer window (e.g. `FEED_START_DELAY_SEC + FEED_TOTAL_TIME_SEC` for a shot) or a
contiguous span in the recorded `frames[]` (e.g. the INTAKE-power-on span). Never invent
a duration constant in `buildEvents()` — if you need a new event type, first make sure
the underlying mechanism state is in `Frame`/recorded somewhere, then derive the
duration from that.

### 11.2 Changing how a category looks
Edit `EFFECT_PROFILES.decode` in the template (around line 148):
```js
SHOOT:  { color:"#ff7a18", glyph:"▲", label:"shoot",  effect:"RING", ray:true },
```
`effect` must be one of the primitives the draw core understands: `NONE`, `RING`
(`fxRing`), `CHEVRONS` (`fxChevrons`), `GAUGE` (`fxGauge`). `ray:true` additionally draws
the ball-flight animation toward the goal (see §11.3).

### 11.3 The ball-flight / `BALL_FLIGHT` mechanism
`BALL_FLIGHT = 900` (ms, near top of the template) is how long an animated artifact
takes to fly from the robot to the goal after a SHOOT event fires. Two things were
tuned together so balls **finish their flight even if the robot has moved on / the
SHOOT window ended**:
- `drawEffects(now)`'s cull check: `const tail = (e.effect==="RING" && e.ray) ?
  BALL_FLIGHT : 200; if (age < 0 || age > dur + tail) continue;` — extends the event's
  visible lifetime past `durationMs` for ray-type events specifically.
- `fxRing`'s muzzle-flash alpha: `(age<=dur?0.9:0)` — the flash itself still respects the
  original window; only the traveling ball persists into the extended tail.

If you add a new ray-type effect, reuse `BALL_FLIGHT` for its tail rather than inventing
a new constant, unless the new artifact genuinely has a different flight time.

### 11.4 A brand-new category (future season)
1. Add the enum value to `Category.java` (there are already unused spares: `SLIDE`,
   `CLAW`, `ARM`, `EXTEND` — reuse one of these first if it semantically fits, to avoid
   a schema change).
2. Add an entry to `EFFECT_PROFILES.decode` (or a new profile keyed by
   `trace.meta.effectProfile` if it's a different season/robot entirely — the viewer
   already supports multiple profiles, just none besides `"decode"` exist yet).
3. If it needs a new primitive (not RING/CHEVRONS/GAUGE/NONE), add a new `fxXxx(...)`
   function and a `case` in `drawEffects`.

---

## 12. Viewer internals cheat-sheet

Everything is in `viewer/autosim.template.html` (one file, ~660 lines: CSS, HTML, JS).
Key globals/functions, useful when scripting via `preview_eval`:

- `TRACES` — `{ autoName: SimTrace, ... }`, all embedded traces.
- `AUTO_NAMES` — `Object.keys(TRACES)`, dropdown order.
- `TRACE` / `PROFILE` / `EVENTS` / `FIELD` / `ROBOT` / `FRAMES` / `TOTAL` — bound to the
  *currently selected* auto; reassigned by `bindAuto(name)`.
- `selectAuto(name)` — switches the active auto (updates the `<select>`, rebinds, resets
  playback).
- `play.t` — current playback time in ms; set directly to scrub.
- `poseAt(t)` — interpolates `{x, y, headingDeg}` from `FRAMES` at time `t` (used for
  both rendering and scripted checks).
- `drawEffects(now)` — renders all overlay effects active at time `now`.
- Video export button — renders an annotated 640p capture of the full playback; see
  `AUTOSIM_DESIGN.md` §15 ("640p annotated video export") for how it works if you need
  to touch it.

The robot footprint sliders (`length`/`width`/`wheelbase offset` in the sidebar) are
viewer-only overrides initialized from `trace.robot.*` — they don't affect the
underlying trace, just the drawn rectangle (useful for what-if footprint exploration
without rebuilding).

---

## 13. Things that look like bugs but aren't

- **Auto durations are longer than you "expect"** — this is the in-place pivot (§7.1)
  consuming time at every heading change. Compare against the previous committed
  `dist/autosim.html`'s summary line if you need a before/after.
- **`getCurrentTValue()` returns 0 for a while after a path starts** — that's the pivot
  phase; FSM transitions gated on t-value are correctly waiting for it.
- **A SHOOT ring's ball keeps animating after the robot has visibly left the shooting
  spot** — intentional (§11.3), models the real flight time of a launched artifact.
- **V3AutoSim's heading near the end of the last-line cycle doesn't exactly match
  V3Auto** — documented simplification (§9.3), not a regression.

---

## 14. If you're an AI agent picking this up cold

Recommended reading order: this file → `docs/AUTOSIM_DESIGN.md` §0 (core idea) and §16
(most recent fidelity work) → the `...Sim.java` most similar to whatever you're changing
→ `SimFollower.java` (the shared kinematic/heading core — small, ~210 lines, read it in
full).

Standing constraints (also in root `CLAUDE.md`, repeated here because they're easy to
violate while focused on `:autosim`):
- All real robot code lives under `TeamCode/`; `FtcRobotController/` is off-limits.
- `:autosim` and `autoshared/` must stay plain Java — no Pedro/Android imports, ever.
- Don't run gradle builds unless asked, except the `:autosim:autosim` regen task via the
  JBR `JAVA_HOME`, which is expected workflow for this module.
- Never `git add`/`commit`/`push` unless explicitly asked.
