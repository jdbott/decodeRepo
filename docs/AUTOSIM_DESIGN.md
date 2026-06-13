# AutoSim Visualizer — Design Document

A selectively-launchable visual simulator/replay tool that renders an FTC autonomous
routine as a top-down animated field view: the robot driving its path, with action
events (shoot / intake / feed / hood / turret) surfaced on a scrubable timeline.

- **Proof-of-concept auto:** `V3FarAuto` ("Far Auto Simple").
- **Hard constraint:** the existing codebase is **not modified**. All new work is new
  files. A **copy** of the auto's FSM (`V3FarAutoSim`) is the test bed; the original
  `V3FarAuto.java` stays byte-for-byte untouched.
- **Relationship to other docs:** this is the *visual front-end* of opportunity **#1
  (`AutoSim`, headless sim/replay)** in [AI_OPPORTUNITIES.md](AI_OPPORTUNITIES.md). It
  is designed to share one simulation core with that headless harness rather than grow
  a second engine. It also dovetails with the eventual auto-base refactor in
  [AUTO_BASE_DESIGN.md](AUTO_BASE_DESIGN.md) (see "Future convergence" below).

> **Status: design only.** No code is written yet. The end of this doc lists the
> decisions that need the captain's input before Phase 0 begins.
>
> **Decisions locked (2026-06-12):** front-end = **Web (HTML5 Canvas)**; build order =
> **predictive sim first** (the `SimFollower` kinematic model, so a run can be watched
> with no robot; replay drops to the Phase 4 stretch); git = **commit + push current
> changes to `master`**. Q3–Q6, Q8, Q9 still open below.

---

## 0. The core architectural idea

Split the tool into two layers that talk through a **plain JSON trace file**:

```
  ┌─────────────────────────────┐        trace.json        ┌──────────────────────────┐
  │  SIM CORE  (pure-Java, JVM) │  ───────────────────────▶ │  VISUALIZER (front-end)  │
  │  • V3FarAutoSim (FSM copy)  │   frames + events +       │  • field + path render   │
  │  • SimFollower (kinematics) │   paths + meta            │  • playback + scrub       │
  │  • virtual clock + stubs    │                           │  • action overlays        │
  └─────────────────────────────┘                           └──────────────────────────┘
        runs off-robot,                                          opens on demand,
     no Android, no hardware                                  reads the trace, draws it
```

Why a file seam in the middle:

- The **sim core** runs on a desktop JVM (no Android, no `hardwareMap`, no real
  follower), so it can run in milliseconds, in CI, and feed the headless assertions of
  #1 — the *same* run that produces a `trace.json` for the eye can produce a pass/fail
  for CI.
- The **visualizer** never imports robot code. It only knows the trace schema. That
  keeps rendering decisions (and any web/JS) fully decoupled from the FTC/Android build,
  and means the *same viewer* can render either a **simulated** run or a **replayed
  real** run (a robot log exported in the same schema). One viewer, two data sources.

This separation is the single most important decision in the design; everything below
follows from it.

---

## 1. Launcher — how the user invokes it

**Not** a Driver Station menu item. The Driver Station runs on the robot; a developer
sim belongs on the laptop, on demand, off the field.

- **Primary (simulate):** a Gradle task, e.g. `./gradlew autosim -Pauto=V3FarAutoSim`,
  that (a) runs the chosen sim copy, (b) writes `docs/autosim/trace.json`, and (c) opens
  the viewer (browser tab or window). An Android Studio **run configuration** wrapping
  the same entry point gives a one-click "▶ Run AutoSim" for students who never touch a
  terminal.
- **Per-auto, global viewer:** the *viewer* is one tool; *which auto* it shows is chosen
  at sim time. Simulatable copies register themselves in a small `AutoRegistry`, so the
  launcher (and later a dropdown in the viewer) can list every auto that has a `…Sim`
  copy. Today that registry has one entry (`V3FarAutoSim`); each future auto that opts in
  adds one line.
- **Replay path:** the on-robot logging copy writes a `trace.json` to the robot's
  storage during a real run; you pull it (adb / file transfer) and open it in the *same*
  viewer. No separate tool.

**Decision flagged:** Gradle task vs. run config vs. double-click HTML as the canonical
launch — not blocking for the design, but pick one "blessed" path so the onboarding doc
can name it. (See Open Questions.)

---

## 2. Visual canvas — field, coordinates, path source

**Coordinate system.** Render in **Pedro field inches**: a 144″ × 144″ square, origin at
the **bottom-left**, +X right, +Y up, heading in **degrees CCW** (0° = +X). This matches
the auto's own pose literals — `START (64.1, 6.75, 180°)` bottom-center facing −X, goal
near `(5, 139)` top-left. Screen mapping is a single affine transform:
`px = x * (canvasPx / 144)`, and **flip Y** (screen Y grows downward). All rendering
works in field units and converts once at draw time, so the same trace renders at any
canvas size. *(Origin/orientation is an assumption to confirm — Open Question Q4.)*

**Alliance.** Traces are stored **blue-native**; the viewer has a Blue/Red toggle that
applies the same mirror `AllianceMirror` uses (`x → 144 − x`, heading negated), so one
trace shows both alliances without re-simulating. This mirrors how the auto itself
treats poses.

**Path data source — emit from the sim, don't re-declare.** The path polylines come
from the **sim copy**, which builds the exact same Pedro `BezierLine`/`BezierCurve`
objects the real auto builds (`buildBasePaths()`, `buildCyclePathsForCurrentCycle()`,
plus the dynamic retreat path) and samples each curve to a polyline (e.g. 30 points via
the curve's parametric form). This makes the **code the single source of truth** for
geometry — the viewer never hand-copies coordinates, so a path edit in the copy shows up
in the picture automatically. Each emitted path carries its control points too, so the
viewer can optionally draw the Bezier handles for debugging.

**On the canvas:**

- **Field backdrop:** 144″ grid, alliance-tinted, a goal reticle at the (mirrored)
  target, and start-pose marker.
- **Path:** full route as a polyline. **Traversed** portion solid/bright, **upcoming**
  portion dimmed/dashed, with sparse direction arrows. Each FSM drive segment is its own
  styled sub-path so you can see where one `followPath` ends and the next begins.
- **Robot:** an oriented ~18″ rounded square with a heading nose. Position/heading come
  from the current frame.
- **Turret:** a short wedge from robot center at the turret's robot-relative angle. On a
  shoot event, a dashed **shot ray** extends from the turret toward the goal reticle.

---

## 3. Action indicators — the visual language

The goal is to convey **what category** of action fires and **when**, without modeling
the mechanism literally. Three coordinated channels:

**(a) Category system (season-generic).** A small enum, instantiated for DECODE but
designed so a future season drops new members in without touching the viewer's core:

| Category | Color | Glyph | DECODE meaning | Fires on |
|---|---|---|---|---|
| `DRIVE`  | slate  | →   | following a path | `follower.followPath(...)` |
| `SHOOT`  | orange | ▲   | a shot leaves the turret | feed reaches the release point |
| `FEED`   | amber  | ⮈   | feeder arm/clutch actuates | `feeder.armShoot/clutchOut` |
| `INTAKE` | green  | ⟳   | intake running to collect | `intake.setPower(>0)` |
| `HOOD`   | violet | ◠   | hood angle change | `hood.setAngle(...)` |
| `TURRET` | cyan   | ⟲   | commanded turret slew | `turret.setAngle(...)` (debounced) |
| `WAIT`   | grey   | ⏱   | dwell / delay state | timer-gated states |

Generic spares (`SLIDE`, `CLAW`, `ARM`, …) live in the enum unused, so next season's
mechanisms map onto existing rendering with zero viewer changes — directly serving the
future-season-generality goal.

**(b) Timeline (bottom rail) — the "when".**
- The track background is segmented into **color bands by `AutoState`**, so the macro
  phase structure (wait → shoot → drive → intake → shoot → …) reads at a glance.
- **Event pins** sit above the track at each event's timestamp, colored/glyphed by
  category. Clicking a pin scrubs to it.
- A draggable **playhead** marks current time.

**(c) On-field pulses — the "where + what", at the moment it fires.**
- `SHOOT`: an expanding orange ring + brief muzzle flash at the turret, fading over
  ~400 ms of sim-time.
- `INTAKE`: green converging chevrons at the intake side while active (steady, not a
  one-shot, since intake runs over a span).
- `FEED`: a small amber flash at the feeder.
- `HOOD`/`TURRET`: a thin arc/gauge tick near the robot.

Events carry a `durationMs`, so one-shot pulses (shoot) and spans (intake-while-driving)
are both representable. A **legend** in the left rail keys color/glyph → category.

---

## 4. Playback controls & timing

- **Transport:** play / pause, **scrub** (drag the playhead), **speed multiplier**
  (0.25× / 0.5× / 1× / 2× / 4×), single-frame step, and **jump-to-next-event**.
- **Timing model — real sim time, normalizable.** Default is **real (sim-derived)
  timing**: the sim stamps frames in real milliseconds, so the genuine dwell is visible
  — the 3 s first-shot delay, the 1.0 s feed window, the 0.25 s intake reverse all read
  truthfully, which is the whole point of validating *timing* off-robot. A **"compress
  dead time"** toggle collapses long waits to a fixed cap for quick review, for when you
  only care about the geometry/order. Real timing is primary; normalized is the
  convenience mode.
- The playhead reads the **dense frame stream** (fixed `dt`, e.g. 20 ms) by binary-search
  on `tMs`, so scrubbing is smooth and O(log n).

---

## 5. Integration with the FSM — reading state without touching V3FarAuto

`V3FarAuto` is a `LinearOpMode`: its logic lives inside one blocking `runOpMode()` with
`while (opModeIsActive())`, real `hardwareMap` lookups, `Constants.createFollower(...)`,
`telemetry`, and `ElapsedTime` (wall clock). You cannot run that off-robot as-is, and
reflection over it is brittle (it blocks, sleeps, and needs an Android `Context` for
`AllianceStore`). So we don't try.

**Chosen strategy: a decoupled copy driven by injected sim collaborators.** Create
`V3FarAutoSim` — a new class that **reproduces the two FSMs verbatim** (`AutoState`,
`FeedState`, the constants, `updateAutoState()`, `updateFeedSequence()`, the path
builders) but replaces the three sources of hardware coupling with seams:

| Real auto depends on | Sim copy uses instead |
|---|---|
| `LinearOpMode` lifecycle (`opModeIsActive`, `waitForStart`, init loop) | a `SimRunner` loop that ticks a **virtual clock** at fixed `dt` until `DONE` or a time cap |
| `ElapsedTime` (`stateTimer`, `feedTimer`, `reverseTimer`) | a `SimClock`-backed stopwatch reading virtual time (so the 3 s delay etc. are simulated, not slept) |
| `Follower` (Pedro + Pinpoint) | **`SimFollower`** — kinematic model (below) implementing `followPath / update / isBusy / getPose` against the sampled path |
| `Intake/Hood/Feeder/Flywheel/Turret` | **recording stubs** with the same method signatures that capture each command into the trace instead of driving motors |
| `AllianceStore`/`AutoStartStore` (Android `Context`) | a plain `alliance` flag passed in |
| `telemetry` | a no-op (or a trace annotation sink) |

Because every collaborator is injected, the FSM body is **pure decision logic over
those interfaces** — exactly the testable substrate #1 wants.

**`SimFollower` fidelity (v1, deliberately coarse).** Point-mass kinematics seeded from
the real measured constants in
[Constants.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java)
(`xVelocity 73.8`, `yVelocity 60.8`, `forwardZeroPowerAcceleration −33.7`, `mass 10.46`):
on `followPath`, compute path arc-length, estimate a traversal time from an average
cruise velocity (with a simple trapezoidal accel/decel and the path's `brakingStrength`
as a decel hint), and interpolate pose along the sampled polyline as virtual time
advances; `isBusy()` is true until arrival (matching the auto's `!follower.isBusy()`
transitions). This is enough to exercise **FSM logic and ordering/timing** — it does
**not** claim to predict real pose error (Open Question Q3). The model is one swappable
class, so a higher-fidelity follower can replace it later without touching the FSM copy.

**Trace emission.** The recording stubs and the runner write events/frames as the sim
ticks. Mechanism calls → `events[]`; every tick → a `frames[]` sample; the path builders
→ `paths[]`. No special instrumentation of *logic* is needed — we observe at the seam.

**The honest cost.** This copy **duplicates the FSM**, so today an edit to the real auto
must be mirrored into the copy (and vice-versa) to stay representative. That is the
accepted price of "don't modify the original," and it's bounded (one auto, ~2 small
FSMs). It collapses later: once [AUTO_BASE_DESIGN.md](AUTO_BASE_DESIGN.md) extracts the
FSM body to talk to a `RobotIO`-style seam, the *same* body runs on both the real
hardware adapter and the sim adapter, and the copy disappears. Building the sim seam now
is a forcing function for that interface. *(If the captain prefers, we can lead with the
**replay** path instead — see Q2 — which needs only a logging line in a copy and zero
FSM duplication, at the cost of needing the robot to generate data.)*

---

## 6. Tech stack

**Recommendation: pure-Java sim core + a zero-build web visualizer (HTML5 Canvas).**

- **Sim core:** plain Java on the desktop JVM, in a new source set / small Gradle module
  (sibling to the headless AutoSim of #1). No Android dependency — it imports only the
  Pedro geometry types (for Bezier sampling) and the copied FSM. Emits `trace.json`.
- **Visualizer:** a single static `autosim.html` + a small JS file using Canvas. No npm,
  no build step, no framework — open the file, it `fetch`es the adjacent `trace.json`.

Why web over the alternatives, for *this* team:

- **Richest visuals for the least code** — Canvas/SVG make the timeline bands, pulse
  rings, and shot rays straightforward; the same UI in Swing is far more layout pain.
- **Maintainability under turnover** — every student can edit HTML/CSS/JS; far fewer can
  wrangle Swing `paintComponent`. Lower ramp = better fit for a 9-person, high-turnover
  team.
- **Trivially shareable** — a `trace.json` + the HTML is a self-contained artifact you
  can drop in a PR, open on any laptop, or host as a static page.
- **Clean decoupling** — the viewer can't accidentally depend on robot code; the Android
  build never has to know the viewer exists.

**Alternatives considered (and why not, for v1):**

- **Swing/JavaFX desktop window** — single-language (all Java), no browser. Viable
  fallback if the captain wants zero web tech; driven by the *same* sim core. Costs more
  UI effort and is harder for students to extend. *(Captain decision — Q1.)*
- **Piggyback FtcDashboard / bylazar Panels field overlay** — already dependencies, draw
  a field canvas, and are great for **live on-robot** telemetry. But they're oriented to
  a running opmode, not off-robot scrub/replay of a pre-computed trace, and bend awkwardly
  to a timeline with action bands. Better as a *future live-mirror* mode than the
  off-robot authoring tool this is.
- **Standalone desktop app (Compose/Electron/etc.)** — overkill; adds a toolchain a
  rookie can't maintain.

---

## 7. Wireframe (popup UI)

```
┌───────────────────────────────────────────────────────────────────────────────┐
│  AutoSim — V3FarAuto         [Blue �҂ Red]   ⏮  ▶/⏸  ⏭     speed[1× ▾]   ⤓json   │ top bar
├───────────────┬───────────────────────────────────────────────────────────────┤
│  LEGEND       │                                                                 │
│  ▲ shoot      │                  ◎ goal                                         │
│  ⟳ intake     │                                                                 │
│  ⮈ feed       │            ·····(upcoming path, dashed)·····                    │
│  ◠ hood       │          ╭─────────────────╮                                    │
│  ⟲ turret     │          │   ▭▶ robot      │  ← oriented, heading nose          │ field
│  → drive      │          │    \ turret wedge│     turret wedge + shot ray       │ canvas
│  ⏱ wait       │   ───────●  (traversed path, solid)                            │ (inches,
│               │       start                                                     │  Y-up)
│  STATE:       │                                                                 │
│  SHOOT_FIRST  │                                                                 │
│  t = 03.41 s  │                                                                 │
│  FW 445/438   │                                                                 │
├───────────────┴───────────────────────────────────────────────────────────────┤
│ WAIT │■■ SHOOT ■│ DRIVE │ SHOOT │■ DRIVE ■│ INTAKE │ DRIVE │ SHOOT │ … (state bands)│
│      ▲          ⮈        ▲                 ⟳         →                ▲           │ timeline
│ 0s   ┃ (playhead)                                                          12.0s  │
└───────────────────────────────────────────────────────────────────────────────┘
```

- **Top bar:** auto name, alliance toggle, transport, speed, export/download.
- **Left rail:** category legend + live state chip, sim-time, flywheel target/actual.
- **Center:** the field canvas (the picture in §2–3).
- **Bottom:** timeline — `AutoState` color bands + category event pins + draggable
  playhead.

---

## 8. Data model (the trace schema)

One JSON document per run. **Dense `frames`** for smooth playback, **sparse `events`**
for action overlays, **`paths`** for geometry, **`meta`** for context. Schema-versioned
so the viewer can evolve.

```jsonc
{
  "schemaVersion": 1,
  "meta": {
    "autoName": "V3FarAuto",
    "alliance": "BLUE",            // trace stored blue-native; viewer mirrors for red
    "simDtMillis": 20,
    "totalMillis": 12000,
    "source": "SIM",              // or "REPLAY" for a real-robot log
    "generatedAt": "2026-06-12T...",
    "follower": { "model": "pointmass-v1", "xVel": 73.8, "yVel": 60.8 }
  },
  "field": { "widthIn": 144, "heightIn": 144, "goal": { "x": 5, "y": 139 } },

  "paths": [
    {
      "id": "toLastLine",
      "kind": "CURVE",                       // LINE | CURVE
      "controlPoints": [ {"x":64.1,"y":12.25}, {"x":45,"y":38}, {"x":12.5,"y":38} ],
      "polyline":      [ {"x":64.1,"y":12.25}, /* …sampled… */ {"x":12.5,"y":38} ],
      "headingDeg": 180,
      "tStartMs": 3600, "tEndMs": 5200       // when this segment is driven
    }
    // … fromLastLineToShoot2, toIntake1, backup…, backToShoot2, retreat …
  ],

  "frames": [                                // one per dt tick
    { "tMs": 0,    "pose": {"x":64.1,"y":6.75,"headingDeg":180},
      "autoState": "WAIT_INITIAL_DELAY", "feedState": "IDLE", "followerBusy": false,
      "flywheelTargetRadS": 445, "flywheelActualRadS": 120,
      "turretDeg": 113, "hoodDeg": 53.5, "intakePower": 0 }
    // … ~600 frames for a 12 s run at 20 ms …
  ],

  "events": [                                // sparse, at transitions
    { "tMs": 3000, "category": "SHOOT",  "label": "first shot",  "durationMs": 400,
      "pose": {"x":64.1,"y":6.75,"headingDeg":180} },
    { "tMs": 3600, "category": "DRIVE",  "label": "toLastLine",  "durationMs": 1600 },
    { "tMs": 6800, "category": "INTAKE", "label": "intake cycle 0","durationMs": 1500 }
    // …
  ]
}
```

Java-side mirror (sim core), sketched:

- `SimTrace { Meta meta; Field field; List<PathSpec> paths; List<Frame> frames; List<ActionEvent> events; }`
- `Frame { long tMs; Pose2d pose; String autoState; String feedState; boolean followerBusy; double flywheelTargetRadS, flywheelActualRadS, turretDeg, hoodDeg, intakePower; }`
- `ActionEvent { long tMs; Category category; String label; long durationMs; Pose2d pose; }`
- `PathSpec { String id; PathKind kind; List<Pt> controlPoints, polyline; double headingDeg; long tStartMs, tEndMs; }`
- `enum Category { DRIVE, SHOOT, FEED, INTAKE, HOOD, TURRET, WAIT, /* generic spares */ SLIDE, CLAW, ARM }`

`Pose2d`/`Pt` are sim-local value types (not Pedro's `Pose`) so the schema layer carries
no Android/Pedro coupling.

---

## 9. Integration strategy for V3FarAuto (proof of concept), concretely

1. **`V3FarAutoSim`** — copy the FSM: `AutoState`, `FeedState`, all `…_DEG/_SEC`
   constants, `updateAutoState()`, `updateFeedSequence()`, `buildBasePaths()`,
   `buildCyclePathsForCurrentCycle()`, the dynamic retreat path. Strip
   `LinearOpMode`/`telemetry`/`hardwareMap`. Constructor takes
   `(SimFollower, sim hardware stubs, SimClock, alliance)`.
2. **`SimFollower`** — implements the follower surface the FSM uses
   (`followPath(path, holdEnd)`, `update()`, `isBusy()`, `getPose()`, `setStartingPose`,
   `setMaxPower`). Samples each Pedro curve to a polyline; advances pose on the virtual
   clock per §5.
3. **Recording stubs** — `SimIntake/SimHood/SimFeeder/SimFlywheel/SimTurret` with the
   real method names (`setPower`, `setAngle`, `armShoot`, `clutchOut`,
   `setTargetVelocity`, …); each call appends to the event/state buffers. A simple
   first-order flywheel spin-up model gives a believable `flywheelActualRadS`.
4. **`SimRunner`** — owns the `SimClock`; loops `flywheel.update → turret → feed → auto`
   at `dt`, samples a `Frame` each tick, stops at `DONE` (or a 30 s cap); serializes
   `SimTrace` → `docs/autosim/trace.json`.
5. **Viewer** — `autosim.html` loads that trace.

The original `V3FarAuto.java` is never imported, never edited.

**Future convergence:** when AUTO_BASE_DESIGN.md lands its `RobotIO`/helper seam, fold
the real auto and `V3FarAutoSim` onto one FSM body with two adapters (real vs. sim).
Until then, keep the copy honest with a small checklist note in the file header.

---

## 10. Phased build plan

- **Phase 0 — Prereqs & scaffold.** Commit/push the current uncommitted changes
  (`HANDOFF.md` + `docs/`) per the workflow rule (master vs. PR — Q7). Resolve Q1
  (stack) and Q2 (sim-vs-replay). Create the `autosim` source set/module, the trace
  schema types, and an empty `autosim.html` that draws a 144″ field grid. *No behavior.*
- **Phase 1 — Render the path (static).** `V3FarAutoSim` builds + samples the paths and
  emits a minimal trace (paths + start pose, no time). Viewer draws field + full path +
  robot at start + goal reticle. **Exit:** the route looks right vs. the field; proves
  coordinate mapping and the file seam end-to-end.
- **Phase 2 — Play back states (kinematic).** Add `SimClock` + `SimFollower` + the
  `SimRunner` loop; emit dense `frames`. Viewer gains play/pause/scrub/speed; the robot
  drives the path; state chip + timeline state-bands update. **Exit:** a full run plays
  start→DONE with plausible timing (3 s delay, feed windows, cycle count = 4).
- **Phase 3 — Action overlays.** Recording stubs emit `events`; viewer renders category
  color bands, event pins, on-field pulses (shoot ring, intake chevrons, feed/hood/turret
  ticks), turret wedge + shot ray, and the legend. **Exit:** every shoot/intake/feed is
  visible at the right time and place; the visual language reads clearly.
- **Phase 4 (optional/stretch).** (a) **Replay** mode — a logging copy writes the same
  schema on-robot; pull + open in the viewer for real-run fidelity. (b) **Multi-auto** —
  registry-driven dropdown. (c) **Assertion overlay** — surface #1's invariants
  (e.g. "ends within 4″ of shoot pose", "fires 6×") as pass/fail badges on the timeline,
  making the sim a regression gate, not just a picture.

---

## 11. Open questions / decisions for the captain (before Phase 0)

1. **Tech stack (gating). — DECIDED: Web visualizer (HTML5 Canvas).** Pure-Java sim core
   emits `trace.json`; a static HTML+Canvas page renders it.
2. **Simulate vs. replay first. — DECIDED: predictive sim first.** Build the
   `SimFollower` kinematic model up front so a run can be watched *before* driving, no
   robot required. Replay of real-robot logs moves to the Phase 4 stretch (it reuses the
   same viewer + schema, so nothing is lost).
3. **Follower fidelity for v1.** Is the coarse point-mass / arc-length-timing model
   acceptable for v1 — i.e. v1 validates **logic, ordering, and timing**, not exact pose
   error? (Matches #1's pragmatism.)
4. **Coordinate convention.** Confirm field 144″×144″, origin bottom-left, +X right, +Y
   up, heading CCW degrees, and that we render blue-native + mirror for red (matching
   `AllianceMirror`).
5. **Path source.** Emit sampled polylines from the sim copy (single source of truth —
   *recommended*) vs. re-declaring paths in the viewer.
6. **Where it lives & launch ergonomics.** New Gradle subproject vs. a `src/test/java`
   harness sharing #1's home; canonical launch = Gradle task vs. Android Studio run
   config vs. double-click HTML.
7. **Git handling now (blocks Phase 0). — DECIDED: commit + push current changes to
   `master`** (matches the repo's master-first rule for small changes). Done before any
   AutoSim code. Note: a stale `Limelight` branch sits 19 commits behind `origin` —
   unrelated, left alone.
8. **Action categories.** Confirm the DECODE set (DRIVE/SHOOT/FEED/INTAKE/HOOD/TURRET/
   WAIT) and that the category enum should be designed season-generic now (slides/claw/
   arm spares) to serve future-season reuse.
9. **One engine or two.** Confirm this should be the **visual front-end of #1's headless
   `AutoSim`** sharing one sim core (strongly recommended) rather than a standalone
   engine.
```
