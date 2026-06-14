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

> **Status: Phases 1–3 built + Phase 4 partial + fidelity tweaks** (see §12–§18). The `autosim/` module simulates
> `V3FarAuto` and `V3ClosePartner` end-to-end and emits one standalone viewer (path render,
> kinematic playback, data-driven action overlays, auto dropdown, 640p video export). Tier-1
> single-source config is in place (§15). Replay mode + assertion overlay remain the stretch.
> §11 records every design decision; all are resolved.
>
> **Decisions locked (2026-06-12):** front-end = **Web (HTML5 Canvas)**; build order =
> **predictive sim first** (the `SimFollower` kinematic model, so a run can be watched
> with no robot; replay drops to the Phase 4 stretch); git = **commit + push current
> changes to `master`**.
>
> **Refined per captain (2026-06-13) — all remaining questions resolved:**
> - **Configurable field-background image** behind the 144″ grid — a per-season bundled
>   default *plus* an in-viewer upload/drag-drop, so field elements are visible across
>   seasons (see §2). Goal reticle and all markers still drawn on top.
> - **Action effects are a data-driven, season-generic "effect profile"** (§3) — the
>   render core knows only generic effect *primitives* (ring, chevrons, flash, arc,
>   band); each season maps its categories onto them in config. DECODE's expanding-orange-
>   ring + muzzle-flash is just the current profile. Supports seasons with **no** shooting/
>   feeding/intaking at all (claws, slides, extensions).
> - **Event durations are derived from the auto's own timing constants** (§3, §8) — e.g.
>   the feed/shoot window is the real ~1.0 s (`FEED_TOTAL_TIME_SEC`), never a guessed value.
> - Follower fidelity = coarse point-mass (Q3 ✓); coordinate convention as written (Q4 ✓);
>   polylines sampled **from the source**, single source of truth (Q5 ✓).
> - **Self-contained in Android Studio, lowest-friction launch** (Q6 ✓): a Gradle task
>   emits **one standalone HTML with the trace + field image inlined**, openable on any
>   device with no server and no extra steps.
> - Category enum **season-generic now** (Q8 ✓); **one shared sim core** with headless
>   AutoSim #1 (Q9 ✓).

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

**Blessed path (lowest-friction, self-contained in Android Studio):** a single Gradle
task, e.g. `./gradlew autosim -Pauto=V3FarAutoSim`, that (a) runs the chosen sim copy,
(b) builds **one standalone `autosim-V3FarAuto.html`** with the trace **and** the field
image **inlined** (base64), and (c) opens it in the default browser. The task shows up in
Android Studio's **Gradle tool window** and is wrapped as a one-click **"▶ Run AutoSim"
run configuration** for students who never touch a terminal.

- **Why standalone-inline, not `fetch`:** a self-contained HTML opens straight off
  `file://` on **any** device — no local server, no CORS, no "serve this folder" step.
  Sharing the run = sharing that one HTML file (drop it in a PR, AirDrop it, open it on a
  phone). That is the "few steps, various devices" bar the captain set.
- **Per-auto, global viewer:** the *viewer template* is one tool; *which auto* it shows is
  chosen at sim time. Simulatable copies register in a small `AutoRegistry`, so the task
  (and later a dropdown in the viewer) can list every auto that has a `…Sim` copy. Today
  that's one entry (`V3FarAutoSim`); each future auto that opts in adds one line.
- **Replay path (Phase 4):** an on-robot logging copy writes the same-schema trace to the
  robot's storage during a real run; pull it (adb / file transfer) and feed it through the
  same task to get a standalone HTML. No separate tool.

---

## 2. Visual canvas — field, coordinates, path source

**Coordinate system.** Render in **Pedro field inches**: a 144″ × 144″ square, origin at
the **bottom-left**, +X right, +Y up, heading in **degrees CCW** (0° = +X). This matches
the auto's own pose literals — `START (64.1, 6.75, 180°)` bottom-center facing −X, goal
near `(5, 139)` top-left. Screen mapping is a single affine transform:
`px = x * (canvasPx / 144)`, and **flip Y** (screen Y grows downward). All rendering
works in field units and converts once at draw time, so the same trace renders at any
canvas size. *(Confirmed by the captain.)*

**Alliance.** Traces are stored **blue-native**; the viewer has a Blue/Red toggle that
applies the same mirror `AllianceMirror` uses (`x → 144 − x`, heading negated), so one
trace shows both alliances without re-simulating. This mirrors how the auto itself
treats poses.

**Field-background image (configurable).** Behind the 144″ grid the viewer renders an
optional **square field image**, so real field elements (goal, gates, launch zones,
next-season obstacles) are visible under the path instead of an abstract grid. The image
is mapped corner-to-corner onto the 0–144″ square via the *same* transform as everything
else, so a path drawn at `(64.1, 6.75)` lands on the right spot of the picture. Two ways
to set it, no rebuild needed for the second:

- **Per-season default (bundled):** a square PNG checked into the repo (e.g.
  `autosim/fields/decode.png`) named in the trace `meta`/`field` (`backgroundImage`). The
  launcher inlines it (base64) into the standalone HTML so the artifact stays
  self-contained. Swapping seasons = swapping which image the profile points at.
- **In-viewer upload / drag-drop:** a file-picker + drag-drop target lets anyone drop a
  square image at runtime (client-side `FileReader` → `drawImage`); it overrides the
  bundled one for that session. Zero build, works on any device — the lowest-friction way
  to try a new field photo.

The image is purely a backdrop layer. The goal reticle, start marker, path, robot, and
all overlays draw **on top**, and an opacity slider lets you dim the photo so the path
stays legible. Non-square images are letterboxed (centered, with a one-line warning)
rather than stretched, so geometry never distorts.

**Path data source — emit from the sim, don't re-declare.** The path polylines come
from the **sim copy**, which builds the exact same Pedro `BezierLine`/`BezierCurve`
objects the real auto builds (`buildBasePaths()`, `buildCyclePathsForCurrentCycle()`,
plus the dynamic retreat path) and samples each curve to a polyline (e.g. 30 points via
the curve's parametric form). This makes the **code the single source of truth** for
geometry — the viewer never hand-copies coordinates, so a path edit in the copy shows up
in the picture automatically. Each emitted path carries its control points too, so the
viewer can optionally draw the Bezier handles for debugging.

**On the canvas (back to front):**

- **Field backdrop:** the optional square field image (above), then the 144″ grid
  (alliance-tinted), a goal reticle at the (mirrored) target, and the start-pose marker.
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

Categories are **data, not hard-coded branches.** The viewer never `switch`es on
"`SHOOT`"; it reads an **effect profile** — a small config object mapping each category to
`{ color, glyph, label, effect-primitive, params }` — and the render core only knows a
handful of season-agnostic **primitives**: `RING` (expanding pulse), `CHEVRONS`
(converging arrows), `FLASH`, `ARC`/`GAUGE-TICK`, and the timeline `BAND`. DECODE ships
one profile (`SHOOT → RING+FLASH orange`, `INTAKE → CHEVRONS green`, …); a future season
ships its own (`CLAW → FLASH`, `SLIDE → GAUGE arc`, `EXTEND → CHEVRONS`) without touching
render code. A season that **never shoots, feeds, or intakes** just omits those categories
and defines its own — the tool has no DECODE assumptions baked into the engine. Spare
members (`SLIDE`, `CLAW`, `ARM`, `EXTEND`, …) already exist in the enum so next year is a
config edit, not a code change.

**(b) Timeline (bottom rail) — the "when".**
- The track background is segmented into **color bands by `AutoState`**, so the macro
  phase structure (wait → shoot → drive → intake → shoot → …) reads at a glance.
- **Event pins** sit above the track at each event's timestamp, colored/glyphed by
  category. Clicking a pin scrubs to it.
- A draggable **playhead** marks current time.

**(c) On-field pulses — the "where + what", at the moment it fires.** This is the current
**DECODE effect profile** (the §3a primitives instantiated for this season):
- `SHOOT`/`FEED`: an expanding orange `RING` + brief muzzle `FLASH` at the turret. The
  feed-and-shoot action in this auto is a **real ~1.0 s window**, so the on-field effect
  *persists for the event's full duration* (a held/pulsing ring) rather than a 400 ms
  blip — the ring's lifetime is the event's `durationMs`, not a cosmetic constant.
- `INTAKE`: green converging `CHEVRONS` at the intake side, shown for as long as intake is
  actually commanded on (a span, often overlapping a drive).
- `HOOD`/`TURRET`: a thin `GAUGE` arc/tick near the robot.

**Durations are sourced from the auto, not guessed.** Every event's `durationMs` is
computed from the same timing the FSM uses — the feed/shoot window is
`FEED_START_DELAY_SEC + FEED_TOTAL_TIME_SEC` (≈ 1.10 s here), a drive segment's duration is
its `SimFollower` traversal time, an intake span runs from `intake.setPower(>0)` until it's
zeroed. So what you see lasting "about a second" on screen is what the robot actually does
for about a second. A **legend** in the left rail keys color/glyph → category, driven off
the same effect profile so it auto-updates when the profile changes.

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
  Pedro geometry types (for Bezier sampling) and the copied FSM. Emits a `SimTrace`.
- **Visualizer:** an HTML+Canvas template (one HTML file, inline `<style>`/`<script>`, no
  npm, no framework). The Gradle task **injects the trace JSON and the field image
  (base64) directly into a copy of the template**, producing one **standalone
  `autosim-<auto>.html`**. Opening that file — on a laptop, a Chromebook, a phone — needs
  no server and no extra steps, which is the self-contained, low-friction bar the captain
  set. The template lives in the repo; the build only fills in the data.

Why web over the alternatives, for *this* team:

- **Richest visuals for the least code** — Canvas/SVG make the timeline bands, pulse
  rings, and shot rays straightforward; the same UI in Swing is far more layout pain.
- **Maintainability under turnover** — every student can edit HTML/CSS/JS; far fewer can
  wrangle Swing `paintComponent`. Lower ramp = better fit for a 9-person, high-turnover
  team.
- **Trivially shareable** — the standalone HTML is one self-contained file (trace + field
  image inlined) you can drop in a PR, AirDrop, or open on any device.
- **Clean decoupling** — the viewer can't accidentally depend on robot code; the Android
  build never has to know the viewer exists.

**Alternatives considered (and why not, for v1):**

- **Swing/JavaFX desktop window** — single-language (all Java), no browser. Driven by the
  *same* sim core, so it stays a viable fallback, but costs more UI effort and is harder
  for students to extend. *(Decided against in favor of web.)*
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
    "effectProfile": "decode",    // which category→effect mapping the viewer applies
    "follower": { "model": "pointmass-v1", "xVel": 73.8, "yVel": 60.8 }
  },
  "field": {
    "widthIn": 144, "heightIn": 144,
    "goal": { "x": 5, "y": 139 },
    "backgroundImage": "autosim/fields/decode.png"   // square; inlined as base64 by the launcher; user upload overrides
  },

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

  "events": [                                // sparse, at transitions; durationMs from the FSM's own timing
    { "tMs": 3000, "category": "SHOOT",  "label": "first shot",  "durationMs": 1100,
      // ≈ FEED_START_DELAY_SEC(0.10) + FEED_TOTAL_TIME_SEC(1.00); not a guessed blip
      "pose": {"x":64.1,"y":6.75,"headingDeg":180} },
    { "tMs": 3600, "category": "DRIVE",  "label": "toLastLine",  "durationMs": 1600 },  // SimFollower traversal time
    { "tMs": 6800, "category": "INTAKE", "label": "intake cycle 0","durationMs": 1500 } // span intake is powered on
    // …
  ]
}
```

Java-side mirror (sim core), sketched:

- `SimTrace { Meta meta; Field field; List<PathSpec> paths; List<Frame> frames; List<ActionEvent> events; }`
- `Field { double widthIn, heightIn; Pt goal; String backgroundImage; }`
- `Frame { long tMs; Pose2d pose; String autoState; String feedState; boolean followerBusy; double flywheelTargetRadS, flywheelActualRadS, turretDeg, hoodDeg, intakePower; }`
- `ActionEvent { long tMs; Category category; String label; long durationMs; Pose2d pose; }` — `durationMs` always set from the source's timing, never a literal.
- `PathSpec { String id; PathKind kind; List<Pt> controlPoints, polyline; double headingDeg; long tStartMs, tEndMs; }`
- `enum Category { DRIVE, SHOOT, FEED, INTAKE, HOOD, TURRET, WAIT, /* generic, unused this season */ SLIDE, CLAW, ARM, EXTEND }`

The category→visual mapping is **not** in Java — it lives in the viewer's effect profile
(`meta.effectProfile` selects it), so reskinning for a new season is a config edit, not a
schema or sim-core change. `Pose2d`/`Pt` are sim-local value types (not Pedro's `Pose`) so
the schema layer carries no Android/Pedro coupling.

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
   at `dt`, samples a `Frame` each tick, stops at `DONE` (or a 30 s cap). Closes out each
   `ActionEvent`'s `durationMs` from the virtual clock (feed span, intake span, drive
   traversal), so durations mirror the FSM, then serializes the `SimTrace`.
5. **Launcher + viewer** — the Gradle task injects that trace and the field image into the
   HTML template, emitting one standalone `autosim-V3FarAuto.html`, and opens it.

The original `V3FarAuto.java` is never imported, never edited.

**Future convergence:** when AUTO_BASE_DESIGN.md lands its `RobotIO`/helper seam, fold
the real auto and `V3FarAutoSim` onto one FSM body with two adapters (real vs. sim).
Until then, keep the copy honest with a small checklist note in the file header.

---

## 10. Phased build plan

- **Phase 0 — Scaffold (decisions all resolved; this is the re-executed Phase 0).**
  Create the `autosim` source set/module; the trace **schema types** (incl. `Field`
  with `backgroundImage`, `Category` with generic spares, `meta.effectProfile`); the
  **Gradle `autosim` task** that injects trace + base64 field image into the HTML
  template and opens the result (+ the wrapping run config); the **HTML template** with an
  inline **effect-profile** config block and an empty Canvas that draws the 144″ grid over
  a configurable square **field-background image** (with the upload/drag-drop control and
  opacity slider stubbed); and check in a placeholder `autosim/fields/decode.png`. *No
  simulation behavior yet — the harness, the launcher, and the field backdrop only.*
- **Phase 1 — Render the path (static).** `V3FarAutoSim` builds + samples the paths and
  emits a minimal trace (paths + start pose, no time). Viewer draws the field image +
  grid + full path + robot at start + goal reticle. **Exit:** the route overlays correctly
  on the field image; uploading a different square image re-backs it without a rebuild;
  proves coordinate mapping and the file seam end-to-end.
- **Phase 2 — Play back states (kinematic).** Add `SimClock` + `SimFollower` + the
  `SimRunner` loop; emit dense `frames`. Viewer gains play/pause/scrub/speed; the robot
  drives the path; state chip + timeline state-bands update. **Exit:** a full run plays
  start→DONE with plausible timing (3 s delay, feed windows, cycle count = 4).
- **Phase 3 — Action overlays (data-driven).** Recording stubs emit `events` with
  `durationMs` taken from the FSM timing; the viewer renders them **through the effect
  profile** — timeline bands/pins, on-field primitives (the DECODE profile's shoot
  ring+flash held for the real feed window, intake chevrons, hood/turret gauge ticks),
  turret wedge + shot ray, and the profile-driven legend. **Exit:** every action is
  visible at the right time, place, **and for its true duration**; swapping the effect
  profile reskins the overlays with no engine change.
- **Phase 4 (optional/stretch).** (a) **Replay** mode — a logging copy writes the same
  schema on-robot; pull + open in the viewer for real-run fidelity. (b) **Multi-auto** —
  registry-driven dropdown. (c) **Assertion overlay** — surface #1's invariants
  (e.g. "ends within 4″ of shoot pose", "fires 6×") as pass/fail badges on the timeline,
  making the sim a regression gate, not just a picture.

---

## 11. Decisions (all resolved)

Every gating question is answered; nothing blocks Phase 0 but the captain's go-ahead.

1. **Tech stack — Web visualizer (HTML5 Canvas).** Pure-Java sim core emits a trace; an
   HTML+Canvas page renders it.
2. **Build order — predictive sim first.** Build the `SimFollower` kinematic model up
   front so a run can be watched *before* driving, no robot required. Real-log replay is
   the Phase 4 stretch (reuses the same viewer + schema).
3. **Follower fidelity — coarse point-mass for v1.** v1 validates **logic, ordering, and
   timing**, not exact pose error. `SimFollower` is one swappable class for later.
4. **Coordinates — confirmed.** 144″×144″, origin bottom-left, +X right, +Y up, heading
   CCW degrees; render blue-native + mirror for red (matching `AllianceMirror`).
5. **Path source — sampled from the sim copy.** Single source of truth; the viewer never
   re-declares geometry.
6. **Home & launch — self-contained in Android Studio, lowest friction.** A Gradle task
   (in the AS Gradle window, wrapped as a run config) emits **one standalone HTML** with
   the trace + field image inlined, openable on any device with no server or extra steps.
7. **Git — committed + pushed to `master`.** Done. (A stale `Limelight` branch sits behind
   `origin` — unrelated, left alone.)
8. **Action categories — DECODE set confirmed, enum season-generic now.** DRIVE/SHOOT/
   FEED/INTAKE/HOOD/TURRET/WAIT plus unused generic spares (SLIDE/CLAW/ARM/EXTEND);
   category→visual mapping is a swappable effect profile (§3).
9. **One shared sim core.** This is the visual front-end of #1's headless `AutoSim`; they
   share one engine, not two.

**Added per captain (2026-06-13):**

10. **Configurable field-background image (§2).** Per-season bundled square image + an
    in-viewer upload/drag-drop, behind the grid, with an opacity slider; goal/markers
    drawn on top; non-square images letterboxed, not stretched.
11. **Season-generic, modular effect profiles (§3).** The render core knows only generic
    primitives (ring/chevrons/flash/arc/band); each season maps its categories onto them
    in config. DECODE's orange-ring + muzzle-flash is the current profile; seasons that
    don't shoot/feed/intake define their own with zero engine changes.
12. **Event durations sourced from the auto (§3, §8, §9).** Every `durationMs` is computed
    from the FSM's real timing (feed window ≈ 1.10 s, drive = traversal time, intake =
    powered span), never a guessed constant.

---

## 12. Phase 1 — as built (2026-06-13)

Phase 0 scaffold + Phase 1 static render are implemented in the new `autosim/` module.
A few implementation choices refined the plan (all reversible, none touch the robot build):

- **Home = a plain-`java` Gradle subproject `:autosim`** (one `include` line in
  `settings.gradle`). Root gradle only shares repositories, so this does **not** apply the
  Android plugin or affect `TeamCode`/`FtcRobotController`. It has **no Android/Pedro
  dependency**, so it runs on any JVM and in CI.
- **Geometry = local De Casteljau, not Pedro's classes.** Because a plain-java module
  can't depend on the Pedro AAR, `V3FarAutoSim` re-declares V3FarAuto's **control points**
  (the acknowledged §5 FSM-copy duplication) and samples them with
  [`Bezier`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/geom/Bezier.java).
  A Bezier is fully defined by its control points, so the sampled curve is mathematically
  identical to Pedro's; the viewer still receives polylines **from the source**, so there's
  no second place to edit coordinates (the original intent of the single-source rule).
- **Field image = SVG placeholder.** [`fields/decode.svg`](../autosim/fields/decode.svg) is
  a full-field schematic; the generator base64-inlines whatever image it's pointed at
  (svg/png/jpg), and the viewer's upload/drag-drop overrides it at runtime. Drop in the
  official DECODE field render to replace it.
- **Run it:** `./gradlew :autosim:autosim` (or run `AutoSimGenerator` from Android Studio)
  → writes the standalone [`dist/autosim.html`](../autosim/dist/autosim.html) (one file with
  *all* registered autos embedded — see §15). Verified headless with the Android Studio JBR;
  placeholders fully substituted and the field image inlined.
- **Deferred to later phases:** Phase 2 (playback) is now built — see §13. Phase 3
  (data-driven action overlays with real durations) is still pending; the effect profile,
  alliance mirror, field-image upload, and opacity controls are already wired in the viewer
  so it only adds event data.

---

## 13. Phase 2 — as built (2026-06-13)

Kinematic playback is implemented. The whole auto now runs headless on a virtual clock and
the viewer animates it.

- **Virtual time:** [`SimClock`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/sim/SimClock.java)
  advances a fixed **20 ms** tick; [`SimStopwatch`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/sim/SimStopwatch.java)
  replaces `ElapsedTime` (reads the clock), so the 3 s delay / 1 s feed / 0.25 s reverse are
  *simulated*, not slept.
- **Kinematic follower:** [`SimFollower`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/sim/SimFollower.java)
  — coarse point-mass with a trapezoidal accel/cruise/decel profile over the path arc length;
  `isBusy()` true until arrival; pose interpolated along the polyline. **Tuning (the one
  judgment call): `CRUISE = 55 in/s`, `ACCEL = 80 in/s²`** — chosen for plausible total time,
  seeded near Pedro's measured caps, and isolated in this one swappable class. They set the
  drive durations; nudge them if you want the sim to match a stopwatch of the real robot.
- **Recording stubs:** `SimIntake/SimFlywheel/SimTurret/SimHood/SimFeeder` mirror the real
  hardware surfaces; the flywheel has a coarse first-order spin-up for a believable actual rad/s.
- **Full two-FSM copy:** [`V3FarAutoSim`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/autos/V3FarAutoSim.java)
  now reproduces both FSMs (`AutoState` + `FeedState`), the timing constants, the path
  builders, and the loop body from V3FarAuto, driven by the seams above. It emits the path
  sequence (with `tStartMs`/`tEndMs`) and a dense frame per tick.
- **Verified headless:** 15 paths, **1445 frames, 28.88 s, ends `DONE`**; 6 shots, 4 intake
  cycles (#0–#3 alternating low/high), 1.00 s feed windows, the intake reverse pulse, and
  monotonic path timing — all confirmed from the trace. Original V3FarAuto untouched.
- **Viewer playback:** play/pause, scrub, **0.25×–4× speed**, a **state-band timeline**
  (colored by `AutoState`, with second ticks and a draggable playhead), the robot animated
  along the path (frame-interpolated) with a live state chip + pose/turret/hood/flywheel/intake
  readouts, and traversed / active / upcoming path styling. Keyboard: space = play, ←/→ = step.
- **Note:** the dense frames make the standalone HTML ~0.5 MB (still a single self-contained
  file that opens instantly). If that ever matters, the frame stride is one constant to raise.
- **Follow-up tweaks (2026-06-13):**
  - **Configurable robot footprint.** A `robot { lengthIn, widthIn, wheelbaseOffsetIn }`
    block in the trace (defaults: 18 × 13, offset 3.5 — the real robot) drives the drawn
    body. The viewer exposes length/width inputs and a ±4″ **wheelbase-offset** slider:
    the pose is the tracked (odometry) point, and a positive offset pushes the body/intake
    *forward* of it, so an asymmetric chassis sits correctly relative to the path. Live
    edits persist per-auto via `localStorage`.
  - **Red-alliance heading fix.** Red mirrors across the vertical centerline (`x → W-x`);
    the heading now mirrors as `180 - h` (was `-h`, which left the front arrow 180° off).
    The robot's perceived front now swaps with the alliance — important once the body is
    asymmetric.

---

## 14. Phase 3 — as built (2026-06-13)

Data-driven action overlays are implemented. The sim now emits `events[]` and the viewer
renders them **through the effect profile** — no DECODE assumptions in the render core.

- **Events are emitted in a post-pass** (`V3FarAutoSim.buildEvents()`), so every `durationMs`
  is sourced, not guessed:
  - **`SHOOT` × 6** — one per `startFeedSequence()`; `durationMs` = the real feed window
    `FEED_START_DELAY_SEC + FEED_TOTAL_TIME_SEC` = **1100 ms**; pose = the shoot pose. First
    fires at **t = 3000 ms** (the 3 s delay).
  - **`INTAKE` × 7 spans** — derived from the dense frames (contiguous runs of
    `intakePower > 0.05`), so the powered duration falls straight out of what the sim did
    (880 ms feed-window bursts, 3.3–4.7 s cycle-drive spans).
  - **`HOOD` / `TURRET`** — single markers at t = 0 (hood set once; turret slews to its aim
    angle). Sorted by time.
- **Effect profile carries a season-agnostic primitive per category** (`effect:
  RING | CHEVRONS | GAUGE | NONE`, plus `ray`). The render core knows only those primitives;
  DECODE maps `SHOOT → RING+FLASH+ray`, `INTAKE → CHEVRONS`, `HOOD/TURRET → GAUGE`. Reskinning
  a season is an edit to one profile object, not to draw code.
- **On-field overlays:** the shot **ring expands and re-pulses for the event's full duration**
  (held ~1.1 s, not a blip) with a muzzle flash and a bright animated **shot ray** to the goal;
  a **launched artifact** (one ball per shot pulse) flies along the aim line, its radius peaking
  at mid-flight (apex height cue) and shrinking into the goal; **intake chevrons** converge on
  the robot's intake side for as long as intake is commanded;
  a **turret wedge** points from the robot toward the goal; a brief **gauge** arc marks
  hood/turret. All draw in field units and mirror correctly for red.
- **Timeline gained event pins:** a category-colored marker (and faint duration bar) at each
  event, sitting above the `AutoState` bands; **click a pin to jump to it**. Transport adds
  **⏮ / ⏭ prev/next-action** buttons and `,` / `.` keys.
- **Legend** is already profile-driven, so it keys color/glyph → category and auto-updates if
  the profile changes.
- **Verified:** regenerated headless (15 paths, 1445 frames, 28.88 s, ends `DONE`) and
  visually in-browser on **both** alliances — shot ring/flash/ray, intake chevrons on the
  correct (mirrored) side, turret aim, and clickable timeline pins all render with no console
  errors. Original `V3FarAuto.java` still untouched.

This closes the core phased plan (Phases 1–3).

---

## 15. Phase 4 (partial) — as built (2026-06-14)

Three Phase-4 items were pulled forward by the captain (replay mode + assertion overlay are
**not** done and remain the stretch).

### Tier-1 single source of truth (the FSM-copy duplication, partly paid down)

The acknowledged §5 cost — every constant/pose/path lives in *both* the real opmode and its
`…Sim` copy — is now removed for the **data** (not yet the control flow; that's Tier 2 / the
AUTO_BASE refactor). Two plain-Java config classes hold the numbers:

- [`autoshared/V3FarAutoConfig.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoshared/V3FarAutoConfig.java)
  and [`autoshared/V3ClosePartnerConfig.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoshared/V3ClosePartnerConfig.java)
  — poses, geometry/control points, headings, timing, turret config, and (for close) the
  distance→shot lookup table. **Plain Java, zero Pedro/Android imports.**
- The package lives in **TeamCode's own source tree** (so the real opmodes compile it with no
  build change) and is **also** compiled into the plain-Java `autosim` module via a filtered
  `srcDir` in [`autosim/build.gradle`](../autosim/build.gradle). One physical file, two modules,
  no inter-module dependency, no Pedro leaking into the sim. Each side adapts the neutral numbers
  into its own types (real builds Pedro `Pose`/`BezierCurve`; sim builds polylines).
- `V3FarAuto.java`, `V3FarAutoSim`, `V3ClosePartner.java`, and `V3ClosePartnerSim` were all
  migrated to reference the config. The migration is a **literal→named-constant swap** — values
  are byte-identical, so robot behavior is unchanged (the V3FarAuto trace is bit-for-bit the same:
  15 paths / 1445 frames / 28.88 s). **Heads-up:** the Android side can't be compiled in this
  headless env, so a one-time Android Studio build is the final confirmation for the two real
  opmodes (the sim side is JBR-verified). This is the forcing function for Tier 2: once the FSM
  *body* is shared too, the `…Sim` copies disappear entirely.

### Second auto: V3ClosePartner

[`V3ClosePartnerSim`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/autos/V3ClosePartnerSim.java)
reproduces the close auto's 25-state machine, three gate cycles, and feed sequence from
`V3ClosePartner.java`, reading geometry/timing from the shared config from day one. `SimFollower`
gained `getCurrentTValue()`, `setMaxPower()` cruise-scaling, `isRobotStuck()`/`breakFollowing()`
to support its t-value- and timer-gated transitions. The real auto's shot-on-move turret/hood
compensation is simplified to straight-line goal tracking + the same `SHOT_TABLE` (readouts only;
path/state/timing/shots are faithful). Verified: 17 paths, 1397 frames, 27.92 s, 6 shots, ends
`DONE`, in-bounds.

### Multi-auto generator + dropdown

- [`AutoRegistry`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/AutoRegistry.java)
  lists the simulatable autos (one line each). The generator builds **every** registered trace and
  embeds them all into **one** `dist/autosim.html` (`TRACES = { name: trace, … }`); a **header
  dropdown** switches between them live (re-binds trace/robot/events/frames, rebuilds legend +
  timeline, resets playback). Robot-footprint tweaks persist per-auto.

### 640p annotated video export

A **"↓ video"** button records the current run+alliance to a `.webm`. It renders to a 640×640
backing canvas (matching the field image) and captures via `MediaRecorder` + `captureStream(0)` +
manual `requestFrame()`, driven by its **own timer** (not `requestAnimationFrame`) so it's
deterministic and never stalls if the tab loses focus. The clip is annotated — all on-field
overlays plus a burned-in HUD (auto name, alliance, time, state) — and downloads as
`autosim-<auto>-<alliance>.webm` (~2× speed so it isn't overlong). Verified end-to-end in-browser.

**Still stretch (not built):** replay mode (on-robot logging in the same schema) and the
assertion/regression overlay.
```

## 16. Post-Phase-4 fidelity tweaks — as built (2026-06-14)

Three captain-requested polish items on top of §15.

### Real heading interpolation + "turn, then drive"

The sim follower previously treated **every** path as a single constant heading that *snapped* at
the path boundary (the "jerk"). `SimFollower` now models Pedro's interpolation modes and pivots
before translating:

- New [`model/HeadingMode`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/model/HeadingMode.java)
  (`CONSTANT` / `TANGENT` / `REVERSE_TANGENT` / `LINEAR`) carried on `PathSpec` via chained setters
  (`.tangent()`, `.reverseTangent()`, `.linear(start,end,endT)`; `CONSTANT` is the default so
  existing constant paths are unchanged). Heading is **not** serialized — it's baked into the
  per-frame poses.
- `SimFollower.getPose()` computes heading point-by-point from the mode: `TANGENT` follows the
  actual polyline tangent at the current arc-length, `REVERSE_TANGENT` is tangent + 180°, `LINEAR`
  slews start→end over the first `endT` of the path.
- **Turn-then-move:** on `followPath`, the robot first pivots in place from its current heading to
  the path's starting heading at `TURN_RATE_DEG_S = 270°/s`, a phase that **consumes sim time**
  (so total durations grew slightly: V3Far 28.88→29.48 s, ClosePartner 27.92→29.56 s). `isBusy()`
  spans turn + translation; `getCurrentTValue()` stays 0 during the pivot so t-value gates fire on
  real path progress.
- [`V3ClosePartnerSim`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/autos/V3ClosePartnerSim.java)
  declares each path's mode to mirror `V3ClosePartner.buildPaths()` exactly (first shot = tangent;
  back-to-shoot / return-from-gate / final = reversed; gate-open = linear; the rest constant).
  V3FarAuto is all-constant and needed no per-path change; both autos benefit from the no-snap
  pivot.
- **Continuity fix:** `lastPose` is now refreshed on every `getPose()`, not only at move
  completion. Without this, autos that chain paths in a single tick (V3FarAuto, no wait state) read
  a stale heading for the next pivot and snapped. Verified: max per-frame heading step is now
  **5.4° = 270°/s × 20 ms** for *both* autos (was a 20° one-frame jump in V3Far).

### Artifacts finish their flight

In the viewer, launched balls now complete their full `BALL_FLIGHT` (900 ms) arc to the goal even
after the shot window / `SHOOT` state ends and the robot drives away — the shot ray, pulses, and
muzzle flash still stop at the window end, but `drawEffects` extends the cull horizon to
`dur + BALL_FLIGHT` for shooting events so in-flight artifacts aren't truncated. (Balls originate
from the fixed shot pose, so they were already independent of the moving robot.)

## 17. Mid-path heading-mode switches — as built (2026-06-14)

Real autos sometimes re-call `setXxxHeadingInterpolation()` on an in-progress path once
`follower.getCurrentTValue()` crosses a threshold — e.g. V3Auto's `toLastLine` starts
`TANGENT` and switches to a constant 180° once `t > 0.2`, and `backToShootFromLastLine`
starts `REVERSE_TANGENT` and switches to a constant -90° once `t > 0.8`. The sim
previously approximated both paths as a single mode for their whole length (documented
as a known simplification in `V3AutoSim`'s class Javadoc).

- New [`PathSpec.thenConstant(afterT, headingDeg)`](../autosim/src/main/java/org/firstinspires/ftc/teamcode/autosim/model/PathSpec.java)
  records an optional switch: once the path's fraction-by-arc-length `t` exceeds
  `afterT`, `SimFollower.headingAt` switches from the path's primary `headingMode` to a
  constant `headingDeg`. Defaults to "no switch" (`headingSwitchT = -1`), so every
  existing path is unaffected.
- `SimFollower.headingAt` was refactored into a small `headingForMode(mode, s, d, t,
  constDeg)` helper so both the pre- and post-switch heading share the same
  TANGENT/REVERSE_TANGENT/LINEAR/CONSTANT logic.
- `V3AutoSim` now declares `.tangent().thenConstant(LAST_LINE_SLOW_TVALUE, LINE_H)` on
  `toLastLine` and `.reverseTangent().thenConstant(0.8, -90.0)` on
  `backToShootFromLastLine`, matching `V3Auto.java` exactly. Its class Javadoc no longer
  lists these as simplifications.
- The switch is an instantaneous target change (matching what the real follower does
  when `setConstantHeadingInterpolation` is called mid-path — a sudden setpoint jump,
  not a smooth pivot like the path-start turn in §16). Verified in-browser: `toLastLine`
  holds its tangent heading (~258.8°→-92.5°) until ~20% of its travel time, then snaps to
  180° for the remainder; `backToShootFromLastLine` holds its reverse-tangent heading
  (~180°→246.7°) until ~80% of its travel time, then snaps to -90°.
- V3FarAutoSim and V3ClosePartnerSim have no mid-path heading switches in their real
  counterparts, so `headingSwitchT` stays at its default (-1) for all their paths — their
  traces are byte-for-byte unchanged except V3Auto's total duration (28.92→29.06 s, +7
  frames) from the two new snap-points slightly perturbing the trapezoidal timing.

> **Superseded by §18:** the "turn-then-move" pivot (§16) and the "instantaneous target
> snap" for mid-path switches (this section) were both replaced by a single continuous
> heading-chase model. The mechanism and verified numbers above are now historical.

## 18. Heading-chase rewrite + path discontinuity indicator — as built (2026-06-14)

Two more captain-requested changes on top of §16/§17.

### Rotate while driving (replaces turn-then-move)

The §16 pivot — stopping to turn in place before translating — didn't match how Pedro's
`Follower` actually behaves (heading control and translation run concurrently). It's
removed:

- `SimFollower` drops `turnSec`/`turnFromDeg`/`turnToDeg` entirely. `followPath()` no
  longer reserves any turn time: `travSec`/`tEndMs` are purely the trapezoidal drive
  time, and `isBusy()`/`getCurrentTValue()` are driven by that drive time from `t=0`.
- `getPose()` instead computes the path's *commanded* heading at the current
  arc-length (`headingAt(active, d)` — unchanged from §17, including `thenConstant`
  switches) and rate-limits the robot's *actual* heading toward it: `step =
  clamp(delta, ±TURN_RATE_DEG_S * dtSec)`, where `dtSec` is the time since the previous
  `getPose()` call (`lastSampleSec`, now updated unconditionally on every call including
  the idle/`active==null` branch). `TURN_RATE_DEG_S` stays `270°/s`, repurposed from "pivot
  speed" to "max heading slew rate while driving."
- This applies uniformly to path-start heading changes AND `thenConstant` mid-path
  switches — no special-casing needed, both are just "target changed, chase it."
- **Durations**, with no reserved turn time, drop back close to the pre-§16 baseline:
  V3FarAuto 29480→28880 ms, V3ClosePartner 29560→27920 ms, V3Auto 29060→27540 ms.
- Verified in-browser: max per-frame heading step remains capped at 5.4°/frame (=270°/s
  × 20 ms) for all three autos; V3Auto's `toLastLine` and `backToShootFromLastLine` now
  show a gradual heading ramp through their `thenConstant` switch points instead of a
  snap.

### Path discontinuity indicator

Added a viewer-only check (no schema/Java changes — operates on the already-serialized
`polyline` endpoints): for each consecutive path pair, if the distance between path
*i*'s last polyline point and path *i+1*'s first polyline point exceeds
`DISCONTINUITY_THRESHOLD_IN = 0.5` inches, it's flagged as a gap in the path plan.

- `computeDiscontinuities()` runs once per `bindAuto()` call and produces a list of
  `{a, b, gap, from, to}`.
- `drawDiscontinuities()` renders each flagged gap as a red dashed line with pulsing
  rings and a "⚠" marker on the canvas, every frame.
- `buildDiscoList()` populates a "Discontinuities ⚠" sidebar section (hidden if empty)
  listing `fromPath → toPath (N.Nin)`.
- V3Auto has several real flagged gaps (e.g. `toLine2 → backToShoot` 3.4in,
  `toGateOpenGate → toGateIntake` 2.2in, `gateExtra → backFromGate` 4.5in and its
  "Again" duplicates) — these are genuine path-planning gaps in `V3Auto.java`'s control
  points, not sim artifacts. V3FarAuto and V3ClosePartner have none above threshold.

See `autosim/AUTOSIM.md` §7.1, §7.2, and §9.4 for the maintainer-facing description of
both changes.

## 19. Hybrid heading model — as built (2026-06-14)

Captain feedback on §18: the "rotate while driving" model was too broad. Smooth
concurrent rotation is correct for `thenConstant` mid-path switches, but path-*start*
heading changes should still be a blocking pivot, as in §16.

- `SimFollower` reinstates `turnSec`/`turnFromDeg`/`turnToDeg`. `followPath()` computes
  `target0 = headingAt(spec, 0.0)` and `turnSec = |shortestDelta(currentHeading,
  target0)| / TURN_RATE_DEG_S`, added on top of `travSec` for `tEndMs`,
  `isBusy()`, and `getCurrentTValue()` (which reports `t=0` during the pivot).
- During the pivot (`elapsed < turnSec`), `getPose()` holds position at the path's
  start point and rate-limits heading toward `target0` — unchanged blocking
  turn-then-move from §16.
- During translation, heading snaps directly to `headingAt(active, d)` each frame,
  *except* once `t > headingSwitchT` (a `thenConstant` switch fires), where it falls
  back to the §18 rate-limited chase from `lastPose.headingDeg` toward the new target —
  concurrent with translation, no reserved time.
- Durations partially revert toward the §16/pre-§18 baseline (path-start pivots are
  reserved again): V3FarAuto 28880→29480 ms, V3ClosePartner 27920→29560 ms, V3Auto
  27540→29060 ms. `toLastLine`/`backToShootFromLastLine`'s `thenConstant` switches
  remain smooth, as in §18.
- As part of the same change, V3Auto's tunables/path geometry/shot table were extracted
  into a new `autoshared/V3AutoConfig.java` (mirroring `V3FarAutoConfig`/
  `V3ClosePartnerConfig`), and both `autos/V3Auto.java` and `autosim/.../V3AutoSim.java`
  now read from it instead of duplicating literals.

See `autosim/AUTOSIM.md` §7.1 for the updated maintainer-facing description.
