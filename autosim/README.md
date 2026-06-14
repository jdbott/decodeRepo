# AutoSim viewer (`:autosim` module)

A small, self-contained tool that renders an FTC autonomous routine as a top-down field
view. See the full design in [docs/AUTOSIM_DESIGN.md](../docs/AUTOSIM_DESIGN.md).

This is a **plain-Java desktop module** with **no Android / Pedro dependency**, so it runs
on any JVM and never touches the robot (TeamCode / FtcRobotController) build.

## What it does

The generator builds a `SimTrace` from a simulatable copy of an auto, then injects that
trace **and** the field image (base64) into the viewer template, producing **one
standalone HTML** you can open on any device — no server, no extra steps.

```
sim core (Java)  ──trace──►  viewer template  ──►  dist/autosim-<auto>.html  (open in any browser)
```

## Run it

From Android Studio: open the **Gradle** tool window → `:autosim` → Tasks → `autosim`
(or run the `AutoSimGenerator` main class). From a terminal:

```bash
./gradlew :autosim:autosim
# then open autosim/dist/autosim-V3FarAuto.html
```

The task prints the output path plus a sanity summary (path count, bounding box, whether
every point is inside the 144" field).

## Phase status

- **Phase 1 (now):** static render — field image + 144" grid + the full path + robot at
  the start pose + goal. Blue/Red mirror toggle; upload/drag-drop or swap
  `fields/decode.svg` to change the field backdrop.
- **Phase 2 (next):** kinematic playback (SimClock + SimFollower) with transport controls.
- **Phase 3:** action overlays driven by the effect profile, with real event durations.

## Layout

```
autosim/
  build.gradle                 plain-java module + `autosim` task
  fields/decode.svg            placeholder full-field image (replace or upload your own)
  viewer/autosim.template.html viewer template (__TRACE_JSON__ / __FIELD_IMAGE_DATAURI__)
  dist/                        generated standalone HTML (committed example included)
  src/main/java/.../autosim/
    geom/      Pt, Pose2d, Bezier (De Casteljau sampler)
    model/     SimTrace, Meta, FieldSpec, PathSpec, Frame, ActionEvent, Category
    json/      TraceWriter (dependency-free JSON)
    autos/     V3FarAutoSim (path-building copy of V3FarAuto, BLUE-NATIVE)
    AutoSimGenerator.java       entry point
```
