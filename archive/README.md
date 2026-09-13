# Archive

Nothing under `archive/` is compiled — it sits outside every Gradle source set. Files keep their
original path, so `archive/decode-2025/TeamCode/...` mirrors where each file used to live in `TeamCode/`.

## `decode-2025/` — DECODE (2025–26) season

Archived during the Pedro Pathing 3 migration (`feature/pedro3-overhaul`). These files use the
Pedro Pathing 2 API, which can't be installed next to Pedro 3.

| File | What it was |
|---|---|
| `autos/V3Auto.java` | Close-side 18-ball auto (FSM) |
| `autos/V3ClosePartner.java` | Close-side triple-gate partner auto |
| `autos/V3FarAuto.java` | Far-side auto |
| `autos/V3FarAutoByYuvi.java`, `autos/NearAutoByYuvi.java` | Alternate far/near autos |
| `teles/TeleOpByYuvi.java` | Alternate teleop (`V3Tele` is the ported main teleop) |
| `pedroPathing/Constants.java` | Pedro 2 constants — keeps last season's tuned drive/PIDF numbers for reference |
| `pedroPathing/Tuning.java` | Pedro 2 tuning opmodes (replaced by AutoTune) |
| `AllianceMirror.java` | Pedro 2 version of the mirroring helper (replaced by `alliance/AllianceMirror`) |
| `autos/visionStuff.py`, `reference/sampleCV.txt` | Old vision experiments |

The autos' tunable configs (`autoshared/*Config.java`) moved into the `:autosim` module, which is
still the only thing that reads them.

## Reviving an archived file

1. Copy it back under `TeamCode/src/main/java/...`.
2. Port it to Pedro 3. `teles/V3Tele.java` is a complete worked example; the call mapping is in `CLAUDE.md`.

The pre-sweep snapshot of older code is also still at git tag `season-2025-decode-archive`.
