# IR Heater Sequence Runner

Synchronized control of a **GRBL-based CNC gantry** and a **DPS3005 power supply**
for automated IR heating experiments.  Includes multi-camera recording, visual
pattern generation, and a manual alignment/jog interface.

## Hardware

- **GRBL controller** (Acmer laser cutter or any GRBL 1.1 gantry) via raw serial
- **DPS3005** programmable DC supply via Modbus RTU (for the IR heater) —
  writes (voltage/current/on-off) retry a few times on transient RS485
  errors, then raise instead of failing silently, so a dropped "turn heater
  off" can no longer go unnoticed
- **Any number of USB cameras** (optional) for recording and alignment preview —
  identified by live preview and assigned a label, since OS device indices
  aren't stable across reconnects (see [Identifying cameras](#identifying-cameras))

> **⚠️ Important: Do NOT use homing (`$H`).**  The homing cycle is
> deliberately disabled in this software.  Running `$H` manually (e.g. via a
> serial terminal) may crash the gantry into its mechanical limits if end-stop
> switches are not properly configured or absent.  Always jog the gantry to a
> known-safe position manually before starting a sequence.

## Quick Start

```bash
# Install dependencies
uv sync

# Design heat-location grids visually
uv run main.py pattern-gui

# Manual alignment with live camera preview
uv run main.py align

# Launch the main sequence GUI
uv run main.py gui

```

## Reproducibility

Every run produces a **run metadata JSON** file (`run_metadata_<timestamp>.json`) in the
output directory (or current directory for dry-runs).  It records:

- Run ID, start/end timestamps (UTC), total duration
- CSV file, loop count, time mode, feedrate
- All hardware ports, baud rates, work-area limits
- Cameras used (`{label: device_index}`), FPS, recording file paths per label (`.mp4`)
- Total steps vs. steps completed, any error message
- `drift_warnings`: any motion-drift warnings raised during the run (see
  [Motion-drift verification](#motion-drift-verification))

Camera recording runs in a **separate process** (`CameraRecorderProcess`) so that
disk I/O from video encoding never blocks the timing-critical gcode loop.  The
main sequence thread handles only GRBL serial, DPS Modbus, and sleep-based timing.

Every recorded `.mp4` gets a matching `<same name>_frames.csv` sidecar logging
the wall-clock UTC timestamp of every written frame. `VideoWriter` assumes a
constant frame rate; if real capture ever dips below the target FPS (USB
contention between multiple cameras, CPU load), the video's internal timeline
silently drifts from wall-clock time. The sidecar gives an exact, independent
record for mapping any frame back to real time — and back to the position/
heater state at that moment via `metadata.started_utc` — without relying on
`frame_index / fps`.

## Project Structure

```
main.py                          # Unified CLI dispatcher
pyproject.toml                   # Dependencies
src/ir-heater/
├── sequence_runner.py           # Core: CSV -> GRBL + DPS + cameras
├── sequence_generator.py        # Old: A<->B oscillation generator
├── pattern_generator.py         # Heat-location dwell-pattern generator (circle/rectangle/line)
├── camera_controller.py         # Multi-camera capture + process-isolated recording
├── camera_setup_dialog.py       # Live-preview camera identification/labeling dialog (PySide6)
├── dps_modbus.py                # DPS5005 Modbus driver
├── dps5005_limits.ini           # DPS voltage/current safety limits
├── gui.py                       # Main sequence-runner GUI (PySide6)
├── alignment_gui.py             # Manual jog + camera preview GUI (PySide6)
├── pattern_utility_gui.py       # Grid/well-plate designer GUI (PySide6)
└── gcodegenerator.py            # Standalone static G-code file writer
```

## CLI Commands

### `run` — Execute a sequence

```bash
uv run main.py run \
    --csv sequence.csv \
    --modbus-port COM4 \
    --grbl-port COM3 \
    --loops 3 \
    --camera top:0 --camera side:1 --cam-fps 15 --record-dir ./capture
```

| Option | Default | Description |
|---|---|---|
| `--csv` | *(required)* | Sequence CSV (see format below) |
| `--loops` | `1` | Repeat the schedule N times |
| `--time-mode` | `step` | `step` = per-row delay, `absolute` = cumulative |
| `--default-feedrate` | `1200` | Fallback feedrate (mm/min) |
| `--dry-run` | off | Parse & print without hardware |
| `--modbus-port` | — | DPS serial port (COM4, /dev/ttyUSB0) |
| `--modbus-address` | `1` | DPS Modbus address |
| `--modbus-baud` | `9600` | DPS baud rate |
| `--grbl-port` | — | GRBL serial port, or `auto` to probe all ports for a GRBL controller |
| `--grbl-baud` | `115200` | GRBL baud rate |
| `--x-max`, `--y-max`, `--z-max` | — | Software work-area clamps (mm) |
| `--camera LABEL:INDEX` | — | Add a camera for recording (repeatable — any number of cameras) |
| `--list-cameras` | — | Probe device indices 0-9 for a responsive camera and exit |
| `--cam-fps` | `15` | Recording framerate |
| `--record-dir` | — | Output directory for `.mp4` files and `run_metadata_*.json` |
| `--return-to-first-position` | off | Return to first CSV position instead of origin |
| `--list-ports` | — | List available serial ports (device + description) and exit |

### `generate` — Old A<->B oscillation generator

```bash
uv run main.py generate \
    --pairs-csv pair_specs.csv \
    --output sequence.csv \
    --default-transition-s 1.5
```

Input pairs CSV columns: `ax, ay, az, bx, by, bz, duration_s, current_a, voltage_v`

Optional: `feedrate`, `transition_s` (time between pair sections).

### `pattern-gen` — Heat-location dwell-pattern generator (circle / rectangle / line)

```bash
uv run main.py pattern-gen \
    --locations heat_locations.csv \
    --output sequence.csv \
    --travel-feedrate 2000 \
    --ring-spacing-mm 2 \      # circle-shape only
    --start-x 0 --start-y 0 --start-z 0   # optional: known gantry start position
```

Input locations CSV columns:

| Column | Description |
|---|---|
| `x`, `y`, `z` | Centre of the heated region (mm) |
| `dwell_time_s` | How long to dwell at this location |
| `radius_mm` | Radius (circle), half-width (rect), or half-length (line) |
| `voltage_v`, `current_a` | Heater setpoints |
| `shape` *(optional)* | `circle` (default), `rectangle`, or `line` |
| `width_mm` *(optional)* | Full width for rectangle; full length for line |
| `height_mm` *(optional)* | Full height for rectangle (ignored for line) |
| `label` *(optional)* | Human-readable name |

There is deliberately no `dwell_feedrate` column — the in-location dwell speed
is *derived* from the dwell path length and `dwell_time_s`
(`path_mm * 60 / dwell_time_s`), not taken as a separate input. An earlier
version accepted both independently, which meant they could silently
disagree: the commanded feedrate might not actually cover the path within
the scheduled time, so the software's timing model and the gantry's real
position could drift apart — exactly the kind of inconsistency this project
depends on avoiding. `--travel-feedrate` (between locations) is unaffected.

`--start-x`/`--start-y`/`--start-z` (all three or none) tell the generator
where the gantry actually is when the sequence starts, so the very first
move gets a real travel time instead of a fixed 0.5s guess. Without them,
the generator prints a warning and falls back to the guess.

Dwell patterns by shape:
- **circle** — concentric rings traced with native GRBL `G2`/`G3` arcs (see
  [Arc moves](#arc-moves-g2g3) below), spaced `--ring-spacing-mm` apart
- **rectangle** — Zigzag raster fill of the rectangle area
- **line** — Straight line traversed back and forth

The output `sequence.csv` is directly compatible with `run`.

### `gui` — Main sequence-runner window

```bash
uv run main.py gui
```

Load a pairs CSV, configure hardware, run sequences with live matplotlib plots
and optional camera recording.  Writes `run_metadata_*.json` alongside
recordings. Click **Configure Cameras…** to identify and label however many
cameras you want to record from (see [Identifying cameras](#identifying-cameras)).

### `align` — Manual alignment & jog window

```bash
uv run main.py align
```

- Live preview for however many cameras you configure in the connect dialog
  (zero-copy `QImage` from OpenCV frames)
- Jog pad: X+ / X- / Y+ / Y- / Z+ / Z- with configurable step size and feedrate
- Go-to coordinate input
- GRBL position polling (every 500 ms)
- Heater ON/OFF toggle with voltage/current spinboxes
- Save named positions and export to CSV

### `pattern-gui` — Visual grid designer

```bash
uv run main.py pattern-gui
```

- Define rectangular grids (well plates) with rows x cols + spacing
- Add multiple grids with different origins and colors
- Add individual custom points
- Zoomable/pannable preview (`QGraphicsView`)
- Export to `heat_locations.csv` for use with `pattern-gen`

## Sequence CSV Format

Required columns (flexible aliases supported):

| Column | Aliases |
|---|---|
| `time` | `time_s`, `dt`, `duration` |
| `current` | `current_a`, `i`, `amps` |
| `voltage` | `voltage_v`, `v`, `volts` |
| `x` | — |
| `y` | — |
| `z` | — |
| `feedrate` *(optional)* | `speed`, `f` |
| `arc_i`, `arc_j` *(optional)* | — |
| `arc_dir` *(optional)* | — |

Example:

```csv
time,current,voltage,x,y,z,feedrate
1.5,1.2,12.0,10,10,1,1200
2.0,1.5,14.0,20,10,1,1400
1.0,1.0,10.0,20,20,1,1200
```

### Arc moves (G2/G3)

A row is an ordinary linear move (`G1`) unless both `arc_i` and `arc_j` are
given, in which case it becomes a circular arc (`G2` clockwise by default, or
`G3` if `arc_dir` is `ccw`) — GRBL's native circular interpolation, computed in firmware
`arc_i`/`arc_j` follow GRBL's own convention: the offset from the *row's
starting point* (i.e. the previous row's `x`/`y`) to the arc's center — not
a bare `i`/`j`, since `i` already aliases the `current` column above and a
column named just `i` would be ambiguous. `pattern-gen`'s circle shape is
the main producer of these rows (see `_ring_segments` in
`pattern_generator.py`); hand-written CSVs can use them directly too. Arc
rows are **not** clamped to `--x-max`/`--y-max`/`--z-max` — clamping only the
endpoint would leave the center offset pointing somewhere that no longer
matches a circle through the (now-clamped) start and end points, so an
out-of-bounds arc is rejected up front with a clear error instead.

## Dependencies

```
matplotlib>=3.10.9     # Plotting
minimalmodbus>=2.1.1   # DPS5005 Modbus
opencv-python>=4.10    # Camera capture
pyserial>=3.5          # Serial (GRBL)
pyside6>=6.7           # Qt GUI framework
pillow>=12.1.1         # Image support
```

## GRBL Notes

- Default baud rate is **115200** (not 250000 like Marlin)
- The GRBL controller must be in **absolute mode (G90)** and **mm units (G21)**
- Soft limits can be set on the controller (`$130`-`$132`) or passed as
  `--x-max` / `--y-max` / `--z-max` CLI flags
- The laser is **not** used — only the motion system. No `M3`/`M4`/`M5` commands
  are sent
- Homing (`$H`) is **deliberately disabled** — do not attempt to home the
  gantry.  See the warning in the Hardware section above.
- If hard/soft limits are configured on the controller, GRBL boots into an
  **Alarm** state on every reset and rejects motion (`error:9`) until
  unlocked.  Since homing is disabled, `GrblController` clears this lock with
  `$X` right after connecting (this does **not** move the gantry — it just
  tells GRBL to trust the current step position).
- Since there's no homing, the *only* origin/zero mechanism is `G92`
  (`GrblController.set_zero`, wired to the align GUI's **Zero Here**
  button) — see its docstring for why it's not persisted across resets.
  `G92` only offsets **WPos** (work position), not **MPos** (raw machine
  position), and GRBL's factory default (`$10=1`) reports MPos — which
  would make every "Zero Here" invisible to both the position readout and
  `run_sequence`'s motion-drift check. `GrblController` forces `$10=2`
  (report WPos) right after connecting for this reason.
- Every command sent to GRBL (`_send_line`) has a bounded 10s timeout and
  raises `TimeoutError` instead of blocking forever if a reply never
  arrives (e.g. a dropped byte on a flaky link). The `align` GUI's jog/go-to
  buttons call GRBL directly on the main Qt thread, so an unbounded wait
  there used to freeze the entire window until the process was killed.
- `pattern_generator.generate_heat_sequence` has no live GRBL connection at
  CSV-generation time, so without an explicit `start_position` it guesses a
  fixed 0.5s for the very first move regardless of the real distance. If
  the gantry is actually far from that first target, `run_sequence` used to
  wait only 0.5s before sending several more lines while GRBL was still
  physically completing a much longer move — the backlog this creates in
  GRBL's planner buffer can eventually stall `ok` replies several steps
  later, surfacing as an unrelated-looking `_send_line` timeout. Since
  `run_sequence` *does* have a live connection right when a run starts, it
  now queries GRBL's actual position there and recomputes the first step's
  timing from the real distance (`_with_live_first_step_timing`) before
  anything else. It only ever lengthens the guess, never shortens it.
- The end-of-run "return to origin / initial position" move used to
  disconnect immediately after GRBL accepted the line (`ok`), not after it
  physically finished — closing the port mid-travel could strand the
  gantry short of the intended final position, which then became the
  *next* run's (wrong) idea of where it was starting from. `run_sequence`
  now waits (`wait_for_hold`, timed to the move's own distance/feedrate)
  for GRBL to actually stop before disconnecting, and warns if it doesn't
  confirm in time.

## Motion-drift verification

There's no real closed loop here — GRBL only tracks its own commanded step
position, no independent (e.g. encoder) feedback exists, and `run_sequence`'s
timing is fundamentally wall-clock/open-loop: a G-code line's `ok` reply just
means GRBL *accepted* it into its planner buffer, not that the motion
finished. Polling GRBL's status and waiting for `Idle` before every move
would give hard guarantees, but it would also force a full stop-and-decelerate
at every waypoint — for the fine-grained dwell patterns that means killing
the smooth continuous motion they're built for (and could unevenly heat
whatever dwells longer at each forced stop).

Instead, `run_sequence` periodically (every `_DRIFT_CHECK_INTERVAL_S` = 5s of
wall-clock time, **not** every step, so it can't affect motion smoothness)
samples GRBL's real position (`?` status query) and compares it against the
schedule's *current target* (the most recently commanded step's X/Y) — a
following-error check, not a "distance covered" check. If the gap exceeds `_DRIFT_WARN_MM` (3mm), it prints a
`WARNING: motion drift` message and appends it to `metadata.drift_warnings` in
the run's JSON log — e.g. a sign that a dwell's implied speed is more than the
hardware can actually sustain. This never gates or slows the command stream;
it's a verification/alerting layer on top of the existing open-loop timing,
not a synchronization mechanism.

## Connecting to custom GRBL boards (e.g. ACMER)

Some laser/CNC controllers (ACMER's included) run a customized GRBL fork on a
non-Arduino MCU that doesn't reset when the serial port opens and may not
print the standard `Grbl ...` banner. `GrblController`'s connect handshake
copes with this automatically:

1. Waits for a banner; if none appears, sends a GRBL soft-reset (`Ctrl-X`) to
   force one.
2. If the banner text doesn't contain `Grbl`, falls back to confirming the
   device via a live status query (`?`) before giving up.
3. Clears a power-up Alarm lock with `$X` (see above) — never `$H`.

**Finding the right COM port:**

```bash
uv run main.py run --list-ports          # list every visible serial port
uv run main.py run --grbl-port auto ...  # probe all ports for a GRBL banner
```

Both `gui` and `align` have **Refresh Ports** / **Auto-detect GRBL** buttons
next to the port fields that do the same thing. Auto-detect only sends the
same reset/handshake bytes used to connect (never G-code), so probing a
non-GRBL device (e.g. the DPS heater) is harmless.

## Identifying cameras

USB webcams don't have a stable identity in OpenCV — the OS can assign a
camera a different device index across reboots or reconnects, and generic
webcams all report the same unhelpful description ("USB Video Device"), so
there's no metadata to tell them apart by. The only reliable way to know
which physical camera is which is to look at it.

Both `gui` and `align` have a **Configure Cameras…** button that opens a
live-preview picker: it probes device indices 0-9, shows a thumbnail from
each one that responds, and lets you check off however many you want to use
(zero, one, two, or more) and give each a label (e.g. `top`, `side`). Camera
identity is tracked by that label everywhere downstream — recording
filenames, `run_metadata_*.json`, and the alignment preview panels — not by
the raw index, so a re-identify after a reconnect doesn't require touching
anything else.

On the CLI, use repeatable `--camera LABEL:INDEX` flags:

```bash
uv run main.py run --list-cameras                        # which indices respond
uv run main.py run --camera top:0 --camera side:1 ...     # record from both
```

### Only one camera streams at a time

Two or more USB cameras opening fine but only one ever showing a live frame
is almost always a **USB bandwidth or power budget problem**, not a software
bug — the second camera is losing an arbitration fight with the first over
a shared hub/host controller. `camera_config.ini`'s `capture_fourcc: MJPG`
(the default) asks each camera for its compressed stream instead of raw
YUY2/uncompressed, which cuts required bandwidth roughly 10x and is usually
enough on its own. If it isn't:

- Put each camera on a **separate USB controller/port**, not the same hub —
  on most PCs/laptops that means physically different ports, not just
  different hub downstream ports.
- If using a hub, use a **powered** one; bus-powered hubs often can't supply
  enough current for two active webcams at once.
- Lower `capture_width`/`capture_height` in `camera_config.ini`.

A camera that opens but then never delivers a frame for `stall_timeout`
seconds (default 5s) now shows an explicit error in the preview panel
instead of sitting on "(not connected)" forever with no explanation.
