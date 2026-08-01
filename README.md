# IR Heater Sequence Runner

Synchronized control of a **GRBL-based CNC gantry** and a **DPS5005 power supply**
for automated IR heating experiments.  Includes dual-camera recording, visual
pattern generation, and a manual alignment/jog interface.

## Hardware

- **GRBL controller** (Acmer laser cutter or any GRBL 1.1 gantry) via raw serial
- **DPS5005** programmable DC supply via Modbus RTU (for the IR heater)
- **Two USB cameras** (optional) for recording and alignment preview

> **⚠️ Important: Do NOT use homing (`$H`).**  The homing cycle is
> deliberately disabled in this software.  Running `$H` manually (e.g. via a
> serial terminal) may crash the gantry into its mechanical limits if end-stop
> switches are not properly configured or absent.  Always jog the gantry to a
> known-safe position manually before starting a sequence.

## Quick Start

```bash
# Install dependencies
uv sync

# Launch the main sequence GUI
uv run main.py gui

# Manual alignment with live camera preview
uv run main.py align

# Design heat-location grids visually
uv run main.py pattern-gui
```

## Reproducibility

Every run produces a **run metadata JSON** file (`run_metadata_<timestamp>.json`) in the
output directory (or current directory for dry-runs).  It records:

- Run ID, start/end timestamps (UTC), total duration
- CSV file, loop count, time mode, feedrate
- All hardware ports, baud rates, work-area limits
- Camera IDs, FPS, recording file paths (`.mp4`)
- Total steps vs. steps completed, any error message

Camera recording runs in a **separate process** (`CameraRecorderProcess`) so that
disk I/O from video encoding never blocks the timing-critical gcode loop.  The
main sequence thread handles only GRBL serial, DPS Modbus, and sleep-based timing.

## Project Structure

```
main.py                          # Unified CLI dispatcher
pyproject.toml                   # Dependencies
src/ir-heater/
├── sequence_runner.py           # Core: CSV -> GRBL + DPS + cameras
├── sequence_generator.py        # Old: A<->B oscillation generator
├── pattern_generator.py         # Heat-location dwell-pattern generator (circle/rectangle/line)
├── camera_controller.py         # Dual USB capture + process-isolated recording
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
    --cam0 0 --cam1 1 --cam-fps 15 --record-dir ./capture
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
| `--cam0`, `--cam1` | — | USB camera device IDs |
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
    --spiral-turns 3          # circle-shape only
```

Input locations CSV columns:

| Column | Description |
|---|---|
| `x`, `y`, `z` | Centre of the heated region (mm) |
| `dwell_time_s` | How long to dwell at this location |
| `radius_mm` | Radius (circle), half-width (rect), or half-length (line) |
| `dwell_feedrate` | Slow feedrate during dwell (mm/min) — separate from travel speed |
| `voltage_v`, `current_a` | Heater setpoints |
| `shape` *(optional)* | `circle` (default), `rectangle`, or `line` |
| `width_mm` *(optional)* | Full width for rectangle; full length for line |
| `height_mm` *(optional)* | Full height for rectangle (ignored for line) |
| `label` *(optional)* | Human-readable name |

Dwell patterns by shape:
- **circle** — Archimedean spiral from centre → radius → back
- **rectangle** — Zigzag raster fill of the rectangle area
- **line** — Straight line traversed back and forth

The output `sequence.csv` is directly compatible with `run`.

### `gui` — Main sequence-runner window

```bash
uv run main.py gui
```

Load a pairs CSV, configure hardware, run sequences with live matplotlib plots
and optional camera recording.  Writes `run_metadata_*.json` alongside
recordings.

### `align` — Manual alignment & jog window

```bash
uv run main.py align
```

- Live dual-camera preview (zero-copy `QImage` from OpenCV frames)
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

Example:

```csv
time,current,voltage,x,y,z,feedrate
1.5,1.2,12.0,10,10,1,1200
2.0,1.5,14.0,20,10,1,1400
1.0,1.0,10.0,20,20,1,1200
```

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
