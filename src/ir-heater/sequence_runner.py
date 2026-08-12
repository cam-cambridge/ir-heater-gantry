from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing
import shutil
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path

import serial as pyserial
from serial.tools import list_ports as _list_ports

from camera_controller import CameraController, CameraRecorderProcess, CameraSpec
from dps_modbus import Dps5005, Import_limits, Serial_modbus


@dataclass(frozen=True)
class SequenceStep:
    time_s: float
    current_a: float
    voltage_v: float
    x: float
    y: float
    z: float
    feedrate: float
    # Arc move (G2/G3): center offset from the *previous* step's position,
    # GRBL's native I/J convention. None on both = ordinary linear move (G1).
    arc_i: float | None = None
    arc_j: float | None = None
    arc_cw: bool = True


@dataclass
class RunMetadata:
    """Captures everything needed to reproduce a run."""

    run_id: str = ""
    started_utc: str = ""
    completed_utc: str = ""
    duration_s: float = 0.0
    completed: bool = False

    # Inputs
    csv_file: str = ""
    csv_format: str = ""  # "raw" (CLI: --csv passed straight in) | "pairs" | "locations"
    source_csv: str = ""  # the small CSV the user picked before generation/looping (GUI runs).
    csv_copy: str = ""  # duplicate of csv_file saved next to this metadata file -- see save_run_metadata.
    loops: int = 1
    cycle_delay_s: float = 0.0  # pause between repeats of the full grid/cycle; see run_sequence.
    steps_per_cycle: int = 0  # len(steps) / loops, i.e. steps in one pass through the grid.
    time_mode: str = "step"
    default_feedrate: float = 1200.0
    dry_run: bool = False
    return_to_origin: bool = True

    # Hardware
    grbl_port: str = ""
    grbl_baud: int = 115200
    x_max: float | None = None
    y_max: float | None = None
    z_max: float | None = None
    dps_port: str = ""
    dps_address: int = 1
    dps_baud: int = 9600

    # Cameras
    cameras: dict[str, int] = field(default_factory=dict)  # label -> device index
    cam_fps: float = 15.0
    record_dir: str = ""
    recording_paths: dict[str, str] = field(default_factory=dict)  # label -> output path

    # Results
    total_steps: int = 0
    steps_completed: int = 0
    error: str = ""
    drift_warnings: list[str] = field(default_factory=list)


def save_run_metadata(meta: RunMetadata, output_dir: Path) -> Path:
    """Write *meta* as a JSON file in *output_dir*."""
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"run_metadata_{meta.run_id}.json"
    data = asdict(meta)
    path.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")
    print(f"Metadata saved -> {path}", flush=True)
    return path


def _normalized_row(row: dict[str, str | None]) -> dict[str, str]:
    return {
        (key.strip().lower() if key else ""): (value or "").strip()
        for key, value in row.items()
    }


def _first_value(row: dict[str, str], aliases: tuple[str, ...]) -> str:
    for name in aliases:
        if name in row:
            return row[name]
    return ""


def _parse_float(row: dict[str, str], aliases: tuple[str, ...], line_number: int) -> float:
    value = _first_value(row, aliases)
    if value == "":
        names = ", ".join(aliases)
        raise ValueError(f"Missing value for one of [{names}] at CSV line {line_number}")
    try:
        return float(value)
    except ValueError as exc:
        names = ", ".join(aliases)
        raise ValueError(f"Invalid numeric value for [{names}] at CSV line {line_number}") from exc


def read_sequence_csv(csv_path: Path, default_feedrate: float) -> list[SequenceStep]:
    if not csv_path.exists():
        raise ValueError(f"CSV file not found: {csv_path}")

    steps: list[SequenceStep] = []
    with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV is empty or missing a header row")

        normalized_fields = {name.strip().lower() for name in reader.fieldnames if name}
        required_aliases = {
            "time": ("time", "time_s", "dt", "duration"),
            "current": ("current", "current_a", "i", "amps"),
            "voltage": ("voltage", "voltage_v", "v", "volts"),
            "x": ("x",),
            "y": ("y",),
            "z": ("z",),
        }
        missing = []
        for logical_name, aliases in required_aliases.items():
            if not any(alias in normalized_fields for alias in aliases):
                missing.append(logical_name)
        if missing:
            raise ValueError(f"CSV missing required columns: {', '.join(sorted(missing))}")

        for line_number, row in enumerate(reader, start=2):
            normalized = _normalized_row(row)
            time_s = _parse_float(normalized, required_aliases["time"], line_number)
            current_a = _parse_float(normalized, required_aliases["current"], line_number)
            voltage_v = _parse_float(normalized, required_aliases["voltage"], line_number)
            x = _parse_float(normalized, required_aliases["x"], line_number)
            y = _parse_float(normalized, required_aliases["y"], line_number)
            z = _parse_float(normalized, required_aliases["z"], line_number)

            feedrate_raw = _first_value(normalized, ("feedrate", "speed", "f"))
            feedrate = default_feedrate if feedrate_raw == "" else float(feedrate_raw)
            if feedrate <= 0:
                raise ValueError(f"Feedrate must be > 0 at CSV line {line_number}")

            if current_a < 0:
                raise ValueError(f"Current must be >= 0 at CSV line {line_number}")
            if voltage_v < 0:
                raise ValueError(f"Voltage must be >= 0 at CSV line {line_number}")

            # Optional arc move (G2/G3). Named arc_i/arc_j rather than the
            # bare i/j GRBL itself uses, since "i" already aliases the
            # current_a column above -- a bare "i" column would be ambiguous.
            arc_i_raw = _first_value(normalized, ("arc_i",))
            arc_j_raw = _first_value(normalized, ("arc_j",))
            arc_i = float(arc_i_raw) if arc_i_raw != "" else None
            arc_j = float(arc_j_raw) if arc_j_raw != "" else None
            if (arc_i is None) != (arc_j is None):
                raise ValueError(
                    f"arc_i and arc_j must both be given or both omitted at CSV line {line_number}"
                )
            arc_dir = _first_value(normalized, ("arc_dir",)).strip().lower()
            if arc_dir not in ("", "cw", "ccw"):
                raise ValueError(f"arc_dir must be 'cw' or 'ccw' at CSV line {line_number}")
            arc_cw = arc_dir != "ccw"

            steps.append(
                SequenceStep(
                    time_s=time_s,
                    current_a=current_a,
                    voltage_v=voltage_v,
                    x=x,
                    y=y,
                    z=z,
                    feedrate=feedrate,
                    arc_i=arc_i,
                    arc_j=arc_j,
                    arc_cw=arc_cw,
                )
            )

    if not steps:
        raise ValueError("CSV must include at least one data row")

    return steps


def _interruptible_sleep(seconds: float, stop_event: threading.Event, poll_interval: float = 0.05) -> bool:
    """Sleep up to *seconds*, waking early if *stop_event* is set.

    Returns True if the sleep was interrupted by the event, False otherwise.
    """
    deadline = time.perf_counter() + seconds
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return False
        if stop_event.wait(timeout=min(poll_interval, remaining)):
            return True


def expand_loop_steps(steps: list[SequenceStep], loops: int) -> list[SequenceStep]:
    if loops < 1:
        raise ValueError("loops must be at least 1")
    if loops == 1:
        return list(steps)
    return steps * loops


def list_serial_ports() -> list[tuple[str, str]]:
    """Return ``(device, description)`` for every serial port visible to the OS."""
    return [(p.device, p.description or p.device) for p in _list_ports.comports()]


def find_grbl_port(
    baudrate: int = 115200,
    timeout_s: float = 2.5,
    exclude: set[str] | None = None,
) -> str | None:
    """Probe every serial port for a GRBL welcome banner; return the first match.

    Only sends the same reset/handshake bytes ``GrblController`` uses to
    connect (never G-code), so probing a non-GRBL device is harmless.  Used
    by ``--grbl-port auto`` and the GUI "Auto-detect" buttons.
    """
    exclude = exclude or set()
    for device, _desc in list_serial_ports():
        if device in exclude:
            continue
        # On Linux, /dev/ttyS* are motherboard 16550 UARTs — GRBL
        # controllers always connect via USB-serial (/dev/ttyUSB* or
        # /dev/ttyACM*).  Skipping them saves ~16 s on machines that
        # expose dozens of unused onboard ports.
        if sys.platform == "linux" and device.startswith("/dev/ttyS"):
            continue
        try:
            probe = pyserial.Serial(device, baudrate, timeout=0.5)
        except (OSError, pyserial.SerialException):
            continue
        try:
            time.sleep(0.3)
            probe.reset_input_buffer()
            probe.write(b"\x18")  # GRBL soft-reset -- forces a fresh banner
            deadline = time.time() + timeout_s
            banner = b""
            while time.time() < deadline:
                waiting = probe.in_waiting
                if waiting:
                    banner += probe.read(waiting)
                    if b"grbl" in banner.lower():
                        return device
                time.sleep(0.1)
        except (OSError, pyserial.SerialException):
            continue
        finally:
            probe.close()
    return None


# Where a "Zero Here" is persisted so a *later* connection (a genuine
# reconnect, or the separate connection the main run GUI opens) can offer to
# restore the same physical zero mark. See GrblController.set_zero /
# reapply_saved_zero for the math and the power-cycle caveat -- this file is
# only meaningful for as long as the controller has stayed continuously
# powered since it was written.
_ZERO_STATE_PATH = Path(__file__).with_name("grbl_zero_state.json")


def _parse_position_xyz(status: str) -> tuple[float, float, float] | None:
    """Extract (X, Y, Z) from a GRBL ``?`` status report's MPos/WPos field."""
    for part in status.split("|"):
        if part.startswith("MPos:") or part.startswith("WPos:"):
            coords = part.split(":", 1)[1].split(",")
            if len(coords) >= 3:
                try:
                    return float(coords[0]), float(coords[1]), float(coords[2])
                except ValueError:
                    return None
    return None


class GrblController:
    """Communicates with a GRBL-based CNC/laser controller via raw serial.

    GRBL's protocol is simple: send a G-code line terminated with ``\\n``,
    then read back ``ok`` or ``error:N``.  No line numbers, no checksums.

    Many CNC/laser boards (e.g. ACMER's) run a customized GRBL fork on a
    non-Arduino MCU that doesn't reset when the serial port is opened, and
    some alter the welcome-banner text.  The connect handshake below copes
    with both: it forces a GRBL soft-reset if no banner shows up on its own,
    and falls back to a live status query if the banner text doesn't match
    the standard ``Grbl`` string.

    Parameters
    ----------
    serial_port:
        e.g. ``COM3`` on Windows or ``/dev/ttyUSB0`` on Linux.  Pass
        ``"auto"`` to probe all visible ports for a GRBL device.
    baudrate:
        GRBL default is 115200 (not 250000 like Marlin).
    connect_timeout_s:
        How long to wait for the ``Grbl ...`` welcome banner.
    x_max, y_max, z_max:
        Optional software-side work-area limits in mm.  Coordinates sent
        via ``send_move`` are clamped to these ranges.  Pass ``None`` to
        skip clamping (rely on GRBL's own ``$130``-``$132`` soft-limits).

        .. warning::
            Homing (``$H``) is **deliberately not supported**.  Do not
            send ``$H`` to the controller — it may crash the gantry into
            its limits if end-stop switches are not configured correctly.
    """

    def __init__(
        self,
        serial_port: str,
        baudrate: int = 115200,
        connect_timeout_s: float = 10.0,
        x_max: float | None = None,
        y_max: float | None = None,
        z_max: float | None = None,
    ):
        if serial_port.strip().lower() == "auto":
            detected = find_grbl_port(baudrate)
            if detected is None:
                raise RuntimeError(
                    "Auto-detect: no GRBL controller found on any serial port."
                )
            serial_port = detected

        self._serial = pyserial.Serial(serial_port, baudrate, timeout=1.0)
        # Give any hardware auto-reset (DTR toggle on Arduino-style boards)
        # time to finish booting before we start reading.
        time.sleep(2.0)
        self._drain_buffer()

        banner = self._read_banner(connect_timeout_s)
        if not banner:
            # Boards without an auto-reset circuit (common on non-Arduino
            # MCUs like the ones ACMER uses) stay running silently on
            # port-open and never print a fresh banner on their own.  Force
            # one with a GRBL soft-reset.
            self._serial.write(b"\x18")
            time.sleep(0.5)
            banner = self._read_banner(connect_timeout_s)

        if banner and b"grbl" not in banner.lower():
            # Got *something* back, just not the standard "Grbl" banner text
            # (some custom forks rename it).  Confirm it's really GRBL by
            # checking it answers the real-time status query.
            self._serial.timeout = 0.5
            if not self._read_status_report():
                self._serial.close()
                raise RuntimeError(
                    f"Unexpected response on {serial_port} (not GRBL?). "
                    f"Received: {banner!r}"
                )

        if not banner:
            self._serial.close()
            raise RuntimeError(
                f"GRBL not detected on {serial_port}.  No response to soft-reset."
            )

        self._serial.timeout = 0.5
        self._drain_buffer()
        self._x_max = x_max
        self._y_max = y_max
        self._z_max = z_max
        self._last_feedrate: float | None = None
        # The G92 offset currently in effect, tracked in software since GRBL
        # only reports WPos/MPos, not the offset itself. Always (0,0,0)
        # right here -- the reset this __init__ just went through (banner or
        # forced soft-reset) clears any previous G92, same as set_zero's
        # docstring explains. Kept updated by set_zero() so raw/absolute
        # position can be recovered later (see reapply_saved_zero).
        self._applied_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)

        print(f"Connected to {banner.decode(errors='replace').strip()}", flush=True)

        # GRBL boots into ALARM state on every reset whenever hard/soft
        # limits are configured, and refuses all motion commands
        # (``error:9``) until unlocked.  Homing ($H) is deliberately never
        # sent by this software (see class docstring warning), so clear the
        # lock directly instead -- this does NOT move the gantry, it just
        # tells GRBL to trust the current step position.
        self._clear_alarm()
        self._ensure_wpos_reporting()

    def _ensure_wpos_reporting(self) -> None:
        """Force ``$10=2`` so ``?`` status reports show ``WPos`` (work
        position) instead of GRBL's factory-default ``MPos`` (raw machine
        position).

        This software's *only* origin/zero mechanism is ``set_zero``
        (``G92``, see its docstring) -- there's no homing. ``G92`` only
        offsets ``WPos``; ``MPos`` is unaffected by it. If the controller is
        left on its default ``$10=1`` (MPos), every "Zero Here" in the align
        GUI would silently do nothing to the reported/displayed position,
        and worse, ``run_sequence``'s motion-drift check (which compares the
        live position against a target that *is* in the zeroed/work frame)
        would report a constant false drift equal to the zero offset on
        every run. ``$10`` is a persisted (non-volatile) GRBL setting, so
        this just re-asserts the value this software needs every connect,
        regardless of what was last configured on the controller.
        """
        resp = self._send_line("$10=2")
        if "error" in resp:
            print(
                f"WARNING: could not set $10=2 (WPos status reporting): {resp}. "
                "Position readout and motion-drift checks may not reflect "
                "G92 zero offsets correctly.",
                flush=True,
            )

    def _clear_alarm(self) -> None:
        """Clear an Alarm lock with ``$X`` if GRBL is currently in one.

        Does **not** home (see class docstring) -- just tells GRBL to trust
        wherever it currently thinks it is.  Called after connecting and
        after every ``soft_reset()``, since a reset re-triggers the same
        Alarm lock whenever hard/soft limits are configured.
        """
        status = self._read_status_report()
        if "Alarm" in status:
            print("GRBL in Alarm state; clearing lock with $X (no homing)...", flush=True)
            unlock_resp = self._send_line("$X")
            print(f"Unlock response: {unlock_resp}", flush=True)

    def _read_banner(self, timeout_s: float) -> bytes:
        """Read whatever text GRBL sends after a reset, up to *timeout_s*."""
        deadline = time.time() + timeout_s
        banner = b""
        while time.time() < deadline:
            waiting = self._serial.in_waiting
            if waiting:
                banner += self._serial.read(waiting)
                if b"grbl" in banner.lower():
                    break
            time.sleep(0.1)
        return banner

    def _read_status_report(self, timeout_s: float = 1.0) -> str:
        """Poll GRBL's real-time status (``?``) and return the ``<...>`` line."""
        self._serial.write(b"?")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = self._serial.readline().decode(errors="replace").strip()
            if line.startswith("<"):
                return line
        return ""

    # ------------------------------------------------------------------
    def _drain_buffer(self) -> None:
        """Discard any stale data sitting in the serial input buffer."""
        while self._serial.in_waiting:
            self._serial.read(self._serial.in_waiting)
            time.sleep(0.05)

    def _send_line(self, line: str, timeout_s: float = 10.0) -> str:
        """Send one G-code / system line and wait for GRBL's response.

        Raises ``TimeoutError`` if no ``ok``/``error`` reply arrives within
        *timeout_s*.  GRBL should always reply, but a dropped byte or a
        wedged serial link must never hang the caller -- this used to loop
        forever, which froze the whole GUI when called from a jog button.
        """
        self._serial.write(line.encode() + b"\n")
        response_parts: list[str] = []
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            resp = self._serial.readline()
            if not resp:
                continue  # per-read timeout – keep waiting up to the deadline
            decoded = resp.decode(errors="replace").strip()
            if decoded:
                response_parts.append(decoded)
            if "ok" in decoded or "error" in decoded:
                return "\n".join(response_parts)
        raise TimeoutError(
            f"No response from GRBL to {line!r} within {timeout_s:.1f}s "
            f"(received so far: {response_parts!r})"
        )

    # ------------------------------------------------------------------
    #  Public API  (compatible with the old PrinterController)
    # ------------------------------------------------------------------

    def send_move(self, x: float, y: float, z: float, feedrate: float) -> None:
        """Queue a linear move to absolute coordinates (mm, G90)."""
        # Clamp to work area if limits are configured
        clamped_x = max(0.0, min(x, self._x_max)) if self._x_max is not None else x
        clamped_y = max(0.0, min(y, self._y_max)) if self._y_max is not None else y
        clamped_z = max(0.0, min(z, self._z_max)) if self._z_max is not None else z
        if (clamped_x, clamped_y, clamped_z) != (x, y, z):
            print(
                f"WARNING: commanded move ({x:.3f}, {y:.3f}, {z:.3f}) exceeds the "
                f"configured work area -- clamped to ({clamped_x:.3f}, {clamped_y:.3f}, "
                f"{clamped_z:.3f}). The gantry will dwell at the clamped position, "
                "not the CSV's original coordinate.",
                flush=True,
            )
        x, y, z = clamped_x, clamped_y, clamped_z

        if self._last_feedrate != feedrate:
            resp1 = self._send_line(f"G1 F{feedrate:.2f}")
            if "error" in resp1:
                print(f"GRBL error setting feedrate: {resp1}", flush=True)
            self._last_feedrate = feedrate
        resp2 = self._send_line(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f}")
        if "error" in resp2:
            print(f"GRBL error on move: {resp2}", flush=True)

    def send_arc(
        self,
        from_x: float,
        from_y: float,
        x: float,
        y: float,
        z: float,
        i: float,
        j: float,
        feedrate: float,
        cw: bool = True,
    ) -> None:
        """Queue a circular arc move (G2/G3) to absolute X/Y (mm, G90).

        *i*, *j* are GRBL's native arc-center offsets: the vector from the
        arc's start point (*from_x*, *from_y*, the previous step's target)
        to its center.

        Unlike ``send_move``, this does **not** clamp to the configured
        work-area limits. Clamping only the endpoint would leave *i*/*j*
        pointing at a center that no longer matches the (clamped) start and
        end points, and GRBL rejects an arc whose endpoint isn't actually on
        the circle its center describes. Instead the full circle's bounding
        box is checked up front and rejected clearly, rather than sent as a
        command GRBL would refuse anyway.
        """
        z = max(0.0, min(z, self._z_max)) if self._z_max is not None else z

        cx, cy = from_x + i, from_y + j
        radius = math.hypot(i, j)
        for limit, lo, hi, axis in (
            (self._x_max, cx - radius, cx + radius, "X"),
            (self._y_max, cy - radius, cy + radius, "Y"),
        ):
            if limit is not None and (lo < 0.0 or hi > limit):
                raise ValueError(
                    f"Arc centered at ({cx:.3f}, {cy:.3f}) r={radius:.3f} would "
                    f"exceed the configured {axis} work area [0, {limit}] -- "
                    "refusing to send (clamping would corrupt the arc geometry)."
                )

        if self._last_feedrate != feedrate:
            self._last_feedrate = feedrate
            feed_word = f" F{feedrate:.2f}"
        else:
            feed_word = ""
        cmd = "G2" if cw else "G3"
        resp = self._send_line(
            f"{cmd} X{x:.3f} Y{y:.3f} Z{z:.3f} I{i:.3f} J{j:.3f}{feed_word}"
        )
        if "error" in resp:
            print(f"GRBL error on arc move: {resp}", flush=True)

    def set_zero(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Redefine the current physical position as (*x*, *y*, *z*) via ``G92``.

        This does **not** move the gantry -- it only changes what GRBL calls
        the current spot.  It's the only way to correlate GRBL's coordinates
        to a physical reference point on hardware with no limit switches /
        homing: without it, GRBL just assumes wherever it happens to be at
        the moment of a reset (power-on, or the soft-reset this class issues
        on connect) is (0,0,0), which may not be true.

        The offset itself is **not persisted by GRBL** -- it's lost on the
        next reset the same way GRBL's whole position model is, so it must
        be re-applied each time a fresh connection is made if it's still
        needed. This method separately writes it to a small state file (see
        ``reapply_saved_zero``) so a *later* connection can restore the same
        physical mark automatically, but that restore is only valid if the
        controller was never fully powered off in between -- see that
        method's docstring.
        """
        pos_before = _parse_position_xyz(self.get_status())
        resp = self._send_line(f"G92 X{x:.3f} Y{y:.3f} Z{z:.3f}")
        if "error" in resp:
            print(f"GRBL error setting zero: {resp}", flush=True)
            return
        if pos_before is not None:
            # raw = the absolute/uncalibrated position GRBL would call MPos,
            # recovered from the just-reported WPos plus whatever offset was
            # in effect *before* this call (both in the same frame, since
            # neither has moved between the status query above and the G92
            # taking effect). This is the value that stays meaningful across
            # a soft-reset/reconnect (unlike WPos, which resets to 0 offset
            # each time) -- see reapply_saved_zero.
            raw = tuple(p + o for p, o in zip(pos_before, self._applied_offset))
            self._applied_offset = (raw[0] - x, raw[1] - y, raw[2] - z)
            self._save_zero_state(self._applied_offset)

    @staticmethod
    def _save_zero_state(offset: tuple[float, float, float]) -> None:
        try:
            _ZERO_STATE_PATH.write_text(
                json.dumps({"offset_xyz": list(offset)}), encoding="utf-8"
            )
        except OSError as exc:
            print(f"WARNING: could not save zero state to {_ZERO_STATE_PATH}: {exc}", flush=True)

    def has_saved_zero(self) -> bool:
        """Whether a previously-saved zero mark exists on disk to reapply."""
        return _ZERO_STATE_PATH.exists()

    def reapply_saved_zero(self) -> bool:
        """Restore the physical zero mark last set by ``set_zero`` (e.g. via
        the align GUI's "Zero Here"), even though this is a fresh connection
        that already lost GRBL's own G92 offset.

        Works because GRBL's *raw* position tracking (what it calls MPos) is
        derived from the stepper step count and survives a soft-reset --
        only the G92 work offset itself gets cleared. So the saved offset
        plus the live raw position is enough to recompute and resend an
        equivalent G92, reproducing the exact same physical reference point.

        .. warning::
            This is only correct if the controller has been continuously
            powered since the zero was saved. A real power-cycle resets
            GRBL's raw position tracking too (there's no homing hardware
            here to re-anchor it), which silently invalidates the saved
            offset -- reapplying it then would zero to the wrong physical
            spot. Callers must only invoke this when they can vouch the
            controller was never powered off in between, and should treat it
            as a deliberate, confirmed action rather than an automatic one.

        Returns ``False`` (does nothing) if no saved zero exists or the
        state file can't be read; ``True`` if it was reapplied.
        """
        try:
            saved = json.loads(_ZERO_STATE_PATH.read_text(encoding="utf-8"))
            offset_saved = tuple(float(v) for v in saved["offset_xyz"])
        except (OSError, ValueError, KeyError, TypeError):
            return False
        pos_now = _parse_position_xyz(self.get_status())
        if pos_now is None:
            return False
        raw_now = tuple(p + o for p, o in zip(pos_now, self._applied_offset))
        target = tuple(r - o for r, o in zip(raw_now, offset_saved))
        self.set_zero(*target)
        return True

    def get_status(self) -> str:
        """Return GRBL's real-time status report (``?`` command)."""
        return self._read_status_report(timeout_s=0.5)

    def soft_reset(self) -> None:
        """Send Ctrl-X (0x18) to soft-reset GRBL."""
        self._serial.write(b"\x18")
        time.sleep(1.0)
        self._drain_buffer()

    def feed_hold(self) -> None:
        """Send a real-time Feed Hold (``!``).

        Unlike G-code lines, real-time commands bypass the normal
        line-by-line ok/error handshake entirely and take effect
        immediately -- this decelerates any in-progress move to a stop
        using GRBL's own configured acceleration (``$120``-``$122``)
        rather than abruptly cutting step pulses, which risks lost steps
        on a machine with no encoder to ever detect that. Returns as soon
        as the byte is written; it does not wait for the hold to actually
        complete (see ``wait_for_hold``).
        """
        self._serial.write(b"!")

    def wait_for_hold(self, timeout_s: float = 10.0) -> bool:
        """Block until GRBL is no longer actively moving.

        Polls status until it's anything other than ``Run`` or the
        in-progress ``Hold:1`` (still decelerating) -- i.e. ``Hold:0``,
        ``Idle``, or ``Alarm`` all count as stopped.  Waiting for this
        before a soft-reset avoids truncating an in-progress deceleration
        ramp, which is the entire point of using Feed Hold over a raw
        reset in the first place.

        Returns False if *timeout_s* elapses first -- treat that as
        best-effort and proceed anyway rather than blocking forever.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            status = self._read_status_report(timeout_s=0.5)
            if status and "Run" not in status and "Hold:1" not in status:
                return True
            time.sleep(0.1)
        return False

    def stop_motion(self) -> None:
        """Bring any in-progress move to an immediate, controlled stop and
        discard whatever's left of the current command queue.

        This is the abort primitive used to cut a running sequence short:
        Feed Hold decelerates smoothly (see ``feed_hold``/``wait_for_hold``)
        instead of yanking the steppers to a halt, then a soft-reset flushes
        the remaining queued G-code so a follow-up move (e.g. returning to
        zero) isn't racing the rest of the original path.

        Best-effort and never raises -- this *is* the abort path, so a
        hiccup here must not get stuck or escalate into a bigger failure of
        its own.
        """
        try:
            self.feed_hold()
            if not self.wait_for_hold():
                print(
                    "WARNING: feed hold did not confirm a stop within the timeout -- "
                    "resetting anyway, but deceleration may not have finished.",
                    flush=True,
                )
        except Exception as exc:
            print(f"WARNING: feed hold did not confirm a clean stop: {exc}", flush=True)
        try:
            self.soft_reset()
            # GRBL forgets its active feed rate across a reset (a bare
            # G1/G2/G3 with no F word since gets error:22) -- invalidate the
            # cache so the next send_move/send_arc is forced to resend it.
            self._last_feedrate = None
            self._clear_alarm()
        except Exception as exc:
            print(f"WARNING: reset after stop failed: {exc}", flush=True)

    def disconnect(self) -> None:
        """Close the serial connection."""
        self._serial.close()


def _parse_mpos(status: str) -> tuple[float, float] | None:
    """Extract (X, Y) from a GRBL ``?`` status report's MPos/WPos field."""
    for part in status.split("|"):
        if part.startswith("MPos:") or part.startswith("WPos:"):
            coords = part.split(":", 1)[1].split(",")
            if len(coords) >= 2:
                try:
                    return float(coords[0]), float(coords[1])
                except ValueError:
                    return None
    return None


def _with_live_first_step_timing(
    steps: list[SequenceStep], printer: GrblController,
) -> list[SequenceStep]:
    """Replace the first step's ``time_s`` with one computed from GRBL's
    actual current position, if that's longer than what's already there.

    ``pattern_generator.generate_heat_sequence`` has no live connection to
    query at CSV-generation time, so when its caller doesn't supply a
    ``start_position`` it falls back to a fixed 0.5s guess for the very
    first move regardless of the real distance (see its docstring). If the
    gantry is actually far from that first target -- e.g. a previous run's
    return-to-origin move didn't fully complete -- the software then waits
    only 0.5s before sending the next several lines while GRBL is still
    physically completing a much longer first move. Each of those extra
    lines still gets queued (GRBL's planner buffer has room for more than
    one), but if the backlog keeps growing across several understated
    steps, GRBL eventually stops returning ``ok`` for new lines until a
    queue slot frees up -- which looks like a silent ``_send_line``
    timeout several steps later, not on the first (undertimed) one itself.
    Recomputing the first step's duration from where GRBL actually is
    closes that gap at its source.

    Never *shortens* the original time_s -- only lengthens it if the real
    distance needs more time than the CSV already budgeted, since the
    original value may have been a deliberately slow first move.
    """
    if not steps:
        return steps
    pos = _parse_mpos(printer.get_status())
    if pos is None:
        return steps
    first = steps[0]
    if first.feedrate <= 0:
        return steps
    real_time_s = math.dist(pos, (first.x, first.y)) * 60.0 / first.feedrate
    if real_time_s <= first.time_s:
        return steps
    return [replace(first, time_s=real_time_s)] + steps[1:]


_DRIFT_CHECK_INTERVAL_S = 5.0
_DRIFT_WARN_MM = 3.0


def _check_motion_drift(
    printer: GrblController,
    now: float,
    target_xy: tuple[float, float],
    metadata: RunMetadata | None,
) -> float:
    """Compare GRBL's real position against where the schedule currently
    expects it to be (``target_xy``, the most recently commanded step's
    target), and warn if they've diverged by more than ``_DRIFT_WARN_MM``.

    This is a *following-error* check -- actual position vs. the schedule's
    target position at the same instant -- not a "distance covered per
    window" check. An earlier version compared the schedule's *total path
    length* traveled in a window (correctly summing arc + segment lengths)
    against the *straight-line displacement* between two sampled points.
    That comparison is only valid for a path that doesn't double back on
    itself -- but every dwell pattern here does (a ring's two semicircle
    arcs return near their start; a line or raster reverses direction
    repeatedly), so a sampling window that happens to catch a
    near-closed loop reports near-zero "coverage" even at perfect real
    speed, while one that catches a half-loop reports a much higher
    fraction -- false drift signals driven by the pattern's own geometry
    and sampling phase, not by real hardware lag. Comparing like-for-like
    positions instead sidesteps that entirely.

    Call this only every few seconds of wall-clock time (see
    ``_DRIFT_CHECK_INTERVAL_S``), never per-step -- a single extra ``?``
    query is cheap, but doing it on every fine-grained dwell segment would
    add serial round-trips right where timing is tightest.

    Returns the updated ``last_check_t`` for the caller to carry into the
    next window.
    """
    status = printer.get_status()
    pos = _parse_mpos(status)
    if pos is not None:
        error_mm = math.dist(target_xy, pos)
        if error_mm > _DRIFT_WARN_MM:
            msg = (
                f"WARNING: motion drift -- gantry is {error_mm:.1f}mm from where "
                f"the schedule currently expects it to be (target "
                f"X{target_xy[0]:.2f} Y{target_xy[1]:.2f}, actual "
                f"X{pos[0]:.2f} Y{pos[1]:.2f}). Physical position may be "
                "lagging the commanded schedule (e.g. dwell_time_s/feedrate "
                "too aggressive for the hardware)."
            )
            print(msg, flush=True)
            if metadata is not None:
                metadata.drift_warnings.append(msg)
    return now


def connect_dps(modbus_port: str, ini_path: Path, address: int, baudrate: int) -> Dps5005:
    serial_modbus = Serial_modbus(modbus_port, address, baudrate, 8)
    limits = Import_limits(str(ini_path))
    return Dps5005(serial_modbus, limits)


def run_sequence(
    steps: list[SequenceStep],
    dps: Dps5005 | None,
    printer: GrblController | None,
    time_mode: str,
    dry_run: bool,
    stop_event: threading.Event | None = None,
    on_step: Callable[[int, int], None] | None = None,
    return_to_origin: bool = True,
    cameras: CameraController | None = None,
    record_dir: Path | None = None,
    metadata: RunMetadata | None = None,
    live_preview: bool = False,
    preview_queue: multiprocessing.Queue | None = None,
    preview_fps: float = 15.0,
    loops: int = 1,
    cycle_delay_s: float = 0.0,
    sequence_csv_path: Path | None = None,
) -> None:
    """Run *steps* against the hardware (or in ``dry_run``/planning-only mode).

    ``loops``/``cycle_delay_s``: *steps* is already the fully expanded,
    flattened list (one full pass through the grid repeated ``loops`` times
    back-to-back -- see ``expand_loop_steps`` and the GUI's own ``rows *
    loops``), not something this function loops over itself. What it *does*
    do with ``loops`` is locate the boundary between one pass and the next
    (``len(steps) // loops``) so it can insert ``cycle_delay_s`` of dwell
    time there -- a plain flattened repeat has no gap between cycles at all
    otherwise, since consecutive steps' ``time_s`` values just pick up where
    the previous cycle left off.

    ``sequence_csv_path``, if given, is the exact CSV *steps* was parsed
    from; it gets duplicated next to the run's metadata JSON (see the
    metadata-saving block below) so the metadata alone is enough to recover
    every parameter of the run without cross-referencing GUI state that may
    have since changed.
    """
    if time_mode not in {"step", "absolute"}:
        raise ValueError("time_mode must be one of: step, absolute")

    if not dry_run and dps is None:
        raise ValueError("dps instance is required when dry_run is False")

    if printer is not None and not dry_run:
        steps = _with_live_first_step_timing(steps, printer)

    home_step: SequenceStep | None = steps[0] if steps else None
    # steps is already the fully flattened loops*cycle list -- see the
    # loops/cycle_delay_s docstring above -- so the boundary between two
    # passes through the grid is just an even split by loop count.
    steps_per_cycle = len(steps) // loops if loops > 0 and len(steps) % max(loops, 1) == 0 else len(steps)
    previous_t = 0.0
    scheduled_elapsed = 0.0
    start_monotonic = time.perf_counter()
    last_voltage: float | None = None
    last_current: float | None = None
    prev_x, prev_y, prev_z = 0.0, 0.0, 0.0

    # --- Periodic motion-drift verification state (see _check_motion_drift) ---
    drift_check_t = start_monotonic

    # --- Camera recording / live preview (separate process — never blocks
    # timing loop, and preview runs at its own rate independent of the
    # record rate; see CameraRecorderProcess) ---
    _cam_proc: CameraRecorderProcess | None = None
    _cam_cmd_q: multiprocessing.Queue | None = None
    _cam_res_q: multiprocessing.Queue | None = None
    if cameras is not None and (record_dir is not None or live_preview):
        _cam_cmd_q = multiprocessing.Queue()
        _cam_res_q = multiprocessing.Queue()
        _cam_proc = CameraRecorderProcess(
            cameras=cameras.camera_specs,
            record_fps=cameras.fps,
            cmd_queue=_cam_cmd_q,
            result_queue=_cam_res_q,
            preview_queue=preview_queue if live_preview else None,
            preview_fps=preview_fps,
        )
        _cam_proc.start()
        # Wait for the cameras to actually finish opening before proceeding.
        # Without this, a short sequence (or a preview-only run, which has
        # no "started" handshake of its own) could run to completion and
        # tear the process back down before it ever captured a frame.
        try:
            ready_msg = _cam_res_q.get(timeout=10.0)
            if ready_msg[0] == "error":
                print(f"WARNING: camera setup failed: {ready_msg[1]}", flush=True)
        except Exception:
            print("WARNING: timed out waiting for cameras to become ready", flush=True)

        if record_dir is not None:
            _cam_cmd_q.put(("start", str(record_dir)))
            # Wait for confirmation (with timeout)
            try:
                msg = _cam_res_q.get(timeout=10.0)
                if msg[0] == "started" and metadata is not None:
                    metadata.recording_paths = dict(msg[1])
                    for label, path in msg[1].items():
                        print(f"Recording {label} -> {path}", flush=True)
            except Exception:
                pass

    if metadata is not None:
        metadata.started_utc = datetime.now(timezone.utc).isoformat()
        metadata.total_steps = len(steps)
        metadata.loops = loops
        metadata.cycle_delay_s = cycle_delay_s
        metadata.steps_per_cycle = steps_per_cycle

    steps_completed = 0
    try:
        if not dry_run:
            assert dps is not None  # narrowed by the entry guard
            dps.onoff("w", 1)

        for index, step in enumerate(steps, start=1):
            if stop_event is not None and stop_event.is_set():
                print("\nStop requested — aborting sequence.", flush=True)
                if printer is not None:
                    printer.stop_motion()
                break

            # --- Delay between cycles (repeats of the full grid) ---
            # Fires once per loop boundary, right before the new cycle's
            # first move/V/I is sent -- the gantry and heater just sit at
            # wherever the previous cycle left them for the extra duration.
            # Checked before steps_completed is advanced so an abort during
            # the pause itself doesn't count this step as executed.
            if (
                cycle_delay_s > 0
                and loops > 1
                and index > 1
                and steps_per_cycle > 0
                and (index - 1) % steps_per_cycle == 0
            ):
                cycle_num = (index - 1) // steps_per_cycle + 1
                print(
                    f"--- Cycle {cycle_num}/{loops}: pausing {cycle_delay_s:.1f}s before "
                    "starting the next pass ---",
                    flush=True,
                )
                if stop_event is not None:
                    interrupted = _interruptible_sleep(cycle_delay_s, stop_event)
                    if interrupted:
                        print("\nStop requested — aborting sequence.", flush=True)
                        if printer is not None:
                            printer.stop_motion()
                        break
                else:
                    time.sleep(cycle_delay_s)
                # Shift the schedule's origin forward by the same amount so
                # the upcoming per-step wait_s calculations (based on
                # start_monotonic + scheduled_elapsed) don't see this pause
                # as the run having fallen behind and try to "catch up" by
                # skipping the next few steps' waits.
                scheduled_elapsed += cycle_delay_s

            steps_completed = index
            if not dry_run:
                assert dps is not None  # narrowed by the entry guard
                if last_voltage != step.voltage_v:
                    dps.voltage_set("w", step.voltage_v)
                    last_voltage = step.voltage_v
                if last_current != step.current_a:
                    dps.current_set("w", step.current_a)
                    last_current = step.current_a

            is_arc = step.arc_i is not None and step.arc_j is not None
            if printer is not None:
                if is_arc:
                    printer.send_arc(
                        prev_x, prev_y, step.x, step.y, step.z,
                        step.arc_i, step.arc_j, step.feedrate, cw=step.arc_cw,
                    )
                else:
                    printer.send_move(step.x, step.y, step.z, step.feedrate)

            prev_x, prev_y, prev_z = step.x, step.y, step.z

            print(
                f"Step {index:04d}: "
                f"t={step.time_s:.3f}s "
                f"V={step.voltage_v:.3f} "
                f"I={step.current_a:.3f} "
                f"X={step.x:.3f} Y={step.y:.3f} Z={step.z:.3f} F={step.feedrate:.1f}"
            )

            if on_step is not None:
                on_step(index, len(steps))

            if time_mode == "step":
                delta_s = max(0.0, step.time_s)
            else:
                # Allow loop wrap-around: if time goes backwards, treat it as a
                # new loop iteration starting from 0.
                if step.time_s < previous_t:
                    previous_t = 0.0
                delta_s = step.time_s - previous_t
                previous_t = step.time_s

            scheduled_elapsed += delta_s
            target_time = start_monotonic + scheduled_elapsed
            wait_s = target_time - time.perf_counter()
            if wait_s > 0:
                if stop_event is not None:
                    interrupted = _interruptible_sleep(wait_s, stop_event)
                    if interrupted:
                        print("\nStop requested — aborting sequence.", flush=True)
                        if printer is not None:
                            printer.stop_motion()
                        break
                else:
                    time.sleep(wait_s)

            # --- Periodic (not per-step) real-position drift check ---
            if printer is not None and not dry_run:
                now = time.perf_counter()
                if (now - drift_check_t) >= _DRIFT_CHECK_INTERVAL_S:
                    drift_check_t = _check_motion_drift(
                        printer, now, (prev_x, prev_y), metadata,
                    )
    except Exception as exc:
        if metadata is not None:
            metadata.error = str(exc)
        raise
    finally:
        # --- Stop camera process ---
        # Wrapped end-to-end: a failure anywhere in here (e.g. the recorder
        # process already died and the queue is broken) must not skip the
        # metadata/heater-off/homing steps below.
        if _cam_proc is not None:
            try:
                if record_dir is not None:
                    _cam_cmd_q.put(("stop",))
                    try:
                        msg = _cam_res_q.get(timeout=10.0)
                        if msg[0] == "stopped":
                            print(f"Recording stopped: {msg[1]}", flush=True)
                    except Exception:
                        pass
                _cam_cmd_q.put(("shutdown",))
                _cam_proc.join(timeout=5.0)
                if _cam_proc.is_alive():
                    _cam_proc.terminate()
            except Exception as exc:
                print(f"WARNING: failed to cleanly stop the camera recorder process: {exc}", flush=True)
                if _cam_proc.is_alive():
                    _cam_proc.terminate()

        # --- Write metadata ---
        # Isolated in its own try/except: a failure here (e.g. disk full,
        # bad permissions) must never prevent the heater-off / homing steps
        # below from running.
        if metadata is not None:
            try:
                metadata.completed_utc = datetime.now(timezone.utc).isoformat()
                metadata.duration_s = time.perf_counter() - start_monotonic
                metadata.steps_completed = steps_completed
                metadata.completed = (steps_completed == metadata.total_steps)
                out_dir = record_dir if record_dir is not None else Path(".")
                if sequence_csv_path is not None and sequence_csv_path.exists():
                    # Duplicate the exact CSV that was executed next to the
                    # metadata JSON, so the JSON alone (via csv_copy) is
                    # enough to recover every per-step parameter of this run
                    # -- the original file may since be overwritten,
                    # regenerated with different settings, or moved.
                    out_dir.mkdir(parents=True, exist_ok=True)
                    csv_copy_path = out_dir / f"sequence_{metadata.run_id}.csv"
                    try:
                        shutil.copy2(sequence_csv_path, csv_copy_path)
                        metadata.csv_copy = str(csv_copy_path)
                    except OSError as exc:
                        print(f"WARNING: failed to copy sequence CSV into {out_dir}: {exc}", flush=True)
                save_run_metadata(metadata, out_dir)
            except Exception as exc:
                print(f"WARNING: failed to save run metadata: {exc}", flush=True)

        # --- Heater off ---
        # Safety-critical, so it gets its own try/except too: a comms hiccup
        # here must not skip the gantry-homing / disconnect steps below.
        if not dry_run and dps is not None:
            try:
                dps.onoff("w", 0)
                print("Light turned off.", flush=True)
            except Exception as exc:
                print(
                    f"WARNING: failed to confirm the heater turned off: {exc}. "
                    "Check the DPS5005 manually.",
                    flush=True,
                )

        # --- Return to a known position and release the serial port ---
        if printer is not None:
            try:
                final_xyz: tuple[float, float, float] | None = None
                final_feedrate = 1200.0
                if return_to_origin:
                    print(
                        "Moving to origin: X=0.000 Y=0.000 Z=0.000",
                        flush=True,
                    )
                    final_xyz = (0.0, 0.0, 0.0)
                    final_feedrate = home_step.feedrate if home_step is not None else 1200.0
                    printer.send_move(*final_xyz, final_feedrate)
                elif home_step is not None:
                    final_xyz = (home_step.x, home_step.y, home_step.z)
                    final_feedrate = home_step.feedrate
                    print(
                        f"Moving to initial position: "
                        f"X={final_xyz[0]:.3f} Y={final_xyz[1]:.3f} Z={final_xyz[2]:.3f}",
                        flush=True,
                    )
                    printer.send_move(*final_xyz, final_feedrate)

                if final_xyz is not None:
                    # send_move only confirms GRBL *accepted* the line into
                    # its queue, not that the move physically finished.
                    # Disconnecting right after that (the old behavior)
                    # could cut the port while the gantry is still
                    # traveling, stranding it short of the intended final
                    # position -- which then becomes the *next* run's
                    # (wrong) idea of where it's starting from. Block here
                    # until GRBL reports it's actually stopped, sized to
                    # the move's own real distance/feedrate rather than a
                    # flat guess.
                    dist = math.dist((prev_x, prev_y, prev_z), final_xyz)
                    move_timeout = max(10.0, dist * 60.0 / final_feedrate + 5.0)
                    if not printer.wait_for_hold(timeout_s=move_timeout):
                        print(
                            "WARNING: gantry did not confirm it reached the "
                            f"final position ({final_xyz[0]:.3f}, {final_xyz[1]:.3f}, "
                            f"{final_xyz[2]:.3f}) within {move_timeout:.1f}s -- it "
                            "may have stopped short. The next run measures its "
                            "actual starting position live, but double-check the "
                            "physical position before trusting it.",
                            flush=True,
                        )
            except Exception as exc:
                print(f"WARNING: failed to return gantry to a safe position: {exc}", flush=True)
            finally:
                # Always release the port, even if the homing move above
                # timed out -- otherwise it's left open/orphaned.
                printer.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run synchronized printer + DPS control from a CSV schedule."
    )
    parser.add_argument("--csv", type=Path, default=None, help="Input schedule CSV path")
    parser.add_argument(
        "--list-ports", action="store_true",
        help="List available serial ports and exit",
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Repeat the schedule this many times (helper loop generation)",
    )
    parser.add_argument(
        "--cycle-delay",
        type=float,
        default=0.0,
        help="Pause this many seconds between repeats of the schedule when --loops > 1 "
             "(no-op for --loops 1)",
    )
    parser.add_argument(
        "--time-mode",
        choices=["step", "absolute"],
        default="step",
        help="Interpret time column as per-step delay or absolute schedule time",
    )
    parser.add_argument(
        "--default-feedrate",
        type=float,
        default=1200.0,
        help="Fallback feedrate when CSV row has no feedrate/speed",
    )

    parser.add_argument("--modbus-port", default="", help="DPS Modbus serial port (required unless --dry-run)")
    parser.add_argument("--modbus-address", type=int, default=1, help="DPS Modbus address")
    parser.add_argument("--modbus-baud", type=int, default=9600, help="DPS Modbus baud rate")
    parser.add_argument(
        "--limits-ini",
        type=Path,
        default=Path(__file__).with_name("dps5005_limits.ini"),
        help="Path to DPS limits ini",
    )

    parser.add_argument(
        "--grbl-port", default="",
        help="GRBL controller serial port (e.g. COM3), or 'auto' to probe all ports",
    )
    parser.add_argument("--grbl-baud", type=int, default=115200, help="GRBL serial baud rate")
    parser.add_argument(
        "--x-max", type=float, default=None,
        help="Work-area X maximum in mm (software clamp; omit to use GRBL soft-limits)",
    )
    parser.add_argument(
        "--y-max", type=float, default=None,
        help="Work-area Y maximum in mm (software clamp; omit to use GRBL soft-limits)",
    )
    parser.add_argument(
        "--z-max", type=float, default=None,
        help="Work-area Z maximum in mm (software clamp; omit to use GRBL soft-limits)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and print schedule without sending commands to hardware",
    )
    parser.add_argument(
        "--return-to-first-position",
        action="store_true",
        default=False,
        help="After the sequence ends return to the first CSV position instead of 0,0,0 (default: return to origin)",
    )
    parser.add_argument(
        "--camera", action="append", default=[], metavar="LABEL:INDEX",
        help="Add a camera as label:device_index for recording (repeatable, "
             "e.g. --camera top:0 --camera side:1). Use --list-cameras to "
             "see what's plugged in.",
    )
    parser.add_argument(
        "--list-cameras", action="store_true",
        help="Probe device indices 0-9 for a responsive camera and exit",
    )
    parser.add_argument("--cam-fps", type=float, default=15.0, help="Camera recording framerate")
    parser.add_argument(
        "--record-dir", type=Path, default=None,
        help="Directory to save camera recordings (required if --camera is set)",
    )
    return parser.parse_args()


def _parse_camera_specs(raw: list[str]) -> list[CameraSpec]:
    specs: list[CameraSpec] = []
    for item in raw:
        label, sep, idx_str = item.partition(":")
        label = label.strip()
        if not sep or not label:
            raise SystemExit(f"error: --camera must be LABEL:INDEX, got {item!r}")
        try:
            idx = int(idx_str.strip())
        except ValueError as exc:
            raise SystemExit(f"error: --camera index must be an integer, got {item!r}") from exc
        specs.append((label, idx))
    return specs


def list_available_cameras(max_index: int = 10) -> list[int]:
    """Return every device index in ``range(max_index)`` that opens successfully."""
    from camera_controller import _open_capture

    found = []
    for index in range(max_index):
        cap = _open_capture(index, attempts=1, retry_delay=0.0)
        if cap.isOpened():
            found.append(index)
        cap.release()
    return found


def _stdin_stop_listener(stop_event: threading.Event) -> None:
    """Block on stdin until the user types 'q' and presses Enter, then signal a stop."""
    try:
        while True:
            line = sys.stdin.readline()
            if line.strip().lower() == "q":
                break
    except Exception:
        pass
    stop_event.set()


def main() -> None:
    args = parse_args()

    if args.list_ports:
        for device, description in list_serial_ports():
            print(f"{device}\t{description}")
        return

    if args.list_cameras:
        for index in list_available_cameras():
            print(index)
        return

    if args.csv is None:
        raise SystemExit("error: --csv is required (unless --list-ports/--list-cameras)")

    steps = read_sequence_csv(args.csv, default_feedrate=args.default_feedrate)
    looped_steps = expand_loop_steps(steps, args.loops)

    if args.dry_run:
        run_sequence(
            looped_steps, dps=None, printer=None, time_mode=args.time_mode, dry_run=True,
            loops=args.loops, cycle_delay_s=args.cycle_delay,
        )
        return

    if not args.modbus_port.strip():
        raise SystemExit("error: --modbus-port is required when not using --dry-run")

    dps = connect_dps(
        modbus_port=args.modbus_port,
        ini_path=args.limits_ini,
        address=args.modbus_address,
        baudrate=args.modbus_baud,
    )
    printer = (
        GrblController(
            args.grbl_port,
            baudrate=args.grbl_baud,
            x_max=args.x_max,
            y_max=args.y_max,
            z_max=args.z_max,
        )
        if args.grbl_port.strip()
        else None
    )

    # --- Cameras ---
    camera_specs = _parse_camera_specs(args.camera)
    cameras: CameraController | None = None
    if camera_specs:
        cameras = CameraController(cameras=camera_specs, fps=args.cam_fps)

    # --- Metadata ---
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    metadata = RunMetadata(
        run_id=run_id,
        csv_file=str(args.csv),
        csv_format="raw",
        source_csv=str(args.csv),
        loops=args.loops,
        cycle_delay_s=args.cycle_delay,
        time_mode=args.time_mode,
        default_feedrate=args.default_feedrate,
        dry_run=args.dry_run,
        return_to_origin=not args.return_to_first_position,
        grbl_port=args.grbl_port,
        grbl_baud=args.grbl_baud,
        x_max=args.x_max,
        y_max=args.y_max,
        z_max=args.z_max,
        dps_port=args.modbus_port,
        dps_address=args.modbus_address,
        dps_baud=args.modbus_baud,
        cameras=dict(camera_specs),
        cam_fps=args.cam_fps,
        record_dir=str(args.record_dir) if args.record_dir else "",
    )

    stop_event = threading.Event()
    listener = threading.Thread(target=_stdin_stop_listener, args=(stop_event,), daemon=True)
    listener.start()
    print("Sequence running. Type 'q' and press Enter at any time to stop.", flush=True)

    run_sequence(
        looped_steps,
        dps=dps,
        printer=printer,
        time_mode=args.time_mode,
        dry_run=False,
        stop_event=stop_event,
        return_to_origin=not args.return_to_first_position,
        cameras=cameras,
        record_dir=args.record_dir,
        metadata=metadata,
        loops=args.loops,
        cycle_delay_s=args.cycle_delay,
        sequence_csv_path=args.csv,
    )


if __name__ == "__main__":
    main()