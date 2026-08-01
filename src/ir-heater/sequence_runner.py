from __future__ import annotations

import argparse
import csv
import json
import multiprocessing
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import serial as pyserial

from camera_controller import CameraController, CameraRecorderProcess
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
    loops: int = 1
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
    cam0_id: int | None = None
    cam1_id: int | None = None
    cam_fps: float = 15.0
    record_dir: str = ""
    recording_paths: list[str] = field(default_factory=list)

    # Results
    total_steps: int = 0
    steps_completed: int = 0
    error: str = ""


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

            steps.append(
                SequenceStep(
                    time_s=time_s,
                    current_a=current_a,
                    voltage_v=voltage_v,
                    x=x,
                    y=y,
                    z=z,
                    feedrate=feedrate,
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


class GrblController:
    """Communicates with a GRBL-based CNC/laser controller via raw serial.

    GRBL's protocol is simple: send a G-code line terminated with ``\\n``,
    then read back ``ok`` or ``error:N``.  No line numbers, no checksums.

    Parameters
    ----------
    serial_port:
        e.g. ``COM3`` on Windows or ``/dev/ttyUSB0`` on Linux.
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
        self._serial = pyserial.Serial(serial_port, baudrate, timeout=1.0)
        self._serial.write(b"\r\n\r\n")
        time.sleep(0.5)

        # Read welcome banner ("Grbl 1.1f ['$' for help]\r\n")
        deadline = time.time() + connect_timeout_s
        banner = b""
        while time.time() < deadline:
            waiting = self._serial.in_waiting
            if waiting:
                banner += self._serial.read(waiting)
                if b"Grbl" in banner:
                    break
            time.sleep(0.1)

        if b"Grbl" not in banner:
            self._serial.close()
            raise RuntimeError(
                f"GRBL not detected on {serial_port}.  Received: {banner!r}"
            )

        self._serial.timeout = 0.5
        self._drain_buffer()
        self._x_max = x_max
        self._y_max = y_max
        self._z_max = z_max

        print(f"Connected to {banner.decode().strip()}", flush=True)

    # ------------------------------------------------------------------
    def _drain_buffer(self) -> None:
        """Discard any stale data sitting in the serial input buffer."""
        while self._serial.in_waiting:
            self._serial.read(self._serial.in_waiting)
            time.sleep(0.05)

    def _send_line(self, line: str) -> str:
        """Send one G-code / system line and wait for GRBL's response."""
        self._serial.write(line.encode() + b"\n")
        response_parts: list[str] = []
        while True:
            resp = self._serial.readline()
            if not resp:
                continue  # timeout – keep waiting
            decoded = resp.decode(errors="replace").strip()
            if decoded:
                response_parts.append(decoded)
            if "ok" in decoded or "error" in decoded:
                break
        return "\n".join(response_parts)

    # ------------------------------------------------------------------
    #  Public API  (compatible with the old PrinterController)
    # ------------------------------------------------------------------

    def send_move(self, x: float, y: float, z: float, feedrate: float) -> None:
        """Queue a linear move to absolute coordinates (mm, G90)."""
        # Clamp to work area if limits are configured
        x = max(0.0, min(x, self._x_max)) if self._x_max is not None else x
        y = max(0.0, min(y, self._y_max)) if self._y_max is not None else y
        z = max(0.0, min(z, self._z_max)) if self._z_max is not None else z

        resp1 = self._send_line(f"G1 F{feedrate:.2f}")
        if "error" in resp1:
            print(f"GRBL error setting feedrate: {resp1}", flush=True)
        resp2 = self._send_line(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f}")
        if "error" in resp2:
            print(f"GRBL error on move: {resp2}", flush=True)

    def get_status(self) -> str:
        """Return GRBL's real-time status report (``?`` command)."""
        self._serial.write(b"?\n")
        return self._serial.readline().decode(errors="replace").strip()

    def soft_reset(self) -> None:
        """Send Ctrl-X (0x18) to soft-reset GRBL."""
        self._serial.write(b"\x18")
        time.sleep(1.0)
        self._drain_buffer()

    def disconnect(self) -> None:
        """Close the serial connection."""
        self._serial.close()


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
) -> None:
    if time_mode not in {"step", "absolute"}:
        raise ValueError("time_mode must be one of: step, absolute")

    if not dry_run and dps is None:
        raise ValueError("dps instance is required when dry_run is False")

    if not dry_run:
        assert dps is not None  # narrowed by the guard above
        dps.onoff("w", 1)

    home_step: SequenceStep | None = steps[0] if steps else None
    previous_t = 0.0
    scheduled_elapsed = 0.0
    start_monotonic = time.perf_counter()
    last_voltage: float | None = None
    last_current: float | None = None

    # --- Camera recording (separate process — never blocks timing loop) ---
    _cam_proc: CameraRecorderProcess | None = None
    _cam_cmd_q: multiprocessing.Queue | None = None
    _cam_res_q: multiprocessing.Queue | None = None
    if cameras is not None and record_dir is not None:
        _cam_cmd_q = multiprocessing.Queue()
        _cam_res_q = multiprocessing.Queue()
        _cam_proc = CameraRecorderProcess(
            cam0_id=cameras.cam0_id,
            cam1_id=cameras.cam1_id,
            fps=cameras.fps,
            cmd_queue=_cam_cmd_q,
            result_queue=_cam_res_q,
        )
        _cam_proc.start()
        _cam_cmd_q.put(("start", str(record_dir)))
        # Wait for confirmation (with timeout)
        try:
            msg = _cam_res_q.get(timeout=10.0)
            if msg[0] == "started" and metadata is not None:
                metadata.recording_paths = [msg[1], msg[2]]
                print(f"Recording cam0 -> {msg[1]}", flush=True)
                print(f"Recording cam1 -> {msg[2]}", flush=True)
        except Exception:
            pass

    if metadata is not None:
        metadata.started_utc = datetime.now(timezone.utc).isoformat()
        metadata.total_steps = len(steps)

    steps_completed = 0
    steps_completed = 0
    try:
        for index, step in enumerate(steps, start=1):
            if stop_event is not None and stop_event.is_set():
                print("\nStop requested — aborting sequence.", flush=True)
                break
            steps_completed = index
            if not dry_run:
                assert dps is not None  # narrowed by the entry guard
                if last_voltage != step.voltage_v:
                    dps.voltage_set("w", step.voltage_v)
                    last_voltage = step.voltage_v
                if last_current != step.current_a:
                    dps.current_set("w", step.current_a)
                    last_current = step.current_a

            if printer is not None:
                printer.send_move(step.x, step.y, step.z, step.feedrate)

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
                        break
                else:
                    time.sleep(wait_s)
    finally:
        # --- Stop camera process ---
        if _cam_proc is not None:
            _cam_cmd_q.put(("stop",))
            try:
                msg = _cam_res_q.get(timeout=10.0)
                if msg[0] == "stopped":
                    print(f"Recording stopped: {msg[1]}, {msg[2]}", flush=True)
            except Exception:
                pass
            _cam_cmd_q.put(("shutdown",))
            _cam_proc.join(timeout=5.0)
            if _cam_proc.is_alive():
                _cam_proc.terminate()

        # --- Write metadata ---
        if metadata is not None:
            metadata.completed_utc = datetime.now(timezone.utc).isoformat()
            metadata.duration_s = time.perf_counter() - start_monotonic
            metadata.steps_completed = steps_completed
            metadata.completed = (steps_completed == metadata.total_steps)
            out_dir = record_dir if record_dir is not None else Path(".")
            save_run_metadata(metadata, out_dir)

        if not dry_run and dps is not None:
            dps.onoff("w", 0)
            print("Light turned off.", flush=True)
        if printer is not None:
            if return_to_origin:
                print(
                    "Moving to origin: X=0.000 Y=0.000 Z=0.000",
                    flush=True,
                )
                origin_feedrate = home_step.feedrate if home_step is not None else 1200.0
                printer.send_move(0.0, 0.0, 0.0, origin_feedrate)
            elif home_step is not None:
                print(
                    f"Moving to initial position: "
                    f"X={home_step.x:.3f} Y={home_step.y:.3f} Z={home_step.z:.3f}",
                    flush=True,
                )
                printer.send_move(home_step.x, home_step.y, home_step.z, home_step.feedrate)
            printer.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run synchronized printer + DPS control from a CSV schedule."
    )
    parser.add_argument("--csv", type=Path, required=True, help="Input schedule CSV path")
    parser.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Repeat the schedule this many times (helper loop generation)",
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

    parser.add_argument("--grbl-port", default="", help="GRBL controller serial port (e.g. COM3)")
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
    parser.add_argument("--cam0", type=int, default=None, help="USB camera 0 device ID for recording")
    parser.add_argument("--cam1", type=int, default=None, help="USB camera 1 device ID for recording")
    parser.add_argument("--cam-fps", type=float, default=15.0, help="Camera recording framerate")
    parser.add_argument(
        "--record-dir", type=Path, default=None,
        help="Directory to save camera recordings (required if --cam0/--cam1 is set)",
    )
    return parser.parse_args()


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

    steps = read_sequence_csv(args.csv, default_feedrate=args.default_feedrate)
    looped_steps = expand_loop_steps(steps, args.loops)

    if args.dry_run:
        run_sequence(looped_steps, dps=None, printer=None, time_mode=args.time_mode, dry_run=True)
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
    cameras: CameraController | None = None
    if args.cam0 is not None or args.cam1 is not None:
        cam0 = args.cam0 if args.cam0 is not None else 0
        cam1 = args.cam1 if args.cam1 is not None else 1
        cameras = CameraController(cam0_id=cam0, cam1_id=cam1, fps=args.cam_fps)

    # --- Metadata ---
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    metadata = RunMetadata(
        run_id=run_id,
        csv_file=str(args.csv),
        loops=args.loops,
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
        cam0_id=args.cam0,
        cam1_id=args.cam1,
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
    )


if __name__ == "__main__":
    main()