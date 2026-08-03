"""Query a GRBL controller's ``$$`` settings, and optionally time a straight
move against a comparable arc move to separate two different explanations
for a "commanded feedrate isn't actually achieved" symptom (see
``sequence_runner.RunMetadata.drift_warnings`` / the motion-drift check in
``run_sequence``):

1. GRBL's own configured max-rate (``$110``/``$111``, mm/min) is silently
   clamping every move below what's being commanded. GRBL never errors on
   this -- it just executes slower than asked, with no feedback.
2. Per-line computation overhead for arc (G2/G3) moves specifically (GRBL
   segments an arc into small chords based on ``$12``, the arc tolerance,
   before it can reply "ok") is eating meaningful time *before* motion even
   starts, as opposed to during it.

These have different symptoms: (1) shows up as the *physical position*
(``?`` status) lagging behind the commanded feedrate even though GRBL
acknowledged ("ok") each line promptly. (2) shows up as a delay *before*
"ok" comes back for arc lines specifically, with physical motion then
proceeding at the expected rate once it starts. The ``--motion-test`` option
measures both separately so you don't have to guess.

Usage::

    python query_grbl_settings.py --port /dev/ttyUSB1
    python query_grbl_settings.py --port auto
    python query_grbl_settings.py --port /dev/ttyUSB1 --motion-test
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_SR_DIR = Path(__file__).parent
if str(_SR_DIR) not in sys.path:
    sys.path.insert(0, str(_SR_DIR))

from sequence_runner import GrblController, _parse_mpos  # noqa: E402

# Settings most relevant to a "commanded feedrate isn't actually achieved"
# symptom -- see module docstring. Full $$ dump is also printed regardless.
_HIGHLIGHT = {
    "$11": "Junction deviation (mm) -- lower = more cornering slowdown at direction changes",
    "$12": "Arc tolerance (mm) -- smaller = more chord segments per arc, more compute per line",
    "$110": "X max rate (mm/min) -- GRBL silently clamps to this, no error, regardless of commanded F",
    "$111": "Y max rate (mm/min) -- same, Y axis",
    "$112": "Z max rate (mm/min) -- same, Z axis",
    "$120": "X acceleration (mm/sec^2) -- low values mean slow ramp-up/down at every corner",
    "$121": "Y acceleration (mm/sec^2)",
    "$122": "Z acceleration (mm/sec^2)",
}


def _parse_dollar_settings(raw: str) -> dict[str, str]:
    settings: dict[str, str] = {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith("$") and "=" in line:
            key, _, value = line.partition("=")
            settings[key.strip()] = value.strip()
    return settings


def print_settings(grbl: GrblController) -> dict[str, str]:
    raw = grbl._send_line("$$", timeout_s=5.0)
    settings = _parse_dollar_settings(raw)

    print("\nAll reported $$ settings:")
    for key in sorted(settings, key=lambda k: (len(k), k)):
        print(f"  {key:6s} = {settings[key]}")

    print("\nMost relevant to a slower-than-commanded motion symptom:")
    for key, note in _HIGHLIGHT.items():
        value = settings.get(key, "(not reported)")
        print(f"  {key:6s} = {value:<10s}  {note}")

    return settings


def _current_xy(grbl: GrblController) -> tuple[float, float]:
    status = grbl.get_status()
    pos = _parse_mpos(status)
    if pos is None:
        raise RuntimeError(f"Could not parse a position out of status report: {status!r}")
    return pos


def _wait_until_settled(
    grbl: GrblController, target_x: float, target_y: float, tolerance_mm: float = 0.1,
    timeout_s: float = 20.0,
) -> tuple[float, bool]:
    """Poll ``?`` status until MPos is within *tolerance_mm* of the target.

    Returns (elapsed_seconds, settled). ``settled=False`` means *timeout_s*
    elapsed without the position ever landing on target -- report this
    plainly rather than guessing at a number.
    """
    start = time.perf_counter()
    deadline = start + timeout_s
    while time.perf_counter() < deadline:
        try:
            x, y = _current_xy(grbl)
        except RuntimeError:
            time.sleep(0.05)
            continue
        if abs(x - target_x) <= tolerance_mm and abs(y - target_y) <= tolerance_mm:
            return time.perf_counter() - start, True
        time.sleep(0.05)
    return time.perf_counter() - start, False


def run_motion_test(grbl: GrblController, distance_mm: float, feedrate: float) -> None:
    print(
        f"\nMotion test: {distance_mm:.1f}mm straight line vs. a comparable-length "
        f"semicircle arc, both at F{feedrate:.1f}."
    )
    print("This WILL move the gantry. Make sure it's clear to move, then confirm.")
    reply = input("Type 'yes' to proceed: ").strip().lower()
    if reply != "yes":
        print("Aborted -- no motion sent.")
        return

    start_x, start_y = _current_xy(grbl)
    print(f"Starting position: X{start_x:.3f} Y{start_y:.3f}")

    # --- Straight line out and back ---
    target_x = start_x + distance_mm
    t0 = time.perf_counter()
    grbl.send_move(target_x, start_y, 0.0, feedrate)
    ack_s = time.perf_counter() - t0
    settle_s, settled = _wait_until_settled(grbl, target_x, start_y)
    print(
        f"Straight line: ok in {ack_s * 1000:.0f}ms, physically settled in "
        f"{settle_s:.2f}s ({'reached target' if settled else 'TIMED OUT, did not reach target'})"
    )
    grbl.send_move(start_x, start_y, 0.0, feedrate)
    _wait_until_settled(grbl, start_x, start_y)

    # --- Semicircle arc of comparable path length (arc length = distance_mm) ---
    radius = distance_mm / 3.14159265
    arc_end_x = start_x  # semicircle returns to the same X, offset in Y
    arc_end_y = start_y + 2 * radius
    t0 = time.perf_counter()
    grbl.send_arc(
        start_x, start_y, arc_end_x, arc_end_y, 0.0,
        i=0.0, j=radius, feedrate=feedrate, cw=False,
    )
    ack_s = time.perf_counter() - t0
    settle_s, settled = _wait_until_settled(grbl, arc_end_x, arc_end_y)
    print(
        f"Semicircle arc (same path length): ok in {ack_s * 1000:.0f}ms, physically "
        f"settled in {settle_s:.2f}s ({'reached target' if settled else 'TIMED OUT, did not reach target'})"
    )
    grbl.send_move(start_x, start_y, 0.0, feedrate)
    _wait_until_settled(grbl, start_x, start_y)
    print(f"Returned to start: X{start_x:.3f} Y{start_y:.3f}")

    print(
        "\nInterpretation: if 'ok' latency is similar for both but the arc's physical "
        "settle time is much longer relative to its commanded duration "
        f"({distance_mm / feedrate * 60:.2f}s expected at F{feedrate:.1f} for "
        f"{distance_mm:.1f}mm), that points at rate-capping/cornering during motion, "
        "not arc computation. If 'ok' itself is slow to come back for the arc line, "
        "that points at GRBL's per-line arc segmentation being the bottleneck instead."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Query GRBL $$ settings and optionally time straight vs. arc motion."
    )
    parser.add_argument(
        "--port", required=True,
        help="Serial port (e.g. /dev/ttyUSB1, COM3), or 'auto' to probe all ports",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--motion-test", action="store_true",
        help="After printing settings, also move the gantry a short distance "
             "(straight line and a comparable arc) to time each separately. "
             "Prompts for confirmation before moving anything.",
    )
    parser.add_argument(
        "--test-distance-mm", type=float, default=10.0,
        help="Path length for --motion-test's straight line and arc (default 10mm)",
    )
    parser.add_argument(
        "--test-feedrate", type=float, default=170.0,
        help="Feedrate (mm/min) for --motion-test -- default matches a typical "
             "slow circle-dwell feedrate, not a fast travel move",
    )
    parser.add_argument(
        "--set-arc-tolerance", type=float, default=None,
        help="Write a new $12 (arc tolerance, mm) before printing settings/running "
             "the motion test. GRBL settings are stored in EEPROM/flash and persist "
             "across resets and power cycles (unlike the real-time feed override) "
             "-- pass --verify-persistence to prove that on this specific board "
             "rather than just taking that on faith.",
    )
    parser.add_argument(
        "--verify-persistence", action="store_true",
        help="After --set-arc-tolerance, disconnect and reconnect fresh (exercising "
             "the same reset path a reboot would) and re-read $12 to confirm the "
             "new value actually survived, instead of just assuming it did.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    grbl = GrblController(args.port, baudrate=args.baud)
    try:
        if args.set_arc_tolerance is not None:
            resp = grbl._send_line(f"$12={args.set_arc_tolerance}")
            if "error" in resp:
                raise SystemExit(f"GRBL rejected $12={args.set_arc_tolerance}: {resp}")
            print(f"Set $12={args.set_arc_tolerance} -- response: {resp}")

        print_settings(grbl)
        if args.motion_test:
            run_motion_test(grbl, args.test_distance_mm, args.test_feedrate)
    finally:
        grbl.disconnect()

    if args.set_arc_tolerance is not None and args.verify_persistence:
        print("\nReconnecting fresh to verify the setting survived a reset...")
        grbl2 = GrblController(args.port, baudrate=args.baud)
        try:
            settings = _parse_dollar_settings(grbl2._send_line("$$", timeout_s=5.0))
            actual = settings.get("$12", "(not reported)")
            print(f"$12 after reconnect: {actual}")
            if actual == str(args.set_arc_tolerance) or abs(
                float(actual) - args.set_arc_tolerance
            ) < 1e-6:
                print("CONFIRMED: setting persisted across a reset.")
            else:
                print(
                    f"WARNING: expected {args.set_arc_tolerance}, got {actual} -- "
                    "did not persist as expected."
                )
        finally:
            grbl2.disconnect()


if __name__ == "__main__":
    main()
