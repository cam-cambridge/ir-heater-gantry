"""Heat-location based toolpath generation.

Instead of the old back-and-forth between fixed position pairs, this module
works with *heat locations*:

- Each location has a **center** point, a **dwell time**, and a **radius**.
- Within the radius the gantry traces concentric rings (native GRBL G2/G3
  arcs) so the IR heater covers the entire circular area uniformly, with far
  fewer G-code lines and mechanically exact circles compared to approximating
  a spiral out of short straight segments. The dwell feedrate is *derived*
  from the path length and the dwell time, not set independently -- see
  ``generate_dwell_rows`` for why that matters.

Output is a ``sequence.csv`` compatible with ``sequence_runner.py``.

Usage (CLI)::

    python pattern_generator.py --locations heat_locations.csv --output sequence.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


# ---------------------------------------------------------------------------
#  Data types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class HeatLocation:
    """One region to heat on the gantry bed — circle, rectangle, or line."""

    x: float          # centre X (mm)
    y: float          # centre Y (mm)
    z: float          # Z height (mm)
    dwell_time_s: float      # how long to spend dwelling at this location
    radius_mm: float         # radius (circle), half-width (rect), or half-length (line)
    voltage_v: float
    current_a: float
    label: str = ""          # optional human-readable name
    shape: str = "circle"    # "circle" | "rectangle" | "line"
    width_mm: float = 0.0    # full width for rectangle; full length for line
    height_mm: float = 0.0   # full height for rectangle (ignored for line)


@dataclass(frozen=True)
class SequenceRow:
    """One row in the output sequence CSV (same shape as sequence_runner expects)."""

    time_s: float
    current_a: float
    voltage_v: float
    x: float
    y: float
    z: float
    feedrate: float
    # Arc move (G2/G3): center offset from the *previous* row's position.
    # None on both = ordinary linear move (G1).
    arc_i: float | None = None
    arc_j: float | None = None
    arc_cw: bool = True


@dataclass(frozen=True)
class _PathSegment:
    """One leg of a dwell path -- either linear or an arc, with its true length."""

    x: float
    y: float
    length_mm: float
    arc_i: float | None = None
    arc_j: float | None = None
    arc_cw: bool = True


# ---------------------------------------------------------------------------
#  Rectangle raster dwell-pattern generation
# ---------------------------------------------------------------------------

def _raster_points(
    cx: float,
    cy: float,
    width: float,
    height: float,
    passes: int = 7,
) -> list[tuple[float, float]]:
    """Generate (x, y) points tracing a zigzag raster over a rectangle.

    Starts at top-left, scans right, steps down, scans left, etc.
    Covers the full rectangle, then reverses direction back to start.

    Parameters
    ----------
    cx, cy:
        Rectangle centre.
    width, height:
        Full width and height (mm).
    passes:
        Number of horizontal scan lines (must be odd for full coverage).
    """
    half_w = width / 2.0
    half_h = height / 2.0
    left = cx - half_w
    right = cx + half_w

    points: list[tuple[float, float]] = []

    # Outward: top-to-bottom zigzag
    for p in range(passes):
        frac = p / max(passes - 1, 1)          # 0 -> 1
        y = cy + half_h - frac * height         # top -> bottom
        if p % 2 == 0:
            points.append((left, y))            # left edge
            points.append((right, y))           # right edge
        else:
            points.append((right, y))           # right edge
            points.append((left, y))            # left edge

    # Inward: bottom-to-top zigzag (reversed)
    inward = list(reversed(points))
    # Avoid duplicate at the seam
    if inward and points and inward[0] == points[-1]:
        inward = inward[1:]

    return points + inward


# ---------------------------------------------------------------------------
#  Line dwell-pattern generation
# ---------------------------------------------------------------------------

def _line_points(
    cx: float,
    cy: float,
    length: float,
    angle_deg: float = 0.0,
    passes: int = 5,
) -> list[tuple[float, float]]:
    """Generate (x, y) points along a straight line, out and back.

    Parameters
    ----------
    cx, cy:
        Line centre point.
    length:
        Total line length (mm).
    angle_deg:
        Angle of the line in degrees (0 = horizontal / X-axis).
    passes:
        Number of full traversals (odd = ends at far end).
    """
    angle = math.radians(angle_deg)
    half = length / 2.0
    dx = half * math.cos(angle)
    dy = half * math.sin(angle)

    start = (cx - dx, cy - dy)
    end = (cx + dx, cy + dy)

    points: list[tuple[float, float]] = [start]
    for p in range(1, passes + 1):
        if p % 2 == 1:
            points.append(end)
        else:
            points.append(start)

    return points


# ---------------------------------------------------------------------------
#  Ring (G2/G3 arc) dwell-pattern generation
# ---------------------------------------------------------------------------

def _ring_segments(cx: float, cy: float, radius: float, ring_spacing_mm: float) -> list[_PathSegment]:
    """Concentric-ring coverage path from centre out to *radius*.

    Steps out from the centre to the innermost ring, traces it as two G3
    (CCW) semicircle arcs -- GRBL's native circular interpolation, computed
    exactly in firmware -- steps out to the next ring, and so on. Compared to
    approximating a spiral with many short straight segments, this needs far
    fewer G-code lines (less serial overhead, less drift risk) and traces
    mechanically exact circles instead of a polygon approximation.

    Rings are spaced roughly *ring_spacing_mm* apart, evenly dividing
    [0, radius] so the outermost ring always lands exactly on *radius*.
    """
    if radius <= 0:
        return []
    num_rings = max(1, round(radius / ring_spacing_mm))
    radii = [radius * (k + 1) / num_rings for k in range(num_rings)]

    segments: list[_PathSegment] = []
    prev_x, prev_y = cx, cy  # start at centre
    for r in radii:
        start_x, start_y = cx + r, cy
        step_dist = math.hypot(start_x - prev_x, start_y - prev_y)
        if step_dist > 1e-9:
            segments.append(_PathSegment(x=start_x, y=start_y, length_mm=step_dist))

        mid_x, mid_y = cx - r, cy
        circumference_half = math.pi * r
        segments.append(_PathSegment(
            x=mid_x, y=mid_y, length_mm=circumference_half,
            arc_i=cx - start_x, arc_j=cy - start_y, arc_cw=False,
        ))
        segments.append(_PathSegment(
            x=start_x, y=start_y, length_mm=circumference_half,
            arc_i=cx - mid_x, arc_j=cy - mid_y, arc_cw=False,
        ))
        prev_x, prev_y = start_x, start_y
    return segments


def _linear_segments(points: list[tuple[float, float]]) -> list[_PathSegment]:
    """Convert a plain (x, y) waypoint path into straight-line ``_PathSegment``s."""
    segments: list[_PathSegment] = []
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        segments.append(_PathSegment(x=x1, y=y1, length_mm=math.hypot(x1 - x0, y1 - y0)))
    return segments


def generate_dwell_rows(loc: HeatLocation, ring_spacing_mm: float = 2.0) -> list[SequenceRow]:
    """Create the slow dwell pattern for a single ``HeatLocation``.

    For ``shape == "circle"``: concentric G2/G3 ring arcs (see ``_ring_segments``).
    For ``shape == "rectangle"``: a zigzag raster fill of the rectangle.
    For ``shape == "line"``: a straight line traversed back and forth.

    The feedrate for every segment is *derived* from the total dwell path
    length and ``dwell_time_s`` (``total_dist_mm * 60 / dwell_time_s``)
    rather than taken as a separate input. This keeps the commanded speed
    and the wall-clock schedule mathematically consistent by construction --
    an earlier version accepted an independent ``dwell_feedrate`` value that
    could silently disagree with what ``dwell_time_s`` implied, causing the
    gantry's actual position to drift away from what the timing loop (and
    therefore the DPS voltage/current schedule) assumed.
    """
    if loc.dwell_time_s <= 0:
        return []

    if loc.shape == "rectangle":
        w = loc.width_mm if loc.width_mm > 0 else loc.radius_mm * 2.0
        h = loc.height_mm if loc.height_mm > 0 else loc.radius_mm * 2.0
        if w <= 0 or h <= 0:
            return []
        passes = max(3, int(h / 2.0))  # ~2 mm line spacing
        segments = _linear_segments(_raster_points(loc.x, loc.y, w, h, passes=passes))
    elif loc.shape == "line":
        length = loc.width_mm if loc.width_mm > 0 else loc.radius_mm * 2.0
        if length <= 0:
            return []
        passes = max(2, int(loc.dwell_time_s / 2.0))
        segments = _linear_segments(_line_points(loc.x, loc.y, length, passes=passes))
    else:
        segments = _ring_segments(loc.x, loc.y, loc.radius_mm, ring_spacing_mm=ring_spacing_mm)

    total_dist = sum(seg.length_mm for seg in segments)
    if total_dist <= 0:
        return []

    # Derive the feedrate that covers the whole path in exactly
    # dwell_time_s, so every segment's time_s (proportional to its share of
    # total_dist) and its commanded speed agree by construction.
    dwell_feedrate = total_dist * 60.0 / loc.dwell_time_s

    rows: list[SequenceRow] = []
    for seg in segments:
        seg_time = (seg.length_mm / total_dist) * loc.dwell_time_s
        if seg_time < 0.001:
            continue
        rows.append(
            SequenceRow(
                time_s=seg_time,
                current_a=loc.current_a,
                voltage_v=loc.voltage_v,
                x=seg.x,
                y=seg.y,
                z=loc.z,
                feedrate=dwell_feedrate,
                arc_i=seg.arc_i,
                arc_j=seg.arc_j,
                arc_cw=seg.arc_cw,
            )
        )

    return rows


# ---------------------------------------------------------------------------
#  Full sequence generation from a list of HeatLocations
# ---------------------------------------------------------------------------

def generate_heat_sequence(
    locations: list[HeatLocation],
    travel_feedrate: float = 2000.0,
    ring_spacing_mm: float = 2.0,
    start_position: Position | None = None,
) -> list[SequenceRow]:
    """Build a full sequence: travel to each location, dwell, repeat.

    Parameters
    ----------
    locations:
        Ordered list of heat locations.
    travel_feedrate:
        Feedrate (mm/min) to use when moving *between* locations (fast).
    ring_spacing_mm:
        Approximate radial spacing between concentric dwell rings for
        circle-shape locations (smaller = more rings = denser coverage).
    start_position:
        Gantry position when the sequence starts, if known (e.g. read off
        the alignment GUI beforehand). Used to compute a real travel time
        for the very first move. If omitted, the first move falls back to a
        fixed 0.5s regardless of actual distance -- fine if the gantry is
        already near location 1, wrong (and silently so) otherwise, so a
        warning is printed when this fallback is used.
    """
    rows: list[SequenceRow] = []
    prev_position: Position | None = start_position

    for loc in locations:
        # --- Travel from the previous position to this one (fast) ---
        if prev_position is not None:
            dx = loc.x - prev_position.x
            dy = loc.y - prev_position.y
            dz = loc.z - prev_position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > 0.001:
                travel_time = dist * 60.0 / travel_feedrate
            else:
                # Zero-distance — still add a tiny step to set V/I
                travel_time = 0.1
            rows.append(
                SequenceRow(
                    time_s=travel_time,
                    current_a=loc.current_a,
                    voltage_v=loc.voltage_v,
                    x=loc.x,
                    y=loc.y,
                    z=loc.z,
                    feedrate=travel_feedrate,
                )
            )
        else:
            # First location and no start_position given: distance is
            # genuinely unknown, so we can't compute a real travel time.
            # This is a fixed guess, not a measurement -- if location 1 is
            # actually far from the gantry's current position, the DPS
            # voltage/current for it will be commanded well before the
            # gantry physically arrives. Pass start_position to fix this.
            print(
                "WARNING: no start_position given -- first move to "
                f"({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f}) uses a fixed 0.5s "
                "guess regardless of actual distance.",
                flush=True,
            )
            rows.append(
                SequenceRow(
                    time_s=0.5,
                    current_a=loc.current_a,
                    voltage_v=loc.voltage_v,
                    x=loc.x,
                    y=loc.y,
                    z=loc.z,
                    feedrate=travel_feedrate,
                )
            )

        # --- Dwell pattern at this location ---
        dwell_rows = generate_dwell_rows(loc, ring_spacing_mm=ring_spacing_mm)
        rows.extend(dwell_rows)

        # Dwell patterns don't necessarily end back at the location's centre
        # (raster ends at a corner, line depends on pass count, and ring arcs
        # end at the outer ring's edge) -- use the real last position so the
        # next location's travel distance/time is computed correctly.
        if dwell_rows:
            last = dwell_rows[-1]
            prev_position = Position(last.x, last.y, last.z)
        else:
            prev_position = Position(loc.x, loc.y, loc.z)

    return rows


# ---------------------------------------------------------------------------
#  CSV I/O
# ---------------------------------------------------------------------------

def read_heat_locations_csv(csv_path: Path) -> list[HeatLocation]:
    """Parse a CSV of heat locations.

    Required columns: ``x, y, z, dwell_time_s, radius_mm, voltage_v, current_a``
    Optional column: ``label``

    There is deliberately no ``dwell_feedrate`` column -- the dwell speed is
    derived from the path length and ``dwell_time_s`` (see
    ``generate_dwell_rows``) so the commanded speed and the schedule can
    never disagree.
    """
    if not csv_path.exists():
        raise ValueError(f"Heat locations CSV not found: {csv_path}")

    locations: list[HeatLocation] = []
    with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV is empty or missing header")

        fields = {n.strip().lower() for n in reader.fieldnames if n}
        required = {"x", "y", "z", "dwell_time_s", "radius_mm", "voltage_v", "current_a"}
        missing = required - fields
        if missing:
            raise ValueError(f"Missing columns: {', '.join(sorted(missing))}")

        for ln, row in enumerate(reader, start=2):
            try:
                raw_shape = (row.get("shape") or "").strip().lower()
                shape = raw_shape if raw_shape in ("circle", "rectangle", "line") else "circle"
                loc = HeatLocation(
                    x=float(row.get("x", 0) or 0),
                    y=float(row.get("y", 0) or 0),
                    z=float(row.get("z", 0) or 0),
                    dwell_time_s=float(row.get("dwell_time_s", 0) or 0),
                    radius_mm=float(row.get("radius_mm", 0) or 0),
                    voltage_v=float(row.get("voltage_v", 0) or 0),
                    current_a=float(row.get("current_a", 0) or 0),
                    label=(row.get("label") or "").strip(),
                    shape=shape,
                    width_mm=float(row.get("width_mm", 0) or 0),
                    height_mm=float(row.get("height_mm", 0) or 0),
                )
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Invalid value at line {ln}: {exc}") from exc
            locations.append(loc)

    if not locations:
        raise ValueError("CSV must contain at least one heat location")
    return locations


def write_sequence_csv(rows: list[SequenceRow], output_csv: Path) -> None:
    """Write a sequence CSV readable by ``sequence_runner.read_sequence_csv``.

    ``arc_i``/``arc_j``/``arc_dir`` are left blank for ordinary linear moves
    and only populated for G2/G3 arc rows (see ``_ring_segments``).
    """
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["time", "current", "voltage", "x", "y", "z", "feedrate", "arc_i", "arc_j", "arc_dir"]
        )
        for row in rows:
            is_arc = row.arc_i is not None and row.arc_j is not None
            writer.writerow([
                f"{row.time_s:.6f}",
                f"{row.current_a:.6f}",
                f"{row.voltage_v:.6f}",
                f"{row.x:.6f}",
                f"{row.y:.6f}",
                f"{row.z:.6f}",
                f"{row.feedrate:.6f}",
                f"{row.arc_i:.6f}" if is_arc else "",
                f"{row.arc_j:.6f}" if is_arc else "",
                ("cw" if row.arc_cw else "ccw") if is_arc else "",
            ])


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a dwell-pattern heat sequence from a list of locations (circle/rectangle/line)."
    )
    parser.add_argument(
        "--locations", type=Path, required=True,
        help="CSV of heat locations (x,y,z,dwell_time_s,radius_mm,voltage_v,current_a)",
    )
    parser.add_argument("--output", type=Path, default=Path("sequence.csv"))
    parser.add_argument("--travel-feedrate", type=float, default=2000.0,
                        help="Feedrate between locations (mm/min)")
    parser.add_argument("--ring-spacing-mm", type=float, default=2.0,
                        help="Radial spacing between concentric dwell rings (circle shape only)")
    parser.add_argument(
        "--start-x", type=float, default=None,
        help="Gantry's known X position when the sequence starts (e.g. from the "
             "alignment GUI) -- lets the first move's timing be computed for real "
             "instead of a fixed 0.5s guess. Must be given with --start-y/--start-z.",
    )
    parser.add_argument("--start-y", type=float, default=None, help="See --start-x")
    parser.add_argument("--start-z", type=float, default=None, help="See --start-x")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    locations = read_heat_locations_csv(args.locations)

    start_coords = (args.start_x, args.start_y, args.start_z)
    if any(c is not None for c in start_coords) and not all(c is not None for c in start_coords):
        raise SystemExit("error: --start-x/--start-y/--start-z must be given together")
    start_position = Position(*start_coords) if all(c is not None for c in start_coords) else None

    rows = generate_heat_sequence(
        locations,
        travel_feedrate=args.travel_feedrate,
        ring_spacing_mm=args.ring_spacing_mm,
        start_position=start_position,
    )
    write_sequence_csv(rows, args.output)
    print(f"Wrote {len(rows)} sequence rows to {args.output}")


if __name__ == "__main__":
    main()
