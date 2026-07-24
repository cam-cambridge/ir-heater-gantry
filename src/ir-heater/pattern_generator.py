"""Heat-location based toolpath generation.

Instead of the old back-and-forth between fixed position pairs, this module
works with *heat locations*:

- Each location has a **center** point, a **dwell time**, and a **radius**.
- Within the radius the gantry traces a slow spiral pattern (at a low
  *dwell feedrate*) so the IR heater covers the entire circular area
  uniformly.

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
class HeatLocation:
    """One region to heat on the gantry bed — circle, rectangle, or line."""

    x: float          # centre X (mm)
    y: float          # centre Y (mm)
    z: float          # Z height (mm)
    dwell_time_s: float      # how long to spend dwelling at this location
    radius_mm: float         # radius (circle), half-width (rect), or half-length (line)
    dwell_feedrate: float    # feedrate during dwell (mm/min, slow)
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
#  Spiral dwell-pattern generation
# ---------------------------------------------------------------------------

def _spiral_points(
    cx: float,
    cy: float,
    radius: float,
    turns: int = 3,
    segments_per_turn: int = 24,
) -> list[tuple[float, float]]:
    """Generate (x, y) points tracing an Archimedean spiral from centre outward.

    Parameters
    ----------
    cx, cy:
        Spiral centre.
    radius:
        Maximum radius (mm).
    turns:
        Number of full revolutions.
    segments_per_turn:
        How many line-segments per revolution (higher = smoother circle).
    """
    total_segments = turns * segments_per_turn
    points: list[tuple[float, float]] = [(cx, cy)]  # start at centre
    for i in range(1, total_segments + 1):
        frac = i / total_segments                # 0 → 1
        theta = 2.0 * math.pi * turns * frac     # angle
        r = radius * frac                        # linearly growing radius
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append((x, y))
    return points


def _points_distance(points: list[tuple[float, float]]) -> float:
    """Total path length (mm) along a sequence of points."""
    total = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        total += math.sqrt(dx * dx + dy * dy)
    return total


def generate_dwell_rows(loc: HeatLocation, spiral_turns: int = 3) -> list[SequenceRow]:
    """Create the slow dwell pattern for a single ``HeatLocation``.

    For ``shape == "circle"``: an Archimedean spiral out-and-back.
    For ``shape == "rectangle"``: a zigzag raster fill of the rectangle.
    For ``shape == "line"``: a straight line traversed back and forth.

    In both cases the ``dwell_feedrate`` (slow) is used for every segment;
    travel between locations uses a separate, faster ``travel_feedrate``.
    """
    if loc.dwell_time_s <= 0:
        return []

    if loc.shape == "rectangle":
        w = loc.width_mm if loc.width_mm > 0 else loc.radius_mm * 2.0
        h = loc.height_mm if loc.height_mm > 0 else loc.radius_mm * 2.0
        if w <= 0 or h <= 0:
            return []
        passes = max(3, int(h / 2.0))  # ~2 mm line spacing
        path = _raster_points(loc.x, loc.y, w, h, passes=passes)
    elif loc.shape == "line":
        length = loc.width_mm if loc.width_mm > 0 else loc.radius_mm * 2.0
        if length <= 0:
            return []
        passes = max(2, int(loc.dwell_time_s / 2.0))
        path = _line_points(loc.x, loc.y, length, passes=passes)
    else:
        # Default: circle spiral
        outward = _spiral_points(loc.x, loc.y, loc.radius_mm, turns=spiral_turns)
        inward_points = _spiral_points(loc.x, loc.y, loc.radius_mm, turns=spiral_turns)
        inward = list(reversed(inward_points))
        path = outward[:-1] + inward

    # Total path length
    total_dist = _points_distance(path)
    if total_dist <= 0:
        return []

    rows: list[SequenceRow] = []
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        seg_dist = math.sqrt(dx * dx + dy * dy)
        # Time for this segment
        seg_time = (seg_dist / total_dist) * loc.dwell_time_s
        if seg_time < 0.001:
            continue
        rows.append(
            SequenceRow(
                time_s=seg_time,
                current_a=loc.current_a,
                voltage_v=loc.voltage_v,
                x=path[i][0],
                y=path[i][1],
                z=loc.z,
                feedrate=loc.dwell_feedrate,
            )
        )

    return rows


# ---------------------------------------------------------------------------
#  Full sequence generation from a list of HeatLocations
# ---------------------------------------------------------------------------

def generate_heat_sequence(
    locations: list[HeatLocation],
    travel_feedrate: float = 2000.0,
    spiral_turns: int = 3,
) -> list[SequenceRow]:
    """Build a full sequence: travel to each location, dwell, repeat.

    Parameters
    ----------
    locations:
        Ordered list of heat locations.
    travel_feedrate:
        Feedrate (mm/min) to use when moving *between* locations (fast).
    spiral_turns:
        Number of spiral revolutions within each dwell radius.
    """
    rows: list[SequenceRow] = []
    prev: HeatLocation | None = None

    for loc in locations:
        # --- Travel from previous location to this one (fast) ---
        if prev is not None:
            dx = loc.x - prev.x
            dy = loc.y - prev.y
            dz = loc.z - prev.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > 0.001:
                travel_time = dist * 60.0 / travel_feedrate
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
                # Zero-distance — still add a tiny step to set V/I
                rows.append(
                    SequenceRow(
                        time_s=0.1,
                        current_a=loc.current_a,
                        voltage_v=loc.voltage_v,
                        x=loc.x,
                        y=loc.y,
                        z=loc.z,
                        feedrate=travel_feedrate,
                    )
                )
        else:
            # First location: move to it
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

        # --- Dwell spiral at this location ---
        dwell_rows = generate_dwell_rows(loc, spiral_turns=spiral_turns)
        rows.extend(dwell_rows)

        prev = loc

    return rows


# ---------------------------------------------------------------------------
#  CSV I/O
# ---------------------------------------------------------------------------

def read_heat_locations_csv(csv_path: Path) -> list[HeatLocation]:
    """Parse a CSV of heat locations.

    Required columns: ``x, y, z, dwell_time_s, radius_mm, dwell_feedrate, voltage_v, current_a``
    Optional column: ``label``
    """
    if not csv_path.exists():
        raise ValueError(f"Heat locations CSV not found: {csv_path}")

    locations: list[HeatLocation] = []
    with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV is empty or missing header")

        fields = {n.strip().lower() for n in reader.fieldnames if n}
        required = {"x", "y", "z", "dwell_time_s", "radius_mm",
                     "dwell_feedrate", "voltage_v", "current_a"}
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
                    dwell_feedrate=float(row.get("dwell_feedrate", 0) or 0),
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
    """Write a sequence CSV readable by ``sequence_runner.read_sequence_csv``."""
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time", "current", "voltage", "x", "y", "z", "feedrate"])
        for row in rows:
            writer.writerow([
                f"{row.time_s:.6f}",
                f"{row.current_a:.6f}",
                f"{row.voltage_v:.6f}",
                f"{row.x:.6f}",
                f"{row.y:.6f}",
                f"{row.z:.6f}",
                f"{row.feedrate:.6f}",
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
        help="CSV of heat locations (x,y,z,dwell_time_s,radius_mm,dwell_feedrate,voltage_v,current_a)",
    )
    parser.add_argument("--output", type=Path, default=Path("sequence.csv"))
    parser.add_argument("--travel-feedrate", type=float, default=2000.0,
                        help="Feedrate between locations (mm/min)")
    parser.add_argument("--spiral-turns", type=int, default=3,
                        help="Spiral revolutions within each dwell radius")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    locations = read_heat_locations_csv(args.locations)
    rows = generate_heat_sequence(
        locations,
        travel_feedrate=args.travel_feedrate,
        spiral_turns=args.spiral_turns,
    )
    write_sequence_csv(rows, args.output)
    print(f"Wrote {len(rows)} sequence rows to {args.output}")


if __name__ == "__main__":
    main()
