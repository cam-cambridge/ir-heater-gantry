"""Graphical front-end for the IR-heater sequence runner.

Launch via::

    python main.py gui

Uses PySide6 (Qt) with:
- FigureCanvasQTAgg for matplotlib embedding
- QThread + Signal for background sequence execution
- QTimer for GUI progress polling
"""

from __future__ import annotations

import csv
import math
import multiprocessing
import queue
import sys
import threading
from pathlib import Path

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

# ---------------------------------------------------------------------------
#  Local imports
# ---------------------------------------------------------------------------
_SR_DIR = Path(__file__).parent
if str(_SR_DIR) not in sys.path:
    sys.path.insert(0, str(_SR_DIR))

from camera_controller import CameraController, CameraSpec
from camera_setup_dialog import CameraSetupDialog
from pattern_generator import (
    generate_heat_sequence,
    read_heat_locations_csv,
    write_sequence_csv as write_locations_sequence_csv,
)
from sequence_generator import (
    generate_sequence_rows,
    read_pair_specs_csv,
    write_sequence_csv as write_pairs_sequence_csv,
)
from sequence_runner import (
    GrblController,
    RunMetadata,
    SequenceStep,
    find_grbl_port,
    list_serial_ports,
    read_sequence_csv,
    run_sequence,
)

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
_DEFAULT_FEEDRATE = 1200.0
# Matches pattern_generator.py's own --travel-feedrate CLI default -- kept a
# separate constant/field from _DEFAULT_FEEDRATE since they control different
# things: this is the fast move *between* heat locations, not the (derived,
# not user-set) dwell speed *within* one, nor the pairs-CSV fallback feedrate.
_DEFAULT_TRAVEL_FEEDRATE = 2000.0
_POLL_MS = 100

# Two unrelated CSV schemas can be dropped on the same "Sequence CSV" field:
# position-pairs (sequence_generator.py: oscillate between two points) and
# heat-locations (pattern_generator.py: dwell at each point with a
# circle/rectangle/line pattern). Their required columns don't overlap, so
# the header alone is enough to tell them apart -- see _detect_csv_format.
_PAIRS_REQUIRED_COLUMNS = {
    "ax", "ay", "az", "bx", "by", "bz", "duration_s", "current_a", "voltage_v",
}
_LOCATIONS_REQUIRED_COLUMNS = {
    "x", "y", "z", "dwell_time_s", "radius_mm", "voltage_v", "current_a",
}


def _detect_csv_format(path: Path) -> str:
    """Sniff a sequence-source CSV's header row. Returns "pairs", "locations",
    or "unknown"."""
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        header = next(csv.reader(handle), [])
    fields = {c.strip().lower() for c in header if c}
    if _PAIRS_REQUIRED_COLUMNS <= fields:
        return "pairs"
    if _LOCATIONS_REQUIRED_COLUMNS <= fields:
        return "locations"
    return "unknown"


def _parse_optional_float(value: str) -> float | None:
    stripped = value.strip()
    if not stripped:
        return None
    try:
        return float(stripped)
    except ValueError:
        return None


_ARC_PLOT_POINTS_PER_SEGMENT = 24


def _arc_plot_points(
    x0: float, y0: float, x1: float, y1: float, i: float, j: float, cw: bool,
    n: int = _ARC_PLOT_POINTS_PER_SEGMENT,
) -> list[tuple[float, float]]:
    """Interpolate points along a G2/G3 arc from (x0,y0) to (x1,y1) for plotting.

    A straight line between just the two endpoints badly misrepresents a
    circle's dwell rings -- each ring is only 2-3 waypoints apart (e.g. the
    two ends of a semicircle), so connecting them with straight chords draws
    a flat zigzag instead of the actual curved path GRBL executes.
    """
    cx, cy = x0 + i, y0 + j
    radius = math.hypot(i, j)
    if radius <= 1e-9:
        return [(x1, y1)]
    start_angle = math.atan2(y0 - cy, x0 - cx)
    if abs(x1 - x0) < 1e-6 and abs(y1 - y0) < 1e-6:
        magnitude = 2 * math.pi  # start == end -> full circle
    else:
        end_angle = math.atan2(y1 - cy, x1 - cx)
        magnitude = (
            (start_angle - end_angle) % (2 * math.pi) if cw
            else (end_angle - start_angle) % (2 * math.pi)
        )
    signed_sweep = -magnitude if cw else magnitude
    return [
        (
            cx + radius * math.cos(start_angle + signed_sweep * k / n),
            cy + radius * math.sin(start_angle + signed_sweep * k / n),
        )
        for k in range(1, n + 1)
    ]


def _dense_path_xy(steps: list[SequenceStep]) -> tuple[list[float], list[float]]:
    """Build an (xs, ys) path for plotting, expanding arc moves (G2/G3) into
    interpolated points instead of a straight line between their endpoints."""
    if not steps:
        return [], []
    xs = [steps[0].x]
    ys = [steps[0].y]
    prev = steps[0]
    for step in steps[1:]:
        if step.arc_i is not None and step.arc_j is not None:
            for px, py in _arc_plot_points(
                prev.x, prev.y, step.x, step.y, step.arc_i, step.arc_j, step.arc_cw
            ):
                xs.append(px)
                ys.append(py)
        else:
            xs.append(step.x)
            ys.append(step.y)
        prev = step
    return xs, ys


# ======================================================================
#  Background worker thread
# ======================================================================

class _SequenceWorker(QThread):
    """Runs the sequence in a background thread, communicating via signals."""

    progress = Signal(int, int)   # (current_step, total_steps)
    done = Signal()
    error = Signal(str)

    def __init__(
        self,
        steps: list[SequenceStep],
        time_mode: str,
        dry_run: bool,
        return_to_origin: bool,
        modbus_port: str,
        modbus_addr: int,
        modbus_baud: int,
        grbl_port: str,
        grbl_baud: int,
        x_max: float | None,
        y_max: float | None,
        z_max: float | None,
        record_cameras: bool,
        camera_specs: list[CameraSpec],
        cam_fps: float,
        record_dir: str,
        live_preview: bool = False,
        preview_fps: float = 15.0,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._stop_event = threading.Event()
        self._steps = steps
        self._time_mode = time_mode
        self._dry_run = dry_run
        self._return_to_origin = return_to_origin
        self._modbus_port = modbus_port
        self._modbus_addr = modbus_addr
        self._modbus_baud = modbus_baud
        self._grbl_port = grbl_port
        self._grbl_baud = grbl_baud
        self._x_max = x_max
        self._y_max = y_max
        self._z_max = z_max
        self._record_cameras = record_cameras
        self._camera_specs = camera_specs
        self._cam_fps = cam_fps
        self._record_dir = record_dir
        self._live_preview = live_preview
        self._preview_fps = preview_fps
        # Created here (GUI thread) rather than in run() (worker thread) so
        # MainWindow can poll it immediately after constructing the worker,
        # without waiting for the thread to actually start.
        self.preview_queue: multiprocessing.Queue | None = (
            multiprocessing.Queue(maxsize=1) if live_preview else None
        )

    def request_stop(self) -> None:
        """Signal the sequence to abort early (thread-safe -- called from the GUI thread)."""
        self._stop_event.set()

    def run(self) -> None:
        dps = None
        printer = None
        cameras = None
        record_dir = None
        try:
            if not self._dry_run:
                ini_path = Path(__file__).with_name("dps5005_limits.ini")
                from sequence_runner import connect_dps as _cdps
                dps = _cdps(
                    modbus_port=self._modbus_port,
                    ini_path=ini_path,
                    address=self._modbus_addr,
                    baudrate=self._modbus_baud,
                )
                if self._grbl_port:
                    printer = GrblController(
                        self._grbl_port,
                        baudrate=self._grbl_baud,
                        x_max=self._x_max,
                        y_max=self._y_max,
                        z_max=self._z_max,
                    )

            recording = self._record_cameras and self._record_dir and self._camera_specs
            if self._camera_specs and (recording or self._live_preview):
                cameras = CameraController(cameras=self._camera_specs, fps=self._cam_fps)
                if recording:
                    record_dir = Path(self._record_dir)

            # --- Metadata ---
            from datetime import datetime, timezone
            run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            metadata = RunMetadata(
                run_id=run_id,
                time_mode=self._time_mode,
                dry_run=self._dry_run,
                return_to_origin=self._return_to_origin,
                grbl_port=self._grbl_port,
                grbl_baud=self._grbl_baud,
                x_max=self._x_max,
                y_max=self._y_max,
                z_max=self._z_max,
                dps_port=self._modbus_port,
                dps_address=self._modbus_addr,
                dps_baud=self._modbus_baud,
                cameras=dict(self._camera_specs),
                cam_fps=self._cam_fps,
                record_dir=self._record_dir,
            )

            def _on_step(index: int, total: int) -> None:
                self.progress.emit(index, total)

            run_sequence(
                self._steps,
                dps=dps,
                printer=printer,
                time_mode=self._time_mode,
                dry_run=self._dry_run,
                stop_event=self._stop_event,
                on_step=_on_step,
                return_to_origin=self._return_to_origin,
                cameras=cameras,
                record_dir=record_dir,
                metadata=metadata,
                live_preview=self._live_preview,
                preview_queue=self.preview_queue,
                preview_fps=self._preview_fps,
            )
        except Exception as exc:
            self.error.emit(str(exc))
        else:
            self.done.emit()


# ======================================================================
#  Main window
# ======================================================================

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IR Heater Sequence Runner")
        self.resize(1200, 750)

        self._looped_steps: list[SequenceStep] = []
        self._generated_csv_path: Path | None = None
        self._worker: _SequenceWorker | None = None
        self._total_steps = 1

        self._build_ui()
        self._refresh_ports()

        # Progress poll timer
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_worker)
        self._poll_timer.start(_POLL_MS)

        # Live camera preview poll timer -- runs continuously; _poll_preview
        # is a no-op whenever no worker/preview_queue is active.
        self._preview_timer = QTimer(self)
        self._preview_timer.timeout.connect(self._poll_preview)
        self._preview_timer.start(_POLL_MS)

    # ------------------------------------------------------------------
    #  UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        # === Top: controls ===
        ctrl = QWidget()
        ctrl_layout = QVBoxLayout(ctrl)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)

        # -- Sequence CSV row (accepts either a position-pairs CSV or a
        # heat-locations CSV -- the format is auto-detected from its header) --
        csv_row = QHBoxLayout()
        csv_row.addWidget(QLabel("Sequence CSV:"))
        self._pairs_csv_le = QLineEdit()
        self._pairs_csv_le.setMinimumWidth(300)
        self._pairs_csv_le.setToolTip(
            "Either a position-pairs CSV (ax,ay,az,bx,by,bz,duration_s,...) or a "
            "heat-locations CSV (x,y,z,dwell_time_s,radius_mm,...) -- the format "
            "is detected automatically from the header row."
        )
        csv_row.addWidget(self._pairs_csv_le, 1)
        browse_btn = QPushButton("Browse\u2026")
        browse_btn.clicked.connect(self._browse_csv)
        csv_row.addWidget(browse_btn)
        ctrl_layout.addLayout(csv_row)

        # -- Connection settings --
        conn_gb = QGroupBox("Hardware")
        conn_layout = QHBoxLayout(conn_gb)

        # Modbus
        conn_layout.addWidget(QLabel("Modbus:"))
        self._modbus_port_cb = QComboBox()
        self._modbus_port_cb.setEditable(True)
        self._modbus_port_cb.setMinimumWidth(100)
        conn_layout.addWidget(self._modbus_port_cb)
        conn_layout.addWidget(QLabel("Addr:"))
        self._modbus_addr_le = QLineEdit("1")
        self._modbus_addr_le.setMaximumWidth(40)
        conn_layout.addWidget(self._modbus_addr_le)
        conn_layout.addWidget(QLabel("Baud:"))
        self._modbus_baud_le = QLineEdit("9600")
        self._modbus_baud_le.setMaximumWidth(60)
        conn_layout.addWidget(self._modbus_baud_le)

        conn_layout.addSpacing(12)
        conn_layout.addWidget(QLabel("GRBL:"))
        self._grbl_port_cb = QComboBox()
        self._grbl_port_cb.setEditable(True)
        self._grbl_port_cb.setMinimumWidth(100)
        conn_layout.addWidget(self._grbl_port_cb)
        conn_layout.addWidget(QLabel("Baud:"))
        self._grbl_baud_le = QLineEdit("115200")
        self._grbl_baud_le.setMaximumWidth(60)
        conn_layout.addWidget(self._grbl_baud_le)

        refresh_ports_btn = QPushButton("Refresh Ports")
        refresh_ports_btn.setToolTip("Re-scan available serial ports")
        refresh_ports_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(refresh_ports_btn)

        auto_grbl_btn = QPushButton("Auto-detect GRBL")
        auto_grbl_btn.setToolTip("Probe every serial port for a GRBL welcome banner")
        auto_grbl_btn.clicked.connect(self._auto_detect_grbl)
        conn_layout.addWidget(auto_grbl_btn)

        conn_layout.addStretch()
        ctrl_layout.addWidget(conn_gb)

        # -- Work area --
        area_gb = QGroupBox("Work Area (mm, optional)")
        area_layout = QHBoxLayout(area_gb)
        area_layout.addWidget(QLabel("X max:"))
        self._x_max_le = QLineEdit()
        self._x_max_le.setMaximumWidth(50)
        area_layout.addWidget(self._x_max_le)
        area_layout.addWidget(QLabel("Y max:"))
        self._y_max_le = QLineEdit()
        self._y_max_le.setMaximumWidth(50)
        area_layout.addWidget(self._y_max_le)
        area_layout.addWidget(QLabel("Z max:"))
        self._z_max_le = QLineEdit()
        self._z_max_le.setMaximumWidth(50)
        area_layout.addWidget(self._z_max_le)
        area_layout.addStretch()
        ctrl_layout.addWidget(area_gb)

        # -- Generator options --
        gen_gb = QGroupBox("Generator")
        gen_vbox = QVBoxLayout(gen_gb)
        gen_layout = QHBoxLayout()
        gen_layout.addWidget(QLabel("Pairs feedrate:"))
        self._feedrate_le = QLineEdit(str(_DEFAULT_FEEDRATE))
        self._feedrate_le.setMaximumWidth(70)
        self._feedrate_le.setToolTip(
            "Position-pairs CSVs only: fallback feedrate for rows with none."
        )
        gen_layout.addWidget(self._feedrate_le)
        gen_layout.addWidget(QLabel("Travel feedrate:"))
        self._travel_feedrate_le = QLineEdit(str(_DEFAULT_TRAVEL_FEEDRATE))
        self._travel_feedrate_le.setMaximumWidth(70)
        self._travel_feedrate_le.setToolTip(
            "Heat-locations CSVs only: fast feedrate for moving BETWEEN heat "
            "locations. The dwell speed WITHIN a location is derived from its "
            "radius/dwell_time_s and is not set here (see pattern_generator.py)."
        )
        gen_layout.addWidget(self._travel_feedrate_le)
        gen_layout.addWidget(QLabel("Transition (s):"))
        self._transition_le = QLineEdit("5.0")
        self._transition_le.setMaximumWidth(60)
        self._transition_le.setToolTip("Position-pairs CSVs only: time to move between pair sections.")
        gen_layout.addWidget(self._transition_le)
        gen_layout.addWidget(QLabel("Loops:"))
        self._loops_le = QLineEdit("1")
        self._loops_le.setMaximumWidth(50)
        gen_layout.addWidget(self._loops_le)
        gen_layout.addStretch()
        gen_vbox.addLayout(gen_layout)
        gen_hint = QLabel(
            "Dwell feedrate (speed WITHIN a heat location) is always computed "
            "automatically from path length ÷ dwell_time_s -- neither field "
            "above sets it. Pairs feedrate only fills in missing values on a "
            "position-pairs CSV; Travel feedrate only applies to the fast move "
            "BETWEEN heat locations."
        )
        gen_hint.setStyleSheet("color: #888; font-style: italic;")
        gen_hint.setWordWrap(True)
        gen_vbox.addWidget(gen_hint)
        ctrl_layout.addWidget(gen_gb)

        # -- Cameras (recording and/or live preview -- independent rates) --
        self._camera_specs: list[CameraSpec] = []
        cam_gb = QGroupBox("Cameras")
        cam_layout = QHBoxLayout(cam_gb)
        self._record_cb = QCheckBox("Record")
        cam_layout.addWidget(self._record_cb)
        configure_cams_btn = QPushButton("Configure Cameras…")
        configure_cams_btn.setToolTip("Identify cameras by live preview and assign labels")
        configure_cams_btn.clicked.connect(self._configure_cameras)
        cam_layout.addWidget(configure_cams_btn)
        self._cameras_summary_label = QLabel("None configured")
        cam_layout.addWidget(self._cameras_summary_label)
        cam_layout.addWidget(QLabel("Record FPS:"))
        self._cam_fps_le = QLineEdit("15")
        self._cam_fps_le.setMaximumWidth(40)
        self._cam_fps_le.setToolTip(
            "Frame rate written to disk. Keep this low (e.g. 1) to save space -- "
            "it's independent of the live preview rate below."
        )
        cam_layout.addWidget(self._cam_fps_le)
        cam_layout.addWidget(QLabel("Dir:"))
        self._record_dir_le = QLineEdit("recordings")
        self._record_dir_le.setMaximumWidth(100)
        cam_layout.addWidget(self._record_dir_le)

        cam_layout.addSpacing(12)
        self._live_preview_cb = QCheckBox("Live Preview")
        self._live_preview_cb.setToolTip(
            "Show a responsive live feed while running, independent of the "
            "(possibly much lower) Record FPS above -- never slows down or "
            "blocks disk recording or GRBL command timing, which run in "
            "separate processes."
        )
        cam_layout.addWidget(self._live_preview_cb)
        cam_layout.addWidget(QLabel("Preview FPS:"))
        self._preview_fps_le = QLineEdit("15")
        self._preview_fps_le.setMaximumWidth(40)
        cam_layout.addWidget(self._preview_fps_le)

        cam_layout.addStretch()
        ctrl_layout.addWidget(cam_gb)

        # -- Sequence options --
        seq_gb = QGroupBox("Sequence")
        seq_layout = QHBoxLayout(seq_gb)
        seq_layout.addWidget(QLabel("Time mode:"))
        self._time_mode_cb = QComboBox()
        self._time_mode_cb.addItems(["step", "absolute"])
        seq_layout.addWidget(self._time_mode_cb)
        self._dry_run_cb = QCheckBox("Dry run")
        seq_layout.addWidget(self._dry_run_cb)
        self._return_cb = QCheckBox("Return to 0,0,0 after run")
        self._return_cb.setChecked(True)
        seq_layout.addWidget(self._return_cb)
        seq_layout.addStretch()

        # Run / Stop
        self._run_btn = QPushButton("Run")
        self._run_btn.clicked.connect(self._on_run)
        seq_layout.addWidget(self._run_btn)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        seq_layout.addWidget(self._stop_btn)

        self._status_label = QLabel("Ready")
        seq_layout.addWidget(self._status_label)
        ctrl_layout.addWidget(seq_gb)

        # -- Progress bar --
        self._progress_bar = QProgressBar()
        self._progress_bar.setMaximum(100)
        self._progress_bar.setValue(0)
        ctrl_layout.addWidget(self._progress_bar)

        root.addWidget(ctrl)

        # === Middle: live camera preview (populated by _configure_cameras) ===
        self._preview_panel = QWidget()
        self._preview_layout = QHBoxLayout(self._preview_panel)
        self._preview_layout.setContentsMargins(0, 0, 0, 0)
        self._preview_labels: dict[str, QLabel] = {}
        self._preview_panel.setVisible(False)
        root.addWidget(self._preview_panel)

        # === Bottom: matplotlib plots ===
        fig = Figure(figsize=(11, 3.5), tight_layout=True)
        self._ax_pos = fig.add_subplot(1, 3, 1)
        self._ax_volt = fig.add_subplot(1, 3, 2)
        self._ax_curr = fig.add_subplot(1, 3, 3)

        # Position is a top-down X-Y path plot (the gantry moves in a plane),
        # not a value-vs-step line chart like Voltage/Current.
        self._ax_pos.set_title("Path (X-Y, mm)", fontsize=9)
        self._ax_pos.set_xlabel("X (mm)", fontsize=8)
        self._ax_pos.set_ylabel("Y (mm)", fontsize=8)
        self._ax_pos.set_aspect("equal", adjustable="datalim")
        self._pos_marker, = self._ax_pos.plot([], [], "o", color="red", markersize=7, zorder=5)

        for ax, title, ylabel in (
            (self._ax_volt, "Voltage", "V"),
            (self._ax_curr, "Current", "A"),
        ):
            ax.set_title(title, fontsize=9)
            ax.set_xlabel("Step", fontsize=8)
            ax.set_ylabel(ylabel, fontsize=8)

        for ax in (self._ax_pos, self._ax_volt, self._ax_curr):
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4)

        self._vline_volt = self._ax_volt.axvline(x=0, color="red", linewidth=1, visible=False)
        self._vline_curr = self._ax_curr.axvline(x=0, color="red", linewidth=1, visible=False)

        self._fig = fig
        self._canvas = FigureCanvasQTAgg(fig)
        root.addWidget(self._canvas, 1)

    # ------------------------------------------------------------------
    #  Serial port selection
    # ------------------------------------------------------------------

    def _refresh_ports(self) -> None:
        """Re-scan available serial ports and repopulate both port combos.

        Item text is the bare device name (e.g. ``COM3``) so it can be used
        directly as a port argument; the human-readable description is
        attached as a tooltip instead of being folded into the text.
        """
        ports = list_serial_ports()
        for combo in (self._modbus_port_cb, self._grbl_port_cb):
            current = combo.currentText().strip()
            combo.clear()
            for device, desc in ports:
                combo.addItem(device)
                combo.setItemData(combo.count() - 1, desc, Qt.ToolTipRole)
            combo.setCurrentText(current)

    def _auto_detect_grbl(self) -> None:
        self._status_label.setText("Probing ports for GRBL…")
        QApplication.processEvents()
        modbus_port = self._modbus_port_cb.currentText().strip()
        exclude = {modbus_port} if modbus_port else None
        port = find_grbl_port(exclude=exclude)
        if port is None:
            self._status_label.setText("Ready")
            QMessageBox.warning(self, "Auto-detect", "No GRBL controller found on any serial port.")
            return
        self._grbl_port_cb.setCurrentText(port)
        self._status_label.setText(f"Found GRBL on {port}")

    # ------------------------------------------------------------------
    #  Camera identification
    # ------------------------------------------------------------------

    def _configure_cameras(self) -> None:
        dlg = CameraSetupDialog(current=self._camera_specs, parent=self)
        if dlg.exec() == CameraSetupDialog.Accepted:
            self._camera_specs = dlg.selected_cameras()
            if self._camera_specs:
                summary = ", ".join(f"{label}:{idx}" for label, idx in self._camera_specs)
            else:
                summary = "None configured"
            self._cameras_summary_label.setText(summary)
            self._rebuild_preview_panel()

    def _rebuild_preview_panel(self) -> None:
        """(Re)build one placeholder QLabel per configured camera. Called
        whenever camera specs change; frames get filled in by _poll_preview
        while a run with Live Preview enabled is active."""
        while self._preview_layout.count():
            item = self._preview_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self._preview_labels.clear()

        for label, index in self._camera_specs:
            cam_label = QLabel(f"{label} (index {index})\n(preview not running)")
            cam_label.setAlignment(Qt.AlignCenter)
            cam_label.setStyleSheet("background: #1a1a1a; color: #888;")
            cam_label.setMinimumSize(240, 180)
            self._preview_layout.addWidget(cam_label)
            self._preview_labels[label] = cam_label

        self._preview_panel.setVisible(bool(self._camera_specs))

    # ------------------------------------------------------------------
    #  CSV loading
    # ------------------------------------------------------------------

    def _browse_csv(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select sequence CSV (pairs or heat-locations)", "",
            "CSV files (*.csv);;All files (*.*)",
        )
        if not path:
            return
        self._pairs_csv_le.setText(path)
        self._load_csv(Path(path))

    def _load_csv(self, path: Path) -> None:
        try:
            feedrate = float(self._feedrate_le.text() or _DEFAULT_FEEDRATE)
            travel_feedrate = float(self._travel_feedrate_le.text() or _DEFAULT_TRAVEL_FEEDRATE)
            transition = float(self._transition_le.text() or 5.0)
            loops = max(1, int(self._loops_le.text() or 1))
        except ValueError as exc:
            QMessageBox.critical(self, "Input Error", f"Invalid parameter: {exc}")
            return

        csv_format = _detect_csv_format(path)
        if csv_format == "unknown":
            QMessageBox.critical(
                self, "Unrecognized CSV",
                "Could not tell whether this is a position-pairs CSV or a "
                "heat-locations CSV from its header row.\n\n"
                f"Pairs format needs: {', '.join(sorted(_PAIRS_REQUIRED_COLUMNS))}\n"
                f"Locations format needs: {', '.join(sorted(_LOCATIONS_REQUIRED_COLUMNS))}",
            )
            return

        try:
            if csv_format == "pairs":
                specs = read_pair_specs_csv(
                    pairs_csv=path,
                    default_feedrate=feedrate,
                    default_transition_s=transition,
                )
            else:
                locations = read_heat_locations_csv(path)
        except Exception as exc:
            QMessageBox.critical(self, "CSV Error", str(exc))
            return

        try:
            if csv_format == "pairs":
                rows = generate_sequence_rows(specs)
            else:
                rows = generate_heat_sequence(locations, travel_feedrate=travel_feedrate)
        except Exception as exc:
            QMessageBox.critical(self, "Generation Error", str(exc))
            return

        looped_rows = rows * loops
        output_path = path.with_stem(f"{path.stem}_loops_{loops}")
        try:
            if csv_format == "pairs":
                write_pairs_sequence_csv(looped_rows, output_path)
            else:
                write_locations_sequence_csv(looped_rows, output_path)
        except Exception as exc:
            QMessageBox.critical(self, "Write Error", str(exc))
            return

        try:
            self._looped_steps = read_sequence_csv(output_path, default_feedrate=feedrate)
        except Exception as exc:
            QMessageBox.critical(self, "Sequence Error", str(exc))
            return

        self._generated_csv_path = output_path
        self._total_steps = max(len(self._looped_steps), 1)
        self._progress_bar.setMaximum(self._total_steps)
        self._plot_planned(self._looped_steps)
        self._status_label.setText(
            f"Generated {output_path.name} ({csv_format}): {len(self._looped_steps)} steps"
        )

    def _plot_planned(self, steps: list[SequenceStep]) -> None:
        xs = list(range(len(steps)))
        x_vals = [s.x for s in steps]
        y_vals = [s.y for s in steps]
        volts = [s.voltage_v for s in steps]
        amps = [s.current_a for s in steps]

        for ax in (self._ax_pos, self._ax_volt, self._ax_curr):
            ax.cla()
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4)

        self._ax_pos.set_title("Path (X-Y, mm)", fontsize=9)
        self._ax_pos.set_xlabel("X (mm)", fontsize=8)
        self._ax_pos.set_ylabel("Y (mm)", fontsize=8)
        self._ax_pos.set_aspect("equal", adjustable="datalim")
        # Arc moves (G2/G3 -- e.g. circle dwell rings) are expanded into
        # interpolated points rather than straight lines between their two
        # endpoints, which would otherwise draw a flat chord/zigzag instead
        # of the actual curved path GRBL executes.
        path_xs, path_ys = _dense_path_xy(steps)
        self._ax_pos.plot(path_xs, path_ys, color="tab:blue", linewidth=1)
        if x_vals:
            self._ax_pos.plot(x_vals[0], y_vals[0], "^", color="tab:green", markersize=8, label="Start")
            self._ax_pos.plot(x_vals[-1], y_vals[-1], "s", color="black", markersize=6, label="End")
            self._ax_pos.legend(fontsize=7)
        self._pos_marker, = self._ax_pos.plot(
            [x_vals[0]] if x_vals else [], [y_vals[0]] if y_vals else [],
            "o", color="red", markersize=7, zorder=5,
        )

        self._ax_volt.set_title("Voltage", fontsize=9)
        self._ax_volt.set_xlabel("Step", fontsize=8)
        self._ax_volt.set_ylabel("V", fontsize=8)
        self._ax_volt.plot(xs, volts, color="tab:orange", linewidth=1)

        self._ax_curr.set_title("Current", fontsize=9)
        self._ax_curr.set_xlabel("Step", fontsize=8)
        self._ax_curr.set_ylabel("A", fontsize=8)
        self._ax_curr.plot(xs, amps, color="tab:green", linewidth=1)

        self._vline_volt = self._ax_volt.axvline(x=0, color="red", linewidth=1, visible=False)
        self._vline_curr = self._ax_curr.axvline(x=0, color="red", linewidth=1, visible=False)
        self._canvas.draw_idle()

    # ------------------------------------------------------------------
    #  Run / Stop
    # ------------------------------------------------------------------

    def _on_run(self) -> None:
        if not self._looped_steps:
            QMessageBox.warning(self, "No sequence",
                                "Please load a pairs CSV first to generate a sequence.")
            return

        dry_run = self._dry_run_cb.isChecked()
        if not dry_run and not self._modbus_port_cb.currentText().strip():
            QMessageBox.critical(self, "Missing port",
                                 "Modbus port is required when not using dry run.")
            return
        if self._record_cb.isChecked() and not self._camera_specs:
            QMessageBox.critical(self, "No cameras configured",
                                 "Click \"Configure Cameras\u2026\" first, or uncheck Record.")
            return
        if self._live_preview_cb.isChecked() and not self._camera_specs:
            QMessageBox.critical(self, "No cameras configured",
                                 "Click \"Configure Cameras\u2026\" first, or uncheck Live Preview.")
            return

        self._run_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._status_label.setText("Running\u2026")
        self._progress_bar.setValue(0)

        # Collect params
        record_dir = self._record_dir_le.text().strip() if self._record_cb.isChecked() else ""
        cam_fps = float(self._cam_fps_le.text() or 15)
        preview_fps = float(self._preview_fps_le.text() or 15)

        self._worker = _SequenceWorker(
            steps=self._looped_steps,
            time_mode=self._time_mode_cb.currentText(),
            dry_run=self._dry_run_cb.isChecked(),
            return_to_origin=self._return_cb.isChecked(),
            modbus_port=self._modbus_port_cb.currentText().strip(),
            modbus_addr=int(self._modbus_addr_le.text() or 1),
            modbus_baud=int(self._modbus_baud_le.text() or 9600),
            grbl_port=self._grbl_port_cb.currentText().strip(),
            grbl_baud=int(self._grbl_baud_le.text() or 115200),
            x_max=_parse_optional_float(self._x_max_le.text()),
            y_max=_parse_optional_float(self._y_max_le.text()),
            z_max=_parse_optional_float(self._z_max_le.text()),
            record_cameras=self._record_cb.isChecked(),
            camera_specs=self._camera_specs,
            cam_fps=cam_fps,
            record_dir=record_dir,
            live_preview=self._live_preview_cb.isChecked(),
            preview_fps=preview_fps,
            parent=self,
        )
        self._worker.progress.connect(self._on_progress)
        self._worker.done.connect(self._on_done)
        self._worker.error.connect(self._on_error)
        self._worker.start()

    def _on_stop(self) -> None:
        if self._worker is not None:
            self._worker.request_stop()
        self._status_label.setText("Stopping\u2026")
        self._stop_btn.setEnabled(False)

    # ------------------------------------------------------------------
    #  Worker signal handlers  (called on main thread)
    # ------------------------------------------------------------------

    def _poll_worker(self) -> None:
        """Fallback poll - no-op since we use signals now."""
        pass

    def _poll_preview(self) -> None:
        """Drain the latest live-preview frame (if any) and repaint its label.

        No-op whenever there's no active worker or Live Preview wasn't
        requested -- cheap to leave running continuously.
        """
        if self._worker is None or self._worker.preview_queue is None:
            return
        try:
            payload: dict[str, bytes] = self._worker.preview_queue.get_nowait()
        except queue.Empty:
            return
        for label, jpeg_bytes in payload.items():
            cam_label = self._preview_labels.get(label)
            if cam_label is None:
                continue
            pix = QPixmap()
            if pix.loadFromData(jpeg_bytes, "JPG"):
                cam_label.setPixmap(pix)

    def _on_progress(self, index: int, total: int) -> None:
        self._progress_bar.setValue(index)
        self._status_label.setText(f"Step {index} / {total}")
        self._set_vlines(index - 1)
        self._canvas.draw_idle()

    def _on_done(self) -> None:
        self._progress_bar.setValue(self._total_steps)
        self._status_label.setText("Done")
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._canvas.draw_idle()
        self._rebuild_preview_panel()

    def _on_error(self, err: str) -> None:
        self._status_label.setText(f"Error: {err}")
        QMessageBox.critical(self, "Sequence error", err)
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._rebuild_preview_panel()

    def _set_vlines(self, step_index: int) -> None:
        for vline in (self._vline_volt, self._vline_curr):
            vline.set_xdata([step_index, step_index])
            vline.set_visible(True)
        if 0 <= step_index < len(self._looped_steps):
            step = self._looped_steps[step_index]
            self._pos_marker.set_data([step.x], [step.y])

    # ------------------------------------------------------------------
    #  Close
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        if self._worker is not None and self._worker.isRunning():
            # _SequenceWorker overrides QThread.run() directly (no exec()), so
            # quit() -- which only asks an event loop to exit -- is a no-op
            # here. request_stop() is the actual cooperative-stop signal that
            # run_sequence's loop checks, so it's the only thing that gets the
            # heater/gantry through their real cleanup path instead of being
            # abandoned mid-run when the window closes.
            self._worker.request_stop()
            self._worker.wait(8000)
        event.accept()


# ======================================================================
#  Entry point
# ======================================================================

def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
