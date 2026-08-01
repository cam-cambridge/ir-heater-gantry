"""Graphical front-end for the IR-heater sequence runner.

Launch via::

    python main.py gui

Uses PySide6 (Qt) with:
- FigureCanvasQTAgg for matplotlib embedding
- QThread + Signal for background sequence execution
- QTimer for GUI progress polling
"""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import Qt, QThread, QTimer, Signal
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

from camera_controller import CameraController
from sequence_generator import (
    generate_sequence_rows,
    read_pair_specs_csv,
    write_sequence_csv,
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
_POLL_MS = 100


def _parse_optional_float(value: str) -> float | None:
    stripped = value.strip()
    if not stripped:
        return None
    try:
        return float(stripped)
    except ValueError:
        return None


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
        cam0_id: int,
        cam1_id: int,
        cam_fps: float,
        record_dir: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
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
        self._cam0_id = cam0_id
        self._cam1_id = cam1_id
        self._cam_fps = cam_fps
        self._record_dir = record_dir

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

            if self._record_cameras and self._record_dir:
                cameras = CameraController(
                    cam0_id=self._cam0_id, cam1_id=self._cam1_id, fps=self._cam_fps,
                )
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
                cam0_id=self._cam0_id,
                cam1_id=self._cam1_id,
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
                stop_event=None,
                on_step=_on_step,
                return_to_origin=self._return_to_origin,
                cameras=cameras,
                record_dir=record_dir,
                metadata=metadata,
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

        # -- Pairs CSV row --
        csv_row = QHBoxLayout()
        csv_row.addWidget(QLabel("Pairs CSV:"))
        self._pairs_csv_le = QLineEdit()
        self._pairs_csv_le.setMinimumWidth(300)
        csv_row.addWidget(self._pairs_csv_le, 1)
        browse_btn = QPushButton("Browse\u2026")
        browse_btn.clicked.connect(self._browse_pairs_csv)
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
        gen_layout = QHBoxLayout(gen_gb)
        gen_layout.addWidget(QLabel("Feedrate:"))
        self._feedrate_le = QLineEdit(str(_DEFAULT_FEEDRATE))
        self._feedrate_le.setMaximumWidth(70)
        gen_layout.addWidget(self._feedrate_le)
        gen_layout.addWidget(QLabel("Transition (s):"))
        self._transition_le = QLineEdit("5.0")
        self._transition_le.setMaximumWidth(60)
        gen_layout.addWidget(self._transition_le)
        gen_layout.addWidget(QLabel("Loops:"))
        self._loops_le = QLineEdit("1")
        self._loops_le.setMaximumWidth(50)
        gen_layout.addWidget(self._loops_le)
        gen_layout.addStretch()
        ctrl_layout.addWidget(gen_gb)

        # -- Camera recording --
        cam_gb = QGroupBox("Camera Recording")
        cam_layout = QHBoxLayout(cam_gb)
        self._record_cb = QCheckBox("Record")
        cam_layout.addWidget(self._record_cb)
        cam_layout.addWidget(QLabel("Cam 0:"))
        self._cam0_le = QLineEdit("0")
        self._cam0_le.setMaximumWidth(30)
        cam_layout.addWidget(self._cam0_le)
        cam_layout.addWidget(QLabel("Cam 1:"))
        self._cam1_le = QLineEdit("1")
        self._cam1_le.setMaximumWidth(30)
        cam_layout.addWidget(self._cam1_le)
        cam_layout.addWidget(QLabel("FPS:"))
        self._cam_fps_le = QLineEdit("15")
        self._cam_fps_le.setMaximumWidth(40)
        cam_layout.addWidget(self._cam_fps_le)
        cam_layout.addWidget(QLabel("Dir:"))
        self._record_dir_le = QLineEdit("recordings")
        self._record_dir_le.setMaximumWidth(100)
        cam_layout.addWidget(self._record_dir_le)
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

        # === Bottom: matplotlib plots ===
        fig = Figure(figsize=(11, 3.5), tight_layout=True)
        self._ax_pos = fig.add_subplot(1, 3, 1)
        self._ax_volt = fig.add_subplot(1, 3, 2)
        self._ax_curr = fig.add_subplot(1, 3, 3)

        for ax, title, ylabel in (
            (self._ax_pos, "Position (mm)", "mm"),
            (self._ax_volt, "Voltage", "V"),
            (self._ax_curr, "Current", "A"),
        ):
            ax.set_title(title, fontsize=9)
            ax.set_xlabel("Step", fontsize=8)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4)

        self._vline_pos = self._ax_pos.axvline(x=0, color="red", linewidth=1, visible=False)
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
    #  CSV loading
    # ------------------------------------------------------------------

    def _browse_pairs_csv(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select pairs CSV", "", "CSV files (*.csv);;All files (*.*)",
        )
        if not path:
            return
        self._pairs_csv_le.setText(path)
        self._load_pairs_csv(Path(path))

    def _load_pairs_csv(self, path: Path) -> None:
        try:
            feedrate = float(self._feedrate_le.text() or _DEFAULT_FEEDRATE)
            transition = float(self._transition_le.text() or 5.0)
            loops = max(1, int(self._loops_le.text() or 1))
        except ValueError as exc:
            QMessageBox.critical(self, "Input Error", f"Invalid parameter: {exc}")
            return

        try:
            specs = read_pair_specs_csv(
                pairs_csv=path,
                default_feedrate=feedrate,
                default_transition_s=transition,
            )
        except Exception as exc:
            QMessageBox.critical(self, "Pairs CSV Error", str(exc))
            return

        try:
            rows = generate_sequence_rows(specs)
        except Exception as exc:
            QMessageBox.critical(self, "Generation Error", str(exc))
            return

        looped_rows = rows * loops
        output_path = path.with_stem(f"{path.stem}_loops_{loops}")
        try:
            write_sequence_csv(looped_rows, output_path)
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
            f"Generated {output_path.name}: {len(self._looped_steps)} steps"
        )

    def _plot_planned(self, steps: list[SequenceStep]) -> None:
        xs = list(range(len(steps)))
        x_vals = [s.x for s in steps]
        y_vals = [s.y for s in steps]
        z_vals = [s.z for s in steps]
        volts = [s.voltage_v for s in steps]
        amps = [s.current_a for s in steps]

        for ax in (self._ax_pos, self._ax_volt, self._ax_curr):
            ax.cla()
            ax.set_xlabel("Step", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4)

        self._ax_pos.set_title("Position (mm)", fontsize=9)
        self._ax_pos.set_ylabel("mm", fontsize=8)
        self._ax_pos.plot(xs, x_vals, label="X", linewidth=1)
        self._ax_pos.plot(xs, y_vals, label="Y", linewidth=1)
        self._ax_pos.plot(xs, z_vals, label="Z", linewidth=1)
        self._ax_pos.legend(fontsize=7)

        self._ax_volt.set_title("Voltage", fontsize=9)
        self._ax_volt.set_ylabel("V", fontsize=8)
        self._ax_volt.plot(xs, volts, color="tab:orange", linewidth=1)

        self._ax_curr.set_title("Current", fontsize=9)
        self._ax_curr.set_ylabel("A", fontsize=8)
        self._ax_curr.plot(xs, amps, color="tab:green", linewidth=1)

        self._vline_pos = self._ax_pos.axvline(x=0, color="red", linewidth=1, visible=False)
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

        self._run_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._status_label.setText("Running\u2026")
        self._progress_bar.setValue(0)

        # Collect params
        record_dir = self._record_dir_le.text().strip() if self._record_cb.isChecked() else ""
        cam0 = int(self._cam0_le.text() or 0)
        cam1 = int(self._cam1_le.text() or 1)
        cam_fps = float(self._cam_fps_le.text() or 15)

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
            cam0_id=cam0,
            cam1_id=cam1,
            cam_fps=cam_fps,
            record_dir=record_dir,
            parent=self,
        )
        self._worker.progress.connect(self._on_progress)
        self._worker.done.connect(self._on_done)
        self._worker.error.connect(self._on_error)
        self._worker.start()

    def _on_stop(self) -> None:
        # Signal the worker to stop (we don't have a threaded stop_event anymore)
        # For now, stopping is a best-effort: the worker doesn't support it via QThread.
        # The sequence_runner's stop_event-based stop mechanism would need a thread-safe flag
        # accessible from here. We'll set the status anyway.
        self._status_label.setText("Stopping\u2026")
        self._stop_btn.setEnabled(False)
        # TODO: integrate a threading.Event accessible to the worker

    # ------------------------------------------------------------------
    #  Worker signal handlers  (called on main thread)
    # ------------------------------------------------------------------

    def _poll_worker(self) -> None:
        """Fallback poll - no-op since we use signals now."""
        pass

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

    def _on_error(self, err: str) -> None:
        self._status_label.setText(f"Error: {err}")
        QMessageBox.critical(self, "Sequence error", err)
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)

    def _set_vlines(self, step_index: int) -> None:
        for vline in (self._vline_pos, self._vline_volt, self._vline_curr):
            vline.set_xdata([step_index, step_index])
            vline.set_visible(True)

    # ------------------------------------------------------------------
    #  Close
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        if self._worker is not None and self._worker.isRunning():
            self._worker.quit()
            self._worker.wait(2000)
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
