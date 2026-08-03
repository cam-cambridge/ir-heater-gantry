"""Manual alignment GUI with camera preview, jog controls, and heater toggling.

Launch via::

    python main.py align

Uses PySide6 (Qt) for:
- Direct OpenCV -> QImage -> QPixmap rendering (no PIL hop)
- QSplitter for resizable camera/control panels
- QStatusBar for connection status
- QDialog for connection settings
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

import cv2
import numpy as np

# ---------------------------------------------------------------------------
#  Local imports
# ---------------------------------------------------------------------------
_SR_DIR = Path(__file__).parent
if str(_SR_DIR) not in sys.path:
    sys.path.insert(0, str(_SR_DIR))

from camera_controller import CameraController, CameraSpec
from camera_setup_dialog import CameraSetupDialog
from dps_modbus import Dps5005, Import_limits, Serial_modbus
from sequence_runner import GrblController, find_grbl_port, list_serial_ports

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
_POLL_MS = 66          # ~15 FPS camera preview
_DEFAULT_BAUD = 115200
_PREVIEW_MAX_W = 480


# ======================================================================
#  Connection dialog
# ======================================================================

class ConnectDialog(QDialog):
    """Modal dialog to configure all hardware connections before launch."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Connect Hardware")
        self.setMinimumWidth(420)

        layout = QVBoxLayout(self)

        # --- GRBL ---
        grbl_gb = QGroupBox("GRBL Controller")
        grbl_form = QFormLayout(grbl_gb)
        self._grbl_port = QComboBox()
        self._grbl_port.setEditable(True)
        self._grbl_baud = QLineEdit(str(_DEFAULT_BAUD))
        self._grbl_baud.setMaximumWidth(80)
        grbl_form.addRow("Port:", self._grbl_port)
        grbl_form.addRow("Baud:", self._grbl_baud)

        grbl_btn_row = QHBoxLayout()
        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self._refresh_ports)
        grbl_btn_row.addWidget(refresh_btn)
        auto_btn = QPushButton("Auto-detect GRBL")
        auto_btn.setToolTip("Probe every serial port for a GRBL welcome banner")
        auto_btn.clicked.connect(self._auto_detect_grbl)
        grbl_btn_row.addWidget(auto_btn)
        grbl_form.addRow("", grbl_btn_row)

        layout.addWidget(grbl_gb)

        # --- Work area ---
        area_gb = QGroupBox("Work Area (mm, optional)")
        area_form = QFormLayout(area_gb)
        self._x_max = QLineEdit()
        self._x_max.setMaximumWidth(70)
        self._y_max = QLineEdit()
        self._y_max.setMaximumWidth(70)
        self._z_max = QLineEdit()
        self._z_max.setMaximumWidth(70)
        area_form.addRow("X max:", self._x_max)
        area_form.addRow("Y max:", self._y_max)
        area_form.addRow("Z max:", self._z_max)
        layout.addWidget(area_gb)

        # --- DPS ---
        dps_gb = QGroupBox("Heater (DPS5005)")
        dps_form = QFormLayout(dps_gb)
        self._dps_port = QComboBox()
        self._dps_port.setEditable(True)
        self._dps_addr = QLineEdit("1")
        self._dps_addr.setMaximumWidth(60)
        self._dps_baud = QLineEdit("9600")
        self._dps_baud.setMaximumWidth(80)
        dps_form.addRow("Port:", self._dps_port)
        dps_form.addRow("Address:", self._dps_addr)
        dps_form.addRow("Baud:", self._dps_baud)
        layout.addWidget(dps_gb)

        # --- Cameras ---
        self._camera_specs: list[CameraSpec] = []
        cam_gb = QGroupBox("Cameras")
        cam_form = QFormLayout(cam_gb)
        configure_cams_btn = QPushButton("Configure Cameras…")
        configure_cams_btn.setToolTip("Identify cameras by live preview and assign labels")
        configure_cams_btn.clicked.connect(self._configure_cameras)
        cam_form.addRow("", configure_cams_btn)
        self._cameras_summary_label = QLabel("None configured")
        cam_form.addRow("Selected:", self._cameras_summary_label)
        self._cam_fps = QLineEdit("15")
        self._cam_fps.setMaximumWidth(50)
        cam_form.addRow("FPS:", self._cam_fps)
        layout.addWidget(cam_gb)

        # --- Buttons ---
        btn_row = QHBoxLayout()
        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(self._on_connect)
        skip_btn = QPushButton("Skip (no hardware)")
        skip_btn.clicked.connect(self._on_skip)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(connect_btn)
        btn_row.addWidget(skip_btn)
        btn_row.addWidget(cancel_btn)
        layout.addLayout(btn_row)

        self._result: dict[str, object] = {}
        self._refresh_ports()

    def _configure_cameras(self) -> None:
        dlg = CameraSetupDialog(current=self._camera_specs, parent=self)
        if dlg.exec() == CameraSetupDialog.Accepted:
            self._camera_specs = dlg.selected_cameras()
            if self._camera_specs:
                summary = ", ".join(f"{label}:{idx}" for label, idx in self._camera_specs)
            else:
                summary = "None configured"
            self._cameras_summary_label.setText(summary)

    def _refresh_ports(self) -> None:
        """Re-scan available serial ports and repopulate both port combos.

        Item text is the bare device name (e.g. ``COM3``) so it can be used
        directly as a port argument; the human-readable description is
        attached as a tooltip instead of being folded into the text.
        """
        ports = list_serial_ports()
        for combo in (self._grbl_port, self._dps_port):
            current = combo.currentText().strip()
            combo.clear()
            for device, desc in ports:
                combo.addItem(device)
                combo.setItemData(combo.count() - 1, desc, Qt.ToolTipRole)
            combo.setCurrentText(current)

    def _auto_detect_grbl(self) -> None:
        dps_port = self._dps_port.currentText().strip()
        exclude = {dps_port} if dps_port else None
        port = find_grbl_port(exclude=exclude)
        if port is None:
            QMessageBox.warning(self, "Auto-detect", "No GRBL controller found on any serial port.")
            return
        self._grbl_port.setCurrentText(port)

    def _on_connect(self) -> None:
        self._result = {
            "grbl_port": self._grbl_port.currentText().strip(),
            "grbl_baud": self._grbl_baud.text().strip(),
            "x_max": self._x_max.text().strip(),
            "y_max": self._y_max.text().strip(),
            "z_max": self._z_max.text().strip(),
            "dps_port": self._dps_port.currentText().strip(),
            "dps_addr": self._dps_addr.text().strip(),
            "dps_baud": self._dps_baud.text().strip(),
            "cameras": self._camera_specs,
            "cam_fps": self._cam_fps.text().strip(),
        }
        self.accept()

    def _on_skip(self) -> None:
        self._result = {"skip": "1"}
        self.accept()

    def result(self) -> dict[str, object]:
        return self._result


# ======================================================================
#  Main alignment window
# ======================================================================

class AlignmentWindow(QMainWindow):
    def __init__(self, cfg: dict[str, object]) -> None:
        super().__init__()
        self.setWindowTitle("IR Heater \u2014 Alignment & Jog")
        self.resize(1280, 720)

        # --- Hardware handles ---
        self._grbl: GrblController | None = None
        self._dps: Dps5005 | None = None
        self._cameras: CameraController | None = None
        self._camera_specs: list[CameraSpec] = cfg.get("cameras") or []
        self._cam_labels: dict[str, QLabel] = {}

        # --- Saved positions ---
        self._saved_positions: list[dict[str, str]] = []

        # Guards re-entrancy into GRBL serial I/O.  A jog/go-to command blocks
        # this (single) GUI thread until GRBL replies, so normally a second
        # click can't be dispatched mid-command -- but QMessageBox.critical()
        # (raised from a timeout) pumps a *nested* Qt event loop, which will
        # happily deliver queued-up clicks from rapid button-mashing while
        # the first call is still unwinding.  That reentrant call reuses the
        # same serial handle concurrently with the outer one, interleaving
        # reads/writes and corrupting the ok/error handshake -- this is the
        # actual cause of the "many jog clicks -> crash" reports.  Buttons
        # are disabled below for the duration of every command so those
        # queued clicks are never delivered in the first place, and the
        # position-poll timer checks the same flag so it can't sneak a `?`
        # query in between either.
        self._hw_busy = False
        self._jog_buttons: list[QPushButton] = []
        self._go_btn: QPushButton | None = None
        self._zero_btn: QPushButton | None = None

        self._build_ui()

        # Connect hardware (deferred so the window paints first)
        if cfg.get("skip") != "1":
            QTimer.singleShot(100, lambda: self._connect_hardware(cfg))
        else:
            self.statusBar().showMessage("No hardware connected (demo mode)")

        # Camera poll timer
        self._cam_timer = QTimer(self)
        self._cam_timer.timeout.connect(self._poll_cameras)
        self._cam_timer.start(_POLL_MS)

    # ------------------------------------------------------------------
    #  UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        splitter = QSplitter(Qt.Horizontal, central)

        # --- Left: camera preview (one panel per configured camera) ---
        cam_panel = QWidget()
        cam_layout = QHBoxLayout(cam_panel)
        cam_layout.setContentsMargins(4, 4, 4, 4)

        if self._camera_specs:
            for label, index in self._camera_specs:
                cam_label = QLabel(f"{label} (index {index})\n(not connected)")
                cam_label.setAlignment(Qt.AlignCenter)
                cam_label.setStyleSheet("background: #1a1a1a; color: #888;")
                cam_label.setMinimumWidth(320)
                cam_layout.addWidget(cam_label)
                self._cam_labels[label] = cam_label
        else:
            placeholder = QLabel("No cameras configured")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("background: #1a1a1a; color: #888;")
            cam_layout.addWidget(placeholder)

        splitter.addWidget(cam_panel)

        # --- Right: controls ---
        ctrl_panel = QWidget()
        ctrl_panel.setMaximumWidth(340)
        ctrl_layout = QVBoxLayout(ctrl_panel)
        ctrl_layout.setContentsMargins(6, 6, 6, 6)

        # ---- Jog ----
        jog_gb = QGroupBox("Jog Control")
        jog_layout = QVBoxLayout(jog_gb)

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step (mm):"))
        self._step_cb = QComboBox()
        self._step_cb.setEditable(True)
        self._step_cb.addItems(["0.1", "0.5", "1.0", "5.0", "10.0", "50.0", "100.0"])
        self._step_cb.setCurrentText("1.0")
        self._step_cb.setMinimumWidth(80)
        step_row.addWidget(self._step_cb)
        step_row.addWidget(QLabel("Feedrate:"))
        self._jog_fr = QLineEdit("600")
        self._jog_fr.setMaximumWidth(70)
        step_row.addWidget(self._jog_fr)
        step_row.addStretch()
        jog_layout.addLayout(step_row)

        # D-pad
        dpad = QGridLayout()
        dpad.setSpacing(4)

        def _btn(text: str, dx: int, dy: int, dz: int) -> QPushButton:
            b = QPushButton(text)
            b.setFixedSize(64, 36)
            b.clicked.connect(lambda: self._jog(dx, dy, dz))
            self._jog_buttons.append(b)
            return b

        dpad.addWidget(_btn("Y+", 0, 1, 0), 0, 1)
        dpad.addWidget(_btn("X-", -1, 0, 0), 1, 0)
        dpad.addWidget(_btn("X+", 1, 0, 0), 1, 2)
        dpad.addWidget(_btn("Y-", 0, -1, 0), 2, 1)
        dpad.addWidget(_btn("Z+", 0, 0, 1), 3, 0)
        dpad.addWidget(_btn("Z-", 0, 0, -1), 3, 2)
        jog_layout.addLayout(dpad)
        ctrl_layout.addWidget(jog_gb)

        # ---- Position ----
        pos_gb = QGroupBox("Position")
        pos_layout = QVBoxLayout(pos_gb)
        self._pos_label = QLabel("X: ---  Y: ---  Z: ---")
        self._pos_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        pos_layout.addWidget(self._pos_label)

        zero_row = QHBoxLayout()
        zero_btn = QPushButton("Zero Here")
        zero_btn.setToolTip(
            "Redefine the current physical position as X0 Y0 Z0 (G92).\n"
            "There are no limit switches / homing on this machine, so GRBL\n"
            "otherwise just assumes wherever it is at connect/reset is zero.\n"
            "Jog to your physical reference mark first, then click this.\n"
            "Does not move the gantry, and does not survive a reconnect --\n"
            "re-zero here again after every reconnect before running a sequence."
        )
        zero_btn.clicked.connect(self._zero_here)
        zero_row.addWidget(zero_btn)
        zero_row.addStretch()
        pos_layout.addLayout(zero_row)
        self._zero_btn = zero_btn

        goto_row = QHBoxLayout()
        goto_row.addWidget(QLabel("Go to X:"))
        self._goto_x = QLineEdit()
        self._goto_x.setMaximumWidth(60)
        goto_row.addWidget(self._goto_x)
        goto_row.addWidget(QLabel("Y:"))
        self._goto_y = QLineEdit()
        self._goto_y.setMaximumWidth(60)
        goto_row.addWidget(self._goto_y)
        goto_row.addWidget(QLabel("Z:"))
        self._goto_z = QLineEdit()
        self._goto_z.setMaximumWidth(60)
        goto_row.addWidget(self._goto_z)
        go_btn = QPushButton("Go")
        go_btn.clicked.connect(self._goto)
        goto_row.addWidget(go_btn)
        self._go_btn = go_btn
        pos_layout.addLayout(goto_row)
        ctrl_layout.addWidget(pos_gb)

        # ---- Heater ----
        heat_gb = QGroupBox("Heater (DPS5005)")
        heat_layout = QVBoxLayout(heat_gb)

        self._heat_btn = QPushButton("Turn ON")
        self._heat_btn.setCheckable(True)
        self._heat_btn.toggled.connect(self._toggle_heater)
        heat_layout.addWidget(self._heat_btn)

        v_row = QHBoxLayout()
        v_row.addWidget(QLabel("V:"))
        self._voltage_sb = QDoubleSpinBox()
        self._voltage_sb.setRange(0, 50)
        self._voltage_sb.setValue(5.0)
        self._voltage_sb.setDecimals(2)
        self._voltage_sb.setSuffix(" V")
        v_row.addWidget(self._voltage_sb)
        apply_v = QPushButton("Set")
        apply_v.clicked.connect(self._apply_voltage)
        v_row.addWidget(apply_v)
        heat_layout.addLayout(v_row)

        c_row = QHBoxLayout()
        c_row.addWidget(QLabel("I:"))
        self._current_sb = QDoubleSpinBox()
        self._current_sb.setRange(0, 10)
        self._current_sb.setValue(1.0)
        self._current_sb.setDecimals(3)
        self._current_sb.setSuffix(" A")
        c_row.addWidget(self._current_sb)
        apply_c = QPushButton("Set")
        apply_c.clicked.connect(self._apply_current)
        c_row.addWidget(apply_c)
        heat_layout.addLayout(c_row)
        ctrl_layout.addWidget(heat_gb)

        # ---- Save Position ----
        save_gb = QGroupBox("Save Position")
        save_layout = QHBoxLayout(save_gb)
        save_layout.addWidget(QLabel("Label:"))
        self._save_label = QLineEdit()
        self._save_label.setMaximumWidth(100)
        save_layout.addWidget(self._save_label)
        save_btn = QPushButton("Save")
        save_btn.clicked.connect(self._save_position)
        save_layout.addWidget(save_btn)
        ctrl_layout.addWidget(save_gb)

        # ---- Saved list ----
        list_gb = QGroupBox("Saved Positions")
        list_layout = QVBoxLayout(list_gb)
        self._list_widget = QListWidget()
        self._list_widget.setMaximumHeight(120)
        list_layout.addWidget(self._list_widget)

        btn_row = QHBoxLayout()
        del_btn = QPushButton("Delete")
        del_btn.clicked.connect(self._delete_position)
        btn_row.addWidget(del_btn)
        export_btn = QPushButton("Export CSV\u2026")
        export_btn.clicked.connect(self._export_csv)
        btn_row.addWidget(export_btn)
        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self._clear_positions)
        btn_row.addWidget(clear_btn)
        list_layout.addLayout(btn_row)
        ctrl_layout.addWidget(list_gb)

        ctrl_layout.addStretch()
        splitter.addWidget(ctrl_panel)

        splitter.setSizes([900, 340])

        # Root layout
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.addWidget(splitter)

        # Status bar
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Connecting\u2026")

        # Position poll timer
        self._pos_timer = QTimer(self)
        self._pos_timer.timeout.connect(self._update_position)
        self._pos_timer.start(500)

    # ------------------------------------------------------------------
    #  Hardware connect / disconnect
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_opt_float(s: str) -> float | None:
        s = s.strip()
        return float(s) if s else None

    def _connect_hardware(self, cfg: dict[str, str]) -> None:
        status_parts: list[str] = []
        try:
            grbl_port = cfg.get("grbl_port", "").strip()
            if grbl_port:
                self._grbl = GrblController(
                    grbl_port,
                    baudrate=int(cfg.get("grbl_baud", _DEFAULT_BAUD)),
                    x_max=self._parse_opt_float(cfg.get("x_max", "")),
                    y_max=self._parse_opt_float(cfg.get("y_max", "")),
                    z_max=self._parse_opt_float(cfg.get("z_max", "")),
                )
                status_parts.append(f"GRBL: {grbl_port}")
            else:
                status_parts.append("No GRBL port")

            dps_port = cfg.get("dps_port", "").strip()
            if dps_port:
                ini_path = Path(__file__).with_name("dps5005_limits.ini")
                serial_modbus = Serial_modbus(
                    dps_port, int(cfg.get("dps_addr", 1)),
                    int(cfg.get("dps_baud", 9600)), 8,
                )
                limits = Import_limits(str(ini_path))
                self._dps = Dps5005(serial_modbus, limits)
                status_parts.append(f"DPS: {dps_port}")

            fps = float(cfg.get("cam_fps", 15))
            if self._camera_specs:
                self._cameras = CameraController(cameras=self._camera_specs, fps=fps)
                self._cameras.start_preview()
                cams_desc = ", ".join(f"{label}:{idx}" for label, idx in self._camera_specs)
                status_parts.append(f"Cams: {cams_desc} @ {fps} FPS")
            else:
                status_parts.append("No cameras configured")

            self.statusBar().showMessage(" | ".join(status_parts))
        except Exception as exc:
            QMessageBox.critical(self, "Connection Error", str(exc))
            self.statusBar().showMessage(f"Error: {exc}")

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._cameras is not None:
            self._cameras.stop_preview()
        if self._dps is not None:
            try:
                self._dps.onoff("w", 0)
            except Exception as exc:
                QMessageBox.warning(
                    self, "Heater Error",
                    f"Could not confirm the heater turned off: {exc}\n"
                    "Check the DPS5005 manually before leaving.",
                )
        if self._grbl is not None:
            try:
                self._grbl.disconnect()
            except Exception:
                pass
        event.accept()

    # ------------------------------------------------------------------
    #  Camera polling
    # ------------------------------------------------------------------

    def _poll_cameras(self) -> None:
        if self._cameras is None:
            return
        frames = self._cameras.get_frames()
        errors = self._cameras.errors()
        for label, cam_label in self._cam_labels.items():
            frame = frames.get(label)
            if frame is not None:
                cam_label.setPixmap(self._ndarray_to_pixmap(frame))
            elif errors.get(label):
                cam_label.setText(f"{label}\n(error: {errors[label]})")

    @staticmethod
    def _ndarray_to_pixmap(frame: np.ndarray) -> QPixmap:
        """Convert OpenCV BGR frame -> QPixmap (zero-copy via QImage)."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        if w > _PREVIEW_MAX_W:
            qimg = qimg.scaledToWidth(_PREVIEW_MAX_W, Qt.SmoothTransformation)
        return QPixmap.fromImage(qimg)

    # ------------------------------------------------------------------
    #  Jog / movement
    # ------------------------------------------------------------------

    def _set_hw_controls_enabled(self, enabled: bool) -> None:
        for btn in self._jog_buttons:
            btn.setEnabled(enabled)
        if self._go_btn is not None:
            self._go_btn.setEnabled(enabled)
        if self._zero_btn is not None:
            self._zero_btn.setEnabled(enabled)

    def _jog(self, dx_sign: int, dy_sign: int, dz_sign: int) -> None:
        grbl = self._grbl
        if grbl is None:
            QMessageBox.warning(self, "No GRBL", "GRBL is not connected.")
            return
        if self._hw_busy:
            # A previous jog/go-to is still in flight (or its error dialog is
            # still up) -- ignore this click rather than re-entering GRBL
            # serial I/O from a second call stacked on top of the first.
            return
        try:
            step = float(self._step_cb.currentText())
        except ValueError:
            step = 1.0
        try:
            fr = float(self._jog_fr.text() or 600)
        except ValueError:
            fr = 600.0

        self._hw_busy = True
        self._set_hw_controls_enabled(False)
        try:
            grbl._drain_buffer()
            grbl._send_line("G91")
            try:
                grbl._send_line(f"G1 F{fr:.1f}")
                grbl._send_line(
                    f"G1 X{dx_sign * step:.3f} Y{dy_sign * step:.3f} Z{dz_sign * step:.3f}"
                )
            finally:
                # Always try to leave GRBL back in absolute mode, even if the
                # move above timed out -- otherwise every subsequent "Go to"
                # or saved-position move would silently be interpreted as a
                # *relative* move instead, sending the gantry to the wrong
                # place.
                grbl._send_line("G90")
        except (TimeoutError, OSError) as exc:
            QMessageBox.critical(self, "Jog Error", str(exc))
        finally:
            self._hw_busy = False
            self._set_hw_controls_enabled(True)

    def _goto(self) -> None:
        if self._grbl is None:
            return
        if self._hw_busy:
            return
        try:
            x = float(self._goto_x.text())
            y = float(self._goto_y.text())
            z = float(self._goto_z.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Enter valid numbers for X, Y, Z")
            return
        try:
            fr = float(self._jog_fr.text() or 600)
        except ValueError:
            fr = 600.0

        self._hw_busy = True
        self._set_hw_controls_enabled(False)
        try:
            self._grbl.send_move(x, y, z, fr)
        except (TimeoutError, OSError) as exc:
            QMessageBox.critical(self, "Go-to Error", str(exc))
        finally:
            self._hw_busy = False
            self._set_hw_controls_enabled(True)

    def _zero_here(self) -> None:
        if self._grbl is None:
            QMessageBox.warning(self, "No GRBL", "GRBL is not connected.")
            return
        if self._hw_busy:
            return
        reply = QMessageBox.question(
            self, "Zero Here",
            "Set the current position as X0 Y0 Z0?\n\n"
            "This redefines coordinates for the rest of this connection -- "
            "make sure you're at the physical reference point you want "
            "sequences to run relative to.",
        )
        if reply != QMessageBox.Yes:
            return

        self._hw_busy = True
        self._set_hw_controls_enabled(False)
        try:
            self._grbl.set_zero()
        except (TimeoutError, OSError) as exc:
            QMessageBox.critical(self, "Zero Error", str(exc))
        finally:
            self._hw_busy = False
            self._set_hw_controls_enabled(True)

    def _update_position(self) -> None:
        if self._grbl is None:
            return
        if self._hw_busy:
            # Don't interleave a `?` status query with an in-flight jog/go-to
            # command's own read of the serial stream.
            return
        try:
            status = self._grbl.get_status()
            for part in status.split("|"):
                for prefix in ("MPos:", "WPos:"):
                    if part.startswith(prefix):
                        coords = part[len(prefix):].split(",")
                        if len(coords) >= 3:
                            self._pos_label.setText(
                                f"X: {coords[0]}  Y: {coords[1]}  Z: {coords[2]}"
                            )
                            return
        except Exception:
            pass

    # ------------------------------------------------------------------
    #  Heater controls
    # ------------------------------------------------------------------

    def _toggle_heater(self, checked: bool) -> None:
        if self._dps is None:
            QMessageBox.warning(self, "No DPS", "Heater (DPS5005) is not connected.")
            self._heat_btn.setChecked(False)
            return
        try:
            self._dps.onoff("w", 1 if checked else 0)
        except IOError as exc:
            QMessageBox.critical(self, "Heater Error", str(exc))
            # The write never reached the DPS -- revert the button so it
            # doesn't claim a state we couldn't actually confirm.
            self._heat_btn.blockSignals(True)
            self._heat_btn.setChecked(not checked)
            self._heat_btn.blockSignals(False)
            return
        self._heat_btn.setText("Turn OFF" if checked else "Turn ON")

    def _apply_voltage(self) -> None:
        if self._dps is None:
            return
        try:
            self._dps.voltage_set("w", self._voltage_sb.value())
        except IOError as exc:
            QMessageBox.critical(self, "Heater Error", str(exc))

    def _apply_current(self) -> None:
        if self._dps is None:
            return
        try:
            self._dps.current_set("w", self._current_sb.value())
        except IOError as exc:
            QMessageBox.critical(self, "Heater Error", str(exc))

    # ------------------------------------------------------------------
    #  Save / manage positions
    # ------------------------------------------------------------------

    def _save_position(self) -> None:
        label = self._save_label.text().strip()
        if not label:
            QMessageBox.warning(self, "No label", "Enter a label for this position.")
            return
        text = self._pos_label.text()
        try:
            parts = text.replace("X:", "").replace("Y:", "").replace("Z:", "").split()
            x, y, z = parts[0], parts[1], parts[2]
        except (IndexError, ValueError):
            x = y = z = "???"
        entry = {"label": label, "x": x, "y": y, "z": z}
        self._saved_positions.append(entry)
        self._list_widget.addItem(f"{label}  ({x}, {y}, {z})")
        self._save_label.clear()

    def _delete_position(self) -> None:
        row = self._list_widget.currentRow()
        if row < 0:
            return
        self._list_widget.takeItem(row)
        del self._saved_positions[row]

    def _clear_positions(self) -> None:
        self._list_widget.clear()
        self._saved_positions.clear()

    def _export_csv(self) -> None:
        if not self._saved_positions:
            QMessageBox.warning(self, "No positions", "Save some positions first.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export positions", "", "CSV files (*.csv)",
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["label", "x", "y", "z"])
            writer.writeheader()
            writer.writerows(self._saved_positions)
        QMessageBox.information(
            self, "Exported",
            f"Saved {len(self._saved_positions)} positions to {path}",
        )


# ======================================================================
#  Entry point
# ======================================================================

def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    dlg = ConnectDialog()
    if dlg.exec() != QDialog.Accepted:
        return
    cfg = dlg.result()
    if not cfg:
        return

    window = AlignmentWindow(cfg)
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
