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

from camera_controller import CameraController
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
        cam_gb = QGroupBox("Cameras")
        cam_form = QFormLayout(cam_gb)
        self._cam0_id = QLineEdit("0")
        self._cam0_id.setMaximumWidth(50)
        self._cam1_id = QLineEdit("1")
        self._cam1_id.setMaximumWidth(50)
        self._cam_fps = QLineEdit("15")
        self._cam_fps.setMaximumWidth(50)
        cam_form.addRow("Cam 0 ID:", self._cam0_id)
        cam_form.addRow("Cam 1 ID:", self._cam1_id)
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

        self._result: dict[str, str] = {}
        self._refresh_ports()

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
            "cam0_id": self._cam0_id.text().strip(),
            "cam1_id": self._cam1_id.text().strip(),
            "cam_fps": self._cam_fps.text().strip(),
        }
        self.accept()

    def _on_skip(self) -> None:
        self._result = {"skip": "1"}
        self.accept()

    def result(self) -> dict[str, str]:
        return self._result


# ======================================================================
#  Main alignment window
# ======================================================================

class AlignmentWindow(QMainWindow):
    def __init__(self, cfg: dict[str, str]) -> None:
        super().__init__()
        self.setWindowTitle("IR Heater \u2014 Alignment & Jog")
        self.resize(1280, 720)

        # --- Hardware handles ---
        self._grbl: GrblController | None = None
        self._dps: Dps5005 | None = None
        self._cameras: CameraController | None = None

        # --- Saved positions ---
        self._saved_positions: list[dict[str, str]] = []

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

        # --- Left: camera preview ---
        cam_panel = QWidget()
        cam_layout = QHBoxLayout(cam_panel)
        cam_layout.setContentsMargins(4, 4, 4, 4)

        self._cam_label0 = QLabel("Camera 0\n(not connected)")
        self._cam_label0.setAlignment(Qt.AlignCenter)
        self._cam_label0.setStyleSheet("background: #1a1a1a; color: #888;")
        self._cam_label0.setMinimumWidth(320)
        cam_layout.addWidget(self._cam_label0)

        self._cam_label1 = QLabel("Camera 1\n(not connected)")
        self._cam_label1.setAlignment(Qt.AlignCenter)
        self._cam_label1.setStyleSheet("background: #1a1a1a; color: #888;")
        self._cam_label1.setMinimumWidth(320)
        cam_layout.addWidget(self._cam_label1)

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

            cam0 = int(cfg.get("cam0_id", 0))
            cam1 = int(cfg.get("cam1_id", 1))
            fps = float(cfg.get("cam_fps", 15))
            self._cameras = CameraController(cam0_id=cam0, cam1_id=cam1, fps=fps)
            self._cameras.start_preview()
            status_parts.append(f"Cams {cam0}/{cam1} @ {fps} FPS")

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
            except Exception:
                pass
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
        f0, f1 = self._cameras.get_frames()
        if f0 is not None:
            self._cam_label0.setPixmap(self._ndarray_to_pixmap(f0))
        elif self._cameras.cam0_error:
            self._cam_label0.setText(f"Camera 0\n(error: {self._cameras.cam0_error})")
        if f1 is not None:
            self._cam_label1.setPixmap(self._ndarray_to_pixmap(f1))
        elif self._cameras.cam1_error:
            self._cam_label1.setText(f"Camera 1\n(error: {self._cameras.cam1_error})")

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

    def _jog(self, dx_sign: int, dy_sign: int, dz_sign: int) -> None:
        grbl = self._grbl
        if grbl is None:
            QMessageBox.warning(self, "No GRBL", "GRBL is not connected.")
            return
        try:
            step = float(self._step_cb.currentText())
        except ValueError:
            step = 1.0
        try:
            fr = float(self._jog_fr.text() or 600)
        except ValueError:
            fr = 600.0
        grbl._drain_buffer()
        grbl._send_line("G91")
        grbl._send_line(f"G1 F{fr:.1f}")
        grbl._send_line(
            f"G1 X{dx_sign * step:.3f} Y{dy_sign * step:.3f} Z{dz_sign * step:.3f}"
        )
        grbl._send_line("G90")

    def _goto(self) -> None:
        if self._grbl is None:
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
        self._grbl.send_move(x, y, z, fr)

    def _update_position(self) -> None:
        if self._grbl is None:
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
        self._dps.onoff("w", 1 if checked else 0)
        self._heat_btn.setText("Turn OFF" if checked else "Turn ON")

    def _apply_voltage(self) -> None:
        if self._dps is not None:
            self._dps.voltage_set("w", self._voltage_sb.value())

    def _apply_current(self) -> None:
        if self._dps is not None:
            self._dps.current_set("w", self._current_sb.value())

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
