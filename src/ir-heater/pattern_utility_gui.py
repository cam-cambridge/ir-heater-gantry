"""Pattern Utility GUI - create grids of heat locations visually.

Launch via::

    python main.py pattern-gui

Uses PySide6 (Qt) with QGraphicsView for zoomable/scrollable preview.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGraphicsScene,
    QGraphicsView,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

# ---------------------------------------------------------------------------
#  Local imports
# ---------------------------------------------------------------------------
_SR_DIR = Path(__file__).parent
if str(_SR_DIR) not in sys.path:
    sys.path.insert(0, str(_SR_DIR))

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
_POINT_RADIUS = 4.0
_CANVAS_SCALE = 2.5  # pixels per mm
_GRID_COLORS = [
    QColor("#2196F3"), QColor("#4CAF50"), QColor("#FF9800"),
    QColor("#9C27B0"), QColor("#00BCD4"), QColor("#E91E63"),
]


# ======================================================================
#  Data model
# ======================================================================

class _Grid:
    """One rectangular array of points (e.g. one well plate)."""

    def __init__(
        self, origin_x: float, origin_y: float,
        rows: int, cols: int,
        spacing_x: float, spacing_y: float,
        label_prefix: str = "",
    ) -> None:
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.rows = rows
        self.cols = cols
        self.spacing_x = spacing_x
        self.spacing_y = spacing_y
        self.label_prefix = label_prefix
        self.color = _GRID_COLORS[len(_grid_registry) % len(_GRID_COLORS)]

    def points(self) -> list[tuple[float, float, str]]:
        pts: list[tuple[float, float, str]] = []
        for r in range(self.rows):
            for c in range(self.cols):
                x = self.origin_x + c * self.spacing_x
                y = self.origin_y + r * self.spacing_y
                label = (
                    f"{self.label_prefix}R{r + 1}C{c + 1}"
                    if self.label_prefix else f"({r},{c})"
                )
                pts.append((x, y, label))
        return pts


class _CustomPoint:
    def __init__(self, x: float, y: float, label: str = "") -> None:
        self.x = x
        self.y = y
        self.label = label


_grid_registry: list[_Grid] = []


# ======================================================================
#  Main window
# ======================================================================

class PatternUtilityWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Pattern Utility \u2014 Heat Location Grids")
        self.resize(1100, 650)

        self._grids: list[_Grid] = []
        self._custom_points: list[_CustomPoint] = []

        self._build_ui()

    # ------------------------------------------------------------------
    #  UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        splitter = QSplitter(Qt.Horizontal, central)

        # --- Left: controls ---
        ctrl = QWidget()
        ctrl.setMaximumWidth(320)
        ctrl_layout = QVBoxLayout(ctrl)
        ctrl_layout.setContentsMargins(6, 6, 6, 6)

        # ---- Grid definition ----
        grid_gb = QGroupBox("Add Grid (Well Plate)")
        grid_form = QFormLayout(grid_gb)

        ox_row = QHBoxLayout()
        self._grid_ox = QLineEdit("0")
        self._grid_ox.setMaximumWidth(60)
        self._grid_oy = QLineEdit("0")
        self._grid_oy.setMaximumWidth(60)
        ox_row.addWidget(QLabel("X:"))
        ox_row.addWidget(self._grid_ox)
        ox_row.addWidget(QLabel("Y:"))
        ox_row.addWidget(self._grid_oy)
        ox_row.addStretch()
        grid_form.addRow("Origin:", ox_row)

        dim_row = QHBoxLayout()
        self._grid_rows = QLineEdit("4")
        self._grid_rows.setMaximumWidth(45)
        self._grid_cols = QLineEdit("6")
        self._grid_cols.setMaximumWidth(45)
        dim_row.addWidget(QLabel("Rows:"))
        dim_row.addWidget(self._grid_rows)
        dim_row.addWidget(QLabel("Cols:"))
        dim_row.addWidget(self._grid_cols)
        dim_row.addStretch()
        grid_form.addRow("Size:", dim_row)

        sp_row = QHBoxLayout()
        self._grid_sx = QLineEdit("9.0")
        self._grid_sx.setMaximumWidth(55)
        self._grid_sy = QLineEdit("9.0")
        self._grid_sy.setMaximumWidth(55)
        sp_row.addWidget(QLabel("Spacing X:"))
        sp_row.addWidget(self._grid_sx)
        sp_row.addWidget(QLabel("Y:"))
        sp_row.addWidget(self._grid_sy)
        sp_row.addStretch()
        grid_form.addRow("Spacing:", sp_row)

        lbl_row = QHBoxLayout()
        self._grid_prefix = QLineEdit("P1_")
        self._grid_prefix.setMaximumWidth(80)
        lbl_row.addWidget(QLabel("Prefix:"))
        lbl_row.addWidget(self._grid_prefix)
        add_grid_btn = QPushButton("Add Grid")
        add_grid_btn.clicked.connect(self._add_grid)
        lbl_row.addWidget(add_grid_btn)
        lbl_row.addStretch()
        grid_form.addRow("", lbl_row)

        ctrl_layout.addWidget(grid_gb)

        # ---- Custom point ----
        cp_gb = QGroupBox("Add Custom Point")
        cp_layout = QHBoxLayout(cp_gb)
        self._cp_x = QLineEdit()
        self._cp_x.setMaximumWidth(55)
        self._cp_x.setPlaceholderText("X")
        self._cp_y = QLineEdit()
        self._cp_y.setMaximumWidth(55)
        self._cp_y.setPlaceholderText("Y")
        self._cp_label = QLineEdit()
        self._cp_label.setMaximumWidth(80)
        self._cp_label.setPlaceholderText("Label")
        cp_layout.addWidget(QLabel("X:"))
        cp_layout.addWidget(self._cp_x)
        cp_layout.addWidget(QLabel("Y:"))
        cp_layout.addWidget(self._cp_y)
        cp_layout.addWidget(QLabel("Lbl:"))
        cp_layout.addWidget(self._cp_label)
        add_cp_btn = QPushButton("Add")
        add_cp_btn.clicked.connect(self._add_custom_point)
        cp_layout.addWidget(add_cp_btn)
        ctrl_layout.addWidget(cp_gb)

        # ---- Dwell params ----
        dwell_gb = QGroupBox("Default Dwell Parameters")
        dwell_form = QFormLayout(dwell_gb)

        t_row = QHBoxLayout()
        self._dwell_time = QLineEdit("30")
        self._dwell_time.setMaximumWidth(55)

        # Shape selector
        t_row.addWidget(QLabel("Shape:"))
        self._dwell_shape_cb = QComboBox()
        self._dwell_shape_cb.addItems(["circle", "rectangle", "line"])
        self._dwell_shape_cb.currentTextChanged.connect(self._on_shape_changed)
        t_row.addWidget(self._dwell_shape_cb)

        t_row.addWidget(QLabel("Time (s):"))
        t_row.addWidget(self._dwell_time)
        t_row.addStretch()
        dwell_form.addRow("", t_row)

        # Radius / Width+Height (stacked based on shape)
        size_stack = QStackedWidget()

        # Page 0: circle radius + ring count
        circle_page = QWidget()
        circle_row = QHBoxLayout(circle_page)
        circle_row.setContentsMargins(0, 0, 0, 0)
        self._dwell_radius = QLineEdit("4")
        self._dwell_radius.setMaximumWidth(55)
        circle_row.addWidget(QLabel("Radius (mm):"))
        circle_row.addWidget(self._dwell_radius)
        self._dwell_num_rings = QLineEdit("5")
        self._dwell_num_rings.setMaximumWidth(40)
        self._dwell_num_rings.setToolTip(
            "Exact number of concentric rings, evenly spaced from centre out "
            "to Radius (the outermost ring always lands exactly on Radius)."
        )
        circle_row.addWidget(QLabel("Rings:"))
        circle_row.addWidget(self._dwell_num_rings)
        circle_row.addStretch()
        size_stack.addWidget(circle_page)

        # Page 1: rectangle width+height
        rect_page = QWidget()
        rect_row = QHBoxLayout(rect_page)
        rect_row.setContentsMargins(0, 0, 0, 0)
        self._dwell_width = QLineEdit("8")
        self._dwell_width.setMaximumWidth(55)
        self._dwell_height = QLineEdit("6")
        self._dwell_height.setMaximumWidth(55)
        rect_row.addWidget(QLabel("W (mm):"))
        rect_row.addWidget(self._dwell_width)
        rect_row.addWidget(QLabel("H (mm):"))
        rect_row.addWidget(self._dwell_height)
        rect_row.addStretch()
        size_stack.addWidget(rect_page)

        # Page 2: line length
        line_page = QWidget()
        line_row = QHBoxLayout(line_page)
        line_row.setContentsMargins(0, 0, 0, 0)
        self._dwell_llen = QLineEdit("10")
        self._dwell_llen.setMaximumWidth(55)
        line_row.addWidget(QLabel("Length (mm):"))
        line_row.addWidget(self._dwell_llen)
        self._dwell_langle = QLineEdit("0")
        self._dwell_langle.setMaximumWidth(45)
        self._dwell_langle.setToolTip("Line orientation in degrees (0 = horizontal / X-axis).")
        line_row.addWidget(QLabel("Angle (°):"))
        line_row.addWidget(self._dwell_langle)
        line_row.addStretch()
        size_stack.addWidget(line_page)

        size_stack.setCurrentIndex(0)  # default: circle
        self._dwell_size_stack = size_stack
        dwell_form.addRow("", size_stack)

        feedrate_hint = QLabel("Dwell feedrate is computed automatically from path length ÷ time.")
        feedrate_hint.setStyleSheet("color: #888; font-style: italic;")
        feedrate_hint.setWordWrap(True)
        dwell_form.addRow("", feedrate_hint)

        r_row = QHBoxLayout()
        self._dwell_repeats = QLineEdit("1")
        self._dwell_repeats.setMaximumWidth(40)
        self._dwell_repeats.setToolTip(
            "Retrace the dwell path this many times within the same dwell "
            "time (feedrate scales up accordingly) -- multiple slower passes "
            "spread across the full dwell duration tend to heat more evenly "
            "than a single continuous sweep."
        )
        r_row.addWidget(QLabel("Repeats:"))
        r_row.addWidget(self._dwell_repeats)
        r_row.addStretch()
        dwell_form.addRow("", r_row)

        m_row = QHBoxLayout()
        self._dwell_z = QLineEdit("0")
        self._dwell_z.setMaximumWidth(55)
        m_row.addWidget(QLabel("Z (mm):"))
        m_row.addWidget(self._dwell_z)
        m_row.addStretch()
        dwell_form.addRow("", m_row)

        e_row = QHBoxLayout()
        self._dwell_v = QLineEdit("13.4")
        self._dwell_v.setMaximumWidth(55)
        self._dwell_i = QLineEdit("1.95")
        self._dwell_i.setMaximumWidth(55)
        e_row.addWidget(QLabel("V:"))
        e_row.addWidget(self._dwell_v)
        e_row.addWidget(QLabel("I (A):"))
        e_row.addWidget(self._dwell_i)
        e_row.addStretch()
        dwell_form.addRow("", e_row)

        ctrl_layout.addWidget(dwell_gb)

        # ---- Actions ----
        act_gb = QGroupBox("Actions")
        act_layout = QHBoxLayout(act_gb)
        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self._clear_all)
        act_layout.addWidget(clear_btn)
        export_btn = QPushButton("Export CSV\u2026")
        export_btn.clicked.connect(self._export_csv)
        act_layout.addWidget(export_btn)
        ctrl_layout.addWidget(act_gb)

        # ---- Summary ----
        self._status_label = QLabel("0 points defined")
        ctrl_layout.addWidget(self._status_label)
        ctrl_layout.addStretch()

        splitter.addWidget(ctrl)

        # --- Right: QGraphicsView preview ---
        self._scene = QGraphicsScene()
        self._view = QGraphicsView(self._scene)
        self._view.setRenderHint(QPainter.Antialiasing)
        self._view.setDragMode(QGraphicsView.ScrollHandDrag)
        self._view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self._view.setBackgroundBrush(QBrush(QColor("#f5f5f5")))
        splitter.addWidget(self._view)

        splitter.setSizes([320, 780])

        # Layout
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.addWidget(splitter)

        # Status bar
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

    def _on_shape_changed(self, text: str) -> None:
        idx = {"circle": 0, "rectangle": 1, "line": 2}.get(text, 0)
        self._dwell_size_stack.setCurrentIndex(idx)

    def _add_grid(self) -> None:
        try:
            ox = float(self._grid_ox.text())
            oy = float(self._grid_oy.text())
            rows = int(self._grid_rows.text())
            cols = int(self._grid_cols.text())
            sx = float(self._grid_sx.text())
            sy = float(self._grid_sy.text())
            prefix = self._grid_prefix.text().strip()
        except ValueError:
            QMessageBox.warning(self, "Input Error",
                                "All grid fields must be numeric (except prefix).")
            return
        if rows < 1 or cols < 1 or sx <= 0 or sy <= 0:
            QMessageBox.warning(self, "Input Error", "Rows/cols >= 1, spacing > 0.")
            return

        grid = _Grid(ox, oy, rows, cols, sx, sy, prefix)
        self._grids.append(grid)
        _grid_registry.append(grid)
        self._redraw()
        self._update_summary()

    def _add_custom_point(self) -> None:
        try:
            x = float(self._cp_x.text())
            y = float(self._cp_y.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "X and Y must be numbers.")
            return
        label = self._cp_label.text().strip()
        self._custom_points.append(_CustomPoint(x, y, label))
        self._cp_x.clear()
        self._cp_y.clear()
        self._cp_label.clear()
        self._redraw()
        self._update_summary()

    def _clear_all(self) -> None:
        self._grids.clear()
        _grid_registry.clear()
        self._custom_points.clear()
        self._redraw()
        self._update_summary()

    def _update_summary(self) -> None:
        n = sum(g.rows * g.cols for g in self._grids) + len(self._custom_points)
        self._status_label.setText(f"{n} points defined")

    # ------------------------------------------------------------------
    #  QGraphicsScene preview
    # ------------------------------------------------------------------

    def _redraw(self) -> None:
        self._scene.clear()

        for grid in self._grids:
            color = grid.color
            brush = QBrush(color)
            pen = QPen(color.darker(130))
            pen.setWidthF(0.5)
            font = QFont("sans-serif", 6)

            for x, y, lbl in grid.points():
                sx = x * _CANVAS_SCALE
                sy = y * _CANVAS_SCALE
                r = _POINT_RADIUS

                ellipse = self._scene.addEllipse(
                    sx - r, sy - r, r * 2, r * 2, pen, brush,
                )
                ellipse.setZValue(1)

                if lbl:
                    text = self._scene.addSimpleText(lbl, font)
                    text.setPos(sx + r + 2, sy - r - 2)
                    text.setBrush(brush)
                    text.setZValue(0)

        # Custom points
        red = QColor("#F44336")
        red_brush = QBrush(red)
        for cp in self._custom_points:
            sx = cp.x * _CANVAS_SCALE
            sy = cp.y * _CANVAS_SCALE
            r = _POINT_RADIUS + 1
            self._scene.addEllipse(sx - r, sy - r, r * 2, r * 2,
                                   QPen(red.darker(130)), red_brush)
            if cp.label:
                text = self._scene.addSimpleText(cp.label)
                text.setPos(sx + r + 2, sy - r - 2)
                text.setBrush(red_brush)

        # Fit
        rect = self._scene.itemsBoundingRect()
        if not rect.isEmpty():
            rect.adjust(-30, -30, 30, 30)
            self._scene.setSceneRect(rect)
            self._view.fitInView(rect, Qt.KeepAspectRatio)

    # ------------------------------------------------------------------
    #  Export to heat_locations CSV
    # ------------------------------------------------------------------

    def _export_csv(self) -> None:
        all_points: list[tuple[float, float, str]] = []
        for grid in self._grids:
            all_points.extend(grid.points())
        for cp in self._custom_points:
            all_points.append((cp.x, cp.y, cp.label))

        if not all_points:
            QMessageBox.warning(self, "No points",
                                "Add grids or points before exporting.")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Export heat locations CSV", "", "CSV files (*.csv)",
        )
        if not path:
            return

        try:
            dwell_time = float(self._dwell_time.text())
            radius = float(self._dwell_radius.text())
            z_val = float(self._dwell_z.text())
            voltage = float(self._dwell_v.text())
            current = float(self._dwell_i.text())
            repeats = int(self._dwell_repeats.text())
            if repeats < 1:
                raise ValueError("repeats must be >= 1")
            num_rings = int(self._dwell_num_rings.text())
            if num_rings < 1:
                raise ValueError("rings must be >= 1")
            shape = self._dwell_shape_cb.currentText()
            width_mm = ""
            height_mm = ""
            angle_deg = ""
            if shape == "rectangle":
                width_mm = f"{float(self._dwell_width.text()):.1f}"
                height_mm = f"{float(self._dwell_height.text()):.1f}"
            elif shape == "line":
                width_mm = f"{float(self._dwell_llen.text()):.1f}"
                angle_deg = f"{float(self._dwell_langle.text()):.1f}"
        except ValueError:
            QMessageBox.warning(self, "Input Error",
                                "All dwell parameters must be numeric "
                                "(repeats and rings must be whole numbers >= 1).")
            return

        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "label", "x", "y", "z", "dwell_time_s", "radius_mm",
                "voltage_v", "current_a",
                "shape", "width_mm", "height_mm", "repeats", "num_rings", "angle_deg",
            ])
            for i, (x, y, lbl) in enumerate(all_points):
                writer.writerow([
                    lbl or f"pt_{i + 1}",
                    f"{x:.3f}", f"{y:.3f}", f"{z_val:.3f}",
                    f"{dwell_time:.1f}", f"{radius:.1f}",
                    f"{voltage:.2f}", f"{current:.2f}",
                    shape, width_mm, height_mm, repeats, num_rings, angle_deg,
                ])

        QMessageBox.information(
            self, "Exported",
            f"Saved {len(all_points)} heat locations to\n{path}",
        )


# ======================================================================
#  Entry point
# ======================================================================

def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = PatternUtilityWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
