"""Live-preview dialog for identifying and labeling cameras.

Generic USB webcams almost always report an identical, unhelpful OS
description ("USB Video Device"), so a dropdown of device indices doesn't
actually tell you which physical camera is which -- and that mapping can
change across reconnects/reboots anyway. The only reliable way is to look
at the live feed. This dialog probes a range of device indices, shows a
live thumbnail for every one that responds, and lets the user check off
however many they want to use and assign each a label.
"""

from __future__ import annotations

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from camera_controller import CameraSpec, _open_capture

_PROBE_MAX_INDEX = 8
_THUMB_W = 220
_THUMB_H = 165
_POLL_MS = 150
_GRID_COLUMNS = 3


def _ndarray_to_pixmap(frame: np.ndarray, max_w: int = _THUMB_W) -> QPixmap:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
    if w > max_w:
        qimg = qimg.scaledToWidth(max_w, Qt.SmoothTransformation)
    return QPixmap.fromImage(qimg)


class _ProbeSlot(QGroupBox):
    """One detected camera index: live thumbnail + include checkbox + label field."""

    def __init__(self, index: int, cap: cv2.VideoCapture, parent: QWidget | None = None) -> None:
        super().__init__(f"Index {index}", parent)
        self.index = index
        self._cap: cv2.VideoCapture | None = cap
        # Some UVC cameras need several read() calls before the sensor
        # starts producing valid frames.  Discard those here so the
        # first QTimer-triggered poll already sees a live image.
        if self._cap is not None:
            for _ in range(10):
                self._cap.read()

        layout = QVBoxLayout(self)
        self._img_label = QLabel("(waiting for frame…)")
        self._img_label.setFixedSize(_THUMB_W, _THUMB_H)
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setStyleSheet("background:#1a1a1a;color:#888;")
        layout.addWidget(self._img_label)

        self.include_cb = QCheckBox("Use this camera")
        layout.addWidget(self.include_cb)

        self.label_edit = QLineEdit(f"cam{index}")
        self.label_edit.setPlaceholderText("Label (e.g. top, side)")
        layout.addWidget(self.label_edit)

    def update_frame(self) -> None:
        if self._cap is None:
            return
        # Try a few reads in case the camera needs a moment to produce a
        # valid frame (common on first poll for UVC cameras with slow
        # auto-exposure / AGC startup).
        for _ in range(3):
            ok, frame = self._cap.read()
            if ok and frame is not None:
                self._img_label.setPixmap(_ndarray_to_pixmap(frame))
                return

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class CameraSetupDialog(QDialog):
    """Probe camera indices, show live thumbnails, and let the user assign labels.

    Construct with the currently-configured cameras (if any) to pre-select
    and pre-label matching indices. Read the result via
    :meth:`selected_cameras` after ``exec()`` returns ``QDialog.Accepted``.
    """

    def __init__(
        self,
        current: list[CameraSpec] | None = None,
        max_probe_index: int = _PROBE_MAX_INDEX,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Identify Cameras")
        self.setMinimumSize(560, 420)

        self._slots: list[_ProbeSlot] = []

        root = QVBoxLayout(self)
        root.addWidget(QLabel(
            "Detected camera indices below, each with a live preview — check "
            "the ones you want to use and give each a label. Physical port "
            "order can change between reconnects, so re-identify visually "
            "rather than assuming an index always means the same camera."
        ))

        grid_host = QWidget()
        self._grid = QGridLayout(grid_host)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grid_host)
        root.addWidget(scroll, 1)

        self._status_label = QLabel("Probing for cameras…")
        root.addWidget(self._status_label)

        self._probe_indices(current or [], max_probe_index)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(_POLL_MS)

    # ------------------------------------------------------------------
    def _probe_indices(self, current: list[CameraSpec], max_probe_index: int) -> None:
        current_by_index = {idx: label for label, idx in current}
        col = row = 0
        for index in range(max_probe_index):
            cap = _open_capture(index, attempts=1, retry_delay=0.0)
            if not cap.isOpened():
                cap.release()
                continue
            slot = _ProbeSlot(index, cap)
            if index in current_by_index:
                slot.include_cb.setChecked(True)
                slot.label_edit.setText(current_by_index[index])
            self._slots.append(slot)
            self._grid.addWidget(slot, row, col)
            col += 1
            if col >= _GRID_COLUMNS:
                col = 0
                row += 1
        self._status_label.setText(
            f"Found {len(self._slots)} camera(s)." if self._slots
            else "No cameras detected. Check connections and try again."
        )

    def _poll(self) -> None:
        for slot in self._slots:
            slot.update_frame()

    # ------------------------------------------------------------------
    def selected_cameras(self) -> list[CameraSpec]:
        """Return ``[(label, index), ...]`` for every checked slot, in index order."""
        result: list[CameraSpec] = []
        seen_labels: set[str] = set()
        for slot in self._slots:
            if not slot.include_cb.isChecked():
                continue
            label = slot.label_edit.text().strip() or f"cam{slot.index}"
            if label in seen_labels:
                label = f"{label}_{slot.index}"
            seen_labels.add(label)
            result.append((label, slot.index))
        return result

    # ------------------------------------------------------------------
    def _release_all(self) -> None:
        self._timer.stop()
        for slot in self._slots:
            slot.release()

    def accept(self) -> None:
        self._release_all()
        super().accept()

    def reject(self) -> None:
        self._release_all()
        super().reject()

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        self._release_all()
        super().closeEvent(event)
