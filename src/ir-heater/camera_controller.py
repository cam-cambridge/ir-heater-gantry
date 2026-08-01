"""Multi USB-camera capture and recording using OpenCV.

Provides a ``CameraController`` that manages an arbitrary number of labeled
cameras simultaneously. Each camera runs its own frame-grab thread so the
main / GUI thread is never blocked by I/O. Cameras are identified by a
user-assigned label (not just an OS device index), since USB webcams can
enumerate at different indices across reboots/reconnects -- see
``camera_setup_dialog.CameraSetupDialog`` for the live-preview picker that
lets a user assign labels to indices by actually looking at the feed.

Usage::

    cam = CameraController(cameras=[("top", 0), ("side", 1)], fps=15.0)
    cam.start_preview()
    frames = cam.get_frames()                # {label: BGR numpy array | None}
    cam.start_recording(Path("output_dir"))
    ...
    cam.stop_recording()
    cam.stop_preview()
"""

from __future__ import annotations

import csv
import multiprocessing
import queue
import re
import sys
import threading
import time
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path
from typing import TextIO

import cv2
import numpy as np

_CV2_CAP_PROP_FPS = 5

# ---------------------------------------------------------------------------
#  Defaults (overridable via camera_config.ini)
# ---------------------------------------------------------------------------
_DEFAULT_FOURCC = "mp4v"
_DEFAULT_JOIN_TIMEOUT = 2.0
_DEFAULT_MIN_FPS = 0.1
_DEFAULT_FPS = 15.0

CameraSpec = tuple[str, int]  # (label, device_id)


def _safe_filename_part(label: str) -> str:
    """Sanitize a user-supplied camera label for use in a filename."""
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", label.strip())
    return cleaned.strip("_") or "cam"


def _open_capture(cam_id: int, attempts: int = 3, retry_delay: float = 0.4) -> cv2.VideoCapture:
    """Open a camera, working around Windows' unreliable auto-selected backend.

    ``cv2.VideoCapture(id)`` on Windows defaults to Media Foundation (MSMF),
    which intermittently reports ``isOpened() == False`` for UVC webcams that
    other apps (e.g. the Windows Camera app, which talks to MF directly, not
    through OpenCV) can open fine. DirectShow (``CAP_DSHOW``) is far more
    reliable for this case, so try it first on Windows, then fall back to
    OpenCV's default backend selection. A short retry loop also helps with
    cameras that need a moment to become available after being released by
    another application.
    """
    backends = [cv2.CAP_DSHOW] if sys.platform == "win32" else []
    cap: cv2.VideoCapture | None = None
    for backend in backends:
        for _ in range(attempts):
            cap = cv2.VideoCapture(cam_id, backend)
            if cap.isOpened():
                return cap
            cap.release()
            time.sleep(retry_delay)
    for _ in range(attempts):
        cap = cv2.VideoCapture(cam_id)
        if cap.isOpened():
            return cap
        cap.release()
        time.sleep(retry_delay)
    return cap


def _load_camera_config(config_path: Path | None = None) -> dict[str, str]:
    """Load camera defaults from an INI-style file, falling back to built-in values."""
    if config_path is None:
        config_path = Path(__file__).with_name("camera_config.ini")
    if not config_path.exists():
        return {}
    result: dict[str, str] = {}
    with config_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or "=" not in line:
                continue
            key, _, val = line.partition("=")
            result[key.strip()] = val.strip().strip('"').strip("'")
    return result


class _CameraThread:
    """Grabs frames from one camera in a background loop."""

    def __init__(
        self,
        cam_id: int,
        fps: float,
        label: str,
        fourcc: str = _DEFAULT_FOURCC,
        min_fps: float = _DEFAULT_MIN_FPS,
        join_timeout: float = _DEFAULT_JOIN_TIMEOUT,
        on_error: Callable[[str, str], None] | None = None,
    ) -> None:
        self._id = cam_id
        self._fps = fps
        self._label = label
        self._fourcc = fourcc
        self._join_timeout = join_timeout
        self._on_error = on_error
        self._cap: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._writer: cv2.VideoWriter | None = None
        self._frame_interval = 1.0 / max(fps, min_fps)
        self._error: str | None = None
        self._frames_csv_file = None
        self._frames_csv_writer = None
        self._frame_index = 0

    # ------------------------------------------------------------------
    def _open(self) -> None:
        self._cap = _open_capture(self._id)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self._id} ({self._label})")
        # Try to set requested FPS (best-effort; many USB cameras ignore this)
        self._cap.set(_CV2_CAP_PROP_FPS, self._fps)

    def _loop(self) -> None:
        try:
            self._open()
        except Exception as exc:
            # Report the failure instead of letting it kill this daemon
            # thread silently -- an uncaught exception here previously left
            # the camera looking permanently "not connected" with no clue
            # why (see _open_capture docstring for the common Windows cause).
            self._error = str(exc)
            if self._on_error is not None:
                self._on_error(self._label, self._error)
            return

        last_write = 0.0
        try:
            while self._running:
                ok, frame = self._cap.read()  # type: ignore[union-attr]
                if not ok:
                    time.sleep(0.01)
                    continue

                with self._lock:
                    self._latest_frame = frame

                # Throttled recording write
                now = time.perf_counter()
                if self._writer is not None and (now - last_write) >= self._frame_interval:
                    self._writer.write(frame)
                    if self._frames_csv_writer is not None:
                        self._frames_csv_writer.writerow(
                            [self._frame_index, datetime.now(tz=UTC).isoformat()]
                        )
                        self._frame_index += 1
                    last_write = now
        finally:
            if self._cap is not None:
                self._cap.release()

    # ------------------------------------------------------------------
    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cam-{self._label}")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=self._join_timeout)

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    @property
    def error(self) -> str | None:
        return self._error

    # --- Recording -------------------------------------------------------

    def start_writer(self, output_dir: Path, fourcc: str | None = None) -> Path:
        """Begin recording to *output_dir* / ``<label>_<timestamp>.mp4``.

        Also opens a ``<label>_<timestamp>_frames.csv`` sidecar logging the
        wall-clock time of every written frame. ``VideoWriter`` assumes a
        constant frame rate, so if real capture ever dips below the target
        fps, the video's internal timeline silently drifts from wall-clock
        time; the sidecar gives an exact, independent record for mapping any
        frame back to real time during post-hoc analysis.

        Returns the output video file path.
        """
        frame = self.get_frame()
        if frame is None:
            detail = f": {self._error}" if self._error else ""
            raise RuntimeError(f"No frame available from camera {self._label!r}{detail}")

        timestamp = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S")
        stem = f"{_safe_filename_part(self._label)}_{timestamp}"
        output_dir.mkdir(parents=True, exist_ok=True)
        out_path = output_dir / f"{stem}.mp4"

        h, w = frame.shape[:2]
        fourcc_code = cv2.VideoWriter_fourcc(*(fourcc if fourcc is not None else self._fourcc))
        self._writer = cv2.VideoWriter(str(out_path), fourcc_code, self._fps, (w, h))
        if not self._writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {out_path}")

        self._frame_index = 0
        self._frames_csv_file = (output_dir / f"{stem}_frames.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._frames_csv_writer = csv.writer(self._frames_csv_file)
        self._frames_csv_writer.writerow(["frame_index", "wall_clock_utc"])
        return out_path

    def stop_writer(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self._frames_csv_file is not None:
            self._frames_csv_file.close()
            self._frames_csv_file = None
            self._frames_csv_writer = None


# ======================================================================


class CameraController:
    """Manages an arbitrary number of labeled USB cameras for preview and recording.

    Parameters
    ----------
    cameras:
        ``[(label, device_id), ...]`` -- labels must be unique and are used
        both as dict keys throughout this API and as output filename
        prefixes when recording.
    fps:
        Target frames per second (best-effort).
    config_path:
        Optional path to a ``camera_config.ini`` file.  When omitted the
        default ``camera_config.ini`` next to this module is used if it
        exists.
    on_error:
        Optional ``(label, message) -> None`` callback invoked from a
        background thread if a camera fails to open. Prefer polling
        :meth:`errors` from the GUI thread over touching widgets in this
        callback directly.
    """

    def __init__(
        self,
        cameras: list[CameraSpec],
        fps: float = _DEFAULT_FPS,
        config_path: Path | None = None,
        on_error: Callable[[str, str], None] | None = None,
    ) -> None:
        labels = [label for label, _ in cameras]
        if len(labels) != len(set(labels)):
            raise ValueError(f"Camera labels must be unique, got: {labels}")

        cfg = _load_camera_config(config_path)
        fourcc = cfg.get("fourcc", _DEFAULT_FOURCC)
        min_fps = float(cfg.get("min_fps", _DEFAULT_MIN_FPS))
        join_timeout = float(cfg.get("join_timeout", _DEFAULT_JOIN_TIMEOUT))

        self._specs: list[CameraSpec] = list(cameras)
        self._threads: dict[str, _CameraThread] = {
            label: _CameraThread(
                cam_id, fps, label, fourcc=fourcc,
                min_fps=min_fps, join_timeout=join_timeout, on_error=on_error,
            )
            for label, cam_id in cameras
        }
        self._fps = fps
        self._recording = False
        self._record_dir: Path | None = None

    # ------------------------------------------------------------------
    #  Lifecycle
    # ------------------------------------------------------------------

    def start_preview(self) -> None:
        """Open every camera and begin background frame grabbing."""
        for thread in self._threads.values():
            thread.start()

    def stop_preview(self) -> None:
        """Stop frame grabbing and release all cameras."""
        if self._recording:
            self.stop_recording()
        for thread in self._threads.values():
            thread.stop()

    # ------------------------------------------------------------------
    #  Frames
    # ------------------------------------------------------------------

    def get_frames(self) -> dict[str, np.ndarray | None]:
        """Return the latest frame from each camera, keyed by label."""
        return {label: thread.get_frame() for label, thread in self._threads.items()}

    def get_frame(self, label: str) -> np.ndarray | None:
        return self._threads[label].get_frame()

    def errors(self) -> dict[str, str | None]:
        """Return the current open/read error (if any) for each camera."""
        return {label: thread.error for label, thread in self._threads.items()}

    # ------------------------------------------------------------------
    #  Recording
    # ------------------------------------------------------------------

    def start_recording(self, output_dir: Path) -> dict[str, Path]:
        """Start writing video files for every camera. Returns ``{label: path}``.

        Raises ``RuntimeError`` if recording is already in progress. If any
        camera fails to start, writers already opened for earlier cameras
        are cleaned up before the exception propagates.
        """
        if self._recording:
            raise RuntimeError("Recording is already in progress")
        self._record_dir = output_dir
        paths: dict[str, Path] = {}
        try:
            for label, thread in self._threads.items():
                paths[label] = thread.start_writer(output_dir)
        except Exception:
            for thread in self._threads.values():
                thread.stop_writer()
            raise
        self._recording = True
        for label, path in paths.items():
            print(f"Recording {label} → {path}", flush=True)
        return paths

    def stop_recording(self) -> None:
        """Finalise every video file."""
        if not self._recording:
            return
        for thread in self._threads.values():
            thread.stop_writer()
        self._recording = False
        print("Recording stopped.", flush=True)

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def camera_specs(self) -> list[CameraSpec]:
        """``[(label, device_id), ...]`` in the order cameras were configured."""
        return list(self._specs)

    # ------------------------------------------------------------------
    #  Snapshots
    # ------------------------------------------------------------------

    def capture_snapshot(self, output_dir: Path, prefix: str = "snap") -> dict[str, Path]:
        """Save the current frame from each camera as a PNG. Returns ``{label: path}``."""
        output_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S_%f")
        paths: dict[str, Path] = {}
        for label, thread in self._threads.items():
            frame = thread.get_frame()
            if frame is None:
                continue
            path = output_dir / f"{prefix}_{_safe_filename_part(label)}_{ts}.png"
            cv2.imwrite(str(path), frame)
            paths[label] = path
        return paths


# ======================================================================
#  Process-based recorder  (isolates disk I/O from timing-critical loop)
# ======================================================================

class CameraRecorderProcess(multiprocessing.Process):
    """Runs multi-camera capture + recording in a separate process.

    Communication via two :class:`~multiprocessing.Queue` instances:

    *cmd_queue* (main → recorder):
        ``("start", output_dir)`` — begin recording
        ``("stop",)``            — finalise video files
        ``("shutdown",)``        — exit the process

    *result_queue* (recorder → main):
        ``("started", {label: path_str, ...})``   — recording has begun
        ``("stopped", {label: path_str, ...})``   — recording finalised
        ``("error", message)``                    — an error occurred
    """

    def __init__(
        self,
        cameras: list[CameraSpec],
        fps: float,
        cmd_queue: multiprocessing.Queue,
        result_queue: multiprocessing.Queue,
        fourcc: str = _DEFAULT_FOURCC,
        min_fps: float = _DEFAULT_MIN_FPS,
    ) -> None:
        super().__init__(daemon=True, name="cam-recorder")
        self._cameras = list(cameras)
        self._fps = fps
        self._cmd_queue = cmd_queue
        self._result_queue = result_queue
        self._fourcc = fourcc
        self._min_fps = min_fps

    def run(self) -> None:
        labels = [label for label, _ in self._cameras]
        caps: list[cv2.VideoCapture] = []
        writers: list[cv2.VideoWriter | None] = [None] * len(self._cameras)
        # Per-frame timestamp sidecars -- see _CameraThread.start_writer's
        # docstring for why VideoWriter's constant-fps assumption needs this.
        frame_logs: list[TextIO | None] = [None] * len(self._cameras)
        frame_counts: list[int] = [0] * len(self._cameras)
        frame_interval = 1.0 / max(self._fps, self._min_fps)
        paths: dict[str, Path] = {}

        def _close_frame_logs() -> None:
            for i, f in enumerate(frame_logs):
                if f is not None:
                    f.close()
                frame_logs[i] = None

        try:
            # --- Open cameras ---
            for label, cam_id in self._cameras:
                cap = _open_capture(cam_id)
                if not cap.isOpened():
                    self._result_queue.put(("error", f"Cannot open camera {cam_id} ({label})"))
                    return
                cap.set(cv2.CAP_PROP_FPS, self._fps)
                caps.append(cap)

            # --- Helper: open writers for every camera immediately ---
            def _open_writers(out_dir: Path, ts: str) -> dict[str, Path]:
                """Create video writers + frame-timestamp sidecars at *out_dir*/<label>_<ts>.*"""
                new_paths: dict[str, Path] = {}
                fourcc_code = cv2.VideoWriter_fourcc(*self._fourcc)

                for i, (label, cap) in enumerate(zip(labels, caps)):
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        self._result_queue.put(
                            ("error", f"No frame from camera {label!r} to initialise writer")
                        )
                        return new_paths  # caller checks writers[i] is not None
                    h, w = frame.shape[:2]
                    stem = f"{_safe_filename_part(label)}_{ts}"
                    p = out_dir / f"{stem}.mp4"
                    writers[i] = cv2.VideoWriter(str(p), fourcc_code, self._fps, (w, h))
                    log_file = (out_dir / f"{stem}_frames.csv").open(
                        "w", newline="", encoding="utf-8"
                    )
                    csv.writer(log_file).writerow(["frame_index", "wall_clock_utc"])
                    frame_logs[i] = log_file
                    frame_counts[i] = 0
                    new_paths[label] = p
                return new_paths

            recording = False
            last_write = 0.0

            while True:
                # --- Blocking wait for next command or frame interval ---
                try:
                    cmd = self._cmd_queue.get(timeout=frame_interval)
                except queue.Empty:
                    cmd = None

                if cmd is not None:
                    action = cmd[0]
                    if action == "shutdown":
                        break
                    elif action == "start":
                        output_dir = Path(cmd[1])
                        output_dir.mkdir(parents=True, exist_ok=True)
                        ts = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S")
                        paths = _open_writers(output_dir, ts)
                        if all(w is not None for w in writers):
                            recording = True
                            self._result_queue.put(
                                ("started", {label: str(p) for label, p in paths.items()})
                            )
                        else:
                            # _open_writers already sent the error message
                            recording = False
                    elif action == "stop":
                        recording = False
                        for w in writers:
                            if w is not None:
                                w.release()
                        writers = [None] * len(self._cameras)
                        _close_frame_logs()
                        self._result_queue.put(
                            ("stopped", {label: str(p) for label, p in paths.items()})
                        )
                        continue

                if not recording:
                    continue

                # --- Capture frames ---
                frames: list[np.ndarray | None] = []
                for cap in caps:
                    ok, frame = cap.read()
                    frames.append(frame if ok else None)

                # --- Throttled write ---
                now = time.perf_counter()
                if (now - last_write) >= frame_interval:
                    frame_ts = datetime.now(tz=UTC).isoformat()
                    for i, frame in enumerate(frames):
                        if frame is None:
                            continue
                        if writers[i] is not None:
                            writers[i].write(frame)
                            log_file = frame_logs[i]
                            if log_file is not None:
                                csv.writer(log_file).writerow([frame_counts[i], frame_ts])
                                frame_counts[i] += 1
                    last_write = now

        except Exception as exc:
            # Top-level safety net — report any unhandled error to the
            # parent process so it isn't lost in a daemon process.
            self._result_queue.put(("error", str(exc)))
        finally:
            for w in writers:
                if w is not None:
                    w.release()
            for cap in caps:
                cap.release()
            _close_frame_logs()
