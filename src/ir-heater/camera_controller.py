"""Dual USB-camera capture and recording using OpenCV.

Provides a ``CameraController`` that manages two cameras simultaneously.
Each camera runs its own frame-grab thread so the main / GUI thread is
never blocked by I/O.

Usage::

    cam = CameraController(cam0_id=0, cam1_id=1, fps=15.0)
    cam.start_preview()
    frame0, frame1 = cam.get_frames()        # latest frames (BGR numpy arrays)
    cam.start_recording(Path("output_dir"))
    ...
    cam.stop_recording()
    cam.stop_preview()
"""

from __future__ import annotations

import multiprocessing
import queue
import threading
import time
from datetime import UTC, datetime
from pathlib import Path

import cv2
import numpy as np

_CV2_CAP_PROP_FPS = 5

# ---------------------------------------------------------------------------
#  Defaults (overridable via camera_config.ini)
# ---------------------------------------------------------------------------
_DEFAULT_FOURCC = "mp4v"
_DEFAULT_JOIN_TIMEOUT = 2.0
_DEFAULT_MIN_FPS = 0.1
_DEFAULT_CAM0_ID = 0
_DEFAULT_CAM1_ID = 1
_DEFAULT_FPS = 15.0


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
    ) -> None:
        self._id = cam_id
        self._fps = fps
        self._label = label
        self._fourcc = fourcc
        self._join_timeout = join_timeout
        self._cap: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._writer: cv2.VideoWriter | None = None
        self._frame_interval = 1.0 / max(fps, min_fps)

    # ------------------------------------------------------------------
    def _open(self) -> None:
        self._cap = cv2.VideoCapture(self._id)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self._id} ({self._label})")
        # Try to set requested FPS (best-effort; many USB cameras ignore this)
        self._cap.set(_CV2_CAP_PROP_FPS, self._fps)

    def _loop(self) -> None:
        self._open()
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
                    last_write = now
        finally:
            if self._cap is not None:
                self._cap.release()

    # ------------------------------------------------------------------
    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cam-{self._id}")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=self._join_timeout)

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    # --- Recording -------------------------------------------------------

    def start_writer(self, output_dir: Path, fourcc: str | None = None) -> Path:
        """Begin recording to *output_dir* / ``cam<id>_<timestamp>.mp4``.

        Returns the output file path.
        """
        frame = self.get_frame()
        if frame is None:
            raise RuntimeError(f"No frame available from camera {self._id}")

        timestamp = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S")
        fname = f"cam{self._id}_{timestamp}.mp4"
        output_dir.mkdir(parents=True, exist_ok=True)
        out_path = output_dir / fname

        h, w = frame.shape[:2]
        fourcc_code = cv2.VideoWriter_fourcc(*(fourcc if fourcc is not None else self._fourcc))
        self._writer = cv2.VideoWriter(str(out_path), fourcc_code, self._fps, (w, h))
        if not self._writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {out_path}")
        return out_path

    def stop_writer(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None


# ======================================================================


class CameraController:
    """Manages two USB cameras for preview and recording.

    Parameters
    ----------
    cam0_id, cam1_id:
        Device IDs for each camera (passed to ``cv2.VideoCapture``).
    fps:
        Target frames per second (best-effort).
    config_path:
        Optional path to a ``camera_config.ini`` file.  When omitted the
        default ``camera_config.ini`` next to this module is used if it
        exists.
    """

    def __init__(
        self,
        cam0_id: int = _DEFAULT_CAM0_ID,
        cam1_id: int = _DEFAULT_CAM1_ID,
        fps: float = _DEFAULT_FPS,
        config_path: Path | None = None,
    ) -> None:
        cfg = _load_camera_config(config_path)
        fourcc = cfg.get("fourcc", _DEFAULT_FOURCC)
        min_fps = float(cfg.get("min_fps", _DEFAULT_MIN_FPS))
        join_timeout = float(cfg.get("join_timeout", _DEFAULT_JOIN_TIMEOUT))

        self._cam0 = _CameraThread(cam0_id, fps, "cam0", fourcc=fourcc,
                                   min_fps=min_fps, join_timeout=join_timeout)
        self._cam1 = _CameraThread(cam1_id, fps, "cam1", fourcc=fourcc,
                                   min_fps=min_fps, join_timeout=join_timeout)
        self._fps = fps
        self._recording = False
        self._record_dir: Path | None = None

    # ------------------------------------------------------------------
    #  Lifecycle
    # ------------------------------------------------------------------

    def start_preview(self) -> None:
        """Open both cameras and begin background frame grabbing."""
        self._cam0.start()
        self._cam1.start()

    def stop_preview(self) -> None:
        """Stop frame grabbing and release cameras."""
        if self._recording:
            self.stop_recording()
        self._cam0.stop()
        self._cam1.stop()

    # ------------------------------------------------------------------
    #  Frames
    # ------------------------------------------------------------------

    def get_frames(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Return the latest frame from each camera (BGR arrays or None)."""
        return self._cam0.get_frame(), self._cam1.get_frame()

    def get_frame0(self) -> np.ndarray | None:
        return self._cam0.get_frame()

    def get_frame1(self) -> np.ndarray | None:
        return self._cam1.get_frame()

    # ------------------------------------------------------------------
    #  Recording
    # ------------------------------------------------------------------

    def start_recording(self, output_dir: Path) -> tuple[Path, Path]:
        """Start writing video files for both cameras.  Returns (path0, path1).

        Raises ``RuntimeError`` if recording is already in progress.
        """
        if self._recording:
            raise RuntimeError("Recording is already in progress")
        self._record_dir = output_dir
        # Start cam0 first; if cam1 fails, clean up cam0's writer.
        p0 = self._cam0.start_writer(output_dir)
        try:
            p1 = self._cam1.start_writer(output_dir)
        except Exception:
            self._cam0.stop_writer()
            raise
        self._recording = True
        print(f"Recording cam0 → {p0}", flush=True)
        print(f"Recording cam1 → {p1}", flush=True)
        return p0, p1

    def stop_recording(self) -> None:
        """Finalise both video files."""
        if not self._recording:
            return
        self._cam0.stop_writer()
        self._cam1.stop_writer()
        self._recording = False
        print("Recording stopped.", flush=True)

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def cam0_id(self) -> int:
        return self._cam0._id

    @property
    def cam1_id(self) -> int:
        return self._cam1._id

    # ------------------------------------------------------------------
    #  Snapshots
    # ------------------------------------------------------------------

    def capture_snapshot(self, output_dir: Path, prefix: str = "snap") -> tuple[Path, Path]:
        """Save the current frame from each camera as a PNG.  Returns (p0, p1)."""
        output_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S_%f")
        p0 = output_dir / f"{prefix}_cam0_{ts}.png"
        p1 = output_dir / f"{prefix}_cam1_{ts}.png"

        f0 = self._cam0.get_frame()
        if f0 is not None:
            cv2.imwrite(str(p0), f0)
        f1 = self._cam1.get_frame()
        if f1 is not None:
            cv2.imwrite(str(p1), f1)
        return p0, p1


# ======================================================================
#  Process-based recorder  (isolates disk I/O from timing-critical loop)
# ======================================================================

class CameraRecorderProcess(multiprocessing.Process):
    """Runs dual-camera capture + recording in a separate process.

    Communication via two :class:`~multiprocessing.Queue` instances:

    *cmd_queue* (main → recorder):
        ``("start", output_dir)`` — begin recording
        ``("stop",)``            — finalise video files
        ``("shutdown",)``        — exit the process

    *result_queue* (recorder → main):
        ``("started", path0, path1)``   — recording has begun
        ``("stopped", path0, path1)``   — recording finalised
        ``("error", message)``          — an error occurred
    """

    def __init__(
        self,
        cam0_id: int,
        cam1_id: int,
        fps: float,
        cmd_queue: multiprocessing.Queue,
        result_queue: multiprocessing.Queue,
        fourcc: str = _DEFAULT_FOURCC,
        min_fps: float = _DEFAULT_MIN_FPS,
    ) -> None:
        super().__init__(daemon=True, name="cam-recorder")
        self._cam0_id = cam0_id
        self._cam1_id = cam1_id
        self._fps = fps
        self._cmd_queue = cmd_queue
        self._result_queue = result_queue
        self._fourcc = fourcc
        self._min_fps = min_fps

    def run(self) -> None:
        caps: list[cv2.VideoCapture] = []
        writers: list[cv2.VideoWriter | None] = [None, None]
        frame_interval = 1.0 / max(self._fps, self._min_fps)
        paths: tuple[Path, Path] = (Path("cam0.avi"), Path("cam1.avi"))

        try:
            # --- Open cameras ---
            for cam_id in (self._cam0_id, self._cam1_id):
                cap = cv2.VideoCapture(cam_id)
                if not cap.isOpened():
                    self._result_queue.put(("error", f"Cannot open camera {cam_id}"))
                    return
                cap.set(cv2.CAP_PROP_FPS, self._fps)
                caps.append(cap)

            # --- Helper: open writers for both cameras immediately ---
            def _open_writers(
                out_dir: Path, ts: str
            ) -> tuple[Path, Path]:
                """Create video writers at *out_dir*/cam{0,1}_{ts}.mp4."""
                p0 = out_dir / f"cam0_{ts}.mp4"
                p1 = out_dir / f"cam1_{ts}.mp4"
                fourcc_code = cv2.VideoWriter_fourcc(*self._fourcc)

                for i, cap in enumerate(caps):
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        self._result_queue.put(
                            ("error", f"No frame from camera {i} to initialise writer")
                        )
                        return p0, p1  # caller checks writers[i] is not None
                    h, w = frame.shape[:2]
                    writers[i] = cv2.VideoWriter(
                        str([p0, p1][i]), fourcc_code, self._fps, (w, h),
                    )
                return p0, p1

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
                        if writers[0] is not None and writers[1] is not None:
                            recording = True
                            self._result_queue.put(
                                ("started", str(paths[0]), str(paths[1]))
                            )
                        else:
                            # _open_writers already sent the error message
                            recording = False
                    elif action == "stop":
                        recording = False
                        for w in writers:
                            if w is not None:
                                w.release()
                        writers = [None, None]
                        self._result_queue.put(
                            ("stopped", str(paths[0]), str(paths[1]))
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
                    for i, frame in enumerate(frames):
                        if frame is None:
                            continue
                        if writers[i] is not None:
                            writers[i].write(frame)
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
