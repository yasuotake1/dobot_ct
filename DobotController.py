from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, List
import socket

from dobot.dobot_api import DobotApiDashboard, DobotApiFeedBack, MyType  # type: ignore


@dataclass(frozen=True)
class RobotState:
    """Latest robot feedback snapshot from port 30004."""
    t_monotonic: float
    robot_mode: int
    current_command_id: int
    q_actual: Tuple[float, float, float, float, float, float]
    tool_vector_actual: Tuple[float, float, float, float, float, float]


class _StateStore:
    """Thread-safe latest-state store with Condition for waiters."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._state: Optional[RobotState] = None

    def set(self, st: RobotState) -> None:
        with self._cv:
            self._state = st
            self._cv.notify_all()

    def snapshot(self) -> Optional[RobotState]:
        with self._lock:
            return self._state

    def wait_until(self, pred: Callable[[RobotState], bool], timeout: float) -> RobotState:
        end = time.monotonic() + float(timeout)
        with self._cv:
            while True:
                st = self._state
                if st is not None and pred(st):
                    return st
                remain = end - time.monotonic()
                if remain <= 0:
                    raise TimeoutError("wait_until timed out")
                self._cv.wait(timeout=remain)


class _FeedbackThread(threading.Thread):
    def __init__(self, fb: DobotApiFeedBack, store: _StateStore, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self._fb = fb
        self._store = store
        self._stop_event = stop_event

    def run(self) -> None:
        # Any socket/parsing failure should raise and stop the program (user preference).
        while not self._stop_event.is_set():
            try:
                a = self._fb.feedBackData()
                if a is None:
                    continue

                # numpy structured array: a["FieldName"][0]
                q = a["QActual"][0]
                tool = a["ToolVectorActual"][0]
                st = RobotState(
                    t_monotonic=time.monotonic(),
                    robot_mode=int(a["RobotMode"][0]),
                    current_command_id=int(a["CurrentCommandId"][0]),
                    q_actual=(float(q[0]), float(q[1]), float(q[2]), float(q[3]), float(q[4]), float(q[5])),
                    tool_vector_actual=(
                        float(tool[0]),
                        float(tool[1]),
                        float(tool[2]),
                        float(tool[3]),
                        float(tool[4]),
                        float(tool[5]),
                    ),
                )
                self._store.set(st)
            except Exception:
                if self._stop_event.is_set():
                    return
                else:
                    raise


class DobotController:
    """
    High-level controller that wraps:
      - Dashboard/control port 29999 (DobotApiDashboard)
      - Feedback streaming port 30004 (DobotApiFeedBack)

    Pose input format (list[float], length 7):
        [a, b, c, d, e, f, mode]

    mode meanings (as defined in dobot_api.py):
        0: pose   (Cartesian/TCP pose)
        1: joint  (J1..J6)

    Notes:
      - mov_j / mov_l select interpolation/path behavior (MovJ vs MovL).
        The meaning of the 6 values is determined solely by mode (0/1).
      - _last_cmd_id is overwritten on every mov_* command and is kept even after timeout.
    """

    DEFAULT_TIMEOUT = 10.0

    def __init__(self, ip: str, dashboard_port: int = 29999, feedback_port: int = 30004) -> None:
        self.ip = ip
        self.dashboard_port = int(dashboard_port)
        self.feedback_port = int(feedback_port)

        self._db: Optional[DobotApiDashboard] = None
        self._fb: Optional[DobotApiFeedBack] = None

        self._store = _StateStore()
        self._stop_event = threading.Event()
        self._fb_thread: Optional[_FeedbackThread] = None

        self._last_cmd_id: Optional[int] = None

    # --------------------
    # Context manager
    # --------------------
    def __enter__(self) -> "DobotController":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # --------------------
    # Lifecycle
    # --------------------
    def connect(self) -> None:
        if self._db is not None or self._fb is not None:
            return
        self._db = DobotApiDashboard(self.ip, self.dashboard_port)
        self._fb = DobotApiFeedBack(self.ip, self.feedback_port)

        self._stop_event.clear()
        self._fb_thread = _FeedbackThread(self._fb, self._store, self._stop_event)
        self._fb_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self._fb = None
        self._db = None
        self._fb_thread = None

    # --------------------
    # Thin wrappers: robot enable/disable, speed/acc, IO
    # --------------------
    def enable_robot(self) -> None:
        self._require_db().EnableRobot()

    def disable_robot(self) -> None:
        self._require_db().DisableRobot()

    def vel_j(self, value: float) -> None:
        self._require_db().VelJ(value)

    def vel_l(self, value: float) -> None:
        self._require_db().VelL(value)

    def acc_j(self, value: float) -> None:
        self._require_db().AccJ(value)

    def acc_l(self, value: float) -> None:
        self._require_db().AccL(value)

    def do(self, index: int, status: int, time_ms: int = -1) -> None:
        self._require_db().DO(index, status, time_ms)

    def di(self, index: int) -> int:
        s = self._require_db().DI(index)
        return int(self._get_value(s))
    
    def clear_error(self) -> str:
        return self._require_db().ClearError()

    # --------------------
    # Motion: async
    # --------------------
    def mov_j(self, pose: List[float]) -> int:
        """Send MovJ and return command id; updates _last_cmd_id."""
        self._validate_pose7(pose)
        s = self._require_db().MovJ(*pose)
        cmd_id = int(self._get_value(s))
        self._last_cmd_id = cmd_id
        return cmd_id

    def mov_l(self, pose: List[float]) -> int:
        """Send MovL and return command id; updates _last_cmd_id."""
        self._validate_pose7(pose)
        s = self._require_db().MovL(*pose)
        cmd_id = int(self._get_value(s))
        self._last_cmd_id = cmd_id
        return cmd_id

    # --------------------
    # Motion: sync
    # --------------------
    def mov_j_wait(self, pose_joints_or_tcp: List[float], timeout: float = DEFAULT_TIMEOUT) -> None:
        cmd_id = self.mov_j(pose_joints_or_tcp)
        self.wait(cmd_id, timeout=timeout)

    def mov_l_wait(self, pose_joints_or_tcp: List[float], timeout: float = DEFAULT_TIMEOUT) -> None:
        cmd_id = self.mov_l(pose_joints_or_tcp)
        self.wait(cmd_id, timeout=timeout)

    # --------------------
    # Wait / busy
    # --------------------
    def last_cmd_id(self) -> Optional[int]:
        return self._last_cmd_id

    def is_busy(self) -> bool:
        """
        Return True if the latest command (_last_cmd_id) is not yet completed.
        A command is considered completed only when:
            robot_mode == 5 AND current_command_id >= _last_cmd_id.
        If no command has been sent, returns False.
        """
        if self._last_cmd_id is None:
            return False
        st = self._store.snapshot()
        if st is None:
            return True
        return (st.robot_mode != 5) or (st.current_command_id < self._last_cmd_id)

    def wait_last(self, timeout: float = DEFAULT_TIMEOUT) -> None:
        if self._last_cmd_id is None:
            return
        self.wait(self._last_cmd_id, timeout=timeout)

    def wait(self, cmd_id: int, timeout: float = DEFAULT_TIMEOUT) -> None:
        """
        Block until the specified command is completed or timeout expires.
        Completion condition:
            robot_mode == 5 AND current_command_id >= cmd_id.
        Raises TimeoutError on timeout.
        """
        target = int(cmd_id)
        self._store.wait_until(lambda st: (st.robot_mode == 5) and (st.current_command_id >= target), timeout=float(timeout))

    # --------------------
    # State getters
    # --------------------
    def robot_mode(self) -> int:
        st = self._require_state()
        return st.robot_mode

    def current_command_id(self) -> int:
        st = self._require_state()
        return st.current_command_id

    def q_actual(self) -> List[float]:
        st = self._require_state()
        return list(st.q_actual)

    def tool_vector_actual(self) -> List[float]:
        st = self._require_state()
        return list(st.tool_vector_actual)

    # --------------------
    # Internals
    # --------------------
    def _require_db(self) -> DobotApiDashboard:
        if self._db is None:
            raise RuntimeError("DobotController is not connected. Call connect() first.")
        return self._db

    def _require_state(self) -> RobotState:
        st = self._store.snapshot()
        if st is None:
            raise RuntimeError("No feedback state received yet (port 30004).")
        return st

    @staticmethod
    def _validate_pose7(pose: List[float]) -> None:
        if not isinstance(pose, list):
            raise TypeError("pose must be list[float] (length 7)")
        if len(pose) != 7:
            raise ValueError(f"pose must have length 7, got {len(pose)}")
        mode = int(pose[6])
        if mode not in (0, 1):
            raise ValueError(f"pose[6] mode must be 0 (pose) or 1 (joint), got {pose[6]!r}")

    @staticmethod
    def _get_value(s: str) -> str:
        # Response format: "...{123}..." (same approach as original test script)
        return s.split("{", 1)[1].split("}", 1)[0]
