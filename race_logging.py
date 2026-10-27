"""
race_logging.py  –  Reusable telemetry helper for TORCS snakeoil clients.
==========================================================================

Public API
----------
    logger = RaceLogger(run_id, version, track, car, results_dir)
    logger.open()                      # ensure CSVs exist, open for append
    logger.on_step(S_d, R_d)          # call once per step, after respond_to_server
    logger.on_shutdown(reason)         # flush, close, write incomplete-lap row
    logger.lap_count                   # number of completed laps this run

Lap-detection algorithm (summary)
----------------------------------
TORCS resets curLapTime to a small value (≈0) when the car crosses the
start/finish line.  We detect a lap completion when ALL of the following
hold:

  1. curLapTime resets  (prev_cur > RESET_THRESHOLD and cur_cur < RESET_THRESHOLD)
  2. lastLapTime > 0    (server actually recorded a lap time)
  3. distRaced advanced by at least MIN_LAP_DIST metres since the last
     confirmed crossing   (guards against spawn-at-start / reconnect)
  4. forward progress   (distRaced is monotonically increasing overall;
     a large drop flags a restart/reconnect and resets state)
  5. The new lastLapTime differs from the PREVIOUS lastLapTime only if we
     have not already recorded it – equal consecutive values are accepted
     (handles the case where the server doesn't update lastLapTime mid-race).

Edge cases that are NOT counted as laps:
  - curLapTime never exceeded RESET_THRESHOLD (car spawned near S/F line)
  - distRaced dropped significantly (restart / reconnect detected)
  - The reset was ambiguous (curLapTime briefly touched 0 then rose again
    without lastLapTime advancing)

Track-data sampling
-------------------
Samples at ≈5 Hz by requiring curLapTime to advance by at least
SAMPLE_INTERVAL_S seconds since the last sample.  Uses simulation time,
not wall-clock time, so paused / slow simulations don't double-sample.

CSV files
---------
  results/lap_data.csv    – one row per completed (or abandoned) lap
  results/track_data.csv  – sensor snapshot rows at ~5 Hz

If a file already exists with data, rows are appended.
If a file exists with an incompatible header, a RuntimeError is raised
and nothing is written (no silent data loss).
If a write fails, IOError propagates immediately.

Assumptions / limitations
--------------------------
- lastLapTime is the authoritative lap time (not wall-clock).
- track[] sensor list always has exactly 19 elements; missing elements
  are written as empty strings.
- "damage_delta" is damage this lap minus damage at the start of this lap.
- speedX is in km/h as reported by TORCS (can be negative while reversing).
- version, track, car are labels supplied by the caller – they are not
  validated against the server.
"""

import csv
import os
import sys

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

RESET_THRESHOLD   = 5.0    # curLapTime (s) above which a reset is "real"
MIN_LAP_DIST      = 200.0  # metres distRaced must advance for a valid lap
RESTART_DROP      = 500.0  # metres drop in distRaced → treat as restart
SAMPLE_INTERVAL_S = 0.2    # seconds of sim-time between track_data rows (~5 Hz)

LAP_HEADER = [
    "run_id", "timestamp_utc", "version", "track", "car",
    "lap_number", "lap_time_seconds", "damage_total", "damage_delta",
    "status", "change_description",
]

TRACK_HEADER = [
    "run_id", "timestamp_utc", "version", "track", "car", "lap_number",
    "curLapTime", "lastLapTime", "distFromStart", "distRaced",
    "speedX", "speedY", "angle", "trackPos", "damage",
    "track_0",  "track_1",  "track_2",  "track_3",  "track_4",
    "track_5",  "track_6",  "track_7",  "track_8",  "track_9",
    "track_10", "track_11", "track_12", "track_13", "track_14",
    "track_15", "track_16", "track_17", "track_18",
    "steer", "accel", "brake", "gear",
]

TRACK_SENSOR_ANGLES = "-45,-19,-12,-7,-4,-2.5,-1.7,-1,-0.5,0,0.5,1,1.7,2.5,4,7,12,19,45"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _utcnow():
    """Return current UTC time as ISO-8601 string (no external deps)."""
    import datetime
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def _ensure_csv(path, expected_header):
    """
    Ensure the CSV at *path* exists and has *expected_header*.

    - Missing or empty file → write header, return.
    - Header matches        → leave file untouched.
    - Header mismatch       → raise RuntimeError (never overwrite data).
    """
    if not os.path.exists(path):
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(expected_header)
        return

    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            existing = next(reader)
        except StopIteration:
            existing = []

    if existing == []:
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(expected_header)
        return

    if existing != expected_header:
        raise RuntimeError(
            f"Incompatible header in {path}.\n"
            f"  Expected : {expected_header}\n"
            f"  Found    : {existing}\n"
            f"  Action   : rename or delete the file, then re-run."
        )
    # Header matches – nothing to do.


def _safe_float(v, default=0.0):
    """Return float(v) or default if v is None / non-numeric."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _track_sensors(S_d):
    """Return the 19 track sensor values as strings, padding with '' if short."""
    raw = S_d.get("track", [])
    if not isinstance(raw, list):
        raw = [raw]
    out = []
    for i in range(19):
        out.append("" if i >= len(raw) else str(raw[i]))
    return out


# ---------------------------------------------------------------------------
# RaceLogger
# ---------------------------------------------------------------------------

class RaceLogger:
    """
    Attach one instance to a snakeoil Client and call on_step / on_shutdown.

    Parameters
    ----------
    run_id      : str  – unique identifier for this execution (UUID or timestamp)
    version     : str  – label for the script version, e.g. "snakeoil3_v1"
    track       : str  – human label passed via --track (not a selector)
    car         : str  – human label for the car (pass "unknown" if not known)
    results_dir : str  – absolute path to the results directory
    """

    def __init__(self, run_id, version, track, car, results_dir):
        self.run_id      = run_id
        self.version     = version
        self.track       = track
        self.car         = car
        self.results_dir = results_dir

        self._lap_path   = os.path.join(results_dir, "lap_data.csv")
        self._track_path = os.path.join(results_dir, "track_data.csv")

        self._lap_fh     = None   # file handle
        self._track_fh   = None
        self._lap_w      = None   # csv.writer
        self._track_w    = None

        self._opened     = False

        # --- lap-detection state ---
        self._lap_index           = 0      # number of completed laps
        self._prev_cur_lap_time   = 0.0    # curLapTime from previous step
        self._prev_last_lap_time  = 0.0    # lastLapTime from previous step
        self._dist_at_lap_start   = 0.0    # distRaced when current lap began
        self._prev_dist_raced     = 0.0    # distRaced from previous step
        self._damage_at_lap_start = 0.0    # damage at the start of current lap
        self._cur_exceeded_thresh = False  # has curLapTime risen above threshold?
        self._last_recorded_llt   = None   # lastLapTime of the most-recent row written

        # --- track-data sampling state ---
        self._last_sample_sim_time = -999.0   # curLapTime at last track row

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self):
        """
        Create results dir and CSVs if needed, then open files for append.
        Raises RuntimeError on header mismatch.  Call once before the loop.
        """
        os.makedirs(self.results_dir, exist_ok=True)
        _ensure_csv(self._lap_path,   LAP_HEADER)
        _ensure_csv(self._track_path, TRACK_HEADER)

        self._lap_fh  = open(self._lap_path,   "a", newline="", encoding="utf-8")
        self._track_fh= open(self._track_path, "a", newline="", encoding="utf-8")
        self._lap_w   = csv.writer(self._lap_fh)
        self._track_w = csv.writer(self._track_fh)
        self._opened  = True

        print(f"[logger] run_id={self.run_id}  version={self.version}  "
              f"track={self.track}  car={self.car}")
        print(f"[logger] lap_data   -> {self._lap_path}")
        print(f"[logger] track_data -> {self._track_path}")
        print(f"[logger] track sensor angles: {TRACK_SENSOR_ANGLES}")

    def close(self):
        """Flush and close open file handles."""
        for fh in (self._lap_fh, self._track_fh):
            if fh and not fh.closed:
                fh.flush()
                fh.close()
        self._opened = False

    # ------------------------------------------------------------------
    # Per-step callback  (call AFTER respond_to_server each step)
    # ------------------------------------------------------------------

    def on_step(self, S_d, R_d):
        """
        Process one simulation step.

        S_d : ServerState.d  – sensor dictionary from the server
        R_d : DriverAction.d – action dictionary as actually sent
                               (clip_to_limits has already run inside __repr__)
        """
        if not self._opened:
            return

        cur  = _safe_float(S_d.get("curLapTime"),   0.0)
        llt  = _safe_float(S_d.get("lastLapTime"),  0.0)
        dist = _safe_float(S_d.get("distRaced"),     0.0)
        dmg  = _safe_float(S_d.get("damage"),        0.0)

        # ── restart / reconnect detection ──────────────────────────────
        if self._prev_dist_raced - dist > RESTART_DROP:
            # distRaced dropped substantially → server restarted the race.
            # Reset all lap-detection state; do NOT write a lap row.
            self._reset_lap_state(dist, dmg)
            self._prev_dist_raced = dist
            self._prev_cur_lap_time = cur
            self._prev_last_lap_time = llt
            return

        # ── lap-crossing detection ──────────────────────────────────────
        if cur > RESET_THRESHOLD:
            self._cur_exceeded_thresh = True

        reset_detected = (
            self._cur_exceeded_thresh
            and self._prev_cur_lap_time > RESET_THRESHOLD
            and cur < RESET_THRESHOLD
        )

        if reset_detected:
            dist_travelled = dist - self._dist_at_lap_start
            valid_distance = dist_travelled >= MIN_LAP_DIST

            if llt > 0.0 and valid_distance:
                # ── Ambiguity check: did lastLapTime actually change? ───
                # If we've already recorded a row with this exact llt
                # AND the distance travelled is suspiciously small,
                # mark as ambiguous.  Equal llt on a genuinely new lap
                # (full lap completed) is accepted.
                if (self._last_recorded_llt is not None
                        and llt == self._last_recorded_llt
                        and dist_travelled < MIN_LAP_DIST * 2):
                    status = "ambiguous"
                    self._write_lap_row(
                        lap_number=self._lap_index + 1,
                        lap_time_seconds=llt,
                        damage_total=dmg,
                        damage_delta=dmg - self._damage_at_lap_start,
                        status=status,
                        change_description="llt unchanged; crossing may be spurious",
                    )
                else:
                    self._lap_index += 1
                    self._write_lap_row(
                        lap_number=self._lap_index,
                        lap_time_seconds=llt,
                        damage_total=dmg,
                        damage_delta=dmg - self._damage_at_lap_start,
                        status="completed",
                        change_description="drive_example unchanged (baseline)",
                    )
                    self._last_recorded_llt = llt

            elif not valid_distance and llt > 0.0:
                # curLapTime reset but car barely moved – spawn or reconnect.
                pass  # silently ignore; no lap row written

            elif llt <= 0.0 and valid_distance:
                # curLapTime reset, car moved, but server gave no lap time.
                self._write_lap_row(
                    lap_number=self._lap_index + 1,
                    lap_time_seconds="",
                    damage_total=dmg,
                    damage_delta=dmg - self._damage_at_lap_start,
                    status="ambiguous",
                    change_description="curLapTime reset but lastLapTime=0",
                )

            # Reset per-lap state regardless of outcome.
            self._reset_lap_state(dist, dmg)

        # ── track-data sampling at ~5 Hz ───────────────────────────────
        # Use simulation curLapTime as the clock.  Reset on negative delta.
        sim_elapsed = cur - self._last_sample_sim_time
        if sim_elapsed < 0:
            # curLapTime just rolled over → sample immediately.
            sim_elapsed = SAMPLE_INTERVAL_S + 1

        if sim_elapsed >= SAMPLE_INTERVAL_S:
            self._write_track_row(S_d, R_d, cur, llt, dist, dmg)
            self._last_sample_sim_time = cur

        # ── advance state ───────────────────────────────────────────────
        self._prev_cur_lap_time  = cur
        self._prev_last_lap_time = llt
        self._prev_dist_raced    = dist

    # ------------------------------------------------------------------
    # Shutdown callback
    # ------------------------------------------------------------------

    def on_shutdown(self, reason="unknown"):
        """
        Call on any exit path (Ctrl+C, server shutdown, step limit, error).
        Writes an incomplete-lap row if the car had made meaningful progress
        in the current lap, then closes all files.
        """
        if not self._opened:
            return

        # Only record an incomplete row if meaningful progress was made.
        # (Avoids polluting lap_data with zero-second entries at startup.)
        dist_this_lap = self._prev_dist_raced - self._dist_at_lap_start
        if dist_this_lap >= MIN_LAP_DIST:
            self._write_lap_row(
                lap_number=self._lap_index + 1,
                lap_time_seconds="",      # blank – not a completed lap
                damage_total=None,
                damage_delta=None,
                status="incomplete",
                change_description=f"shutdown: {reason}",
            )

        self.close()
        print(f"[logger] closed. completed laps this run: {self._lap_index}")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def lap_count(self):
        """Number of completed (status='completed') laps this run."""
        return self._lap_index

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _reset_lap_state(self, dist_raced, damage):
        self._dist_at_lap_start   = dist_raced
        self._damage_at_lap_start = damage
        self._cur_exceeded_thresh = False
        self._last_sample_sim_time = -999.0

    def _write_lap_row(self, lap_number, lap_time_seconds,
                       damage_total, damage_delta, status, change_description):
        row = [
            self.run_id,
            _utcnow(),
            self.version,
            self.track,
            self.car,
            lap_number,
            lap_time_seconds if lap_time_seconds != "" else "",
            "" if damage_total is None else damage_total,
            "" if damage_delta is None else f"{damage_delta:.1f}",
            status,
            change_description,
        ]
        self._lap_w.writerow(row)
        self._lap_fh.flush()

    def _write_track_row(self, S_d, R_d, cur, llt, dist, dmg):
        tsens = _track_sensors(S_d)
        row = [
            self.run_id,
            _utcnow(),
            self.version,
            self.track,
            self.car,
            self._lap_index,   # current lap index (0-based during a lap)
            f"{cur:.3f}",
            f"{llt:.3f}",
            f"{_safe_float(S_d.get('distFromStart')):.1f}",
            f"{dist:.1f}",
            f"{_safe_float(S_d.get('speedX')):.2f}",
            f"{_safe_float(S_d.get('speedY')):.2f}",
            f"{_safe_float(S_d.get('angle')):.5f}",
            f"{_safe_float(S_d.get('trackPos')):.4f}",
            f"{dmg:.1f}",
        ] + tsens + [
            f"{_safe_float(R_d.get('steer')):.4f}",
            f"{_safe_float(R_d.get('accel')):.4f}",
            f"{_safe_float(R_d.get('brake')):.4f}",
            int(_safe_float(R_d.get('gear'))),
        ]
        self._track_w.writerow(row)
        # Track data is flushed lazily (every ~5 Hz row) to avoid overhead.
        # The close() call does a final flush.
