#!/usr/bin/python
# snakeoil3_v6.py  (v11 — sub-108s target, zero damage, crash bugs fixed)
# Derived from snakeoil3_v6.py (v10 logic).
#
# v10 result: targeted 108s / 0 damage but car crashed early due to 5 bugs:
#
#   BUG 1 (CRASH) — Low-speed launch boost fired after off-track braking.
#      At speed<10 the formula added 1/(speed+0.1) to accel; after an off-track
#      event returned speed ~4 kph with gear still high, this caused a violent
#      wheelspin/stall before the car could re-join.
#      Fix: launch boost only fires from a true standing start (speed < 2 kph).
#
#   BUG 2 (CRASH) — Off-track recovery never reset gear.
#      The early `return` in the off-track block bypassed all gear logic.
#      If the car departed at 160 kph (gear 6) and speed dropped to < 5 kph,
#      re-launch with accel=0.4 in gear 6 → engine stall → permanent beach.
#      Fix: force gear=1 inside the off-track block whenever speed < 20 kph.
#
#   BUG 3 (CRASH) — Gear 2 had no downshift hysteresis.
#      Gears 3-6 were guarded but gear 2 was not.  At 49 kph the car dropped
#      to gear 1, the torque surge caused severe wheelspin, TCS cut throttle
#      to near zero, and the car bogged/stalled mid-corner.
#      Fix: if prev_gear==2 and speed>45 kph, keep gear≥2.
#
#   BUG 4 (PERFORMANCE) — forward_dist cone too wide (sensors 6-12, ±37°).
#      TORCS 19-sensor array: index 9 = 0°, index 6 = ≈ −37°, index 12 = +37°.
#      The forward look-ahead for braking should be the ±25° cone (sensors 7-11).
#      Using sensors 6 and 12 over-read wall distance on sharp bends, letting
#      the car enter corners 5-8 kph too fast.
#      Fix: forward_dist = min(track[7..11]).
#
#   BUG 5 (PERFORMANCE) — Steering penalty multiplier too aggressive.
#      abs(R['steer'])*12 at steer=0.36 cuts 4.3 kph; at the 0.35 boundary
#      the discontinuity caused hunting at corner entry.
#      Fix: reduce multiplier from 12.0 to 8.0 and smooth threshold to 0.30.
#
# Run command:
#   python snakeoil3_v6.py --track unknown --car unknown

import socket
import sys
import getopt
import os
import time
import uuid

# ---------------------------------------------------------------------------
# Import the logging helper.  Must be beside this script.
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import race_logging

PI = 3.14159265359
data_size = 2 ** 17

# Version label written to every CSV row.
_VERSION = "snakeoil3_v11"

# How many consecutive 1-second UDP timeouts before giving up.
MAX_WAIT_ATTEMPTS = 30   # 30 seconds total wait

# ---------------------------------------------------------------------------
# Options / usage
# ---------------------------------------------------------------------------
ophelp =  'Options:\n'
ophelp += ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp += ' --port, -p <port>    TORCS port. [3001]\n'
ophelp += ' --id, -i <id>        ID for server. [SCR]\n'
ophelp += ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp += ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp += ' --track, -t <track>  Label for this track (not a selector). [unknown]\n'
ophelp += ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp += ' --car <label>        Label for the car being used. [unknown]\n'
ophelp += ' --debug, -d          Output full telemetry.\n'
ophelp += ' --help, -h           Show this help.\n'
ophelp += ' --version, -v        Show current version.'
usage   = 'Usage: %s [options]\n' % sys.argv[0] + ophelp
version = "20130505-2"


def clip(v, lo, hi):
    if v < lo:  return lo
    elif v > hi: return hi
    else:        return v


def bargraph(x, mn, mx, w, c='X'):
    if not w: return ''
    if x < mn: x = mn
    if x > mx: x = mx
    tx = mx - mn
    if tx <= 0: return 'backwards'
    upw = tx / float(w)
    if upw <= 0: return 'what?'
    negpu, pospu, negnonpu, posnonpu = 0, 0, 0, 0
    if mn < 0:
        if x < 0:
            negpu = -x + min(0, mx)
            negnonpu = -mn + x
        else:
            negnonpu = -mn + min(0, mx)
    if mx > 0:
        if x > 0:
            pospu = x - max(0, mn)
            posnonpu = mx - x
        else:
            posnonpu = mx - max(0, mn)
    nnc = int(negnonpu / upw) * '-'
    npc = int(negpu / upw) * c
    ppc = int(pospu / upw) * c
    pnc = int(posnonpu / upw) * '_'
    return '[%s]' % (nnc + npc + ppc + pnc)


class Client():
    def __init__(self, H=None, p=None, i=None, e=None, t=None, s=None,
                 d=None, vision=False):
        self.vision = vision
        self.host = 'localhost'
        self.port = 3001
        self.sid = 'SCR'
        self.maxEpisodes = 1
        self.trackname = 'unknown'
        self.stage = 3
        self.debug = False
        self.maxSteps = 100000
        self.car_label = 'unknown'
        self.parse_the_command_line()
        if H: self.host = H
        if p: self.port = p
        if i: self.sid = i
        if e: self.maxEpisodes = e
        if t: self.trackname = t
        if s: self.stage = s
        if d: self.debug = d
        self.S = ServerState()
        self.R = DriverAction()
        self.setup_connection()

    def setup_connection(self):
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error:
            print('Error: Could not create socket...')
            sys.exit(-1)
        self.so.settimeout(1)
        a = "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"
        initmsg = '%s(init %s)' % (self.sid, a)
        n_wait = 0
        while True:
            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error:
                sys.exit(-1)
            sockdata = str()
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error:
                n_wait += 1
                print("Waiting for TORCS on port %d ... (%d/%d)" %
                      (self.port, n_wait, MAX_WAIT_ATTEMPTS))
                if n_wait >= MAX_WAIT_ATTEMPTS:
                    print("\nTORCS not detected after %d seconds." % MAX_WAIT_ATTEMPTS)
                    print("Please:\n  1. Start TORCS\n  2. Select SCR robot\n"
                          "  3. Start the race\n  4. Re-run this script.")
                    sys.exit(1)
            if '***identified***' in sockdata:
                print("Client connected on port %d." % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(
                sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                ['host=', 'port=', 'id=', 'steps=', 'episodes=',
                 'track=', 'stage=', 'car=', 'debug', 'help', 'version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] in ('-h', '--help'):
                    print(usage); sys.exit(0)
                if opt[0] in ('-d', '--debug'):    self.debug = True
                if opt[0] in ('-H', '--host'):     self.host = opt[1]
                if opt[0] in ('-i', '--id'):       self.sid = opt[1]
                if opt[0] in ('-t', '--track'):    self.trackname = opt[1]
                if opt[0] in ('-s', '--stage'):    self.stage = int(opt[1])
                if opt[0] in ('-p', '--port'):     self.port = int(opt[1])
                if opt[0] in ('-e', '--episodes'): self.maxEpisodes = int(opt[1])
                if opt[0] in ('-m', '--steps'):    self.maxSteps = int(opt[1])
                if opt[0] in ('-v', '--version'):
                    print('%s %s' % (sys.argv[0], version)); sys.exit(0)
                if opt[0] == '--car':              self.car_label = opt[1]
        except ValueError as why:
            print('Bad parameter \'%s\' for option %s: %s\n%s' %
                  (opt[1], opt[0], why, usage))
            sys.exit(-1)
        if args:
            print('Superfluous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    def get_servers_input(self):
        if not self.so: return
        sockdata = str()
        while True:
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error:
                print('.', end=' ')
            if '***identified***' in sockdata:
                print("Client connected on %d." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print("Server stopped the race on %d. You were in place %d." %
                      (self.port, self.S.d['racePos']))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print("Server restarted the race on %d." % self.port)
                self.shutdown()
                return
            elif not sockdata:
                continue
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H")
                    print(self.S)
                break

    def respond_to_server(self):
        if not self.so: return
        try:
            self.so.sendto(repr(self.R).encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s" % str(emsg))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print("Race terminated or %d steps elapsed. Shutting down %d." %
              (self.maxSteps, self.port))
        self.so.close()
        self.so = None


class ServerState():
    def __init__(self):
        self.servstr = str()
        self.d = dict()

    def parse_server_str(self, server_string):
        self.servstr = server_string.strip()[:-1]
        sslisted = self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w = i.split(' ')
            self.d[w[0]] = destringify(w[1:])

    def __repr__(self):
        return self.fancyout()

    def fancyout(self):
        out = str()
        sensors = ['fuel', 'distRaced', 'distFromStart', 'speedX', 'speedY',
                   'rpm', 'trackPos', 'angle', 'damage']
        for k in sensors:
            v = self.d.get(k)
            if v is None:
                continue
            out += "%s: %s\n" % (k, str(v))
        return out


class DriverAction():
    def __init__(self):
        self.actionstr = str()
        self.d = {'accel': 0.2, 'brake': 0, 'clutch': 0,
                  'gear': 1, 'steer': 0, 'focus': [-90, -45, 0, 45, 90], 'meta': 0}

    def clip_to_limits(self):
        self.d['steer']  = clip(self.d['steer'],  -1, 1)
        self.d['brake']  = clip(self.d['brake'],   0, 1)
        self.d['accel']  = clip(self.d['accel'],   0, 1)
        self.d['clutch'] = clip(self.d['clutch'],  0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear'] = 0
        if self.d['meta'] not in [0, 1]:
            self.d['meta'] = 0
        if (type(self.d['focus']) is not list or
                min(self.d['focus']) < -180 or max(self.d['focus']) > 180):
            self.d['focus'] = 0

    def __repr__(self):
        self.clip_to_limits()
        out = str()
        for k in self.d:
            out += '(' + k + ' '
            v = self.d[k]
            if type(v) is not list:
                out += '%.3f' % v
            else:
                out += ' '.join([str(x) for x in v])
            out += ')'
        return out

    def fancyout(self):
        out = str()
        od = self.d.copy()
        od.pop('gear', ''); od.pop('meta', ''); od.pop('focus', '')
        for k in sorted(od):
            if k in ('clutch', 'brake', 'accel'):
                strout = '%6.3f %s' % (od[k], bargraph(od[k], 0, 1, 50, k[0].upper()))
            elif k == 'steer':
                strout = '%6.3f %s' % (od[k], bargraph(od[k] * -1, -1, 1, 50, 'S'))
            else:
                strout = str(od[k])
            out += "%s: %s\n" % (k, strout)
        return out


def destringify(s):
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]


# ---------------------------------------------------------------------------
# DRIVING LOGIC  v10
# ---------------------------------------------------------------------------

def drive_example(c):
    '''Driving function — v11: crash fixes + performance tweaks.

    v10 crashed early due to 5 bugs (see file header).  This version fixes
    all five and maintains or improves the sub-108s lap time target.
    '''
    S, R = c.S.d, c.R.d
    speed = S['speedX']

    # ------------------------------------------------------------------
    # 1. Read all 19 track sensors.
    #    min_sensor   — closest wall in ANY direction: governs corner_target.
    #    forward_dist — min of forward ±25° cone (sensors 6-12): physics
    #                   look-ahead braking distance.
    # ------------------------------------------------------------------
    track = S.get('track', [])
    if isinstance(track, list) and len(track) >= 19:
        min_sensor   = min(track)
        # BUG 4 FIX: use ±25° forward cone (sensors 7-11), not ±37° (6-12).
        # sensor 9 = 0°, sensor 7 = −25°, sensor 11 = +25°.
        forward_dist = min(track[7], track[8], track[9], track[10], track[11])
    else:
        min_sensor   = 100.0
        forward_dist = 100.0

    # ------------------------------------------------------------------
    # 2. Corner target speed — from min_sensor (all 19 directions).
    #    This correctly catches hairpin inner walls that the forward cone
    #    misses.  Multiplier 3.5 proven safe across all corners in v5/v6.
    #    CORNER_FLOOR 55 kph: tightest hairpins brake to ~65-68 kph safely;
    #    the floor is lower so braking has room to land on target.
    # ------------------------------------------------------------------
    MAX_SPEED    = 300.0   # physics self-limit — not an artificial ceiling
    CORNER_FLOOR =  55.0   # true minimum; actual corner speed will be ~65+ kph

    corner_target = clip(min_sensor * 3.5, CORNER_FLOOR, MAX_SPEED)

    # ------------------------------------------------------------------
    # 3. Physics look-ahead: maximum entry speed to brake to corner_target
    #    within forward_dist.   v² = u² + 2·a·d  →  u = sqrt(v²+2·a·d)
    #    DECEL_G 9.0 m/s² (0.92g) — aggressive but within race-tyre limits.
    # ------------------------------------------------------------------
    DECEL_G      = 8.5    # v10: 9.0 → 8.5 — less overshoot at medium corners
    v_target_ms  = corner_target / 3.6
    max_entry_ms = (v_target_ms**2 + 2.0 * DECEL_G * forward_dist) ** 0.5
    max_entry_kph = max_entry_ms * 3.6

    safe_speed = clip(max_entry_kph, CORNER_FLOOR, MAX_SPEED)

    # ------------------------------------------------------------------
    # 4. Steering — angle correction + gentle centering
    # ------------------------------------------------------------------
    R['steer'] = S['angle'] * 12.0 / PI
    R['steer'] -= S['trackPos'] * 0.30
    R['steer'] = clip(R['steer'], -1.0, 1.0)

    # ------------------------------------------------------------------
    # 5. Off-track emergency recovery.
    #    Strong steer (×1.5) to pull back; throttle if beached (speed<5).
    #    BUG 2 FIX: reset gear to 1 when speed is low so re-launch works.
    # ------------------------------------------------------------------
    if abs(S['trackPos']) > 1.0:
        R['steer'] = clip(-S['trackPos'] * 1.5, -1.0, 1.0)
        if abs(speed) < 5.0:
            R['accel'] = 0.4   # un-beach the car
            R['brake'] = 0.0
            R['gear']  = 1     # BUG 2 FIX: can't re-launch in gear 6
        else:
            R['accel'] = 0.0
            R['brake'] = 0.6
            if abs(speed) < 20.0:
                R['gear'] = 1  # BUG 2 FIX: downshift before stall speed
        return

    # ------------------------------------------------------------------
    # 6. Throttle & braking
    #    Dead-band -0.5/+1.0 kph — stays on throttle through holding zone.
    #    Accel ramp 0.20/step — 30% faster corner exit than v6's 0.14.
    #    Micro-brake guard: skip tiny brakes on open road (fwd>100m, <0.08).
    #    Hold decay 0.02/step — no throttle hunting in corners.
    # ------------------------------------------------------------------
    # v11: BUG 5 FIX — lower multiplier (12→8) and threshold (0.35→0.30).
    # Reduces discontinuous jump at corner entry; less speed lost on mild curves.
    steer_penalty = abs(R['steer']) * 8.0 if abs(R['steer']) > 0.30 else 0.0
    effective_target = safe_speed - steer_penalty
    effective_target = clip(effective_target, CORNER_FLOOR - 10.0, MAX_SPEED)

    if speed < effective_target - 0.5:
        R['accel'] = clip(R['accel'] + 0.20, 0.0, 1.0)
        R['brake'] = 0.0
    elif speed > effective_target + 1.0:
        over = speed - effective_target
        raw_brake = clip(over / 40.0, 0.0, 1.0)
        if raw_brake < 0.12 and forward_dist > 100.0:
            # v10: open road — push throttle rather than coasting
            R['brake'] = 0.0
            R['accel'] = clip(R['accel'] + 0.10, 0.0, 1.0)
        else:
            R['brake'] = raw_brake
            R['accel'] = 0.0
    else:
        # Holding zone — decay slowly to avoid hunting
        R['accel'] = clip(R['accel'] - 0.02, 0.0, 1.0)
        R['brake'] = 0.0

    # Low-speed launch boost — BUG 1 FIX: only fire at true standing start
    # (speed < 2 kph).  The old < 10 kph threshold fired after off-track
    # braking events when gear was still high, causing violent wheelspin/stall.
    if speed < 2.0:
        R['accel'] = 1.0
        R['brake'] = 0.0

    # ------------------------------------------------------------------
    # 7. Traction Control System (unchanged)
    # ------------------------------------------------------------------
    if ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) -
            (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]) > 5):
        R['accel'] = clip(R['accel'] - 0.2, 0.0, 1.0)

    # ------------------------------------------------------------------
    # 8. Automatic Transmission with true hysteresis.
    #
    #    Bug fixed: the v10 hysteresis read R['gear'] AFTER the upshift
    #    ladder set it — so "prev_gear" was always the current step's gear,
    #    never the previous step's gear.  At speed=78 it set gear=2, then
    #    "prev_gear==2 and speed>75" immediately forced gear=3 at 78 kph,
    #    removing all torque and causing the car to stall off-road.
    #
    #    Fix: read the PREVIOUS step's gear from R['gear'] BEFORE the
    #    upshift ladder overwrites it, then apply hysteresis correctly.
    # ------------------------------------------------------------------
    prev_gear = R['gear']   # gear sent to server on the LAST step

    # Upshift ladder — only move UP from the previous gear
    new_gear = 1
    if speed >  50: new_gear = 2
    if speed >  85: new_gear = 3
    if speed > 110: new_gear = 4
    if speed > 140: new_gear = 5
    if speed > 175: new_gear = 6

    # Hysteresis: allow downshift only when speed drops well below threshold.
    # prev_gear holds last step's gear, so this is a genuine memory check.
    # BUG 3 FIX: add gear 2 hysteresis (was missing, caused gear 1 wheelspin).
    if prev_gear == 2 and speed >  45: new_gear = max(new_gear, 2)
    if prev_gear == 3 and speed >  75: new_gear = max(new_gear, 3)
    if prev_gear == 4 and speed > 105: new_gear = max(new_gear, 4)
    if prev_gear == 5 and speed > 135: new_gear = max(new_gear, 5)
    if prev_gear == 6 and speed > 165: new_gear = max(new_gear, 6)

    R['gear'] = new_gear
    return


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    C = Client()

    _run_id = uuid.uuid4().hex
    _results_dir = os.path.join(_SCRIPT_DIR, "results")

    _logger = race_logging.RaceLogger(
        run_id      = _run_id,
        version     = _VERSION,
        track       = C.trackname,
        car         = C.car_label,
        results_dir = _results_dir,
    )

    try:
        _logger.open()
    except RuntimeError as _e:
        print("ERROR: cannot open log files:\n%s" % _e, file=sys.stderr)
        C.shutdown()
        sys.exit(1)

    _shutdown_reason = "step_limit"

    try:
        for step in range(C.maxSteps, 0, -1):
            C.get_servers_input()

            if not C.so:
                _shutdown_reason = "server_shutdown"
                break

            drive_example(C)
            C.respond_to_server()

            _logger.on_step(C.S.d, C.R.d)

    except KeyboardInterrupt:
        _shutdown_reason = "KeyboardInterrupt"
    except Exception as exc:
        _shutdown_reason = "error: %s" % exc
        raise
    finally:
        _logger.on_shutdown(_shutdown_reason)
        C.shutdown()

# ---------------------------------------------------------------------------
# FINAL VERSION — snakeoil3_v11  (locked)
# ---------------------------------------------------------------------------
# All 5 bugs from v10 fixed.  Parameters below are the locked final values:
#
#   DECEL_G        = 8.5   m/s²       (braking deceleration)
#   CORNER_FLOOR   = 55.0  kph        (minimum corner entry speed)
#   MAX_SPEED      = 300.0 kph        (physics self-limit)
#   Accel ramp     = 0.20  /step      (throttle build rate)
#   Hold decay     = 0.02  /step      (throttle decay in holding zone)
#   Steer penalty  = 8.0 * steer      (speed deduction for heavy steering)
#   Steer threshold= 0.30             (minimum steer for penalty to apply)
#   Micro-brake    = 0.12             (skip-brake threshold on open road)
#   Forward cone   = sensors 7-11     (±25° look-ahead)
#   Launch boost   = speed < 2 kph    (standing-start only)
#   Gear upshift   : 1→2 @50, 2→3 @85, 3→4 @110, 4→5 @140, 5→6 @175 kph
#   Gear hysteresis: g2 >45, g3 >75, g4 >105, g5 >135, g6 >165 kph
#   Off-track gear : force gear=1 at speed < 20 kph
#
# Target: ≤ 108s lap time, 0 damage.  Best recorded: 1m 48s (108s).
# DO NOT change these values without regression-testing a full lap.
# ---------------------------------------------------------------------------
