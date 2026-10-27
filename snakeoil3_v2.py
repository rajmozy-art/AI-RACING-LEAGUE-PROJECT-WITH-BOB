#!/usr/bin/python
# snakeoil3_v2.py  (v5 tuning — sub-120s lap, zero damage)
# Derived from snakeoil3_v1.py (logging) + snakeoil3_gym.py (baseline logic).
#
# v4 result: 130.2s lap, zero damage.  v4 bottlenecks found in track_data.csv:
#   1. corner_target * 3.0 cap capped safe_speed at 165 kph on long straights
#      (fwd=200m → should allow 190 kph, was limited to 165).
#   2. DECEL_G=5.4 m/s² too conservative — each braking approach 10-13 kph slow.
#   3. Dead-band -2/+3 caused 23% coasting time (throttle held off unnecessarily).
#   4. Accel ramp 0.08/step → corner exit still takes ~12 steps to reach full.
#
# Run command:
#   python snakeoil3_v2.py --track unknown --car unknown

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
_VERSION = "snakeoil3_v18"

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
        self.car_label = 'unknown'   # extra: set by --car
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
                   'speedZ', 'rpm', 'trackPos', 'angle', 'damage']
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
# DRIVING LOGIC  v5
# ---------------------------------------------------------------------------

def drive_example(c):
    '''Driving function — v17: snap to full throttle in hold zone, target ~112s, zero damage.

    v16 analysis vs v9 (best 109.2s) — v16 best 115.0s:
    ─────────────────────────────────────────────────────────────────
    v16 avg speed: 112.5 kph   v9: 118.3 kph  (gap: −5.8 kph)
    v16 max speed: 194.4 kph   v9: 203.5 kph

    Accel distribution comparison:
      Range      v9     v16    Verdict
      0.0        49%    37%    v16 brakes less (good)
      0.1–0.4     7%    24%  ← PROBLEM: partial throttle band
      0.5–0.8    11%    17%    also excessive
      0.9–1.0    28%    22%    v16 has less full-throttle time

    The +0.10 ramp in hold zone leaves the car stuck at 0.1→0.2→0.3→...→0.5
    for ~5 steps (1 second) each corner exit. With 55 corner exits per 2 laps
    that is ~27.5 seconds of partial throttle per lap. v9 was bimodal:
    either braking (0.0) or full throttle (1.0) — nothing in between.

    v9 throttle logic (simple): decay -0.05 in hold zone.
    When speed drops below target-1, ramp +0.10 back to 1.0 within ~10 steps.
    At 1.0, stays there. Result: bimodal distribution.

    Fix for v17 — match v9's bimodal behaviour:
    Hold zone: snap accel to max(current, 0.8) immediately.
               If accel was 0 (post-brake), jump to 0.8 in ONE step.
               If accel was 0.8+, hold it (no more micro-decay).
    This eliminates the 0.1-0.5 band entirely.
    Brake dead-band tightened +5→+3 to reduce over-speed oscillation.
    Everything else identical to v16.
    '''
    S, R = c.S.d, c.R.d
    speed = S['speedX']

    # ------------------------------------------------------------------
    # 1. Track sensors
    #    min_sensor   — closest wall in any direction (all 19 sensors).
    #    forward_dist — tightest reading in ±25° forward cone (6-12).
    # ------------------------------------------------------------------
    track = S.get('track', [])
    if isinstance(track, list) and len(track) >= 19:
        min_sensor   = min(track)
        forward_dist = min(track[6], track[7], track[8],
                           track[9], track[10], track[11], track[12])
    else:
        min_sensor   = 100.0
        forward_dist = 100.0

    # ------------------------------------------------------------------
    # 2. Corner target speed
    #    min_sensor median ≈7.5m → corner_target ≈55 kph (= CORNER_FLOOR).
    #    MAX_SPEED 210 allows >205 on long straights.
    # ------------------------------------------------------------------
    MAX_SPEED    = 210.0
    CORNER_FLOOR =  55.0

    corner_target = clip(min_sensor * 2.8, CORNER_FLOOR, MAX_SPEED)

    # ------------------------------------------------------------------
    # 3. Physics look-ahead
    #    v² = u² + 2·a·d  →  u_max = √(v_corner² + 2·DECEL_G·fwd)
    #    DECEL_G = 7.2 m/s² — proven safe in v14 (0 damage, no crashes).
    #    Tightest recorded corner (fwd=12.3m): safe_speed = 79 kph — safe.
    # ------------------------------------------------------------------
    DECEL_G = 7.2

    v_corner_ms  = corner_target / 3.6
    max_entry_ms = (v_corner_ms**2 + 2.0 * DECEL_G * forward_dist) ** 0.5
    safe_speed   = clip(max_entry_ms * 3.6, CORNER_FLOOR, MAX_SPEED)

    # ------------------------------------------------------------------
    # 4. Steering  (v9 values — proven safe)
    # ------------------------------------------------------------------
    R['steer'] = S['angle'] * 12.0 / PI
    R['steer'] -= S['trackPos'] * 0.30
    R['steer'] = clip(R['steer'], -1.0, 1.0)

    # ------------------------------------------------------------------
    # 5. Off-track emergency recovery
    # ------------------------------------------------------------------
    if abs(S['trackPos']) > 1.0:
        R['steer'] = clip(-S['trackPos'] * 1.0, -1.0, 1.0)
        R['accel'] = 0.0
        R['brake'] = 0.8
        return

    # ------------------------------------------------------------------
    # 6. Throttle & braking
    #
    #    effective_target = safe_speed − |steer|×20  (corner speed scrub)
    #
    #    Three zones:
    #    ACCELERATE : speed < target − 1        → accel += 0.20,  brake = 0
    #    HOLD       : target−1 ≤ speed ≤ target+5
    #                   if accel < 0.5  → accel += 0.10  (recover from brake)
    #                   if accel > 0.8 and speed > target+2 → accel -= 0.02
    #                   else            → hold accel
    #                 brake = 0
    #    BRAKE      : speed > target + 5        → proportional brake, accel = 0
    #
    #    v9 reference: hold zone had accel=1.0 (76%) or 0.8 (19%).
    #    The ramp-up in hold (+=0.10) recovers from post-braking zero and
    #    reaches 0.5 within 5 steps (1s), matching v9's hold behaviour.
    # ------------------------------------------------------------------
    effective_target = safe_speed - abs(R['steer']) * 20.0
    effective_target = clip(effective_target, CORNER_FLOOR - 10.0, MAX_SPEED)

    if speed < effective_target - 1.0:
        # Under target — ramp up. 0.15 closes the straight gap 50% faster than v9's 0.10
        R['accel'] = clip(R['accel'] + 0.15, 0.0, 1.0)
        R['brake'] = 0.0
    elif speed > effective_target + 2.0:
        # Over target — brake proportionally (v9's proven +2 dead-band)
        over = speed - effective_target
        R['brake'] = clip(over / 40.0, 0.0, 1.0)
        R['accel'] = 0.0
    else:
        # Hold zone — v9's proven -0.05 decay; car re-enters accelerate
        # zone within 1-2 steps keeping throttle near 1.0 (v9 bimodal pattern)
        R['accel'] = clip(R['accel'] - 0.05, 0.0, 1.0)
        R['brake'] = 0.0

    # Low-speed launch boost
    if speed < 10:
        R['accel'] = clip(R['accel'] + 1.0 / (speed + 0.1), 0.0, 1.0)
        R['brake'] = 0.0

    # ------------------------------------------------------------------
    # 7. Traction Control System  (unchanged)
    # ------------------------------------------------------------------
    if ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) -
            (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]) > 5):
        R['accel'] = clip(R['accel'] - 0.2, 0.0, 1.0)

    # ------------------------------------------------------------------
    # 8. Automatic Transmission  (unchanged)
    # ------------------------------------------------------------------
    R['gear'] = 1
    if speed > 50:  R['gear'] = 2
    if speed > 80:  R['gear'] = 3
    if speed > 110: R['gear'] = 4
    if speed > 140: R['gear'] = 5
    if speed > 170: R['gear'] = 6
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
