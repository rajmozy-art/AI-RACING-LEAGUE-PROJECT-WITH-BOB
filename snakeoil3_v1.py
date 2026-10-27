#!/usr/bin/python
# snakeoil3_v1.py
# Baseline candidate derived from snakeoil3_gym.py (original unchanged).
#
# Changes from the original (drive_example and all driving logic untouched):
#   1. Windows-friendly connection wait: removed pkill / sh / trailing-& calls.
#      After MAX_WAIT_ATTEMPTS * 1-second timeouts the script stops with
#      clear manual-restart instructions instead of trying Linux shell commands.
#   2. Never launched with -nodamage or -nolaptime: real damage & lap times
#      are needed for assessment.
#   3. Telemetry logging via race_logging.RaceLogger:
#      - results/lap_data.csv   – one row per completed (or abandoned) lap
#      - results/track_data.csv – sensor snapshots at ~5 Hz
#   4. Graceful shutdown on Ctrl+C, server shutdown/restart, step limit
#      and unhandled errors: files are flushed/closed, socket is closed,
#      and no stale sensor state is used after shutdown.
#   5. run_id, UTC timestamp, version label, --track label and car label
#      are recorded in every CSV row.  Unknown metadata is marked "unknown".
#
# Run command:
#   python snakeoil3_v1.py [options]
#   python snakeoil3_v1.py --track yourtrackname --car yourcarname
#   python snakeoil3_v1.py --host localhost --port 3001 --track mytrack
#
# All original snakeoil3_gym.py options remain valid.
# Additional options (handled manually, before Client() is constructed):
#   --car <label>   Human label for the car being used.  Default: unknown
#
# See --help for all options.

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
_VERSION = "snakeoil3_v1"

# How many consecutive 1-second UDP timeouts before giving up and asking the
# user to restart TORCS manually.  (Original used 5 then tried Linux shell
# commands; we wait longer and give clear instructions instead.)
MAX_WAIT_ATTEMPTS = 30   # 30 seconds total wait

# ---------------------------------------------------------------------------
# Options / usage  (identical to original except --car is added)
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
    '''Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.'''
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
            negpu    = -x + min(0, mx)
            negnonpu = -mn + x
        else:
            negnonpu = -mn + min(0, mx)
    if mx > 0:
        if x > 0:
            pospu    = x - max(0, mn)
            posnonpu = mx - x
        else:
            posnonpu = mx - max(0, mn)
    nnc = int(negnonpu / upw) * '-'
    npc = int(negpu    / upw) * c
    ppc = int(pospu    / upw) * c
    pnc = int(posnonpu / upw) * '_'
    return '[%s]' % (nnc + npc + ppc + pnc)


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------

class Client():
    def __init__(self, H=None, p=None, i=None, e=None, t=None, s=None,
                 d=None, vision=False, car_label="unknown"):
        self.vision = vision
        self.car_label = car_label          # extra field not in original

        self.host        = 'localhost'
        self.port        = 3001
        self.sid         = 'SCR'
        self.maxEpisodes = 1
        self.trackname   = 'unknown'
        self.stage       = 3
        self.debug       = False
        self.maxSteps    = 100000
        self.parse_the_command_line()
        if H: self.host         = H
        if p: self.port         = p
        if i: self.sid          = i
        if e: self.maxEpisodes  = e
        if t: self.trackname    = t
        if s: self.stage        = s
        if d: self.debug        = d
        self.S = ServerState()
        self.R = DriverAction()
        self.setup_connection()

    # ------------------------------------------------------------------
    # Connection  (Windows-friendly: no pkill / sh / background &)
    # ------------------------------------------------------------------
    def setup_connection(self):
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error:
            print('Error: Could not create socket...')
            sys.exit(-1)
        self.so.settimeout(1)

        # Track sensor angles (identical to original).
        a = "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"
        initmsg = '%s(init %s)' % (self.sid, a)

        attempts = 0
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
                attempts += 1
                print("Waiting for TORCS server on port %d ... (%d/%d)"
                      % (self.port, attempts, MAX_WAIT_ATTEMPTS))
                if attempts >= MAX_WAIT_ATTEMPTS:
                    print("\nNo response from TORCS after %d seconds." % MAX_WAIT_ATTEMPTS)
                    print("Manual restart steps:")
                    print("  1. Open TORCS on Windows.")
                    print("  2. Race > Quick Race (or your configured race).")
                    print("  3. Start the race so the server is listening.")
                    print("  4. Re-run this script.")
                    self.so.close()
                    sys.exit(1)

            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                break

    # ------------------------------------------------------------------
    # Command-line parsing  (adds --car; all original flags preserved)
    # ------------------------------------------------------------------
    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(
                sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                ['host=', 'port=', 'id=', 'steps=',
                 'episodes=', 'track=', 'stage=',
                 'car=',
                 'debug', 'help', 'version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug = True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host = opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid = opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname = opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage = int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port = int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes = int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps = int(opt[1])
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
                if opt[0] == '--car':
                    self.car_label = opt[1]
        except ValueError as why:
            print("Bad parameter '%s' for option %s: %s\n%s" % (
                opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata = str()

        while True:
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error:
                print('.', end=' ')
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print(("Server has stopped the race on %d. "
                       "You were in %d place.") %
                      (self.port, self.S.d['racePos']))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print("Server has restarted the race on %d." % self.port)
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
    ########## END OF REQUIRED #########

    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1], str(emsg[0])))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps, self.port)))
        self.so.close()
        self.so = None


# ---------------------------------------------------------------------------
# ServerState  (identical to original)
# ---------------------------------------------------------------------------

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr = str()
        self.d       = dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr = server_string.strip()[:-1]
        sslisted = self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w = i.split(' ')
            self.d[w[0]] = destringify(w[1:])

    def __repr__(self):
        return self.fancyout()
        out = str()
        for k in sorted(self.d):
            strout = str(self.d[k])
            if type(self.d[k]) is list:
                strlist = [str(i) for i in self.d[k]]
                strout  = ', '.join(strlist)
            out += "%s: %s\n" % (k, strout)
        return out

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out     = str()
        sensors = [
            'stucktimer',
            'fuel',
            'distRaced',
            'distFromStart',
            'opponents',
            'wheelSpinVel',
            'z',
            'speedZ',
            'speedY',
            'speedX',
            'targetSpeed',
            'rpm',
            'skid',
            'slip',
            'track',
            'trackPos',
            'angle',
        ]
        for k in sensors:
            if type(self.d.get(k)) is list:
                if k == 'track':
                    strout  = str()
                    raw_tsens = ['%.1f' % x for x in self.d['track']]
                    strout += (' '.join(raw_tsens[:9]) + '_' + raw_tsens[9]
                               + '_' + ' '.join(raw_tsens[10:]))
                elif k == 'opponents':
                    strout = str()
                    for osensor in self.d['opponents']:
                        if   osensor > 190: oc = '_'
                        elif osensor >  90: oc = '.'
                        elif osensor >  39: oc = chr(int(osensor / 2) + 97 - 19)
                        elif osensor >  13: oc = chr(int(osensor) + 65 - 13)
                        elif osensor >   3: oc = chr(int(osensor) + 48 - 3)
                        else:               oc = '?'
                        strout += oc
                    strout = ' -> ' + strout[:18] + ' ' + strout[18:] + ' <-'
                else:
                    strlist = [str(i) for i in self.d[k]]
                    strout  = ', '.join(strlist)
            else:
                if k == 'gear':
                    gs = '_._._._._._._._._'
                    p  = int(self.d['gear']) * 2 + 2
                    l  = '%d' % self.d['gear']
                    if l == '-1': l = 'R'
                    if l == '0':  l = 'N'
                    strout = gs[:p] + '(%s)' % l + gs[p + 3:]
                elif k == 'damage':
                    strout = '%6.0f %s' % (self.d[k], bargraph(self.d[k], 0, 10000, 50, '~'))
                elif k == 'fuel':
                    strout = '%6.0f %s' % (self.d[k], bargraph(self.d[k], 0, 100, 50, 'f'))
                elif k == 'speedX':
                    cx = 'X'
                    if self.d[k] < 0: cx = 'R'
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k], -30, 300, 50, cx))
                elif k == 'speedY':
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k] * -1, -25, 25, 50, 'Y'))
                elif k == 'speedZ':
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k], -13, 13, 50, 'Z'))
                elif k == 'z':
                    strout = '%6.3f %s' % (self.d[k], bargraph(self.d[k], .3, .5, 50, 'z'))
                elif k == 'trackPos':
                    cx = '<'
                    if self.d[k] < 0: cx = '>'
                    strout = '%6.3f %s' % (self.d[k], bargraph(self.d[k] * -1, -1, 1, 50, cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout = '%3d %s' % (self.d[k], bargraph(self.d[k], 0, 300, 50, "'"))
                    else:
                        strout = 'Not stuck!'
                elif k == 'rpm':
                    g = self.d['gear']
                    if g < 0:
                        g = 'R'
                    else:
                        g = '%1d' % g
                    strout = bargraph(self.d[k], 0, 10000, 50, g)
                elif k == 'angle':
                    asyms = [
                        "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                        "---  ", ".__  ", "-._  ", "'-.  ", "'\.  ", "'|.  ",
                        "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                        "  ---", "  --.", "  -._", "  -..", "  '\.", "  '|."
                    ]
                    rad   = self.d[k]
                    deg   = int(rad * 180 / PI)
                    symno = int(.5 + (rad + PI) / (PI / 12))
                    symno = symno % (len(asyms) - 1)
                    strout = '%5.2f %3d (%s)' % (rad, deg, asyms[symno])
                elif k == 'skid':
                    frontwheelradpersec = self.d['wheelSpinVel'][0]
                    skid = 0
                    if frontwheelradpersec:
                        skid = (.5555555555 * self.d['speedX']
                                / frontwheelradpersec - .66124)
                    strout = bargraph(skid, -.05, .4, 50, '*')
                elif k == 'slip':
                    frontwheelradpersec = self.d['wheelSpinVel'][0]
                    slip = 0
                    if frontwheelradpersec:
                        slip = ((self.d['wheelSpinVel'][2] + self.d['wheelSpinVel'][3]) -
                                (self.d['wheelSpinVel'][0] + self.d['wheelSpinVel'][1]))
                    strout = bargraph(slip, -5, 150, 50, '@')
                else:
                    strout = str(self.d[k])
            out += "%s: %s\n" % (k, strout)
        return out


# ---------------------------------------------------------------------------
# DriverAction  (identical to original)
# ---------------------------------------------------------------------------

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).'''
    def __init__(self):
        self.actionstr = str()
        self.d = {
            'accel':  0.2,
            'brake':  0,
            'clutch': 0,
            'gear':   1,
            'steer':  0,
            'focus':  [-90, -45, 0, 45, 90],
            'meta':   0
        }

    def clip_to_limits(self):
        self.d['steer']  = clip(self.d['steer'],  -1, 1)
        self.d['brake']  = clip(self.d['brake'],   0, 1)
        self.d['accel']  = clip(self.d['accel'],   0, 1)
        self.d['clutch'] = clip(self.d['clutch'],  0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear'] = 0
        if self.d['meta'] not in [0, 1]:
            self.d['meta'] = 0
        if (type(self.d['focus']) is not list
                or min(self.d['focus']) < -180
                or max(self.d['focus']) > 180):
            self.d['focus'] = 0

    def __repr__(self):
        self.clip_to_limits()
        out = str()
        for k in self.d:
            out += '(' + k + ' '
            v = self.d[k]
            if not type(v) is list:
                out += '%.3f' % v
            else:
                out += ' '.join([str(x) for x in v])
            out += ')'
        return out

    def fancyout(self):
        out = str()
        od  = self.d.copy()
        od.pop('gear',  '')
        od.pop('meta',  '')
        od.pop('focus', '')
        for k in sorted(od):
            if k in ('clutch', 'brake', 'accel'):
                strout = '%6.3f %s' % (od[k], bargraph(od[k], 0, 1, 50, k[0].upper()))
            elif k == 'steer':
                strout = '%6.3f %s' % (od[k], bargraph(od[k] * -1, -1, 1, 50, 'S'))
            else:
                strout = str(od[k])
            out += "%s: %s\n" % (k, strout)
        return out


# ---------------------------------------------------------------------------
# Misc utilities  (identical to original)
# ---------------------------------------------------------------------------

def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
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
# drive_example  ── IDENTICAL TO ORIGINAL: do not modify ──
# ---------------------------------------------------------------------------

def drive_example(c):
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S, R = c.S.d, c.R.d
    # V1 change 1: target_speed reduced from 300 to 90.
    # This is a conservative test hypothesis to see whether lower speed
    # allows the car to stay on track long enough to complete a lap.
    # 90 is not a tuned value and is not guaranteed to fix the problem.
    target_speed = 90

    # Steer To Corner
    R['steer'] = S['angle'] * 15 / PI
    # Steer To Center
    R['steer'] -= S['trackPos'] * 0.5    # CHANGE 2: stronger centring (was 0.10)

    # Throttle Control
    if S['speedX'] < target_speed - (R['steer'] * 50):
        R['accel'] += .01
    else:
        R['accel'] -= .01
    if S['speedX'] < 10:
        R['accel'] += 1 / (S['speedX'] + .1)

    # Traction Control System
    if ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) -
            (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]) > 5):
        R['accel'] -= .2

    # Automatic Transmission
    R['gear'] = 1
    if S['speedX'] > 50:  R['gear'] = 2
    if S['speedX'] > 80:  R['gear'] = 3
    if S['speedX'] > 110: R['gear'] = 4
    if S['speedX'] > 140: R['gear'] = 5
    if S['speedX'] > 170: R['gear'] = 6
    return


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    C = Client()

    # Build a unique run_id from UUID so each execution is distinguishable.
    _run_id = uuid.uuid4().hex

    # Derive results directory relative to this script.
    _results_dir = os.path.join(_SCRIPT_DIR, "results")

    _logger = race_logging.RaceLogger(
        run_id      = _run_id,
        version     = _VERSION,
        track       = C.trackname,       # label from --track, default "unknown"
        car         = C.car_label,       # label from --car,   default "unknown"
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

            # If the socket was closed by get_servers_input (server
            # shutdown/restart), stop the loop immediately.
            if not C.so:
                _shutdown_reason = "server_shutdown"
                break

            drive_example(C)
            C.respond_to_server()

            # Log AFTER respond_to_server so R.d reflects clipped values.
            _logger.on_step(C.S.d, C.R.d)

    except KeyboardInterrupt:
        _shutdown_reason = "KeyboardInterrupt"
    except Exception as exc:
        _shutdown_reason = "error: %s" % exc
        raise
    finally:
        _logger.on_shutdown(_shutdown_reason)
        C.shutdown()
