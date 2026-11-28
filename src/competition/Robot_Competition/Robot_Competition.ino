/*
  LINE ROBOT - CONTEST FIRMWARE (official contest track)                   version C3 (from C2)
  --------------------------------------------------------------------------------
  VERSION C3.31 (28/11): CIRCLE CLUSTER rebuilt from the REAL sensor data (C3.30 2 mm capture; sensor spacing ~12 mm = D1-D8 edges 88 mm,
                    line 16 mm at home / 20 mm at the contest). The simulator now uses that sensor model and reproduces the real failure
                    (C3.30 logic: 18/96 circle runs ok, robot loops like the real log).
                    ENTRY: a SEPARATE trace appears at D8 / D1 (white gap to the line), slides in and merges with the line within 90 mm, and
                    stays beside the line for 16 mm (a T branch crossed at an angle is gone after <= 10 mm), while driving straight.
                    ARC FOLLOWING: line candidates come from the raw channels (a 1-channel line is real at 12 mm spacing; readLine() dropped it
                    as noise and the robot jumped to a crossing arc); world line rebuilt with 12 mm per channel (CX_PITCH).
                    Simulation, real sensor model, line 16 and 20 mm, R 96-130 mm, touching / overlapping / mirrored, 6 variants: 94/96 on the
                    S-path (2 runs dip 23 mm at the last touching point but stay on it); 1/816 false entries (that run still finishes);
                    square / hexagon / pulse / circle-exit / avoid benches unchanged. CAPTURE 2mm log kept.
  VERSION C3.30 (28/11): DIAGNOSTIC for the circle cluster (real log 28/11: C3.29 never entered arc mode - the real line covers ~1 channel,
                    the joining arcs only widened the trace and never made a 'junction'). Adds "CAPTURE 2mm" to the log: 8 analog channels + heading
                    every 2 mm from 20 cm before the first wide trace to 80 cm after. Driving logic = C3.29.
  VERSION C3.29 (28/11): CIRCLE CLUSTER = FOLLOW THE ARCS (user rule: stay on the line, no driving straight across).
                    ENTRY: driving straight, one-sided split, and the joining arc SLID in from the outer channel >= 10 mm (CX_SLIDE; corners / T /
                    square cluster 0-6 mm) - also works on a fast approach where the 2 traces blur into one.
                    The robot follows the S-path along the arcs: top of a bottom circle -> bottom of a top circle -> ... -> exit line. Followed line
                    is rebuilt in world coordinates (odometry + bar position); its heading at the bar decides the side: right of hRef -> keep the LEFT
                    edge, left of hRef -> keep the RIGHT edge (hysteresis 8 deg, no flip back within 30 mm). The followed line is tracked by
                    continuity; only where it FORKS (touching point) the chosen side is taken. Exit: straight line on hRef for 50 mm. Speed CX_V 34 %.
                    Simulation (R 96-130 mm, touching / overlapping, mirrored, slow and fast approach, 10 noise variants): 140/140, sensor bar always
                    within 12 mm of the line (C3.26: 16-34 mm off, C3.28: up to 52 mm); 0/408 false entries on square / hexagon / pulse / zigzag /
                    curve tracks; other benches unchanged.
  VERSION C3.28 (28/11): circle cluster from the 28/11 C3.27 log (confirmation came 160 mm late, after the corner logic had swung the robot
                    +-20 deg; the straight run then left the cluster and drove 300 mm over white): enter straight-through mode AT the first slow one-sided
                    split; reference = approach heading hRef and the line point under the SENSOR BAR; exit on one plain line anywhere on the bar after 5.25 R (R measured
                    from the spacing of the touching points, 110 mm until measured); 120 mm of white = off the cluster -> stop the straight run and search toward the line.
  VERSION C3.27 (28/11): CIRCLE CLUSTER STRAIGHT THROUGH (log 28/11: 160 s looping; circles R 10-11 cm ~ the 115 mm sensor offset -> the robot
                    spun on each circle). Entry = while driving straight a junction splits slowly to ONE side, confirmed by the next junction 40-200 mm
                    later (traces on both sides / full bar). Then: hold the entry heading, keep the lateral offset from the entry line ~0 by odometry,
                    ignore the arcs, and take the line again when a single clean centered line is seen for 40 mm after 380 mm (exit line); none by
                    1100 mm -> normal following. Log events 'E' / 'e'. Simulation (R 100/105/110 mm, 7 variants each): 21/21 straight through,
                    ~1.8 s instead of 2.1-2.4 s; no false trigger on squares, hexagons, pulses, zigzags.
  VERSION C3.26 (28/11): CIRCLE CLUSTER: square-grid mode ('Q': stop at the junction center and pivot) was latched by mistake at the touching
                    points of the circles (black covers the bar there too) -> the robot pivoted inside the circles and looped (simulation: every slow /
                    failed circle run). Now grid mode is only confirmed on a PURE T (no line straight on after it) and is cancelled as soon as the
                    robot rolls round an arc >= 70 deg (square edges are straight). Simulation: circles 51 -> 52/54 and less time, squares and
                    hexagons unchanged. USER_CFG = your parameters of 28/11.
  VERSION C3.25 (28/11): ARC line catch: as soon as the line is under the bar (2 reads instead of 3) the robot FOLLOWS it with PD right away
                    and, when the line slips off the bar while crossing at an angle, pivots toward the side the line continues on. The merge arc was
                    dropped for ARC (28/11 log: it ran ~2x wider than planned and lost the line for 1.3 s).
  VERSION C3.24 (28/11): from the 28/11 C3.22 log (2nd brick: 1300 mm/s at 294 mm -> slid into it, pushed 4 s; line catch took 1.2 s):
                    (1) ENTRY: roll into the curve only if v <= sqrt(2 x 2500 x (VL - 110)); faster -> hard brake (brev 60 / bms 80) + back up if
                    too close (v3.8), then the same curve from standstill; (2) the VL53 slow zone (obs + 150 mm) slows to that safe speed instead of a
                    fixed 35 %; (3) OUT: nose < 70 mm or pushing without moving -> back up 60 mm, turn 20 deg further out (max 3 times);
                    stuck while parallel -> lane blocked -> other side; (4) LINE CATCH without searching: merge / grab run through the wheel PID
                    (raw 25-45 % PWM sits on the new gearbox's dead band -> crawled and turned too tight), grab exits after 80 ms; retm 0.
  VERSION C3.23 (28/11): ARC avoid with a BLOCKED LANE (track edge / wall / object beside the line, 28/11 log: pushed into it for 1.6 s):
                    the VL53 is now watched while driving out and parallel; an object < 200 mm ahead that is not the brick -> stop, back up behind
                    the start point and avoid on the OTHER side. Both sides blocked -> stop (avoid failed). On the way back: stop instead of pushing.
  VERSION C3.22 (28/11): ARC avoid follows 'retm' on the way back: retm 1 = brake on the line, center, turn (backs up if it overshot) like avmode 2;
                    retm 0 = merge without stopping. Real track: the merge overshot and lost the line.
  VERSION C3.21 (28/11): ARC avoid from the 28/11 log (brick seen at 240 mm at 1250 mm/s): (1) short reverse brake down to the out speed before
                    curving (the out curve overshot to 63 deg and used 300 mm); (2) the last 70 mm before the line are driven at 35 deg instead of 65,
                    so the slow photoresistors stay over the line longer; (3) give up only 0.5 x lat past the line (odometry drifts after a fast curve);
                    (4) the fallback sweep pivots at 46 % (34 % did not turn the robot: stuck 1.4 s).
  VERSION C3.20 (28/11): ARC avoid did not catch the line on the way back: it slowed down by the WHEEL AXLE position, but at 65 deg the sensor
                    bar reaches the line ~100 mm earlier -> it crossed at full return speed and the photoresistors missed it. Now it slows by the SENSOR BAR
                    position (70 mm before the line) to the old-gearbox catch speed (~440 mm/s); if it still passes the line it turns back toward it.
  VERSION C3.19 (28/11): ARC (parabola-like) obstacle avoid = avmode 3. Same plan as avmode 2 (out -> parallel -> back), but the robot does NOT
                    stop when it sees the brick, does NOT pivot to the out angle and does NOT stop at the line: it keeps rolling and steers one smooth
                    curve with odometry + gyro (out at 'aout', curve onto the offset 'lat', hold it past the brick, back at 'aback', merge into the line).
                    Brick orientation unknown -> lateral offset 'lat' 215, pass the long side (205) + 'pass'. Brick closer than 0.55 x obs -> old stop + pivot.
  VERSION C3.18 (25/11): OLED speed buttons stop at the limit (sp 80, vmax 100); before, 73 + 3 x 10 gave vmax 103.
  VERSION C3.17 (25/11): tuning WITHOUT the USB cable: the web page (WiFi LINE-ROBOT, 192.168.4.1) gets a COMMAND box that sets any parameter
                    ("vmax 80;alat 4500", name-value pairs only) and shows the full parameter line (same as 'export') to copy from the phone.
                    'wcal' now measures at 40 % and 80 % (the 25/11 run at 50/80 gave dband 33.4 / vfull 1645: the curve bends below 60 %, so the
                    feed-forward gave ~30 % too much speed at cluster / inner-wheel speeds). Default dband 25: fits the floor test from 40 to 100 %.
  VERSION C3.16 (25/11): from the Test_WheelSpeed floor run (40/60/80/100 % -> 330/695/1150/1583 mm/s, no motion below ~28 % PWM):
                    (1) 'wcal' measures at 50 % AND 80 % and fits a line -> real vfull (the old 50 % x 2 read ~35 % low: 1027-1131 instead of ~1580)
                    plus the start-up dead band 'dband' (new parameter, %); (2) wheel speed PID feed-forward = dband + pct x (100 - dband) / 100, so the PID
                    no longer has to pull the dead band itself (faster response in and out of bends); (3) TURN_MIN 26 -> 32 (was below the dead band);
                    (4) USER_CFG: vfull 1583, dband 28.5 and the C3.11b percentages x 0.731 (= 1158 / 1583) -> same REAL speeds and steering as C3.11b
                    (vclust 22). Simulation with the dead band (vs C3.15 at vfull 1131): same success rate, ~20-30 % less time.
  VERSION C3.15 (25/11): from the 25/11 log: (1) hexagon Y detection only when the split is symmetric around the centered line, and extending hexagon
                    mode no longer blocks square grid mode ('H' fired inside the square cluster -> grid mode off -> random loops); (2) pivots for the
                    lower-torque gearbox: TURN_MIN 20 -> 26, stall kick after 80 ms (+20%/100 ms); (3) for 600 mm after going round a circle, 'running backward'
                    needs 120 mm of evidence instead of 60 (2 wrong 'D' turns near the finish); (4) log distance column no longer stops at 32000 mm.
  VERSION C3.14 (25/11): longer 'wcal' for the new motors: 50% for 1.6 s, speed printed every 100 ms, vfull from the last 0.6 s only
                    (old 0.8 s / 60% run gave vfull 1027-1067, below the old motors - likely still accelerating). Needs 1.5 m clear ahead.
  VERSION C3.13 (21/11): = C3.11b + USER_CFG for the NEW MOTORS (gearbox ~20.7:1, both wheels cpr 576.3, ~1.4x faster):
                    cpr 576.3, vfull 1640 (estimate - run 'wcal'), speed percentages x 0.71 (cspd 38, vclust 25 (simulation: 28 -> square cluster worse), vout/vpar 49, vret 42, vturn 49),
                    kp 10.2, kd 0.39, sp 80. After flashing: 'reset' (loads USER_CFG) -> 'cal' -> 'wcal' -> 'lag'.
  VERSION C3.11 (11/11): HEXAGON (honeycomb) CLUSTER COMPENSATED LIKE THE SQUARE CLUSTER: Y-junction (2 branches splitting fast, gap >= 4 channels within 30 mm) -> hexagon
                    mode (log 'H'): LOCK the main heading = heading when entering the cluster (no drifting along the slanted edges), at every junction pick the branch that brings the robot back to the main
                    heading, DISABLE the C3.9 circle-exit rule (60-degree corners made the robot think it was curving -> wrong branch -> circled one cell).
                    Curving continuously > 10 cm inside the cluster -> temporarily re-enable the circle rule. USER_CFG: track 128, aout 58, out 76, par 500.
  VERSION C3.11b: entering the hexagons RIGHT AFTER A BEND: main heading = robot heading at the first Y-junction (not the old hRef from before the bend), and hexagon
                    mode still turns on when the robot has just come out of a bend (HEX_GZ 70 -> 400).
  VERSION C3.10 (10/11): START INSIDE THE BLACK BOX: drive STRAIGHT holding the gyro heading until the sensor bar leaves the box + START_GO (60 mm), only then follow the line
                    (old: followed the line right inside the black box -> robot kept wobbling). Log 'P'.
  VERSION C3.9 (10/11): 1. CIRCLE CLUSTER EXIT: while curving (|gzSlow| > CIRC_GZ 40 deg/s, outside square clusters), at junctions take the branch on the
                       OUTER side of the arc (right bend -> left branch). The old version compared with the cluster entry heading: after turning > 180 deg (3/4 turn) wrap180 flipped the sign
                       -> picked the INNER side -> circled forever. Circle turn-commit threshold 90 -> 45 deg/s (CIRC_COMMIT).
                    2. ANTI-STALL WHEN PIVOTING: both wheels still > 150 ms while a turn is commanded -> ramp PWM up to +30% (log: 'D' stuck 7 s).
                    3. SPINNING - STOPPED only counts rotation when TURNING NEARLY IN PLACE (> 1 deg/mm): rolling around small circles no longer triggers a false stop.
  VERSION C3.8 (10/11): OBSTACLE AVOID - SEARCH FOR THE LINE IN THE RIGHT DIRECTION: when the line catch fails after avoiding, the robot sweeps toward WHERE THE LINE CONTINUES
                    (line heading at the corner, on a curve offset another 45 deg toward the curve) instead of by avoid side. Old: avoid left -> sweep RIGHT first
                    ~100 deg, caught the line in reverse then turned 180 (lost 2-3 s on left bends). Simulation: the re-search step is 0.4-0.9 s faster.
  VERSION C3.7 (08/11): OBSTACLE ON A BEND (AVOID_ON_CURVE = true): when the obstacle is seen while the robot is curving (heading change > 15 deg in the last 20 cm or
                    turning > 40 deg/s) -> avoid TOWARD THE BEND (left bend avoid left, right bend avoid right). While driving parallel / returning, if the line appears on
                    half of the sensor bar on the avoid side (left channels when avoiding left) -> CATCH THE LINE NOW with PD (log 'Y'), no stop-center-turn back to the
                    old heading (on a bend the old heading is already wrong -> spinning). Obstacle on a straight: avoid by 'side' as before.
  VERSION C3.6 (08/11): from C3.5 + your FIX-001 (sharp zigzag: release the corner lock once the line is centered for 20 mm, reset the filter after a line search,
                    when the line is re-caught slow down to cspd). Added 2 fixes for the 3-SQUARE CLUSTER (simulation: 141 -> 146/150 passes, spins 7 -> 3):
                    - Remembers a 3 m path (was 1 m): when the robot circled a cluster for more than 1 m it forgot the entry line -> drove back out the entry.
                    - While turning at a cluster corner, if the sensor bar sweeps over a line with the nose close to the main heading (< 50 deg) = exit branch -> accept
                      right away (before: when the square corner and the exit branch were < 11.5 cm apart the sensor bar had already passed the exit -> spinning).
  VERSION C3.5 (07/11): 3-SQUARE CLUSTER: - 'vclust' (default 40%): separate speed inside the cluster (cspd 54 before) -> less overshoot from inertia.
                                   - 'pclust': side chosen at a perpendicular T-junction inside a cluster (0 left, 1 right, 2 = by pref). Set per track:
                                     whichever side the entry line is offset to (relative to the center of the first square), pick that side (in the driving direction).
                                     pref is still kept for circles / hexagons.
                    Plays HAPPY BIRTHDAY at the finish like version B (press any button to stop; disable: FIN_SONG = false).
  VERSION C3.4 (07/11): REMOVED parabolic avoidance (avmode 3) - keeps FLOWING avoidance avmode 2 (out 260, par 350, aout/aback 65 as before).
                    Square cluster 'grid' (log 'Q'): perpendicular T on the main heading -> slow down, at every junction take the branch closest to the heading
                    most, side branch -> bring the wheel axle to the junction center then turn into it. ONLY enabled in SQUARE clusters: Y-junctions (hexagon / honeycomb)
                    not enabled -> hexagons run smoothly like C3 (C3.3 false-triggered in hexagons -> stalling, jerking). cluster 0 = off.
  VERSION C3.1 (06/11): 'cluster 1' (default): INSIDE SQUARE CLUSTERS (>= 2 REAL T / 4-way junctions within 50 cm; L corners, square pulses not counted) reuses the v3.8 handling (version B):
                    KEEP HEADING (jmode 1: drifted left -> turn right to compensate, drifted right -> turn left), follow the branch edge firmly, on line loss go to
                    apex then turn. Outside clusters it is still like C3 (smooth, no stalling). 'cluster 0' = like C3.
  VERSION C3 (05/11): - REMOVED 'xv' (reach the apex then turn) -> no more stalling on square pulses. 90-degree corners use the gemini rolling arc as in version C.
                  - ANTI U-TURN (uturn 1): the robot remembers the PATH just driven (last 1 m). When searching for the line it does NOT accept a line lying on the old path
                    but facing backward (log 'u'), keeps turning to find the onward branch (log 'w'). While following the line, if it finds itself running BACKWARD on the
                    old path >= 60 mm -> turn 180 (log 'D'). Applies to square pulses, touching circles, after obstacle avoidance.
                    Only when it has turned all the way and ONLY the old line remains (dead end) does it turn back. uturn 0 = off.
                  - The old WRONG-WAY GUARD (heading compare 145 deg) false-triggered on sharp S-bends -> replaced by an old-path check (more accurate).
                  - TOUCHING CIRCLES: the reference heading (hRef) is not dragged along while the robot rolls around a circle; at the joint, while the robot is
                    heading off >= 35 deg and rolling backward on the heading-keeping branch side -> turn DECISIVELY onto that branch (up to 450 ms) -> no more circling.
                  - Unchanged: gemini line-follow controller, avscan 0 (avoid only follows the line when returning), jnmode 1. Your parameters are kept.
  VERSION C2 (05/11): - xv 1: 90-degree corner WITH A CLEAR SIGN -> reach the apex then turn (v3.8 way) -> no more spinning on square pulses.
                    Curves / S-bends / arcs (no corner sign) still use the high-speed rolling arc. Back to version C: xv 0.
                  - avscan 0: while avoiding, follow the line ONLY when RETURNING and only accept a line after >= 40% of the lateral offset (no false catch of the border line).
                    avscan 1: reads the line also while going out / parallel, like C.
                  - jnmode 1: within 60 cm of a junction / cluster -> v3.8-style line following (correct branch turns: touching circles, squares,
                    hexagons). Straights / bends / S-curves still use the gemini controller. jnmode 0: gemini always, like C.
  VERSION C (04/11): KEEPS the whole LINE FOLLOWING algorithm of gemini_V3 (line follow, corners, arcs, wrong-way guard).
    Only the OBSTACLE AVOID / LINE RE-CATCH part goes back to the verified (simulated) v3.8 logic:
    - REMOVED the 'target line filter' (only accepted a line within +-55 mm of the old line): coordinate error after 2 turns + a line changing direction
      (obstacle between 2 bends) made the robot SKIP the very line it had to catch -> line never caught.
    - REMOVED the 1.3x speed boost while avoiding (AV_SPEED_BOOST): crossed the line too fast for the photoresistor bar to confirm. For faster avoidance: raise vout/vpar.
    - Recovery: crossed the line -> drive onto it then turn away from the obstacle ('W'), following backward -> turn 180 ('D'),
      returning to a line that changed direction -> keep turning the same way, sweep toward where the line continues, too close to the brick -> back up ('B').
  2 cm line on white + junctions (square clusters, hexagon clusters) + 90-degree corners/zigzags + OBSTACLE AVOIDANCE 10x15 cm.
  v3.8 AVOID: - brick seen -> HARD BRAKE, too close (< obs x 0.75) then BACK UP to a safe distance before turning out (log 'B')
          - VL53 LONG RANGE mode -> sees the brick earlier at high speed
          - returning to a line that changed direction (between 2 bends): keep turning the SAME way to find it, no reverse sweep (~3 s faster)
  v3.7 OBSTACLE BETWEEN 2 BENDS: the parallel segment CROSSES the line -> bring the wheel axle onto the line then turn toward the
    direction the track continues (log 'W'). Line re-caught while running BACKWARD toward the obstacle -> turn 180 deg (log 'D').
  v3.6 SMOOTH CORNERING (idea from Tung's code, keeps the 90-DEGREE CORNER / square cluster handling of v3.5):
    - large line error -> strong braking (brake 0.85) + stronger kp (kpb 1.25)
    - line touches the edge without a junction -> PIVOT (inner wheel reverses) -> S-bends / zigzags lose the line less (piv 2.5)
    Old way: brake 0.45;kpb 1;piv 0
  v3.5 FLOWING AVOID (avmode 2): pivot out once in place, then STEER continuously into the parallel and return segments, with
    no more stop-and-turn -> avoids ~0.8-1 s faster (simulation 3.2 s -> 2.4 s). avmode 1 = as before. Straight segments while avoiding accelerate twice as fast.
  v3.4.1: - New curve handling ('curve 1', log 'q') OFF BY DEFAULT (curve 0) because it made the robot circle in hexagons.
            Enable only for testing - only matters when diving FAST from a long straight into a bend.
          - Added the 'VL53 last run' log line: nearest object the VL53 saw -> shows why it did not avoid.
  v3.3 KEEP PARAMETERS: type 'export' -> prints 1 line with ALL parameters. Paste that line back into the Serial Monitor = everything reloaded.
    Paste into USER_CFG (YOUR PARAMETERS section below) = the new code takes your parameters as defaults right away.
    Several commands at once: kp 12;kd 0.4;sp 70  (separated by ;)
  v3.2 SMOOTH RUN (simulation: 7-20% faster, wobble around the line ~20% lower, clusters 52/52):
    - PER-WHEEL SPEED PID (encoder): both wheels turn at the commanded speed even with a weak wheel / sagging battery. Measure once:
        command 'wcal' OR press SET on the INFO page (page 8) -> the robot drives straight ~2 m (50 % then 80 %), saves 'vfull' and 'dband'.
        Not measured -> the robot runs exactly like v3.1. Off: wpid 0.  Gains: wkp 1.5, wki 6.
    - SENSOR LAG COMPENSATION (slow photoresistor bar): 'lag' = latency (ms) measured with Test_LineSensor (START -> res). Default 15.
        Fast sensor (QTR/TCRT): lag 5. Off: lag 0.
    - alat (mm/s2, default 0 = off): sharp curves slow down automatically.
  v3.0 RUN MODES (READY page, SET button to change; or command 'mode 0/1/2', or the web page):
    NORMAL : as before.
    DEMO   : drive safely once, at the FINISH -> SAVE THE MAP (track shape every 25 mm + the ACTUAL turn side at each line loss).
    FAST   : by the map: straights at 'vfast', slow down BEFORE corners by 'mpre' mm, on line loss turn IMMEDIATELY to the right side.
             Still follows the line / avoids real obstacles -> works even if the track differs slightly; map wrong 2 times -> back to NORMAL.
    Commands: map (view the map) | mapclr (clear) | vfast 100 | mpre 250.  The map is kept after power-off.
  PINS (pin map v5):
    TB6612 : PWMA 38, AIN2 39, AIN1 40 (LEFT) | BIN1 41, BIN2 42, PWMB 2 (RIGHT)
    Encoder: LEFT A21 B47 | RIGHT A48 B1        QTR-8RA: D1..D8 -> 14,13,12,11,10,7,6,5 (D1 = LEFT)
    I2C    : SDA 8, SCL 9, 400 kHz (MPU6050 + VL53L0X + OLED SSD1306 0.96" 0x3C)   Buttons: START 18 | SET 17 | MODE 16
    Buzzer 15 | Battery sense: GPIO 4 (divider ratio 4.33, as in the Antigravity code that measured correctly)
  Libraries: Adafruit_VL53L0X + Adafruit GFX (same as the code you are using). The OLED is driven directly, sending
  16-byte packets at a time -> the screen updates in REAL TIME even while running without slowing the line loop.
  BUTTONS + OLED SCREEN (8 pages):
        MODE  = next page
        SET   = action of the current page (raise speed / calibrate / swap avoid side ...)
        START = RUN from any page (keep the robot still ~1 s for the gyro, then it runs). Press any button = STOP.
    Pages: 1 READY | 2 CORNER SPEED | 3 STRAIGHT SPEED | 4 SENSOR (live view of 8 channels + VL53 + angle)
           5 CALIBRATE | 6 OBSTACLE AVOID | 7 RUN RESULT | 8 INFO
    While RUNNING: the screen shows time, distance, speed, battery, state (updated 4 times/s).
  HOW THE ROBOT HANDLES THE TRACK:
    - Normal line: PD on the centroid of 8 channels; steady straight -> accelerate to vmax.
    - WIDE black (>= 4 channels) or 2 BRANCHES: FOLLOW THE LEFT EDGE (left-hand rule) -> T turns left,
      Y takes the left branch, right branch + straight goes straight.
      v2.9 'jmode 1' (KEEP HEADING, default): inside square / hexagon CLUSTERS, if the robot is LEFT of the previous straight by > 20 deg
      before, then at the next junction follow RIGHT (and vice versa) -> alternating left-right turns, passing THROUGH the cluster. (Always following left in a hexagon
      hexagon = circles one cell forever -> 'SPINNING'.) 'jmode 0' = always follow the 'pref' side as before.
    - v2.9 STUCK CHANNEL: one channel reports black ALONE (both neighbors white) > 40% of the time -> auto-ignore that channel (log 'STUCK CH').
      Manual disable: command 'mask' (D1 = 1, D2 = 2, ... D8 = 128).
    - 90-degree corner / zigzag to the RIGHT: robot overshoots, loses the line -> drives SENS_AX further (wheel axle at the corner) -> turns right.
    - Obstacle = BRICK 205x95x95 (VL53 < obs). 2 MODES (web: 'Avoid mode'):
      MANUAL (default): turn 'aout' -> diagonal 'out' mm -> turn parallel -> drive 'par' mm -> turn 'aback' -> back to the line.
      RETURN TO LINE (v2.8): while turning or returning diagonally, if the sensor bar touches the line -> NO stop: keep driving until the line is centered,
      enters an arc R = sens*cot(angle/2) -> the wheel axle lands on the line just as the robot becomes parallel -
> PD line following right away.
      AUTO: DIAGONAL AVOID MEASURING THE SIZE: turns out (at least 'aout' 45) while using the VL53 to measure the obstacle edge ->
      computes the obstacle width -> picks the out angle (45..90) so the nose corner does not hit the obstacle corner, picks the lateral offset (>= 'lat')
      and the parallel distance (brick ACROSS -> 95 along the line, brick ALONG -> 205; the wheel axle passes the back of the brick by another 'pass' mm) -> diagonal 'aback' (45) back to the line -> line centered on the sensor bar
      -> drive another 'sens' (wheel axle onto the line) -> turn back to the old heading. Turn angle limited each time. At most 'nobs' obstacles (0 = unlimited).
    - FINISH: >= 7 channels black continuously over 'finlen' mm (black box) AND driven > 'fin' mm.
  SERIAL 115200 (Newline) - every numeric command is SAVED AUTOMATICALLY:
    kp 14 | kd 0.35 | sp 40 (corner entry speed) | vmax 60 (straight speed) | cal | go | raw | flip | info
    obs 130 (mm VL53) | lat 180 | pass 380 | sens 60 (mm from the sensor bar to the wheel axle - MEASURE WITH A RULER)
    fin 1500 (minimum mm before accepting the finish) | nobs 2 | side 0/1 (avoid left/right) | pref 0/1 (follow left/right edge)
    cpr 1400 | cprcnt (measure cpr by hand, 10 turns) | wheel 43 | track 100 | vcal 8.05 | btn | scan | reset
  READ THE LOG WITHOUT USB (WiFi): the robot opens WiFi "LINE-ROBOT" password 12345678 when STOPPED
    (off while running / calibrating / on the sensor page because WiFi uses ADC2 shared with 4 line channels). Connect a phone,
    open a browser at 192.168.4.1 -> view results, event log, 50 ms trace, edit parameters, download log.txt to share.
    OLED page 9 = WIFI (SET on/off).
  FIRST TIME: cal (SET) -> raw to view the 8 channels -> run with sp 30 vmax 30 -> raise gradually. After each run the robot prints an EVENT LOG
  events with distance (mm) -> used to tune fin, sens, obs.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <WebServer.h>
#define SSD1306_WHITE 1
#define SSD1306_BLACK 0
// Types are declared at the TOP of the file: the Arduino IDE generates function prototypes above, if the struct is below it reports 'does not name a type'
struct Ev { float d; float t; char c; float h; uint8_t nb; };
struct Tr { uint16_t t; int32_t d; int8_t p; uint8_t v; char ph; uint8_t nb; uint16_t vl; int16_t h; uint8_t m; };
// ================= YOUR PARAMETERS =================
// Loaded automatically when: the code is flashed for the first time (no parameters yet), or after 'reset'. New code still KEEPS the parameters stored in the robot.
// UPDATE: type 'export' in the Serial Monitor -> copy the whole line -> paste between the 2 double quotes below (replace the old line).
const char* USER_CFG = "kp 10.6;kd 0.4;sp 80;vmax 100;cspd 39;vfast 73;mpre 250;tlead 20;jmode 1;pref 0;mask 0;auto 1;cpr 576.3;wheel 48;track 128;sens 115;fin 500;fgo 180;vk 1;obs 400;lat 190;pass 40;nobs 0;side 0;avmode 3;aout 58;aback 65;out 76;par 300;vout 51;vpar 51;vret 44;vturn 51;retm 0;brev 75;bms 90;dec 50;roled 0;wifi 1;lag 8;wpid 1;wkp 1.5;wki 6;alat 3800;vfull 1583;curve 1;brake 0.78;kpb 1.45;piv 1.75;uturn 1;avscan 0;jnmode 1;cluster 1;vclust 22;pclust 2;dband 25";   // 28/11: your tuned parameters after C3.25 (obs 400, brev 75, bms 90, par 300)
// ================= EDIT HERE =================
bool REV_L = true;     // LEFT motor reversed (confirmed)
bool REV_R = false;
const float PIVOT_PCT = 34;     // line search / corner turn speed
const float AV_PCT = 60;        // straight speed while avoiding obstacles
// C3.19 ARC avoid (avmode 3): steering gains and geometry
const float ARC_KH = 2.5f;      // % wheel difference per degree of heading error
const float ARC_KD = 0.04f;     // damping on the gyro turn rate
const float ARC_LOOK = 120;     // look-ahead (mm) when steering onto the lateral offset 'lat'
const float ARC_CATCH_DEG = 35;  // C3.21: approach angle over the last ARC_SLOW mm before the line
const float ARC_SLOW = 70;      // C3.20: slow to the catch speed when the sensor bar is this close (mm) to the line
const float ARC_DECEL = 2500;   // C3.24: braking the robot can really do on the mat (mm/s2, log: it SLIDES when braked harder)
const float ARC_CLEAR = 110;    // C3.24: room (mm) still needed in front of the brick to curve out once the speed is safe
const float ARC_LANE = 200;     // C3.23: object closer than this (mm) in the avoid lane -> lane blocked, use the other side
const float ARC_FACE = 30;      // the wheel axle must be 'lat' out when the nose is still this far (mm) before the brick face
const float AV_TURN = 60;       // max turn speed while avoiding
const float AV_SPEED_BOOST = 1.30f; // speed up the avoid segments without changing the wheel ratio
const float ROBOT_HALF_W = 80;  // half the robot width (robot ~157 mm wide)
const float REAR_MM = 35;       // robot tail behind the wheel axle
const float OB_LONG = 205, OB_SHORT = 95;   // OBSTACLE = BRICK 205 x 95 x 95 mm
const float CATCH_PCT = 38;     // speed right after touching the line on return
const float TURN_MIN = 32,   // C3.16: 26 -> 32 (loaded robot does not move below ~28 % PWM) | C3.15: 20 -> 26: the 20.7:1 gearbox stalled 0.3-0.5 s at the end of turns
            TURN_TOL = 3.0f, KP_T = 1.4f, KD_T = 0.07f;   // gyro-controlled turning
const float CX_V = 34;          // C3.29: speed (%) while following the circle arcs
const float CX_HYS = 8;         // C3.29: line heading must pass hRef by this much (deg) before the followed edge changes side
const float CX_BASE = 24;       // C3.29: line heading measured over this length of the followed line (mm)
const float CX_FORK = 1.3f;     // C3.29: two line candidates within this (channels) of the followed line = it forks -> chosen side
const float CX_ARM = 90;        // C3.31: a separate trace at D8 / D1 must merge with the line within this (mm) = circle cluster entry
const float CX_HOLD = 16;       // C3.31: ... and stay beside the line (black beyond the line on that side) this long (mm); T branch: <= 9
const float CX_PITCH = 12;      // C3.31: sensor spacing (mm, measured: D1-D8 edges 88 mm); the line on the bar -> world position
const float CX_STR = 50;        // C3.29: straight line on hRef this long (mm) = exit line
const float CX_MAX = 1400;      // C3.27: no exit line by then -> back to normal line following
const float CIRC_NO_REV = 600; // C3.15: this far (mm) after going round a circle, 'running backward' needs 120 mm of evidence instead of 60
const float HEX_GAP = 4;       // C3.11: white gap between the 2 branches (channels) ...
const float HEX_SPLIT = 30;    // ... must appear within this many mm from the start of the junction
const float HEX_GZ = 400;      // C3.11b: a Y-junction still counts as hexagon while the robot is curving (|gzSlow| < 400 deg/s, i.e. almost always). Previously 70 -> right after a bend into the hexagons
                               //  hexagon mode did NOT turn on -> the circle rule ran inside the cluster -> circling. Touching circles are already told apart by the branch-split gap (HEX_GAP/HEX_SPLIT). Old way: 70
const float HEX_CV = 100;      // C3.11: curving continuously for more than this distance (mm) = circle, temporarily disable hexagon mode
const float HEX_MM = 450;      // C3.11: how far (mm) after the last Y-junction counts as OUT of the hexagon cluster
const float START_GO = 60;     // C3.10: after the sensor bar leaves the START box, drive straight this many mm more before following the line
const float START_MAX = 600;   // C3.10: maximum straight distance (mm) from the start (black box too long / misplaced -> give up)
const float CIRC_COMMIT = 45;  // C3.9: curving threshold to COMMIT to the outer branch (old 90 deg/s -> large circles / slow runs never committed)
const float CIRC_GZ = 40;      // C3.9: curving > 40 deg/s -> at junctions take the branch on the OUTER side of the arc (circle exit). 0 = off (as C3.8)
const bool AVOID_ON_CURVE = true;      // C3.7: obstacle on a bend -> avoid toward the bend + catch the line as soon as it appears on the avoid side. false = always by 'side'
const float SWEEP_DEG = 120;    // line search turn of at most 120 deg each way (anti-spin)
// Guard after line loss: if the line is re-caught but the robot has turned close to 180 deg
// compared with the heading at line loss, it is the old line in reverse -> turn 180 deg to fix.
// Use 145 deg so it is not confused with a 90-degree corner skewed by inertia.
const float WRONG_WAY_DEG = 145.0f;
const float WRONG_WAY_FIX_PCT = 42.0f;
const uint8_t WRONG_WAY_MAX_FIX = 3;       // limit per run, avoids an infinite loop
// Target line catch while avoiding: no fixed timing. Uses the sensor coordinates
// relative to the original line, to ignore the opposite line while going around.
const float AV_EXIT_TOL_MM = 55.0f;        // lateral tolerance around the target line (mm)
const bool VL_LONG = true;      // v3.8: VL53 LONG RANGE mode (sees the brick earlier, ~30 reads/s). If the VL53 reports wrong / noisy -> false
const float SLOW_MM = 350;      // VL53 sees an object < 35 cm -> slow down
const float FIN_LEN = 45;       // mm of continuous black to accept the finish box
const float GAP_MM = 60;        // v2.9.6: line lost in the MIDDLE (not a corner) -> drive straight up to 60 mm to find it before turning
const float FLOOR_CUT = 0.35f;  // background level removed when computing the line position (the floor is not perfectly white)
const uint32_t I2C_HZ = 400000; // if I2C is flaky (MPU/VL53 lost) lower to 100000
const float VBAT_RATIO = 4.33f; // battery divider ratio (from code that measured it correctly)
const float VBAT_LOW = 7.0f;    // 2S battery below this -> LOW BATTERY warning
// ===============================================
#define PIN_PWMA 38
#define PIN_AIN2 39
#define PIN_AIN1 40
#define PIN_BIN1 41
#define PIN_BIN2 42
#define PIN_PWMB 2
#define ENC_LA 21
#define ENC_LB 47
#define ENC_RA 48
#define ENC_RB 1
#define PIN_SDA 8
#define PIN_SCL 9
#define BTN_START 18
#define BTN_SET 17
#define BTN_MODE 16
#define PIN_BUZ 15
#define PIN_VBAT 4
#define MPU_ADDR 0x68
const uint8_t LPIN[8] = {14, 13, 12, 11, 10, 7, 6, 5};
const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif
// ---------- parameters stored in NVS ----------
struct CfgV21 {   // version v1.3 .. v2.1 (to migrate parameters)
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
};
struct CfgV22 {   // version v2.2 (to migrate parameters)
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // speed while avoiding (%)
};
struct CfgV24 {   // version v2.3-v2.4
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
};
struct CfgV27 {   // version v2.5-v2.7
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
};
struct CfgV28 {   // version v2.8
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
};
struct Cfg {
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
  // v3.2 SMOOTH RUN
  float sdly;          // sensor latency (ms): per-channel lag compensation (command 'lag'). 0 = off
  uint8_t wpid;        // 1 = per-wheel SPEED PID (encoder) | 0 = direct PWM like v3.1
  float vfull;         // wheel speed at 100% PWM (mm/s) - measured with command 'wcal'. 0 = not measured -> PID turns itself off
  float wkp, wki;      // wheel speed PID gains
  float alat;          // lateral acceleration limit when entering bends (mm/s2): the sharper the bend the slower. 0 = off
  uint8_t curveOld;    // v3.4: 0 = curve/S-bend line loss -> brake + turn NOW (new) | 1 = like a 90-degree corner (old)
  // v3.6 SMOOTH CORNERING (learned from Tung's code): the larger the line error the SLOWER + STRONGER steering, near the edge PIVOT so the line is not lost
  float brakeK;          // brake by error: 0.45 (old) .. 0.85 (new): at 2.5 channels error -> (1 - hamK) of the speed remains
  float kpBoost;       // error >= 1.8 channels: kp x kpBoost (1.0 = old, 1.25 = new)
  float pivE;          // line touches the edge (|error| >= pivE) and it is NOT a junction -> pivot with the inner wheel reversing. 0 = off
  uint8_t cornerMode;  // C3: command 'uturn': 1 = ANTI U-TURN (remember the driven path) | 0 = off. (C2 used this slot for 'xv' - removed)
  uint8_t avScan;      // C2: 1 = read the line also while GOING OUT / PARALLEL | 0 = only when RETURNING (narrow track, no false catch of the border line)
  uint8_t jnMode;      // C2: 1 = within 60 cm of a JUNCTION / CLUSTER: v3.8-style following (correct branch turns: touching circles, squares) | 0 = gemini always
  uint8_t clusterMode;     // C3.1: 1 = inside a SQUARE CLUSTER / consecutive junctions: OLD v3.8 handling (keep heading compensating left/right + reach the apex then turn) | 0 = as C3
  uint8_t vCluster;        // C3.5: speed (%) inside a SQUARE CLUSTER (grid). Slower means less overshoot from inertia
  uint8_t pCluster;        // C3.5: side chosen at a perpendicular T inside a SQUARE CLUSTER: 0 left, 1 right, 2 = by pref
};
struct CfgC31 {   // version C3.1 - C3.4 (before 07/11)
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
  // v3.2 SMOOTH RUN
  float sdly;          // sensor latency (ms): per-channel lag compensation (command 'lag'). 0 = off
  uint8_t wpid;        // 1 = per-wheel SPEED PID (encoder) | 0 = direct PWM like v3.1
  float vfull;         // wheel speed at 100% PWM (mm/s) - measured with command 'wcal'. 0 = not measured -> PID turns itself off
  float wkp, wki;      // wheel speed PID gains
  float alat;          // lateral acceleration limit when entering bends (mm/s2): the sharper the bend the slower. 0 = off
  uint8_t curveOld;    // v3.4: 0 = curve/S-bend line loss -> brake + turn NOW (new) | 1 = like a 90-degree corner (old)
  // v3.6 SMOOTH CORNERING (learned from Tung's code): the larger the line error the SLOWER + STRONGER steering, near the edge PIVOT so the line is not lost
  float brakeK;          // brake by error: 0.45 (old) .. 0.85 (new): at 2.5 channels error -> (1 - hamK) of the speed remains
  float kpBoost;       // error >= 1.8 channels: kp x kpBoost (1.0 = old, 1.25 = new)
  float pivE;          // line touches the edge (|error| >= pivE) and it is NOT a junction -> pivot with the inner wheel reversing. 0 = off
  uint8_t cornerMode;  // C3: command 'uturn': 1 = ANTI U-TURN (remember the driven path) | 0 = off. (C2 used this slot for 'xv' - removed)
  uint8_t avScan;      // C2: 1 = read the line also while GOING OUT / PARALLEL | 0 = only when RETURNING (narrow track, no false catch of the border line)
  uint8_t jnMode;      // C2: 1 = within 60 cm of a JUNCTION / CLUSTER: v3.8-style following (correct branch turns: touching circles, squares) | 0 = gemini always
  uint8_t clusterMode;     // C3.1: 1 = inside a SQUARE CLUSTER / consecutive junctions: OLD v3.8 handling (keep heading compensating left/right + reach the apex then turn) | 0 = as C3
};
struct CfgC2 {   // version C2 / C3 (before 06/11)
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
  // v3.2 SMOOTH RUN
  float sdly;          // sensor latency (ms): per-channel lag compensation (command 'lag'). 0 = off
  uint8_t wpid;        // 1 = per-wheel SPEED PID (encoder) | 0 = direct PWM like v3.1
  float vfull;         // wheel speed at 100% PWM (mm/s) - measured with command 'wcal'. 0 = not measured -> PID turns itself off
  float wkp, wki;      // wheel speed PID gains
  float alat;          // lateral acceleration limit when entering bends (mm/s2): the sharper the bend the slower. 0 = off
  uint8_t curveOld;    // v3.4: 0 = curve/S-bend line loss -> brake + turn NOW (new) | 1 = like a 90-degree corner (old)
  // v3.6 SMOOTH CORNERING (learned from Tung's code): the larger the line error the SLOWER + STRONGER steering, near the edge PIVOT so the line is not lost
  float brakeK;          // brake by error: 0.45 (old) .. 0.85 (new): at 2.5 channels error -> (1 - hamK) of the speed remains
  float kpBoost;       // error >= 1.8 channels: kp x kpBoost (1.0 = old, 1.25 = new)
  float pivE;          // line touches the edge (|error| >= pivE) and it is NOT a junction -> pivot with the inner wheel reversing. 0 = off
  uint8_t cornerMode;  // C3: command 'uturn': 1 = ANTI U-TURN (remember the driven path) | 0 = off. (C2 used this slot for 'xv' - removed)
  uint8_t avScan;      // C2: 1 = read the line also while GOING OUT / PARALLEL | 0 = only when RETURNING (narrow track, no false catch of the border line)
  uint8_t jnMode;      // C2: 1 = within 60 cm of a JUNCTION / CLUSTER: v3.8-style following (correct branch turns: touching circles, squares) | 0 = gemini always
};
struct CfgC1 {   // version C (gemini_V3) before 05/11
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
  // v3.2 SMOOTH RUN
  float sdly;          // sensor latency (ms): per-channel lag compensation (command 'lag'). 0 = off
  uint8_t wpid;        // 1 = per-wheel SPEED PID (encoder) | 0 = direct PWM like v3.1
  float vfull;         // wheel speed at 100% PWM (mm/s) - measured with command 'wcal'. 0 = not measured -> PID turns itself off
  float wkp, wki;      // wheel speed PID gains
  float alat;          // lateral acceleration limit when entering bends (mm/s2): the sharper the bend the slower. 0 = off
  uint8_t curveOld;    // v3.4: 0 = curve/S-bend line loss -> brake + turn NOW (new) | 1 = like a 90-degree corner (old)
  // v3.6 SMOOTH CORNERING (learned from Tung's code): the larger the line error the SLOWER + STRONGER steering, near the edge PIVOT so the line is not lost
  float brakeK;          // brake by error: 0.45 (old) .. 0.85 (new): at 2.5 channels error -> (1 - hamK) of the speed remains
  float kpBoost;       // error >= 1.8 channels: kp x kpBoost (1.0 = old, 1.25 = new)
  float pivE;          // line touches the edge (|error| >= pivE) and it is NOT a junction -> pivot with the inner wheel reversing. 0 = off
};
struct CfgV35 {   // version v3.4 - v3.5
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
  // v3.2 SMOOTH RUN
  float sdly;          // sensor latency (ms): per-channel lag compensation (command 'lag'). 0 = off
  uint8_t wpid;        // 1 = per-wheel SPEED PID (encoder) | 0 = direct PWM like v3.1
  float vfull;         // wheel speed at 100% PWM (mm/s) - measured with command 'wcal'. 0 = not measured -> PID turns itself off
  float wkp, wki;      // wheel speed PID gains
  float alat;          // lateral acceleration limit when entering bends (mm/s2): the sharper the bend the slower. 0 = off
  uint8_t curveOld;    // v3.4: 0 = curve/S-bend line loss -> brake + turn NOW (new) | 1 = like a 90-degree corner (old)
};
struct CfgV32 {   // version v3.2 - v3.3
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
  // v3.2 SMOOTH RUN
  float sdly;          // sensor latency (ms): per-channel lag compensation (command 'lag'). 0 = off
  uint8_t wpid;        // 1 = per-wheel SPEED PID (encoder) | 0 = direct PWM like v3.1
  float vfull;         // wheel speed at 100% PWM (mm/s) - measured with command 'wcal'. 0 = not measured -> PID turns itself off
  float wkp, wki;      // wheel speed PID gains
  float alat;          // lateral acceleration limit when entering bends (mm/s2): the sharper the bend the slower. 0 = off
};
struct CfgV31 {   // version v3.0 - v3.1
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
  float mFast, mPre;   // v3.0 FAST RUN by map: straight speed (%), look-ahead distance to slow down (mm)
};
struct CfgV296 {   // version v2.9.6
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
  float finGo;         // v2.9.6: sensor bar touches the FINISH BOX edge -> drive this many mm further then stop (so the robot sits in the MIDDLE of the box)
};
struct CfgV295 {   // version v2.9 - v2.9.5
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs;
  int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax;
  float cpr, wheel, track;
  float obs, lat, pass, sensAx, finMin;
  float vcal;
  float aout, aback; uint8_t wifi;
  uint8_t avMode;          // 0 = AUTO (measure the brick with the VL53), 1 = MANUAL (distances you set)
  float outMm, parMm;      // MANUAL: diagonal-out distance, parallel distance (mm)
  float avSpd;             // (v2.2) avoid speeds - kept for compatibility
  // v2.8: per-segment avoid speeds + BRAKING
  float spdOut, spdPar, spdRet, spdTurn;   // % speed: diagonal out, parallel, diagonal back, turn
  float brkRev, brkMs, decMm;              // braking: reverse force (%), time (ms), slowdown distance before the stop point (mm)
  // v2.8
  uint8_t retMode;     // return-to-line method: 0 = FOLLOW DIRECTLY (arc, v2.4) | 1 = STOP - CENTER - TURN (v2.3, stable)
  uint8_t runOled;     // 1 = screen updates while running | 0 = off (more regular line loop)
  float turnLead;      // turn inertia compensation (ms): stop early based on turn rate
  float cornerSpd;     // v2.8: speed when approaching a 90-degree corner / sharp bend (%)
  uint8_t maskBits;    // v2.9: disable channels manually (bit0 = D1 ... bit7 = D8)
  uint8_t jMode;       // v2.9: junctions inside a CLUSTER (squares / hexagons): 1 = KEEP HEADING (pick the branch that brings the robot back to the main heading), 0 = always by 'pref'
  uint8_t noAuto;      // v2.9.4: 0 = AUTO-disable stuck channels while running (default), 1 = no auto-disable (stored in a pad byte, size unchanged)
};
// version v1.2 (to migrate old parameters without losing calibration / cpr)
struct CfgV12 {
  uint32_t magic; uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip, pref, side, nobs; int8_t sgnL, sgnR, gsgn;
  float kp, kd; int base, vmax; float cpr, wheel, track; float obs, lat, pass, sensAx, finMin; float vcal;
};
const uint32_t MAGIC = 0xF1A1D01A, MAGIC_C31 = 0xF1A1D019, MAGIC_C2 = 0xF1A1D018, MAGIC_C1 = 0xF1A1D017, MAGIC_V40 = 0xF1A1D016, MAGIC_V39 = 0xF1A1D015, MAGIC_V38 = 0xF1A1D014, MAGIC_V35 = 0xF1A1D013, MAGIC_V34 = 0xF1A1D012, MAGIC_V32 = 0xF1A1D011, MAGIC_V31 = 0xF1A1D010, MAGIC_V296 = 0xF1A1D00F, MAGIC_V295 = 0xF1A1D00E, MAGIC_V28 = 0xF1A1D00D, MAGIC_V27 = 0xF1A1D00C, MAGIC_V24 = 0xF1A1D00B, MAGIC_V22 = 0xF1A1D00A, MAGIC_V21 = 0xF1A1D009, MAGIC_V20 = 0xF1A1D008, MAGIC_V19 = 0xF1A1D007, MAGIC_V13 = 0xF1A1D006, MAGIC_V12 = 0xF1A1D005;
Cfg cfg; Preferences prefs;
void runCmdLine(String s, bool verbose);
void cfgSave();
void cfgDefault() {
  cfg = Cfg(); cfg.magic = MAGIC;
  for (int i = 0; i < 8; i++) { cfg.lmin[i] = 0; cfg.lmax[i] = 4095; }
  cfg.blackHigh = 1; cfg.calOk = 0; cfg.flip = 0; cfg.pref = 0; cfg.side = 0; cfg.nobs = 2;
  cfg.sgnL = 1; cfg.sgnR = 1; cfg.gsgn = 1;
  cfg.kp = 14; cfg.kd = 0.35f; cfg.base = 40; cfg.vmax = 55;
  cfg.cpr = 1400; cfg.wheel = 48; cfg.track = 130;
  cfg.obs = 120; cfg.lat = 185; cfg.pass = 455; cfg.sensAx = 120; cfg.finMin = 500;   // per the real robot pin map
  cfg.vcal = 1.0f;
  cfg.lat = 165; cfg.pass = 80; cfg.aout = 45; cfg.aback = 45; cfg.avMode = 1; cfg.outMm = 260; cfg.parMm = 280; cfg.avSpd = 60; cfg.wifi = 1;
  cfg.spdOut = 60; cfg.spdPar = 50; cfg.spdRet = 45; cfg.spdTurn = 55; cfg.brkRev = 35; cfg.brkMs = 50; cfg.decMm = 50;   // diagonal avoid 45/45 (simulation: no contact, faster than square avoid)
  cfg.retMode = 1; cfg.runOled = 0; cfg.turnLead = 20;
  cfg.cornerSpd = 45; cfg.maskBits = 0; cfg.jMode = 1; cfg.noAuto = 0; cfg.finGo = 180; cfg.mFast = 100; cfg.mPre = 250;
  cfg.sdly = 15; cfg.wpid = 1; cfg.vfull = 0; cfg.wkp = 1.5f; cfg.wki = 6; cfg.alat = 0; cfg.curveOld = 1; cfg.brakeK = 0.85f; cfg.kpBoost = 1.25f; cfg.pivE = 2.5f; cfg.cornerMode = 1; cfg.avScan = 0; cfg.jnMode = 1; cfg.clusterMode = 1; cfg.vCluster = 40; cfg.pCluster = 2;
  runCmdLine(USER_CFG, false);   // YOUR parameters (top of the file) override the defaults
}
void cfgLoad() {
  prefs.begin("thidau", false);   // NVS name kept from the first firmware so data already stored on the robot survives updates
  Cfg t; CfgV12 o; CfgV21 q; CfgV22 w; CfgV24 y; CfgV27 z; CfgV28 x8; CfgV295 x9; CfgV296 xa; CfgV31 xb; CfgV32 xc; CfgV35 xd;
  if (prefs.getBytes("c", &t, sizeof(t)) == sizeof(t) && t.magic == MAGIC) {
    cfg = t;
    if (cfg.avMode > 3) { cfg.avMode = 2; cfgSave(); }   // C3.19: avmode 3 = ARC avoid (new); anything above -> flowing avoid
    return;
  }
  { CfgC31 c31;   // version C3.1 - C3.4 -> C3.5: KEEP EVERYTHING (calibration, parameters), add vclust 40 / pclust 2
    if (prefs.getBytes("c", &c31, sizeof(c31)) == sizeof(c31) && c31.magic == MAGIC_C31) {
      cfgDefault(); uint32_t m = cfg.magic;
      memcpy(&cfg, &c31, sizeof(c31)); cfg.magic = m; cfg.vCluster = 40; cfg.pCluster = 2;
      if (cfg.avMode > 2) { cfg.avMode = 2; cfg.aout = 65; cfg.aback = 65; cfg.outMm = 260; cfg.parMm = 350; }   // avmode 3 removed
      cfgSave();
      Serial.println("Parameters migrated to C3.5 (added vclust 40: speed inside square clusters | pclust 2: side chosen in cluster = by pref)");
      return;
    } }
  { CfgC2 c2;   // version C2 / C3 -> C3.1: KEEP EVERYTHING (calibration, parameters), add 'cluster 1'
    if (prefs.getBytes("c", &c2, sizeof(c2)) == sizeof(c2) && c2.magic == MAGIC_C2) {
      cfgDefault(); uint32_t m = cfg.magic;
      memcpy(&cfg, &c2, sizeof(c2)); cfg.magic = m; cfg.clusterMode = 1;
      Serial.println("Parameters migrated to C3.1 (cluster 1: square cluster handled the old v3.8 way)");
      return;
    } }
  { CfgC1 c1;   // old version C -> C2: KEEP EVERYTHING (calibration, parameters), add 2 new options
    if (prefs.getBytes("c", &c1, sizeof(c1)) == sizeof(c1) && c1.magic == MAGIC_C1) {
      cfgDefault(); uint32_t m = cfg.magic;
      memcpy(&cfg, &c1, sizeof(c1)); cfg.magic = m; cfg.cornerMode = 1; cfg.avScan = 0; cfg.jnMode = 1; cfg.clusterMode = 1;
      Serial.println("Parameters migrated to C3 (uturn 1: anti U-turn | avscan 0: avoid only follows line when returning | jnmode 1)");
      return;
    } }
  if (prefs.getBytes("c", &t, sizeof(t)) == sizeof(t) && (t.magic == MAGIC_V40 || t.magic == MAGIC_V39 || t.magic == MAGIC_V38)) {
    cfg = t; cfg.magic = MAGIC;
    cfg.kp = 14.5f; cfg.kd = 0.55f; cfg.base = 76; cfg.vmax = 100; cfg.cornerSpd = 54;
    cfg.alat = 3800; cfg.brakeK = 0.78f; cfg.kpBoost = 1.45f; cfg.pivE = 1.75f;
    cfgSave();
    Serial.println("Auto-updated to v4.1: Fix 3 squares + Fix 90-degree L corner + Fix 60cm U shape");
    return;
  }
  if (prefs.getBytes("c", &xd, sizeof(xd)) == sizeof(xd) && (xd.magic == MAGIC_V35 || xd.magic == MAGIC_V34)) {   // v3.4/3.5 -> v3.6: keep everything
    cfgDefault(); uint32_t m = cfg.magic; bool from34 = xd.magic == MAGIC_V34;
    memcpy(&cfg, &xd, sizeof(xd)); cfg.magic = m; if (from34) cfg.curveOld = 1;
    cfg.brakeK = 0.85f; cfg.kpBoost = 1.25f; cfg.pivE = 2.5f;
    Serial.println("Parameters migrated to v3.6 (+ smooth cornering: brake 0.85 kpb 1.25 piv 2.5 | old way: brake 0.45;kpb 1;piv 0)");
    return;
  }
  if (prefs.getBytes("c", &xc, sizeof(xc)) == sizeof(xc) && xc.magic == MAGIC_V32) {   // from v3.2/3.3: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &xc, sizeof(xc)); cfg.magic = m; cfg.curveOld = 1;
    Serial.println("Parameters migrated from v3.3 (+ v3.4: new curve handling - 'curve 0' for the old way)");
    return;
  }
  if (prefs.getBytes("c", &xb, sizeof(xb)) == sizeof(xb) && xb.magic == MAGIC_V31) {   // from v3.0/3.1: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &xb, sizeof(xb)); cfg.magic = m;
    Serial.println("Parameters migrated from v3.1 (+ v3.2: lag/vfull taken from USER_CFG; if wrong type 'wcal', 'lag')");
    return;
  }
  if (prefs.getBytes("c", &xa, sizeof(xa)) == sizeof(xa) && xa.magic == MAGIC_V296) {   // from v2.9.6: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &xa, sizeof(xa)); cfg.magic = m; cfg.mFast = 100; cfg.mPre = 250;
    Serial.println("Parameters migrated from v2.9.6");
    return;
  }
  if (prefs.getBytes("c", &x9, sizeof(x9)) == sizeof(x9) && x9.magic == MAGIC_V295) {   // from v2.9 - v2.9.5: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &x9, sizeof(x9)); cfg.magic = m; cfg.finGo = 180; cfg.mFast = 100; cfg.mPre = 250;
    Serial.println("Parameters migrated from v2.9.x (+ fgo 180 mm: stop mid finish box)");
    return;
  }
  if (prefs.getBytes("c", &x8, sizeof(x8)) == sizeof(x8) && x8.magic == MAGIC_V28) {   // from v2.8: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &x8, sizeof(x8)); cfg.magic = m; cfg.maskBits = 0; cfg.jMode = 1;
    Serial.println("Parameters migrated from v2.8");
    return;
  }
  if (prefs.getBytes("c", &z, sizeof(z)) == sizeof(z) && z.magic == MAGIC_V27) {   // from v2.5-2.7: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &z, sizeof(z)); cfg.magic = m; cfg.cornerSpd = 45; cfg.maskBits = 0; cfg.jMode = 1;
    Serial.println("Parameters migrated from v2.7 (+ 90-degree corner entry speed 45%)");
    return;
  }
  if (prefs.getBytes("c", &y, sizeof(y)) == sizeof(y) && y.magic == MAGIC_V24) {   // from v2.3/2.4: keep everything
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &y, sizeof(y)); cfg.magic = m;
    cfg.retMode = 1; cfg.runOled = 0; cfg.turnLead = 20;
    Serial.println("Parameters migrated from v2.4 (return to line: STOP-ALIGN-TURN, screen while running: OFF)");
    return;
  }
  if (prefs.getBytes("c", &w, sizeof(w)) == sizeof(w) && w.magic == MAGIC_V22) {   // from v2.2: keep everything, add speeds + braking
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &w, sizeof(w)); cfg.magic = m;
    cfg.spdOut = w.avSpd; cfg.spdPar = min(w.avSpd, 50.0f); cfg.spdRet = min(w.avSpd, 45.0f); cfg.spdTurn = 55;
    cfg.brkRev = 35; cfg.brkMs = 50; cfg.decMm = 50;
    Serial.println("Parameters migrated from v2.2 + added per-segment and straight speeds");
    return;
  }
  if (prefs.getBytes("c", &q, sizeof(q)) == sizeof(q) &&
      (q.magic == MAGIC_V13 || q.magic == MAGIC_V19 || q.magic == MAGIC_V20 || q.magic == MAGIC_V21)) {   // keep all old parameters
    cfgDefault(); uint32_t m = cfg.magic;
    memcpy(&cfg, &q, sizeof(q)); cfg.magic = m;
    if (q.magic != MAGIC_V21) { cfg.aout = 45; cfg.aback = 45; cfg.pass = 80; }
    cfg.avMode = 1; cfg.outMm = 260; cfg.parMm = 280; cfg.avSpd = 60;
    cfg.spdOut = 60; cfg.spdPar = 50; cfg.spdRet = 45; cfg.spdTurn = 55; cfg.brkRev = 35; cfg.brkMs = 50; cfg.decMm = 50;
    Serial.println("Old parameters migrated; MANUAL avoid: diagonal out 260, parallel 280 (adjust on web)");
    return;
  }
  cfgDefault();
  if (prefs.getBytes("c", &o, sizeof(o)) == sizeof(o) && o.magic == MAGIC_V12) {   // keep calibration + old pin map
    memcpy(cfg.lmin, o.lmin, sizeof o.lmin); memcpy(cfg.lmax, o.lmax, sizeof o.lmax);
    cfg.blackHigh = o.blackHigh; cfg.calOk = o.calOk; cfg.flip = o.flip; cfg.pref = o.pref; cfg.side = o.side; cfg.nobs = o.nobs;
    cfg.sgnL = o.sgnL; cfg.sgnR = o.sgnR; cfg.gsgn = o.gsgn; cfg.kp = o.kp; cfg.kd = o.kd; cfg.base = o.base; cfg.vmax = o.vmax;
    cfg.cpr = o.cpr; cfg.wheel = o.wheel; cfg.track = o.track; cfg.obs = o.obs; cfg.sensAx = o.sensAx; cfg.finMin = o.finMin; cfg.vcal = o.vcal;
    Serial.println("Parameters migrated from v1.2 (lat/pass reset for the new avoid method)");
  }
}
void cfgSave() { prefs.putBytes("c", &cfg, sizeof(cfg)); }
// ---------- encoder ----------
volatile int32_t rawL = 0, rawR = 0;
volatile uint8_t prevL = 0, prevR = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() { uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB); rawL += QDEC[(prevL << 2) | s]; prevL = s; }
void IRAM_ATTR isrR() { uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB); rawR += QDEC[(prevR << 2) | s]; prevR = s; }
float mmL() { return cfg.sgnL * rawL * PI * cfg.wheel / cfg.cpr; }
float mmR() { return cfg.sgnR * rawR * PI * cfg.wheel / cfg.cpr; }
float dist() { return (mmL() + mmR()) * 0.5f; }
// ---------- motor ----------
void motorOne(int side, float pct) {
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (side ? REV_R : REV_L) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  digitalWrite(in1, pct > 0); digitalWrite(in2, pct < 0);
  PWM_OUT(pwm, side, (int)(fabsf(pct) * PWM_MAX / 100.0f));
}
float vL = 0, vR = 0;                    // measured wheel speed (mm/s), updated in update()
// C3.9 ANTI-STALL WHEN PIVOTING: a turn command (wheels opposite) but both wheels still STOPPED > 150 ms (static friction, weak battery)
// -> ramp up PWM (+10%/100 ms, max +30%) until the wheels turn; keep that boost until the end of the turn.
// (log 10/11: 180-degree turn 'D' stuck at -325 deg -> stood still for 7 s because 38% turn PWM could not move the robot)
float kickB = 0; uint32_t tKick = 0;
void drive(float l, float r) {
  if (l * r < 0 && fabsf(l) >= 10 && fabsf(r) >= 10) {
    uint32_t now = millis();
    if (fabsf(vL) > 30 || fabsf(vR) > 30) tKick = now;
    else if (now - tKick > 80) kickB = max(kickB, min(30.0f, (now - tKick - 80) * 0.2f));   // C3.15: start after 80 ms, +20%/100 ms (was 150 ms, +10%)
    l += l > 0 ? kickB : -kickB; r += r > 0 ? kickB : -kickB;
  } else { kickB = 0; tKick = millis(); }
  motorOne(0, l); motorOne(1, r);
}
// v3.2 PER-WHEEL SPEED PID: command % -> target speed (mm/s) = % x vfull. PWM = % (as before) + correction from the speed error
// measured by the encoders -> both wheels turn at the RIGHT speed: a weak wheel / sagging battery / different friction does not pull the robot aside, faster acceleration.
float wiL = 0, wiR = 0; uint32_t tDV = 0;
bool wpidOk() { return cfg.wpid && cfg.vfull > 100; }
// C3.24: the highest speed (mm/s) from which the robot can still slow down and curve out before an object 'mm' ahead
float vSafeMM(float mm);
// C3.24: % of the old-gearbox speed (C3.11b tuned the raw numbers at vfull 1158) -> same REAL speed through the wheel PID
float oldPct(float pct) { return wpidOk() ? pct * 1158.0f / cfg.vfull : pct; }
float dBand = 25.0f;   // C3.16: PWM % below which the loaded robot does not move (measured by 'wcal', stored apart from cfg: key "db")
// C3.16: measured speed is a straight line above the dead band: v = vfull x (PWM - dband) / (100 - dband) -> PWM needed for pct
float feedFwd(float pct) { return fabsf(pct) < 1 ? pct : (pct > 0 ? 1 : -1) * (dBand + fabsf(pct) * (100.0f - dBand) / 100.0f); }
float wheelPid(float pct, float v, float &integ, float dt) {
  if (pct * integ < 0 || fabsf(pct) < 1) integ = 0;            // direction change / stop -> clear the integral
  float err = (pct * cfg.vfull / 100.0f - v) * 100.0f / cfg.vfull;   // speed mismatch as %
  integ = constrain(integ + cfg.wki * err * dt, -25.0f, 25.0f);
  return feedFwd(pct) + cfg.wkp * err + integ;
}
void driveV(float l, float r) {
  if (!wpidOk()) { drive(l, r); return; }
  uint32_t now = micros(); float dt = (now - tDV) * 1e-6f; tDV = now;
  if (dt > 0.05f) { wiL = wiR = 0; dt = 0; }                    // not called for a while (just turned / braked) -> restart
  drive(wheelPid(l, vL, wiL, dt), wheelPid(r, vR, wiR, dt));
}
void coast() { drive(0, 0); }
void brake() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); PWM_OUT(PIN_PWMA, 0, PWM_MAX);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
}
// ---------- MPU ----------
bool mpuOk = false; float gz = 0, bgz = 0, hdg = 0, encHPrev = 0; int gyroFail = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR); if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x80); delay(100); mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x19, 0x00); mpuWrite(0x1B, 0x18);
  return true;
}
bool readGz(float &o) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 2) != 2) return false;
  int16_t h = Wire.read(); int16_t r = (h << 8) | Wire.read(); o = r / 16.4f; return true;
}
float encHeading();
void calibrateGyro() {
  coast(); double s = 0; int n = 0; float g; uint32_t t0 = millis();
  while (millis() - t0 < 1000) { if (readGz(g)) { s += g; n++; } delay(2); }
  bgz = n ? s / n : 0; hdg = 0; encHPrev = encHeading();
}
// ---------- VL53 (non-blocking) ----------
uint16_t vlMinRun = 8190; uint32_t vlN = 0;   // v3.4.1: nearest VL53 reading during the run + read count (diagnoses missing avoidance)
Adafruit_VL53L0X vl; bool vlOk = false, vlOn = true; uint16_t vlMM = 8190; uint32_t vlT = 0;
void vlPoll() {
  if (!vlOk || millis() - vlT < 10) return;
  vlT = millis();
  if (!vl.isRangeComplete()) return;          // continuous ranging: only read when a new result exists, no waiting
  uint16_t r = vl.readRangeResult();
  vlMM = (r != 0xFFFF && vl.readRangeStatus() == 0) ? r : 8190;
  vlN++; if (vlMM < vlMinRun) vlMinRun = vlMM;
}
// ---------- buzzer + buttons ----------
void beep(int n, int on = 70) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(70); } }
// C3.5 (from version B) AT FINISH: plays HAPPY BIRTHDAY (~8 s). Press any button to stop. PASSIVE buzzer -> correct pitch;
// ACTIVE buzzer (beeps by itself) -> only the rhythm is right. Disable the song: FIN_SONG = false.
const bool FIN_SONG = true;
void syncButtons();
void playBirthday() {
  const uint16_t G4 = 392, A4 = 440, B4 = 494, C5 = 523, D5 = 587, E5 = 659, F5 = 698, G5 = 784;
  const uint16_t nt[] = {G4, G4, A4, G4, C5, B4,   G4, G4, A4, G4, D5, C5,   G4, G4, G5, E5, C5, B4, A4,   F5, F5, E5, C5, D5, C5};
  const float    du[] = {.75, .25, 1, 1, 1, 2,     .75, .25, 1, 1, 1, 2,     .75, .25, 1, 1, 1, 1, 2.5,    .75, .25, 1, 1, 1, 3};
  const int beat = 380;   // ms / beat
  for (unsigned i = 0; i < sizeof(nt) / sizeof(nt[0]); i++) {
    int d = (int)(du[i] * beat);
    tone(PIN_BUZ, nt[i]);
    uint32_t t0 = millis(); bool stop = false;
    while (millis() - t0 < (uint32_t)(d * 0.85f)) { if (digitalRead(BTN_START) == LOW || digitalRead(BTN_SET) == LOW || digitalRead(BTN_MODE) == LOW) { stop = true; break; } delay(2); }
    noTone(PIN_BUZ);
    if (stop) break;
    delay((int)(d * 0.15f));
  }
  noTone(PIN_BUZ); pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  syncButtons();
}
const uint8_t BTN[3] = {BTN_START, BTN_SET, BTN_MODE};
bool bDown[3] = {false, false, false}; uint32_t bT[3] = {0, 0, 0};
int pressed() {
  for (int i = 0; i < 3; i++) {
    bool r = digitalRead(BTN[i]) == LOW;
    if (r != bDown[i] && millis() - bT[i] > 30) { bDown[i] = r; bT[i] = millis(); if (r) return i; }
  }
  return -1;
}
void syncButtons() { for (int i = 0; i < 3; i++) { bDown[i] = digitalRead(BTN[i]) == LOW; bT[i] = millis(); } }
// ---------- common update ----------
void traceTick();
bool aborted = false, spinStop = false, tracing = false;
float odoX = 0, odoY = 0;   // v3.7: robot coordinates (mm) - used to know where the obstacle is while avoiding
float spinAcc = 0; uint32_t tSpin = 0;   // ANTI-SPIN: turned > 720 deg (2 turns) within 3 s while running -> STOP THE ROBOT
// ROBOT HEADING (deg, counter-clockwise = positive): integrated gyro; on gyro read error / saturation -> use encoders
// wait for that measurement (no angle jump) -> no more spinning from a wrong angle.
float encHeading() { return (mmR() - mmL()) / cfg.track * 57.2958f; }
float heading() { return hdg; }
void update() {
  static uint32_t t = micros();
  uint32_t now = micros();
  if (now - t >= 3000) {
    float dt = (now - t) * 1e-6f; t = now;
    float eh = encHeading(), dEnc = eh - encHPrev; encHPrev = eh;
    float ddS = 0;
    { static float dPrev = 0; float dn = dist(), dd = dn - dPrev; dPrev = dn; ddS = dd;   // v3.7: robot position (x, y) from the encoders + heading
      if (fabsf(dd) < 50) { odoX += dd * cosf(hdg * 0.0174533f); odoY += dd * sinf(hdg * 0.0174533f); } }
    float g = 0; bool ok = mpuOk && readGz(g) && fabsf(g) < 1990;
    float hPrev = hdg;
    if (ok) { gz = cfg.gsgn * (g - bgz); hdg += gz * dt; gyroFail = 0; }
    else {
      gz = dEnc / dt; hdg += dEnc;
      if (mpuOk && ++gyroFail > 30) { mpuOk = false; Serial.println("!! MPU lost repeatedly -> switching to encoder heading"); }
    }
    { static float pL = 0, pR = 0; static uint32_t tw = 0; uint32_t nw = micros();   // wheel speed every >= 8 ms
      if (nw - tw >= 8000) { float dw = (nw - tw) * 1e-6f, a = mmL(), b = mmR();
        if (dw < 0.1f) { vL = 0.5f * vL + 0.5f * (a - pL) / dw; vR = 0.5f * vR + 0.5f * (b - pR) / dw; }
        pL = a; pR = b; tw = nw; } }
    if (tracing) {
      if (millis() - tSpin > 3000) { tSpin = millis(); spinAcc = 0; }
      // C3.9: only count rotation when TURNING NEARLY IN PLACE (> 1 deg / mm, radius < 6 cm). The old version also counted rolling around small circles
      //  (R 15 cm, 0.6 m/s = 690 deg / 3 s) -> wrongly stopped inside the circle cluster while rolling around and searching for the exit.
      if (fabsf(hdg - hPrev) > 1.0f * fabsf(ddS)) spinAcc += fabsf(hdg - hPrev);
      if (spinAcc > 720 && !spinStop) { spinStop = true; aborted = true; }
    }
  }
  traceTick();
  vlPoll();
  if (pressed() >= 0) aborted = true;
}
void waitMs(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms && !aborted) update(); }
// ---------- line sensor ----------
uint16_t raw[8]; float val[8];
int nBlack = 0, nBlob = 0, b0s = 0, b0e = 0, bLs = 0, bLe = 0;   // first branch (leftmost) and last branch (rightmost)
uint32_t adc2Zero = 0;   // number of ADC2 reads (D1..D4) returning 0 - a sign that WiFi is holding ADC2
int maxRun = 0;   // longest run of consecutive black (channels)
bool sStuck[8] = {false}; uint8_t stuckEv = 0;   // stuck channels (auto-ignored), channels that were ever stuck during the run
bool runActive = false;   // stuck channels are auto-disabled only during a CONTEST RUN (sensor page / raw / calibration always show REAL values)
bool inAvoid = false;  // while avoiding an obstacle: do NOT use the stuck-channel filter (a line under one edge channel is normal)
float lineC = 0;      // line center (before a junction) - to know which side a wide trace flares to
float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float jDh = 0, hRefG = 0; bool gridG = false; bool hexG = false;   // hexG: C3.11 inside a HEXAGON (honeycomb) cluster
//   // gridG: in grid mode (square cluster)   // C3: heading offset from hRef (deg), used at junctions
// C3 DECISIVE TURN AT JUNCTIONS (touching circles): chose the HEADING-KEEPING branch but the branch only flashed past the sensor bar (robot rolling along
// old circle) -> HOLD the turn command toward that side until the heading is close to hRef (<= 20 deg) or 450 ms pass. Only while following the line.
bool gClus = false;   // C3.1: inside a CLUSTER (>= 2 junctions within 45 cm) and 'cluster 1' -> v3.8-style junction / corner handling
int8_t jCom = 0, jPend = 0; uint32_t tJCom = 0; float jPendD = 0; float gzSlow = 0;   // gzSlow: turn rate filtered over ~0.25 s (curving around a circle CONTINUOUSLY)
 bool onOldPath(float lat, float r = 35.0f, float back = 600.0f); float jRecD = -1e9;   // jRecD: position where the line was just re-caught after a loss (search turn)
bool jCommitOK();
uint8_t jPref = 0;   // side followed at the current junction (0 left, 1 right): = cfg.pref, or chosen automatically when jMode = 1
uint8_t lineMask = 0; float pos = 0, posC = 0, peakV = 0; bool seen = false, junc = false;
void readLine() {
  float sw = 0, s = 0, peak = 0; nBlack = 0; nBlob = 0; lineMask = 0;
  for (int i = 0; i < 8; i++) {
    { uint8_t pn = LPIN[cfg.flip ? 7 - i : i]; uint16_t a1 = analogRead(pn), a2 = analogRead(pn), a3 = analogRead(pn), a4 = analogRead(pn);
      raw[i] = (a1 + a2 + a3 + a4) / 4;   // v2.9.5: read 4 times and average -> motor noise (PWM 20 kHz) drops ~2x
      if (a1 == 0 && pn >= 11 && pn <= 14) adc2Zero++; }
    float span = max(1, (int)cfg.lmax[i] - (int)cfg.lmin[i]);
    float v = constrain(((int)raw[i] - (int)cfg.lmin[i]) / span, 0.0f, 1.0f);
    if (!cfg.blackHigh) v = 1 - v;
    val[i] = v;
  }
  // v3.2 LAG COMPENSATION (slow sensor, e.g. photoresistor): a channel responds like a 'first-order lag' with time constant 'sdly' ms:
  //   read = real + lag  ->  real ~= read + sdly x (rate of change of read). After compensation a channel reports black/white AS SOON AS the line arrives.
  {
    static float vPrev[8]; static uint32_t tPrevR = 0; static bool okPrev = false;
    uint32_t nw = micros(); float dtr = (nw - tPrevR) * 1e-6f; tPrevR = nw;
    bool use = cfg.sdly > 0 && okPrev && dtr > 0.0003f && dtr < 0.03f;
    for (int i = 0; i < 8; i++) {
      float v0 = val[i];
      if (use) val[i] = constrain(v0 + cfg.sdly * 0.001f * (v0 - vPrev[i]) / dtr, 0.0f, 1.0f);
      vPrev[i] = v0;
    }
    okPrev = true;
  }
  // 0. STUCK CHANNEL: a channel reports black ALONE (both sides white) > 40% of the time (e.g. D8 at the wrong height / misreading the brick)
  //    -> automatically IGNORE that channel (treated as white) until it reads normally again. Or disable it manually on the web page.
  {
    static uint16_t isoCnt[8] = {0}, winN = 0;
    if (runActive && !inAvoid && !cfg.noAuto) {
      // "stuck" channel: reports black ALONE while the REAL line (>= 2 adjacent black channels) is elsewhere, >= 2 channels away.
      // (a channel with the real line under it is NOT counted -> never wrongly disables a channel that sees the line)
      for (int i = 0; i < 8; i++) {
        bool l = i > 0 && val[i - 1] > 0.4f, r = i < 7 && val[i + 1] > 0.4f;
        if (!(val[i] > 0.5f && !l && !r)) continue;
        bool other = false;
        for (int j = 0; j < 7; j++) if (abs(j - i) >= 2 && abs(j + 1 - i) >= 2 && val[j] > 0.5f && val[j + 1] > 0.5f) other = true;
        if (other) isoCnt[i]++;
      }
      // only count while REALLY seeing the line (>= 2 adjacent channels): when the line is lost / searching, KEEP the previous verdict
      bool realLine = false;
      for (int j = 0; j < 7; j++) if (val[j] > 0.5f && val[j + 1] > 0.5f) realLine = true;
      if (realLine && ++winN >= 200) {
        for (int i = 0; i < 8; i++) {
          bool was = sStuck[i];
          if (isoCnt[i] > 50) sStuck[i] = true; else if (isoCnt[i] < 5) sStuck[i] = false;   // > 25% of reads are lost
          if (sStuck[i] && !was) stuckEv |= 1 << i;
          isoCnt[i] = 0;
        }
        winN = 0;
      }
    }
    for (int i = 0; i < 8; i++) if ((sStuck[i] && runActive && !inAvoid && !cfg.noAuto) || ((cfg.maskBits >> i) & 1)) val[i] = 0;
  }
  // 1. black channel (1 "missing" channel between 2 black channels still counts as black)
  bool bk[8];
  for (int i = 0; i < 8; i++) {
    bk[i] = val[i] > 0.5f || (i > 0 && i < 7 && val[i] > 0.2f && val[i - 1] > 0.5f && val[i + 1] > 0.5f);
    if (val[i] > 0.5f) lineMask |= 1 << i;
    peak = max(peak, val[i]);
  }
  // 2. split into consecutive "black traces". A 2 cm line always covers >= 2 adjacent channels.
  //    A trace on ONE isolated channel (e.g. outer channel D1/D8 misreading the brick edge, a shadow) when a real line exists -> IGNORE (noise)
  int rs[8], re[8], nr = 0;
  for (int i = 0; i < 8; i++) if (bk[i]) { if (i == 0 || !bk[i - 1]) rs[nr] = i; if (i == 7 || !bk[i + 1]) re[nr++] = i; }
  int big = 0; for (int k = 0; k < nr; k++) if (re[k] - rs[k] >= 1) big++;
  bool drop[8] = {false};
  maxRun = 0;
  for (int k = 0; k < nr; k++) {
    int w = re[k] - rs[k] + 1;
    if (w == 1 && big > 0) { drop[rs[k]] = true; continue; }
    maxRun = max(maxRun, w); nBlack += w; nBlob++;
    if (nBlob == 1) { b0s = rs[k]; b0e = re[k]; }
    bLs = rs[k]; bLe = re[k];
  }
  for (int i = 0; i < 8; i++) {
    if (drop[i]) continue;
    float v = val[i];
    if (v > FLOOR_CUT) { float w = (v - FLOOR_CUT) / (1 - FLOOR_CUT); sw += w; s += w * (i - 3.5f); }   // remove background
  }
  // on/off with hysteresis so it does not flicker LINE <-> LINE LOST
  peakV = peak;
  if (sw <= 0) seen = false; else if (peak >= 0.60f) seen = true; else if (peak < 0.45f) seen = false;
  if (seen && sw > 0) posC = s / sw;
  junc = nBlob >= 2 || nBlack >= 5;   // a 2 cm line covers only 2-4 channels; >= 5 channels or 2 branches means a junction
  if (!seen) return;
  if (!junc) {
    pos = posC;
    if (jPend != 0) {
      if (!jCommitOK() || dist() - jPendD > 120.0f) jPend = 0;
      else if (fabsf(posC - lineC) < 1.2f && dist() - jPendD > 15.0f) { jCom = jPend; tJCom = millis(); jPend = 0; }   // straight line still present -> real T-junction
    }
    if (jCom != 0 && jCommitOK()) {
      if (millis() - tJCom > 450 || fabsf(wrap180(heading() - hRefG)) <= 20.0f) jCom = 0;
      else {   // prefer the black trace on the chosen side; none -> steer hard toward that side to catch the new branch
        float c0 = 0.5f * (b0s + b0e) - 3.5f, cL = 0.5f * (bLs + bLe) - 3.5f;
        float c = jCom > 0 ? cL : c0;
        pos = (jCom > 0 ? c > lineC + 0.6f : c < lineC - 0.6f) ? c : jCom * 2.6f;
        return;   // keep the old lineC (old branch) for comparison
      }
    }
    lineC = posC; return;
  }
  // v2.9.3: 1 WIDE BLACK TRACE flaring to only ONE side of the previous line center = CORNER (only 1 branch) -> follow that edge,
  // NOT by the preferred side. (old: always followed the LEFT edge -> left corners smooth, right corners pulled straight then turned wrong)
  int side = jPref;
  // Follow the edge only for a SINGLE-BRANCH L CORNER (one side touches the edge, the other still has >= 2 white channels).
  // If it is a T / 4-WAY JUNCTION inside the 3-square cluster (black covering both sides b0s <= 1 && b0e >= 6 or 2 branches) -> FOLLOW jPref (jMode 1)!
  float edgePos;
  if (gClus) {
    // C3.1 SQUARE CLUSTER OLD STYLE (v3.8): one trace clearly flaring to one side (>= 2.5 channels) = corner -> follow that side; otherwise (T / 4-way)
    // -> by jPref (KEEP HEADING: drifted left then turn right to compensate, drifted right then turn left). FOLLOW the 2 outermost channels of the branch.
    if (nBlob == 1) {
      float extL = lineC - (b0s - 3.5f), extR = (b0e - 3.5f) - lineC;
      if (extR > extL + 2.5f) side = 1; else if (extL > extR + 2.5f) side = 0;
    }
    if (side == 0) { int a = b0s, b = min(b0s + 1, b0e); pos = (a + b) * 0.5f - 3.5f; }
    else           { int b = bLe, a = max(bLe - 1, bLs);  pos = (a + b) * 0.5f - 3.5f; }
    edgePos = (side == 0) ? ((float)b0s - 3.5f) : ((float)bLe - 3.5f);
  } else {
  if (nBlob == 1 && !(b0s <= 1 && b0e >= 6)) {
    float extL = lineC - (b0s - 3.5f), extR = (b0e - 3.5f) - lineC;
    if (extR > extL + 2.0f) side = 1; else if (extL > extR + 2.0f) side = 0;
  }
  edgePos = (side == 0) ? ((float)b0s - 3.5f) : ((float)bLe - 3.5f);
  pos = 0.78f * edgePos + 0.22f * posC;
  }
  // C3 TOUCHING CIRCLES: the robot is FAR OFF heading (>= 35 deg vs the cluster entry heading) and the junction has a branch on the HEADING-KEEPING SIDE
  // -> FOLLOW the edge of that branch firmly -> turn decisively onto the right branch, no further looping on the old circle.
  // only when the robot is ROLLING BACKWARD on that side (e.g. circling the old loop): L corners / square pulses (driving straight into the corner) are unaffected
  if (jCommitOK() && fabsf(jDh) >= 35.0f && side == jPref && (side ? gzSlow > (hexG ? 90.0f : CIRC_COMMIT) : gzSlow < -(hexG ? 90.0f : CIRC_COMMIT)) && dist() - jRecD > 200.0f) {
    float ext = side ? (bLe - 3.5f) - lineC : lineC - (b0s - 3.5f);
    if (ext >= 1.0f && !onOldPath(edgePos)) { pos = edgePos * 1.15f; jCom = side ? 1 : -1; tJCom = millis(); }
  }
  // C3.1 SQUARE CLUSTER (cluster 1): robot driving PERPENDICULAR to the main heading (>= 60 deg), junction with a branch on the HEADING-KEEPING SIDE -> REMEMBER it.
  //   After the junction the STRAIGHT line still continues (= real T-junction, not an L corner) -> turn decisively onto that branch (see the 'jPend' branch above).
  //   (the exit branch at the side only passes the sensor bar for ~0.1 s -> v3.8 cannot turn in time -> robot circles the cluster)
}
// ---------- event log ----------
// RING BUFFER: always keeps the 150 NEWEST events (not filled at the start of the run like the old version)
const int NEV = 150; Ev evs[NEV]; int nEv = 0; uint32_t tRun0 = 0; float d0 = 0; int cntL = 0, cntJ = 0;
void logEv(char c) {
  if (c == 'L') cntL++;
  if (c == 'J') cntJ++;
  Ev &e = evs[nEv % NEV]; e.d = dist() - d0; e.t = (millis() - tRun0) / 1000.0f; e.c = c; e.h = heading(); e.nb = nBlack; nEv++;
}
const Ev& evAt(int k) { int first = nEv > NEV ? nEv - NEV : 0; return evs[(first + k) % NEV]; }
int evCount() { return nEv > NEV ? NEV : nEv; }
// detailed trace every 50 ms (max 35 s) -> view over WiFi / Serial
char phase = '-';
bool jCommitOK() { return runActive && !inAvoid && cfg.jMode && (phase == '-'); }     // '-' line following | 'S' line search | '1'..'6' avoid steps | 'C' line re-caught
const int NTR = 700; Tr tr[NTR]; int nTr = 0; float curSpd = 0; uint32_t tTr = 0;
// Line filter while avoiding. Ref is the line through the sensor bar position when
// avoid start; the target line at Z must lie close to this line. The opposite line at Y
// will have a larger lateral offset and is ignored, even if it appears early.
bool avExitFilter = false;
// C2 (avscan 0) ENTRY GATE: when RETURNING only accept a line once the sensor bar is >= 40% back inside the parallel offset
// (narrow track: the border line lies right under the robot while driving parallel -> not caught by mistake)
bool avGate = false; float avGRX = 0, avGRY = 0, avGRH = 0, avGLim = 0; int avGS = 1;
float avCross() {   // how far (mm) the sensor bar is offset toward the avoid side from the original line
  float h = heading() * DEG_TO_RAD, sx = odoX + cosf(h) * cfg.sensAx, sy = odoY + sinf(h) * cfg.sensAx, rh = avGRH * DEG_TO_RAD;
  return avGS * (-(sx - avGRX) * sinf(rh) + (sy - avGRY) * cosf(rh));
}
bool goingBack(float deg);
bool avGateOK() { return !avGate || avCross() <= avGLim; }
float avRefX = 0, avRefY = 0, avRefH = 0;
bool obstacleExitLineTarget();
// C3.30 CLUSTER CAPTURE (diagnostic): every 2 mm the 8 analog channels (0-F, F = black) + heading. Starts at the first wide trace
//  (>= 3 channels) or 2 traces after 150 mm of the run, keeps 100 samples before and 400 after (~1 m) -> "CAPTURE 2mm" in the log.
struct Cap { int16_t d, h; uint32_t v; };
const int NCAP = 500; Cap cap[NCAP]; int nCap = 0, capTrig = -1; float capLastD = -1e9;
void traceTick() {
  if (tracing && (capTrig < 0 || nCap - capTrig < 400) && dist() - capLastD >= 2.0f) {
    capLastD = dist(); Cap &c = cap[nCap % NCAP]; nCap++;
    c.d = (int16_t)(dist() - d0); c.h = (int16_t)(heading() * 10); c.v = 0;
    for (int k = 0; k < 8; k++) c.v |= (uint32_t)min(15, (int)(val[k] * 15.99f)) << (4 * k);
    if (capTrig < 0 && dist() - d0 > 150.0f && (nBlob >= 2 || (seen && bLe - b0s >= 2))) capTrig = nCap - 1;
  }
  if (!tracing || millis() - tTr < 50) return;
  tTr = millis();
  Tr &r = tr[nTr % NTR]; nTr++;              // ring buffer: keeps the LAST 35 s (where the robot runs into trouble)
  r.t = (millis() - tRun0) / 10; r.d = (int32_t)(dist() - d0);   // C3.15: int32 (int16 stuck at 32000 mm after 32 m)
  r.p = seen ? (int8_t)(posC * 10) : 127; r.v = (uint8_t)constrain(curSpd, 0.0f, 100.0f);
  r.ph = phase == '-' ? (junc ? 'J' : 'L') : phase; r.nb = nBlack; r.vl = vlMM; r.h = (int16_t)(heading() * 10); r.m = lineMask;
}
// ---------- gyro-assisted motion ----------
// pivot to the absolute angle 'target'. Stops when reached / overshot / timed out by angle.
bool turnHitLine = false;
float lastTurnErr = 0;   // angle error after the last turn (deg) - logged   // turnTo(..., true): the line crossed the sensor bar during the turn
float turnTo(float target, float maxPct, bool watchLine = false) {
  float e0 = target - heading();
  uint32_t t0 = millis(), tmax = 350 + (uint32_t)(fabsf(e0) * 9);   // 90 deg -> max ~1.2 s
  int hits = 0; turnHitLine = false;
  while (!aborted && millis() - t0 < tmax) {
    update();
    if (watchLine) {                                 // turning and the line (e.g. a bend) is already under the sensor bar -> stop turning, follow the line now
      readLine();
      if (peakV >= 0.6f && maxRun >= 2 && avGateOK() && !(inAvoid && goingBack(100.0f))) { if (++hits >= 3) { turnHitLine = true; logEv('T'); break; } } else hits = 0;   // >= 2 ADJACENT channels make a line
    }
    float e = target - heading();
    // stop EARLY based on the current turn rate (inertia): remaining < expected coast angle = |gz| * turnLead
    if (fabsf(e) < TURN_TOL + fabsf(gz) * cfg.turnLead * 0.001f || e * e0 < 0) break;
    float u = constrain(KP_T * e - KD_T * gz, -maxPct, maxPct);
    if (fabsf(u) < TURN_MIN) u = u >= 0 ? TURN_MIN : -TURN_MIN;
    drive(-u, u);
  }
  brake(); waitMs((uint32_t)max(30.0f, cfg.brkMs)); coast();
  lastTurnErr = heading() - target;
  return lastTurnErr;
}
// Check whether the line just seen is the target line to rejoin after the obstacle.
// Uses the coordinates of the sensor point (not the axle center), because the sensor is
// 115 mm, so using the axle center would drift noticeably while the robot follows an arc.
bool obstacleExitLineTarget() {
  if (!avExitFilter) return true;
  if (!(seen && nBlack >= 1)) return false;
  float h = heading() * DEG_TO_RAD;
  float sx = odoX + cosf(h) * cfg.sensAx;
  float sy = odoY + sinf(h) * cfg.sensAx;
  float rh = avRefH * DEG_TO_RAD;
  // Perpendicular distance from the sensor to the original line.
  float cross = -(sx - avRefX) * sinf(rh) + (sy - avRefY) * cosf(rh);
  return fabsf(cross) <= AV_EXIT_TOL_MM;
}
// drive straight holding heading h. stopLine: stop when the line is seen (after at least minMm). returns true if the line was seen
bool nb2 = false;
float enterPos = 0;   // on which side the line first touches the sensor bar (-: left, +: right) -> decides which way to turn
float lastOver = 0;   // distance the robot SLIDES after the brake command (mm) - logged to tune braking
// HARD BRAKE while driving forward: reverse pulse brkRev% for brkMs, then short-circuit. Measures the slide distance.
void hardStop(float dirSign = 1) {
  float d0s = dist(); uint32_t t0 = millis();
  if (cfg.brkRev > 0) {
    float lastD = dist(); uint32_t tChk = millis();
    while (millis() - t0 < (uint32_t)cfg.brkMs) {
      update(); drive(-dirSign * cfg.brkRev, -dirSign * cfg.brkRev);
      if (millis() - tChk >= 10) {                      // every 10 ms: robot nearly stopped (< 0.5 mm/10 ms) -> stop reversing, avoids backing up
        float dd = dist(); if (millis() - t0 > 15 && (dd - lastD) * dirSign < 0.5f) break; lastD = dd; tChk = millis();
      }
    }
  }
  brake(); t0 = millis();
  while (millis() - t0 < (uint32_t)max(20.0f, cfg.brkMs * 0.6f)) update();
  coast();
  lastOver = fabsf(dist() - d0s);
}
// drive straight holding heading h, SLOW DOWN in the last decMm + HARD BRAKE -> stops at the right distance. stopLine: stop when the line is seen (after minMm).
// needClear: only start reading the line AFTER the sensor bar has left the old line by >= 40 mm (used for the diagonal-out segment)
// flow (v3.5 FLOWING avoid): no slowdown at the end of a segment, NO braking at the end -> blends into the next segment (the robot steers around toward the new heading)
// C3.7 OBSTACLE ON A BEND: avSideGrab = +1 (avoid LEFT on a left bend) / -1 (avoid RIGHT on a right bend) / 0 (none). While driving PARALLEL
//      or DIAGONAL RETURN ('4', '6') when the line already covers HALF the sensor bar on the avoid side (left channels when avoiding left) -> catch the line NOW (no stop - center
//      - turning back to the old heading: on a bend the old heading is already wrong -> robot circles searching for the line).
int8_t avSideGrab = 0; bool avSideHit = false;
bool sideLineNow() { return avSideGrab != 0 && (phase == '4' || phase == '6') && seen && peakV >= 0.6f && maxRun >= 2 && (avSideGrab > 0 ? posC <= 0.5f : posC >= -0.5f); }
bool straightMM(float mm, float h, float pct, bool stopLine, float minMm = 0, bool stopOnLine = true, bool needClear = false, bool flow = false) {
  float s0 = dist(); uint32_t t0 = millis(), tmax = 500 + (uint32_t)(mm * 10);
  int hits = 0; bool cleared = !needClear; float clr0 = s0;
  while (!aborted) {
    update();
    float done = dist() - s0;
    if (done >= mm || millis() - t0 > tmax) break;
    if (avSideGrab != 0 && (phase == '4' || phase == '6')) {   // C3.7: bend -> a line appearing on the avoid side is caught immediately (skips the avscan entry gate)
      if (!stopLine) readLine();
      static int sh = 0; sh = sideLineNow() ? sh + 1 : 0;
      if (sh >= 2) { sh = 0; avSideHit = true; logEv('Y'); return true; }
    }
    // REAL line seen: NEW read + 3 in a row, at least once >= 2 black channels -> BRAKE NOW (do not slide past the line)
    if (stopLine && done >= minMm) {
      readLine();
      if (!cleared) { if (maxRun >= 1 || peakV >= 0.5f) clr0 = dist(); else if (dist() - clr0 >= 40) cleared = true; }
      if (cleared && peakV >= 0.6f && nBlack >= 1 && avGateOK()) {
        if (++hits == 1) { enterPos = posC; nb2 = false; }
        if (maxRun >= 2) nb2 = true;
        if (hits >= 3 && nb2) { logEv('T'); if (stopOnLine) hardStop(); return true; }
      } else hits = 0;
    }
    float base = min(pct, (flow ? 40 : 25) + (millis() - t0) * (wpidOk() ? 0.3f : 0.15f));   // v3.5: with the wheel PID -> accelerates twice as fast
    float remain = mm - done;
    if (!flow && cfg.decMm > 1 && remain < cfg.decMm) base = min(base, 22 + (pct - 22) * remain / cfg.decMm);   // slow down before the stop point
    float corr = constrain(2.0f * (h - heading()) - 0.05f * gz, -base * 0.6f, base * 0.6f);
    driveV(base - corr, base + corr);
  }
  if (!flow) hardStop();
  return false;
}
// line search turn, ANGLE LIMITED: toward dir (+1 = RIGHT) up to a1 deg, then back the other way up to a2 deg (from the start)
// fastDeg > 0: turn FAST (fastPct) for the first fastDeg degrees (the line surely is not there yet, e.g. a 90-degree corner), then slow down to catch the line
// minTurn: in the FIRST turn direction, before turning minTurn deg only accept a line RIGHT IN THE MIDDLE of the sensor bar (|pos| < 0.8).
// (real log: right after starting to turn it already 'saw' the trace ......## at the edge -> stopped turning at the wrong place -> lost the line)
// v3.1 ANTI U-TURN IN CLUSTERS: inside square / hexagon clusters (many junctions close together), when turning to search for the line
// DO NOT accept a line that makes the robot reverse > 150 deg from the cluster entry heading (hRef) -> avoids looping back / running out backward.
// Turned all the way and only that line exists -> only then accept it (never lose the line because of this rule).
float sweepAvoidH = 1e9, backH = 1e9; int nBack = 0;
// C3 ANTI U-TURN (independent of the track shape): the robot remembers the PATH just driven (sensor bar position + heading, every 20 mm, last 1 m).
// When searching for the line: if the sensor bar lies ON the old path and the robot faces OPPOSITE to the driven direction (> 120 deg) = it is turning back and running in reverse
// -> NOT accepted, keep turning to find another branch. Narrow square pulse (2 parallel edges in opposite directions), hairpin, intersecting circles
// still used because the new line is NOT on the old path. Only when it has turned all the way and only the old line remains -> accept it (this rule never stops the robot).
const int NCR = 150; float crX[NCR], crY[NCR], crH[NCR], crD[NCR]; int crN = 0; float crLastD = -1e9;
void crumbReset() { crN = 0; crLastD = -1e9; }
int nRev = 0;   // number of consecutive points running BACKWARD on the old path (while following the line)
bool goingBack(float deg);
void crumbAdd() {
  float d = dist(); if (fabsf(d - crLastD) < 20) return; crLastD = d;
  nRev = goingBack(150.0f) ? nRev + 1 : 0;
  float h = heading(), r = h * DEG_TO_RAD; int i = crN++ % NCR;
  crX[i] = odoX + cosf(r) * cfg.sensAx; crY[i] = odoY + sinf(r) * cfg.sensAx; crH[i] = h; crD[i] = d;
}
bool onOldPath(float lat, float r, float back) {   // is the point on the sensor bar (lateral offset lat channels, + = right) on the path just driven?
  float h = heading() * DEG_TO_RAD, bx = odoX + cosf(h) * cfg.sensAx + sinf(h) * lat * 10.0f, by = odoY + sinf(h) * cfg.sensAx - cosf(h) * lat * 10.0f, d = dist();
  int n = crN < NCR ? crN : NCR;
  for (int k = 0; k < n; k++) { if (d - crD[k] > back || d - crD[k] < 60) continue; float dx = bx - crX[k], dy = by - crY[k]; if (dx * dx + dy * dy < r * r) return true; }
  return false;
}
bool goingBack(float deg = 100.0f) {   // the sensor bar is on the driven path (< 45 mm) and the robot faces OPPOSITE to the direction it passed (> deg degrees)
  if (!cfg.cornerMode) return false;
  float h = heading(), r = h * DEG_TO_RAD, bx = odoX + cosf(r) * cfg.sensAx, by = odoY + sinf(r) * cfg.sensAx, d = dist();
  int n = crN < NCR ? crN : NCR;
  for (int k = 0; k < n; k++) {
    if (fabsf(d - crD[k]) > 3000) continue;
    float dx = bx - crX[k], dy = by - crY[k];
    if (dx * dx + dy * dy < 45.0f * 45.0f && fabsf(wrap180(h - crH[k])) > deg) return true;
  }
  return false;
}   // backH: heading when the U-TURN line was seen (to turn back if no other line exists)
bool sweepFind(int dir, float a1, float a2, int minNb = 1, float maxPos = 2.0f, float pivPct = PIVOT_PCT, float fastDeg = 0, float fastPct = 60, float minTurn = 0) {
  float h0 = heading();
  for (int k = 0; k < 2; k++) {
    int d = k ? -dir : dir; float lim = k ? a2 : a1; uint32_t t0 = millis();
    while (!aborted && millis() - t0 < 2500) {
      update(); readLine();
      // C3.6: inside a square cluster, turning at a corner and the sensor bar sweeps over a line with the nose CLOSE to the main heading (< 50 deg) = exit branch -> accept now
      bool early = k == 0 && (h0 - heading()) * d < minTurn && !(gridG && fabsf(wrap180(heading() - hRefG)) < 50.0f);   // early: only accept a line RIGHT IN THE MIDDLE (no off-center trace at the edge)
      if (seen && fabsf(posC) < (early ? 0.8f : maxPos) && maxRun >= minNb) {
        if ((sweepAvoidH < 1e8 && fabsf(wrap180(heading() - sweepAvoidH)) > 150) || goingBack()) { if (backH > 1e8) { backH = heading(); nBack++; logEv('u'); } }
        else { brake(); waitMs(20); return true; }
      }
      if ((h0 - heading()) * d > lim) break;    // turning RIGHT makes the heading decrease
      float turned = (h0 - heading()) * d;
      float pp = (k == 0 && turned < fastDeg) ? fastPct : pivPct;
      drive(d * pp, -d * pp);
    }
    brake(); waitMs(40);
  }
  coast();
  return false;
}
// driving diagonally, JUST touched the line: slow down, wait for the line to reach the middle of the sensor bar, drive sensAx further (wheel axle onto the line),
// then turn toward side s (+1 = left) until the line is centered. Every step is bounded.
bool catchLine(int s, float hRef, bool useRef = true) {
  phase = 'C'; logEv('C');
  // 1. Drive slowly along the current heading, ONLY count it as caught when a NEW line reaches the middle of the sensor bar
  //    (or the line just passed through the middle). Not really seen -> return false, no blind turn.
  const float hTravel = heading();
  // 0. Inertia may have SLID the robot PAST the line after braking -> if the line is no longer seen, BACK UP SLOWLY to find it (max 120 mm)
  readLine();
  if (!(seen && nBlack >= 1)) {
    float b0 = dist(); uint32_t tb = millis();
    while (!aborted && b0 - dist() < 120 && millis() - tb < 1200) {
      update(); readLine();
      if (seen && nBlack >= 1) break;
      float corr = constrain(2.0f * (hTravel - heading()), -10.0f, 10.0f);
      drive(-25 - corr, -25 + corr);
    }
    brake(); waitMs(30);
    if (seen) logEv('R');
  }
  float s0 = dist(); uint32_t t0 = millis();
  float lastP = 0; bool hadSeen = false, centered = false;
  while (!aborted && dist() - s0 < 100 && millis() - t0 < 900) {
    update(); readLine();
    if (seen && nBlack >= 1) {
      if (fabsf(posC) < 0.6f || (hadSeen && posC * lastP < 0)) { centered = true; break; }
      lastP = posC; hadSeen = true;
    }
    float corr = constrain(2.0f * (hTravel - heading()), -12.0f, 12.0f);
    drive(CATCH_PCT - corr, CATCH_PCT + corr);
  }
  if (!centered) { brake(); waitMs(30); logEv('Z'); return false; }
  // 2. Drive sensAx further: the wheel axle is on the line (the sensor bar is ahead of the axle)
  s0 = dist(); t0 = millis();
  while (!aborted && dist() - s0 < max(0.0f, cfg.sensAx - 10) && millis() - t0 < 1100) {
    update(); float corr = constrain(2.0f * (hTravel - heading()), -12.0f, 12.0f); drive(CATCH_PCT - corr, CATCH_PCT + corr);
  }
  brake(); waitMs(40);
  // 3. Turn BACK TO THE OLD LINE HEADING (hRef) - the turn direction is always known, no guessing. Straight line: at hRef the line is centered.
  //    Meets a line in the middle earlier (curved line) -> stop right away.
  // useRef = false (line caught while arcing - curved line): line heading unknown -> turn AWAY from the side the line touched first,
  // max 100 deg, stop when the line is centered.
  if (!useRef) hRef = heading() + (enterPos > 0.8f ? 100 : enterPos < -0.8f ? -100 : 100 * s);
  float e0 = hRef - heading(); int dir = e0 >= 0 ? 1 : -1;     // +1 = turn LEFT
  // v3.8: the line may have CHANGED DIRECTION (obstacle between 2 bends): reached hRef without seeing the line -> KEEP TURNING the same way up to 110 deg more
  //       (old: stop at hRef then sweep the opposite side -> lost 3-4 s)
  const float over = useRef ? 110.0f : 0.0f;
  t0 = millis();
  while (!aborted && millis() - t0 < 2600) {
    update(); readLine();
    float e = hRef - heading();
    if (seen && maxRun >= 2 && fabsf(posC) < 1.6f && (useRef ? (fabsf(e) < 0.6f * fabsf(e0) + 5 || e * dir < 0) : fabsf(e0 - e) > 4)) { brake(); waitMs(30); return true; }
    if (e * dir <= -over) break;                // past the old heading + over
    float u = (e * dir > 0) ? constrain(KP_T * fabsf(e), TURN_MIN, AV_TURN) : (float)PIVOT_PCT + 4;
    drive(-dir * u, dir * u);
  }
  brake(); waitMs(40); readLine();
  if (seen && nBlack >= 1 && fabsf(posC) < 2.5f) return true;      // curved line: the sensor bar is slightly off but PD can still follow
  // 4. Not seen yet (e.g. curved line): small sweep both ways, angle limited, needs >= 2 black channels
  logEv('Z');
  return sweepFind(s > 0 ? -1 : 1, 60, 120, 2, 2.5f);   // v3.8: sweep first toward where the line continues (old version swept back toward the obstacle)
}
// arc toward the line (line curving away, e.g. obstacle on a bend avoided on the outer side): move forward while turning gradually
bool arcFind(int t, float maxDeg, float maxMm) {   // t = +1 turn RIGHT, -1 turn LEFT
  float h0 = heading(), s0 = dist(); uint32_t t0 = millis(); int hits = 0;
  float base = AV_PCT * 0.7f;
  while (!aborted && dist() - s0 < maxMm && millis() - t0 < 4000) {
    update(); readLine();
    if (peakV >= 0.6f && nBlack >= 1) { if (++hits == 1) { enterPos = posC; nb2 = false; } if (maxRun >= 2) nb2 = true; if (hits >= 3 && nb2) { logEv('T'); return true; } } else hits = 0;
    if ((h0 - heading()) * t > maxDeg) break;
    if (t > 0) drive(base, base * 0.35f); else drive(base * 0.35f, base);
  }
  brake(); waitMs(40); coast();
  return false;
}
// FOLLOW THE LINE WHILE MOVING (no stop): the robot returns diagonally to the line at angle psi, as soon as it touches the line
//  1. keep driving (no stop) until the line reaches the MIDDLE of the sensor bar,
//  2. enter an ARC of radius R = sens * cot(psi/2): the arc brings the wheel axle onto the line just as the robot is parallel to it,
//  3. arc done -> return true, the PD line loop continues right away. No stop - center - turn like the old way.
bool mergeLine(float hRef) {
  phase = 'M'; logEv('M');
  const float v = wpidOk() ? oldPct(CATCH_PCT + 5) : constrain(min(cfg.spdRet, CATCH_PCT + 5), 25.0f, 45.0f);   // C3.24: real ~500 mm/s via the PID
  const float hTravel = heading();
  // 1. keep driving until the line reaches the middle of the sensor bar
  float s0 = dist(); uint32_t t0 = millis(); float lastP = 0; bool had = false, centered = false;
  while (!aborted && dist() - s0 < 110 && millis() - t0 < 900) {
    update(); readLine();
    if (seen && nBlack >= 1) {
      if (fabsf(posC) < 0.6f || (had && posC * lastP < 0)) { centered = true; break; }
      lastP = posC; had = true;
    }
    float corr = constrain(2.0f * (hTravel - heading()), -10.0f, 10.0f);
    driveV(v - corr, v + corr);
  }
  if (!centered) return false;                     // let catchLine (with backing up) handle it
  // 2. arc toward the line heading
  float psi = hRef - heading();                    // + = needs to turn left
  if (fabsf(psi) < 8) return true;
  const int dir = psi > 0 ? 1 : -1;
  float R = cfg.sensAx / tanf(fabsf(psi) * 0.5f * DEG_TO_RAD);
  R = constrain(R, cfg.track * 0.5f, 900.0f);
  const float inner = v * (R - cfg.track * 0.5f) / R, outer = v * (R + cfg.track * 0.5f) / R;
  t0 = millis();
  while (!aborted && millis() - t0 < 2000) {
    update(); readLine();
    float e = hRef - heading();
    if (e * dir <= 3) break;                       // (nearly) parallel to the line
    if (dir > 0) driveV(inner, outer); else driveV(outer, inner);   // C3.24: PID keeps the wheel-speed RATIO (raw PWM near the dead band turned far too tight)
  }
  return true;                                     // PD keeps following (small line error is corrected by PD)
}
// drive straight (mm > 0 forward, < 0 BACKWARD) holding heading h, hard brake at the end. Used to bring the wheel axle to the 90-degree corner point.
void moveMM(float mm, float h, float pct, bool fix = true) {
  if (fabsf(mm) < 5) return;
  const int dir = mm > 0 ? 1 : -1; float s0 = dist(); uint32_t t0 = millis();
  float lastD = s0, v = 0; uint32_t tv = millis();
  while (!aborted && millis() - t0 < 1500) {
    update();
    float done = (dist() - s0) * dir;
    if (millis() - tv >= 15) { float dd = dist(); v = 0.6f * v + 0.4f * fabsf(dd - lastD) * 1000.0f / (millis() - tv); lastD = dd; tv = millis(); }
    // stop EARLY based on speed (braking distance ~ v * 0.04 s) so it does not slide past the target point
    if (done >= fabsf(mm) - 3 - v * 0.04f) break;
    float corr = constrain(2.0f * (h - heading()), -10.0f, 10.0f);
    driveV(dir * pct - corr, dir * pct + corr);
  }
  hardStop(dir);
  // still off by > 15 mm (slid past / not reached) -> correct once at low speed
  float err = mm - (dist() - s0);
  if (fix && fabsf(err) > 15) moveMM(err, h, 25, false);
}
// FOLLOW THE LINE NOW (line seen while avoiding, e.g. obstacle between 2 bends): do not finish the whole avoid sequence,
// switch to low-speed PD line following, keep the line steady ~0.2 s then return to the main line loop.
// v3.7: position of the obstacle just avoided (coordinates) - so the line is not followed BACK toward it
float avObX = 0, avObY = 0; bool avObOn = false; float avObOffD = 0;
float obsRelG() { return wrap180(atan2f(avObY - odoY, avObX - odoX) * 57.2958f - heading()); }
bool facingObs() { return avObOn && hypotf(avObX - odoX, avObY - odoY) < 450 && fabsf(obsRelG()) < 45; }
float vSafeMM(float mm) { return sqrtf(2.0f * ARC_DECEL * max(0.0f, mm - ARC_CLEAR)); }
// C3.25 hint: which side the line will leave the sensor bar (+ = right) when the robot is crossing it at an angle -> pivots the right way at once
bool grabLine(float hint = 0) {
  phase = 'G'; logEv('G');
  const float base = oldPct(32);                   // C3.24: was raw 32 % = just above the dead band of the new gearbox -> crawled
  float lastE = (hint != 0 && fabsf(posC) < 1.5f) ? hint : posC; uint32_t t0 = millis(), tPrev = micros(), tLost = 0, tOk = 0;
  while (!aborted && millis() - t0 < 1200) {
    update(); readLine();
    if (facingObs()) { brake(); waitMs(20); return false; }   // v3.7: following BACKWARD toward the obstacle -> stop, let the caller turn 180
    uint32_t now = micros(); float dt = max(1e-4f, (now - tPrev) * 1e-6f); tPrev = now;
    if (!(seen && nBlack >= 1)) {                   // the line just passed -> turn toward the side it was seen
      if (!tLost) tLost = millis();
      if (millis() - tLost > 500) { brake(); waitMs(30); return false; }
      int sg = lastE > 0 ? 1 : -1;                   // +1 = line on the RIGHT
      drive(sg * (PIVOT_PCT + 10), -sg * (PIVOT_PCT + 10));   // C3.24: +10 % (34 % barely turns the new gearbox)
      continue;
    }
    tLost = 0;
    float e = posC, de = (e - lastE) / dt; lastE = e;
    float u = cfg.kp * e + cfg.kd * de;
    driveV(base + u, base - u);
    if (fabsf(e) < 0.8f) { if (!tOk) tOk = millis(); if (millis() - tOk > 80) return true; } else tOk = 0;   // C3.24: 80 ms (was 150)
  }
  return seen;
}
// ---------- obstacle avoidance ----------
int nObs = 0;
// Result of the latest avoid (written to the log)
float avAng = 0, avHalfW = 0, avLat = 0, avPass = 0, avGap = 0, avLen = 0, avOut = 0, avOverOut = 0, avOverPar = 0, avE1 = 0, avE3 = 0, avE5 = 0;
bool avoidObstacle() {
  nObs++; logEv('O');
  // v3.8 HARD BRAKE + BACK UP TO A SAFE DISTANCE: the VL53 sees the brick late (e.g. 22 cm) while the robot runs fast -> the robot slides
  // reached the brick then turned out and swept into it. Now: reverse-brake, read the VL53 again, closer than 'obs x 0.75' then BACK UP (max 15 cm).
  Serial.printf("[OBSTACLE #%d] seen at %u mm\n", nObs, vlMM);
  // C3.19 ARC avoid: keep rolling (no stop, no pivot) unless the brick is already too close to curve around it
  // C3.24: roll straight into the curve only when the speed can still be shed in the distance seen; too fast (long straight) -> hard brake
  //        first (+ back up if too close) like v3.8, then the same curve from standstill. Log 28/11: 1300 mm/s at 294 mm -> slid into the brick.
  const bool arcMode = cfg.avMode == 3;
  const float v0 = 0.5f * (vL + vR), vSafe = vSafeMM(vlMM);
  const bool arcRoll = arcMode && v0 <= vSafe;
  if (arcRoll) Serial.printf("  ARC avoid: no stop (%.0f mm/s <= safe %.0f)\n", v0, vSafe);
  else {
  if (arcMode) Serial.printf("  ARC avoid: too fast (%.0f mm/s > safe %.0f) -> hard brake first\n", v0, vSafe);
  hardStop(); waitMs(60);
  {
    const float want = cfg.obs * 0.75f; float b0 = dist(), hb = heading(); uint32_t tb = millis();
    if (vlMM < want) logEv('B');
    while (!aborted && vlMM < want && b0 - dist() < 150 && millis() - tb < 1500) {
      update(); float corr = constrain(2.0f * (hb - heading()), -8.0f, 8.0f);
      driveV(-28 - corr, -28 + corr);
    }
    if (b0 - dist() > 3) { hardStop(-1); waitMs(40); }
  }
  }
  const float gap = min((float)vlMM, cfg.obs);    // distance from VL53 to the obstacle face AFTER stopping / backing up
  // v3.7: remember the obstacle POSITION (coordinates) -> after re-catching the line, check whether the robot is running BACK toward it
  avObX = odoX + cosf(heading() * 0.0174533f) * (gap + cfg.sensAx * 0.5f + 50); avObY = odoY + sinf(heading() * 0.0174533f) * (gap + cfg.sensAx * 0.5f + 50); avObOn = true;
  auto obsRel = [&]() -> float { return obsRelG(); };   // obstacle angle relative to the robot heading (+ = left)
  auto obsDist = [&]() -> float { return hypotf(avObX - odoX, avObY - odoY); };
  // line caught but FACING the obstacle (+-75 deg, < 70 cm) = following the track backward -> turn 180 deg and catch again
  auto fixDir = [&]() {
    if (obsDist() > 700 || fabsf(obsRel()) > 75) return;
    logEv('D'); phase = 'D';
    float h2 = heading() + (obsRel() > 0 ? -180.0f : 180.0f);   // turn away from the obstacle
    turnTo(h2, cfg.spdTurn, false);
    readLine(); if (!(seen && maxRun >= 2)) sweepFind(1, 50, 100, 2, 2.0f);
  };
  // C3.7: CURVING (heading change > 15 deg in the last 20 cm, or turning > 40 deg/s) -> avoid TOWARD THE BEND (left bend avoid left), otherwise by 'side'
  float dhC = 0; { float d = dist(), best = 1e9; int n = crN < NCR ? crN : NCR;
    for (int k = 0; k < n; k++) { float e = fabsf(d - 200.0f - crD[k]); if (e < best && e < 60.0f) { best = e; dhC = wrap180(heading() - crH[k]); } } }
  const int curveS = (AVOID_ON_CURVE && (dhC > 15.0f || gzSlow > 40.0f)) ? 1 : (AVOID_ON_CURVE && (dhC < -15.0f || gzSlow < -40.0f)) ? -1 : 0;
  const int s = curveS != 0 ? curveS : (cfg.side ? -1 : 1);   // +1 = avoid on the LEFT, -1 = avoid on the RIGHT
  avSideGrab = curveS; avSideHit = false;
  if (curveS) Serial.printf("  ON A CURVE %s (heading change %+.0f deg / 20 cm) -> avoid %s, grab the line as soon as it appears on that side\n", curveS > 0 ? "LEFT" : "RIGHT", dhC, curveS > 0 ? "LEFT" : "RIGHT");
  const float h = heading();
  // C3.8: heading of the line AHEAD (to sweep for the line in the RIGHT DIRECTION on the first try): line heading h, on a bend offset further toward the bend.
  //  sweepFind: +1 = turn RIGHT -> turn toward the side that brings the heading toward hExp. Old version swept by s (avoid left -> sweep RIGHT first)
  //  -> on a left bend the robot swept ~180 deg to the right, caught the line in reverse and then had to turn back (lost 2-3 s).
  const float hExp = h + curveS * 45.0f;
  auto dirExp = [&]() -> int { return wrap180(hExp - heading()) > 0 ? -1 : 1; };
  const float nose = cfg.sensAx + 10;              // robot nose distance from the wheel axle (VL53 at the nose)
  const float faceX = nose + gap;                  // obstacle face distance from the wheel axle
  const float ab = constrain(cfg.aback, 20.0f, 90.0f), sinB = sinf(ab * DEG_TO_RAD);
  float ang, lat, outDist, passMm, halfW = 0, lenAlong = 0;
  phase = '1'; logEv('1');
  const bool flowAvoid = cfg.avMode >= 2;   // v3.5 FLOWING AVOID: after the first turn-out, NO more stop-and-turn: the robot STEERS continuously into the parallel and return segments (~1 s faster)
  if (arcMode) {
    // C3.19 ARC: brick orientation unknown -> lateral offset 'lat' (215 clears a brick lying ACROSS the line), pass the LONG side + 'pass'
    ang = constrain(cfg.aout, 30.0f, 85.0f); lat = max(cfg.lat, 120.0f); outDist = 0; passMm = OB_LONG + cfg.pass;
  } else if (cfg.avMode >= 1) {
    // ===== MANUAL: drive EXACTLY the distances you set (adjust on the web: 'Diagonal out', 'Parallel') =====
    ang = constrain(cfg.aout, 30.0f, 90.0f);
    turnTo(h + ang * s, cfg.spdTurn);                  // the turn OUT is still a pivot in place (needs the lateral offset right away so it does not sweep into the brick)
    outDist = max(30.0f, cfg.outMm);
    lat = outDist * sinf(ang * DEG_TO_RAD);          // actual lateral offset from the line
    passMm = max(0.0f, cfg.parMm);
  } else {
    // 1. TURN OUT + MEASURE OBSTACLE WIDTH WITH VL53: when the VL53 beam leaves the obstacle edge (distance jumps) -> compute half the obstacle width.
    //    Minimum out angle = aout (45). Wide / close obstacle -> turn further (max 90) until this angle drives diagonally
    //    so the NOSE CORNER does not touch the obstacle face before getting far enough sideways.
    halfW = 75; ang = 0; bool edge = false;
    float aMin = constrain(cfg.aout, 30.0f, 90.0f);
    uint32_t t0 = millis();
    while (!aborted && millis() - t0 < 1800) {
      update();
      ang = (heading() - h) * s;                      // how many degrees turned outward
      if (!edge && (vlMM > gap + 150 || vlMM > 2000) && ang > 3) {
        edge = true;
        float th = max(0.0f, ang - 5) * DEG_TO_RAD;   // subtract ~5 deg because the VL53 beam is a cone
        halfW = constrain(nose * sinf(th) + (faceX - nose * cosf(th)) * tanf(th), 40.0f, 300.0f);
      }
      if (ang >= 88) break;
      if (ang >= aMin && edge) {
        // check: driving diagonally at this angle, at what lateral offset of the wheel axle does the nose corner on the obstacle side touch the obstacle face?
        float a = ang * DEG_TO_RAD;
        float fwdAtFace = faceX - nose * cosf(a) - ROBOT_HALF_W * sinf(a);          // how far the wheel axle moves before the nose corner reaches the obstacle face
        float latAtFace = max(0.0f, fwdAtFace) * tanf(a) + nose * sinf(a) - ROBOT_HALF_W * cosf(a);
        if (latAtFace >= halfW + 20) break;           // this angle is safe
      }
      float target = 90;                              // keep turning (max 90)
      float e = target - ang;
      float u = constrain(KP_T * e, TURN_MIN + 5, cfg.spdTurn);
      drive(-s * u, s * u);
    }
    brake(); waitMs(40); coast();
    ang = constrain((heading() - h) * s, 20.0f, 95.0f);
    if (!edge) halfW = 150;                           // no edge seen (very wide obstacle) -> avoid wide
    lat = max(cfg.lat, halfW + ROBOT_HALF_W + 30);
    const float a = ang * DEG_TO_RAD;
    const float fwdOut = lat / tanf(max(a, 0.35f));   // wheel axle advance while driving diagonally
    // BRICK 205x95: measured width > 150 mm -> brick lies ACROSS the line (only 95 mm along the line), otherwise ALONG it (205 mm).
    // The wheel axle must pass the BACK FACE of the brick by another 'pass' mm (clearing the robot tail) before turning back.
    lenAlong = (!edge || 2 * halfW > 0.5f * (OB_LONG + OB_SHORT)) ? OB_SHORT : OB_LONG;
    passMm = max(60.0f, faceX + lenAlong + cfg.pass - fwdOut);
    outDist = lat / sinf(a);

  }
  avAng = ang; avHalfW = halfW; avLat = lat; avPass = passMm; avGap = gap; avLen = lenAlong; avOut = outDist;
  Serial.printf("  %s | out angle %.0f | diagonal %.0f (lateral %.0f) | parallel %.0f\n", arcMode ? "ARC" : cfg.avMode >= 2 ? "FLOW" : cfg.avMode ? "MANUAL" : "AUTO", ang, outDist, lat, passMm);
  avE1 = cfg.avMode >= 1 ? lastTurnErr : 0;
  // FROM HERE ON READ THE LINE CONTINUOUSLY: line seen (e.g. a line curving back close) -> FOLLOW NOW, do not finish the avoid sequence
  auto grabDone = [&]() -> bool {
    avGate = false;
    bool ok;
    // v3.7 CROSSING THE LINE (e.g. obstacle between 2 bends: the parallel segment crosses the line behind at a right angle): wide black trace >= 4 channels
    // -> move the WHEEL AXLE onto the line then TURN away from the obstacle (the way the track continues). Old: PD follow -> line slips past -> stop.
    if (nBlack >= 4 && (phase == '2' || phase == '4')) {
      logEv('W'); float rel = obsRel();
      moveMM(cfg.sensAx - 10, heading(), 30);
      int dir = rel < 0 ? -1 : 1;                    // obstacle on the RIGHT -> turn LEFT (sweepFind: +1 = RIGHT)
      ok = sweepFind(dir, 130, 130, 2, 1.8f, PIVOT_PCT + 4, 0, 60, 30);
    } else {
      ok = grabLine();
      if (!ok && facingObs()) { fixDir(); ok = grabLine(); }   // following back toward the obstacle -> turn 180 then follow again
      if (!ok) ok = sweepFind(dirExp(), 40, 80, 2, 2.5f);   // C3.8: sweep first toward where the line continues
    }
    if (ok) fixDir();
    phase = '-'; if (ok) { logEv('o'); return true; } return false; };
  // C2 (avscan 0): narrow track -> do NOT read the line while GOING OUT / PARALLEL (avoid catching the border line), only when RETURNING
  const bool scanOut = cfg.avScan != 0;
  avGRH = h; avGRX = odoX + cosf(h * DEG_TO_RAD) * cfg.sensAx; avGRY = odoY + sinf(h * DEG_TO_RAD) * cfg.sensAx; avGS = s; avGate = false;
  bool found = false; int sA = s;   // C3.23: side actually used (ARC may switch side once when its lane is blocked)
  if (arcMode) {
    // ===== C3.19 ARC AVOID (avmode 3): the avmode 2 plan (out -> parallel -> back to the line) as ONE smooth curve, no stops =====
    // Odometry + gyro in the frame of the old line: x = forward along it, y = sideways toward the avoid side (wheel axle).
    // OUT: steer at 'aout' until half of 'lat', then curve onto the offset 'lat' so it is reached before the nose gets to the brick face.
    // PARALLEL: hold 'lat' until the axle is past the back face (long side 205) + 'pass'. BACK: steer at 'aback' toward the line, merge without stopping.
    const float x0 = odoX, y0 = odoY, c0 = cosf(h * DEG_TO_RAD), s0 = sinf(h * DEG_TO_RAD);
    auto ax = [&]() { return c0 * (odoX - x0) + s0 * (odoY - y0); };
    auto ay = [&]() { return sA * (-s0 * (odoX - x0) + c0 * (odoY - y0)); };
    const float L = lat, xNeed = faceX - nose - ARC_FACE, xRet = faceX + passMm;
    auto steer = [&](float psi, float v) {          // heading h + s*psi (deg); rolling turn, the inner wheel may run slightly backward
      float e = wrap180(h + sA * psi - heading());
      float corr = constrain(ARC_KH * e - ARC_KD * gz, -1.35f * v, 1.35f * v);
      driveV(v - corr, v + corr);
    };
    // C3.21: seen at straight speed (log: 1250 mm/s) -> the out curve ran wide (63 deg, 300 mm forward) -> short reverse brake down to the out speed first
    { const float vOut = cfg.spdOut * cfg.vfull / 100.0f; uint32_t tb = millis();
      while (!aborted && 0.5f * (vL + vR) > vOut + 100 && millis() - tb < 120) { update(); drive(-cfg.brkRev, -cfg.brkRev); }
      if (millis() - tb > 5) Serial.printf("  ARC: braked %lu ms to %.0f mm/s\n", (unsigned long)(millis() - tb), 0.5f * (vL + vR)); }
    uint32_t tA = millis(); bool laneFail = false;
    // C3.23 LANE BLOCKED (track edge / wall / object beside the line): while driving out or parallel, an object < ARC_LANE mm ahead whose hit point
    // lies beside the brick (outside its half-width) but inside the lane the robot sweeps -> stop, back up, avoid on the other side.
    auto laneBlocked = [&](bool par) -> bool {
      (void)par;
      if (!vlOk || vlMM <= 20 || vlMM >= ARC_LANE) return false;
      float rel = wrap180(heading() - h) * sA;      // + = pointing further out
      float hitY = ay() + (nose + vlMM) * sinf(rel * DEG_TO_RAD);   // sideways position of the object the VL53 sees
      return hitY > OB_LONG * 0.5f + 20 && hitY < L + ROBOT_HALF_W + 20;   // beside the brick AND inside the lane the robot will sweep
    };
    const float catchV = cfg.vfull > 100 ? CATCH_PCT * 1158.0f / cfg.vfull : CATCH_PCT;   // C3.20: same REAL catch speed as on the old gearbox (~440 mm/s)
    for (int tryN = 0; tryN < 2; tryN++) {
      bool blocked = false, stall = false; int nRec = 0; float dProg = dist(); uint32_t tProg = millis();
      phase = '2';
      while (!aborted && millis() - tA < 1500) {      // OUT
        update();
        float x = ax(), y = ay();
        if (y >= L - 15 || x >= xRet) break;
        if (laneBlocked(false)) { blocked = true; break; }
        // C3.24: nose < 70 mm from something, or pushing without moving (log: stuck 1.5 s against the brick at -52 deg) -> back up, turn further out
        if (millis() - tProg > 250) { stall = dist() - dProg < 8; dProg = dist(); tProg = millis(); }
        if ((vlOk && vlMM > 20 && vlMM < 70) || stall) {
          if (++nRec > 3) { blocked = true; break; }
          logEv('B'); hardStop(); stall = false;
          moveMM(-60, heading(), 30);
          turnTo(heading() + sA * 20, cfg.spdTurn);
          dProg = dist(); tProg = millis(); continue;
        }
        float psi = atan2f(L - y, max(ARC_LOOK, xNeed - x)) * 57.2958f;
        if (y < 0.5f * L) psi = max(psi, ang);
        steer(constrain(psi, 0.0f, 85.0f), cfg.spdOut);
      }
      if (!blocked) { phase = '4'; logEv('4'); }
      while (!blocked && !aborted && millis() - tA < 4000) {      // PARALLEL
        update();
        if (avSideGrab != 0) { readLine(); static int sh = 0; sh = sideLineNow() ? sh + 1 : 0; if (sh >= 2) { sh = 0; avSideHit = true; logEv('Y'); return grabDone(); } }
        if (ax() >= xRet) break;
        if (laneBlocked(true)) { blocked = true; break; }
        if (millis() - tProg > 300) { if (dist() - dProg < 8) { blocked = true; break; } dProg = dist(); tProg = millis(); }   // C3.24: stuck while parallel
        steer(constrain(atan2f(L - ay(), ARC_LOOK) * 57.2958f, -35.0f, 35.0f), cfg.spdPar);
      }
      if (!blocked) break;
      // C3.23: lane blocked -> stop, face the old heading, back up behind the start point, then the same curve on the OTHER side
      logEv('B'); hardStop();
      Serial.printf("  ARC: lane blocked (VL53 %u mm) -> %s\n", vlMM, tryN ? "both sides blocked" : "other side");
      if (tryN) { laneFail = true; break; }
      turnTo(h, cfg.spdTurn);
      moveMM(-(ax() + 40), h, 30);
      sA = -sA; avGS = sA; avSideGrab = 0; tA = millis();
    }
    if (!scanOut) { avGLim = avCross() * 0.6f; avGate = true; }   // only accept a line once the sensor bar is back inside the offset
    phase = '6'; logEv('5'); int hits = 0;
    while (!laneFail && !aborted && millis() - tA < 6000) {      // BACK to the line
      update(); readLine();
      if (vlOk && vlMM > 20 && vlMM < 90) { logEv('B'); hardStop(); break; }   // C3.23: something right ahead -> do not push into it
      if (avSideGrab != 0) { static int sh2 = 0; sh2 = sideLineNow() ? sh2 + 1 : 0; if (sh2 >= 2) { sh2 = 0; avSideHit = true; logEv('Y'); found = true; break; } }
      if (peakV >= 0.6f && nBlack >= 1 && avGateOK()) {
        if (++hits == 1) { enterPos = posC; nb2 = false; }
        if (maxRun >= 2) nb2 = true;
        if (hits >= 2 && nb2) { logEv('T'); found = true; break; }   // C3.25: 2 reads (was 3) -> caught while the line is still near the bar edge
      } else hits = 0;
      float bar = avCross();                        // C3.20: SENSOR BAR offset from the line (was the axle: the bar is ~100 mm closer at 65 deg)
      if (bar < -0.5f * L) break;                   // C3.21: well past the line (odometry can be ~50-100 mm off after a fast curve: was -45)
      // C3.21: near the line flatten the approach to ARC_CATCH_DEG -> the bar stays over the 2 cm line ~2x longer (photoresistors are slow)
      if (bar < ARC_SLOW) steer(-min(ab, ARC_CATCH_DEG), catchV); else steer(-ab, cfg.spdRet);
    }
  } else {
    phase = '2'; bool hit = straightMM(outDist, h + ang * s, cfg.spdOut, scanOut, 0, false, true, flowAvoid); avOverOut = flowAvoid ? 0 : lastOver;
    if (hit) return grabDone();
    if (!flowAvoid) { phase = '3'; turnTo(h, cfg.spdTurn, scanOut); avE3 = lastTurnErr;
      if (turnHitLine) return grabDone(); }
    // 4. parallel. Only reads the line in the SECOND HALF (wheel axle past the obstacle front) - to catch the line when the obstacle is on a bend
    //    a curved line crossing the path. Needs >= 2 black channels, 3 reads in a row -> no false catch on the obstacle edge / dirt.
    // only read the line once the sensor bar is level with the obstacle front (before that it only sees the obstacle edge)
    phase = '4'; logEv('4'); found = straightMM(passMm, h, cfg.spdPar, scanOut, 0, false, true, flowAvoid); avOverPar = flowAvoid ? 0 : lastOver;
    if (found) return grabDone();
    if (!scanOut) { avGLim = avCross() * 0.6f; avGate = true; }   // from here on only accept a line once the sensor bar is >= 40% back inside the offset
    if (!flowAvoid) { phase = '5'; logEv('5'); turnTo(h - ab * s, cfg.spdTurn, true);   // turning and the line is seen -> follow now
      found = turnHitLine; avE5 = lastTurnErr;
      if (found) return grabDone(); }
    const float need = lat / sinB;
    const float retMin = 0;                          // read the line NOW (a curved line may already be ahead of the sensor bar after the turn)
    // 6. diagonal back: drive fast until the sensor bar is near the line, then SLOW DOWN (CATCH_PCT) so the line is not crossed too far
    const float fastLen = max(0.0f, need - cfg.sensAx - 50.0f / sinB);   // slow down when the sensor bar is ~5 cm from the line
    // touches the line -> NO stop, follow the line now (mergeLine). Only if that fails use the old way (catchLine).
    const bool stopL = cfg.retMode == 1;              // STOP-ALIGN-TURN: brake as soon as the line is touched
    if (!found) { phase = '6'; found = straightMM(fastLen, h - ab * s, cfg.spdRet, true, retMin, stopL); }
    if (!found) found = straightMM(1.8f * need - fastLen, h - ab * s, min(cfg.spdRet, CATCH_PCT), true, 0, stopL);
  }
  if (found && avSideHit) { avSideGrab = 0; return grabDone(); }
  if (found && arcMode) {
    // C3.25 CATCH AND GO: the line is under the bar -> follow it NOW with PD (no merge arc: on the 28/11 log the planned arc ran ~2x wider and
    // lost the line). The robot crosses at ~40-60 deg, so the line leaves the bar on the side it must turn to -> pivot that way at once.
    bool okG = grabLine(-sA * 1.5f);
    if (!okG && facingObs()) { fixDir(); okG = grabLine(); }
    if (!okG) okG = sweepFind(sA > 0 ? -1 : 1, 110, 170, 2, 2.5f, PIVOT_PCT + 12);
    if (okG) { fixDir(); phase = '-'; logEv('o'); return true; }
  }   // C3.7: on a bend - follow the line immediately on the bend side
  bool arc = false, ok = false; avGate = false;
  if (!found && arcMode) {                         // C3.20: ARC passed the line without seeing it -> turn back toward the old heading (the line is behind the bar)
    logEv('X'); ok = sweepFind(sA > 0 ? -1 : 1, 110, 170, 2, 2.5f, PIVOT_PCT + 12);   // C3.21: stronger pivot (34 % stalled on the new gearbox)
    if (ok) { fixDir(); phase = '-'; logEv('o'); return true; }
  }
  if (!found) { phase = '7'; logEv('7'); found = arcFind(sA, 150, 900); arc = true; }
  if (found && !arc && cfg.retMode == 0) ok = mergeLine(h);   // C3.22: ARC follows 'retm' too (1 = stop-align-turn, catches the line even after overshooting)
  if (found && !ok) ok = catchLine(sA, h, !arc);
  if (!ok) { logEv('X'); ok = sweepFind(dirExp(), 130, 170, 2, 2.5f); }   // C3.8: sweep first toward where the line continues (heading h / bend side)   // v3.7: wider sweep (the line may have changed direction - obstacle between 2 bends); if following backward, fixDir turns back
  if (ok) fixDir();
  phase = '-';
  if (ok) { logEv('o'); return true; }
  Serial.println("!! line not recovered after avoidance");
  return false;
}
// ---------- screen hooks (defined in the UI section) ----------
void uiBusy(const char* title, const char* l1, const char* l2);
void wifiStop();
void logToSerial();
void uiGo();
void uiRun(float tS, float dMm, float spd, const char* st);
void uiRunStep();
float readVbat();
int calGood = -1;
struct Res { bool has; char why[24]; float t, d, v; int obs, lost, junc; } res = {false, "", 0, 0, 0, 0, 0, 0};
void uiAfterRun();
void uiAfterCal();
// ---------- calibration (with self-learned encoder + gyro signs) ----------
void calibrate() {
  wifiStop();
  Serial.println("\n=== CALIBRATION: robot sweeps over the line for 5 s (sensor bar must be ON THE LINE) ===");
  uiBusy("CALIBRATE", "Robot turning", "over the line ~5 s");
  beep(1, 150); delay(800);
  uint16_t mn[8], mx[8], pv[8]; double sum[8] = {0}; uint32_t n = 0;
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; pv[i] = 0xFFFF; }
  int32_t l0 = rawL, r0 = rawR; double gsum = 0; int gn = 0; bool first = true;
  const float seq[5][2] = {{1, 350}, {-1, 700}, {1, 700}, {-1, 700}, {1, 350}};   // +1 = turn RIGHT
  for (auto &st : seq) {
    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)st[1]) {
      drive(st[0] * 28, -st[0] * 28);
      for (int i = 0; i < 8; i++) { uint8_t pn = LPIN[cfg.flip ? 7 - i : i]; uint32_t a = 0; for (int k = 0; k < 4; k++) a += analogRead(pn);
        uint16_t r = a / 4; sum[i] += r;   // mean of 4 samples, same as while running
        // v3.2: min/max must hold for >= 2 CONSECUTIVE reads -> one noise spike (e.g. at motor start) does not corrupt the level / invert black-white
        if (pv[i] != 0xFFFF) { mn[i] = min(mn[i], max(r, pv[i])); mx[i] = max(mx[i], min(r, pv[i])); }
        pv[i] = r; }
      if (first && mpuOk) { float g; if (readGz(g)) { gsum += g - bgz; gn++; } }
      n++; delay(2);
    }
    if (first) {   // step 1: turn RIGHT -> left wheel forward, right wheel backward, gyro (CCW positive) must be NEGATIVE
      int32_t dl = rawL - l0, dr = rawR - r0;
      if (labs(dl) > 30) cfg.sgnL = dl > 0 ? 1 : -1; else Serial.println("  !! LEFT encoder not counting");
      if (labs(dr) > 30) cfg.sgnR = dr < 0 ? 1 : -1; else Serial.println("  !! RIGHT encoder not counting");
      if (gn > 20) cfg.gsgn = (gsum / gn) < 0 ? 1 : -1;
      Serial.printf("  encoder sign L %+d R %+d | gyro sign %+d\n", cfg.sgnL, cfg.sgnR, cfg.gsgn);
      first = false;
    }
  }
  brake(); delay(150); coast();
  int good = 0, highVotes = 0; uint8_t badMask = 0;
  for (int i = 0; i < 8; i++) {
    float mean = sum[i] / max<uint32_t>(n, 1);
    if (mx[i] - mn[i] > 300) good++;
    if (mean < (mn[i] + mx[i]) / 2.0f) highVotes++;
    bool bad = mx[i] - mn[i] < 150 || mn[i] >= 4090 || mx[i] <= 5;   // v2.9.5: channel value NOT changing (loose wire / stuck at 4095) -> do NOT save wrong numbers
    bool oldOk = cfg.calOk && cfg.lmax[i] - cfg.lmin[i] >= 150;
    if (!bad) { cfg.lmin[i] = mn[i]; cfg.lmax[i] = mx[i]; }
    Serial.printf("  D%d: min %4u max %4u diff %4u %s\n", i + 1, mn[i], mx[i], mx[i] - mn[i],
                  bad ? (oldOk ? "<-- ERROR: value NOT changing (check wire) - KEEP old value" : "<-- ERROR: value NOT changing (check wire / channel)") : mx[i] - mn[i] > 300 ? "" : "<-- WEAK");
    if (bad) badMask |= 1 << i;
  }
  cfg.blackHigh = highVotes >= 4;
  cfg.calOk = good >= 7;
  if (badMask) { Serial.print("!! FAULTY CHANNELS DURING CALIBRATION:"); for (int i = 0; i < 8; i++) if (badMask >> i & 1) Serial.printf(" D%d", i + 1);
    Serial.println("  -> fix the wiring then recalibrate, or disable the channel with the mask command"); beep(3, 120); }
  Serial.printf("=> %d/8 channels good, black = %s -> %s\n", good, cfg.blackHigh ? "HIGH" : "LOW", cfg.calOk ? "PASS, saved" : "FAIL");
  cfgSave(); calGood = good;
  uiAfterCal();
  if (cfg.calOk) beep(2); else beep(1, 600);
}
// =====================================================================
//   v3.0 TRACK MAP: DEMO (record) -> FAST (run by the map)
// =====================================================================
// DEMO: normal (safe) run, every 25 mm record the heading change -> track "shape"; every line loss records
//       position + ACTUAL side turned (after the search turn finishes). Reaching the finish -> saved to flash (kept after power-off).
// FAST: still follows the line / avoids obstacles normally, but:
//   - segments the map says are STRAIGHT (no heading change in the next 200-250 mm) -> run at 'vfast' %
//   - approaching a curve / corner / obstacle (look ahead 'mpre' mm) -> slow down EARLY (no overshoot)
//   - line lost at a known place -> turn IMMEDIATELY to the side taken in the demo (no wrong-side turn), dashed line -> keep straight
//   - at every corner / obstacle RE-ALIGN the position to the map (encoder error does not accumulate)
const int MAP_STEP = 25, NPROF = 1800, NNODE = 220;       // 25 mm/sample -> max 45 m
struct MNode { uint16_t d2; int16_t h; char t; int8_t s; };   // d2 = position / 2 (mm), h = robot heading at that point (deg), t: L line lost, g dashed line, O obstacle, o avoid done, F finish; s: +1 right -1 left
int8_t prof[NPROF]; int nProf = 0; MNode mnode[NNODE]; int nNode = 0;
bool mapOk = false; float mapT = 0, mapLen = 0;
uint8_t runMode = 0;                                      // 0 NORMAL | 1 DEMO (record map) | 2 FAST (follow map)
const char* MODEN[3] = {"NORMAL", "DEMO", "FAST"};
float mapOff = 0, lastProfH = 0; int kNode = 0, mapBad = 0; bool recOn = false, mapUse = false;
Preferences pmap;
void mapSave() {
  pmap.begin("bando", false);   // NVS name kept from the first firmware so data already stored on the robot survives updates
  pmap.putBytes("p", prof, nProf); pmap.putBytes("n", mnode, nNode * sizeof(MNode));
  float h[4] = {(float)nProf, (float)nNode, mapT, mapLen}; pmap.putBytes("h", h, sizeof h);
  pmap.end();
}
void mapLoad() {
  pmap.begin("bando", true);   // NVS name kept from the first firmware so data already stored on the robot survives updates
  float h[4] = {0, 0, 0, 0};
  if (pmap.getBytes("h", h, sizeof h) == sizeof h && h[0] > 0 && h[0] <= NPROF && h[1] >= 0 && h[1] <= NNODE) {
    nProf = (int)h[0]; nNode = (int)h[1]; mapT = h[2]; mapLen = h[3];
    mapOk = pmap.getBytes("p", prof, nProf) == (size_t)nProf && pmap.getBytes("n", mnode, nNode * sizeof(MNode)) == nNode * sizeof(MNode);
  }
  pmap.end();
  if (!mapOk) { nProf = nNode = 0; }
}
void mapClear() { nProf = nNode = 0; mapOk = false; mapSave(); }
float mapPos() { return dist() - d0 - mapOff; }
void mapNode(char t, int8_t sd, float at = -1, float hAt = 1e9) {   // at: position (mm from the start), hAt: heading - default = now
  if (!recOn || nNode >= NNODE) return;
  float md = max(0.0f, at >= 0 ? at : dist() - d0); if (hAt > 1e8) hAt = heading();
  mnode[nNode].d2 = (uint16_t)min(65535.0f, md / 2); mnode[nNode].h = (int16_t)constrain(lroundf(hAt), -32000L, 32000L); mnode[nNode].t = t; mnode[nNode].s = sd; nNode++;
}
void mapRecTick() {                                       // called every loop during DEMO: record the heading change every 25 mm
  if (!recOn) return;
  float md = dist() - d0;
  while (nProf < NPROF && md >= (nProf + 1) * MAP_STEP) {
    float dh = heading() - lastProfH; lastProfH = heading();
    prof[nProf++] = (int8_t)constrain(lroundf(dh), -127L, 127L);
  }
}
// total |heading change| (deg) within 'look' mm ahead of map position md
float mapTurnAhead(float md, float look) {
  int a = max(0, (int)(md / MAP_STEP)), b = min(nProf, (int)((md + look) / MAP_STEP) + 1); float sum = 0;
  for (int i = a; i < b; i++) sum += abs(prof[i]);
  return sum;
}
// map point (type t1/t2) closest to position md within +-win mm, from kNode onward (skips points already passed)
int mapFind(float md, const char* types, float win, float h = 1e9) {   // h: current heading -> the point must have the SAME HEADING (+-50 deg)
  while (kNode < nNode && mnode[kNode].d2 * 2.0f < md - win) kNode++;
  int best = -1; float bd = win;                          // pick the CLOSEST point (not the first one) -> no confusion with the neighboring corner
  for (int j = kNode; j < nNode && j < kNode + 8; j++) {
    float nd = mnode[j].d2 * 2.0f;
    if (nd > md + win) break;
    if (h < 1e8 && fabsf(h - mnode[j].h) > 50) continue;
    if (strchr(types, mnode[j].t) && fabsf(nd - md) <= bd) { bd = fabsf(nd - md); best = j; }
  }
  return best;
}
void mapSync(int j) { if (j < 0) return; mapOff += mapPos() - mnode[j].d2 * 2.0f; kNode = j + 1; }
bool mapNear(float md, char t, float ahead) {             // is there a point of type t in [md, md+ahead]?
  for (int j = kNode; j < nNode; j++) { float nd = mnode[j].d2 * 2.0f; if (nd > md + ahead) break; if (mnode[j].t == t && nd >= md - 20) return true; }
  return false;
}
void mapPrint() {
  Serial.printf("MAP: %s | %d heading samples (%.1f m) | %d points | demo %.2f s\n", mapOk ? "YES" : "NONE", nProf, nProf * MAP_STEP / 1000.0f, nNode, mapT);
  for (int j = 0; j < nNode; j++) Serial.printf("  %c %6.0f mm heading %+5d %s\n", mnode[j].t, mnode[j].d2 * 2.0f, mnode[j].h, mnode[j].s > 0 ? "RIGHT" : mnode[j].s < 0 ? "LEFT" : "");
}
// ---------- CONTEST RUN ----------
void runCourse() {
  wifiStop();
  if (!cfg.calOk) { Serial.println("!! NOT CALIBRATED - press SET first"); uiBusy("ERROR", "NOT CALIBRATED", "Page 5 -> SET"); beep(1, 600); delay(1200); return; }
  Serial.printf("\n=== RUN  sp %d vmax %d kp %.1f kd %.2f | MPU %s VL53 %s ===\n",
                cfg.base, cfg.vmax, cfg.kp, cfg.kd, mpuOk ? "OK" : "NO", vlOk ? "OK" : "NO");
  uiBusy("GET READY", "KEEP ROBOT STILL", "gyro calibrating...");
  beep(1, 120);
  if (mpuOk) calibrateGyro(); else delay(1000);
  uiGo();
  syncButtons(); aborted = false;
  mapOff = 0; kNode = 0; mapBad = 0; lastProfH = 0; recOn = runMode == 1; mapUse = runMode == 2 && mapOk;
  if (recOn) { nProf = nNode = 0; mapOk = false; }
  if (cfg.wpid && !wpidOk()) Serial.println("!! Wheel speed PID NOT active: type 'wcal' (measure wheel speed) once");
  Serial.printf("MODE: %s%s\n", MODEN[runMode], runMode == 2 && !mapOk ? " - NO MAP -> NORMAL run" : "");
  runActive = true; avObOn = false; avExitFilter = false; avGate = false; adc2Zero = 0; stuckEv = 0; nBack = 0; for (int i = 0; i < 8; i++) sStuck[i] = false; nEv = 0; cntL = 0; cntJ = 0; nObs = 0; vlOn = true; spinStop = false; spinAcc = 0; tSpin = millis(); tRun0 = millis(); d0 = dist(); nTr = 0; tracing = true; nCap = 0; capTrig = -1; capLastD = -1e9; phase = '-'; vlMinRun = 8190; vlN = 0;
  float lastErr = 0, lastPos = 0, spd = cfg.base, finStart = -1, hFin = 0;
  // FIX-001: the filter belongs to the current line-following session, do not keep samples from before the turn/search.
  float smoothPos = 0, filtDe = 0;
  uint8_t lineCtl = 0;   // 0: needs init, 1: nearJ, 2: filtered line following
  uint32_t tPrev = micros(), tStraight = millis(), tP = 0, loops = 0;
  int hint = 0; bool wasJunc = false; const char* why = "STOP BUTTON";
  float hRef = heading(); jPref = cfg.pref; lastProfH = heading(); crumbReset(); gClus = false;
  uint8_t wrongWayFixes = 0;    // number of times a reversed line catch was corrected in this run
  float jD[4] = {-1e9, -1e9, -1e9, -1e9}; int jI = 0;   // v3.1: positions of the last 4 junctions
  auto inCluster = [&]() { int n = 0; for (float x : jD) if (dist() - x < 450) n++; return n >= 2; };
  // C3.1 REAL JUNCTION (T / 4-way: 2 separate black traces, or black covering the WHOLE bar) - L corners / square pulses (flare on one side only) DO NOT count.
  // >= 2 real junctions within 50 cm = inside a SQUARE CLUSTER -> only then use the old handling (cluster 1).
  float jT[4] = {-1e9, -1e9, -1e9, -1e9}; int jTI = 0; bool jTwo = false; float jS0 = 0;
  // C3.3 'GRID' MODE IN SQUARE CLUSTERS (cluster 1): entering a cluster (perpendicular T - black covering the WHOLE bar) -> slow down, at EVERY junction:
  // knows whether the junction has a LEFT / RIGHT branch (black trace flaring to the edge) and whether the STRAIGHT line continues (20 mm after the junction) -> pick the branch with the heading CLOSEST
  // main heading (hRef) (equal -> pref). Side branch -> bring the WHEEL AXLE to the junction CENTER then turn into the branch (log 'Q').
  // (before: the PID only passed the side branch for ~0.1 s -> missed the branch -> robot circled / ran backward depending on the cluster offset up or down)
  bool gridOn = false; float gridJ = -1e9, gridCand = -1e9; uint8_t clusterPref = 0;
  float cxArmR = -1e9, cxArmL = -1e9, cxSlide = 0, cxPendD = 0, cxHoldD = 0; int cxPend = 0;   // C3.31 circle entry: where a separate trace appeared at D8 / D1, its slide to the line (mm)
  float cxCandD = -1e9;   // C3.27 circle cluster candidate (distance where the split was seen)
  int qMB = 0; float hexW = -1e9, hRefHex = 0, dCv = -1e9, cvH0 = 0, dCircle = -1e9;   // C3.11 hexagon cluster: hexW = position of the last Y-junction, hRefHex = locked main heading
  float hexD = -1e9, sqCandD = -1e9; bool qPend = false; float qL = 0, qR = 0, qLc = 0, qD0 = 0, qD1 = 0, qH0 = 0; bool qFull = false; bool qFast = false;
  auto inSqCluster = [&]() { int n = 0; for (float x : jT) if (dist() - x < 500) n++; return n >= 2; };   // >= 2 junctions within 45 cm = inside a CLUSTER
  float lastRD = -1e9, lastLD = -1e9, lastBD = -1e9, lastShapeD = -1e9, lastRecD = -1e9;   // lastBD: both edges AT ONCE (real T-junction)   // v2.9.1: position (encoder) when a branch last touched the RIGHT / LEFT edge of the sensor bar
  int cornerSide = 0; uint32_t tCorner = 0, tCentered = millis(); float dCornerBar = 0, lastSeenD = dist();   // 90-degree corner: side, when seen, sensor bar position at the corner
  float centeredD = -1e9;
  uint8_t nR = 0, nL = 0; bool heldEdgeLogged = false;
  // C3.10 START INSIDE THE BLACK BOX: the sensor bar sees almost all black (>= 6 channels) -> do NOT follow the line (the wide black trace makes PD wobble left and right),
  // drive STRAIGHT holding the gyro heading, ramp up, until the sensor bar leaves the box (<= 4 black channels, stable), then drive START_GO mm more before following the line.
  // Line right after the box off-center (|pos| > 1, robot placed slightly skewed / sharp bend) or no line seen -> stop early and let PD / line search handle it.
  readLine();
  if (nBlack >= 6) {
    phase = 'P'; logEv('P');
    const float h0 = heading(); const float s0 = dist(); float dOut = -1e9; uint32_t t0 = millis();
    while (!aborted && millis() - t0 < 2500 && dist() - s0 < START_MAX) {
      update(); readLine();
      if (nBlack >= 6) dOut = -1e9;                                  // still inside the black box
      else if (dOut < -1e8) { if (nBlack <= 4) dOut = dist(); }      // just left the box edge
      else if (dist() - dOut >= START_GO || (seen && nBlack >= 1 && fabsf(posC) > 1.0f) || (!seen && dist() - dOut >= 30)) break;
      float b = min((float)cfg.base, 35.0f + (millis() - t0) * 0.08f);
      float corr = constrain(2.0f * (h0 - heading()) - 0.05f * gz, -b * 0.5f, b * 0.5f);
      driveV(b - corr, b + corr);
    }
    phase = '-'; tPrev = micros(); tStraight = millis(); lastErr = 0; hRef = heading();
  }
  while (!aborted) {
    update(); readLine(); loops++; mapRecTick();
    uint32_t now = micros(); float dt = max(1e-4f, (now - tPrev) * 1e-6f); tPrev = now;
    float d = dist() - d0;
    // FINISH (v2.9.6): wide black box. From when the sensor bar touches the box edge: HOLD HEADING straight (no PD -> not pulled left),
    // black continuously for FIN_LEN mm = definitely the finish box -> drive on until 'fgo' mm from the box edge then brake -> robot sits in the middle of the box.
    // If the black ends before FIN_LEN (just a crossbar / T-junction) -> normal line following.
    if (d > cfg.finMin && (nBlack >= 7 || (finStart >= 0 && nBlack >= 5))) {
      if (finStart < 0) { finStart = d; hFin = heading(); }
      if (d - finStart >= FIN_LEN) {
        why = "TO FINISH"; logEv('F'); phase = 'F'; mapNode('F', 0);
        float rest = cfg.finGo - (d - finStart);
        if (rest > 5) moveMM(rest, hFin, min(max(spd, 30.0f), 45.0f), false); else hardStop();
        break;
      }
      float b = min(spd, 45.0f);
      float corr = constrain(2.0f * (hFin - heading()) - 0.05f * gz, -b * 0.6f, b * 0.6f);
      driveV(b - corr, b + corr);
      continue;
    } else finStart = -1;
    // OBSTACLE
    if (vlOn && vlOk && seen && !junc && vlMM > 20 && vlMM < cfg.obs) {
      inAvoid = true; jPref = cfg.pref; gClus = false;            // while avoiding: follow the line on a fixed preferred side, as in v2.8
      if (mapUse) mapSync(mapFind(mapPos(), "O", 350));
      mapNode('O', 0);
      bool okA = avoidObstacle(); avSideGrab = 0;
      mapNode('o', 0); if (mapUse) mapSync(mapFind(mapPos(), "o", 400));
      inAvoid = false; hRef = heading(); tStraight = millis(); cornerSide = 0; lastRD = lastLD = lastBD = -1e9; avObOffD = dist() + 400;
      if (!okA) { why = "OBSTACLE AVOID FAILED"; break; }
      if (cfg.nobs && nObs >= cfg.nobs) vlOn = false;   // nobs 0 = unlimited
      lineCtl = 0; centeredD = -1e9; nR = nL = 0;
      lastErr = 0; tPrev = micros(); spd = min(cfg.base * 0.7f, 45.0f); continue;
    }
    // v3.7: 40 cm after the avoid: if running TOWARD the obstacle (following backward) -> turn 180 and catch the line again
    if (avObOn) {
      if (dist() > avObOffD) avObOn = false;
      else if (seen && facingObs()) {
        logEv('D'); phase = 'D'; float h2 = heading() + (obsRelG() > 0 ? -180.0f : 180.0f);
        turnTo(h2, cfg.spdTurn, false); readLine(); if (!(seen && maxRun >= 2)) sweepFind(1, 50, 100, 2, 2.0f);
        lineCtl = 0; centeredD = -1e9; nR = nL = 0;
        phase = '-'; lastErr = 0; tPrev = micros(); spd = min(spd, cfg.cornerSpd); avObOn = false; continue;
      }
    }
    // remember the branch/corner side for use when the line is lost
    if (junc) {
      static uint32_t tJ = 0;
      if (!wasJunc && millis() - tJ > 300) { logEv('J'); jD[jI++ & 3] = dist(); }
      tJ = millis();
    }
    if (junc && !wasJunc) { jTwo = false; jS0 = dist(); }
    if (junc && (nBlob >= 2 || (b0s <= 1 && b0e >= 6))) jTwo = true;   // 2 separate traces, or black COVERING THE WHOLE BAR (T from the base, 4-way)
    if (!junc && wasJunc && jTwo && dist() - jS0 >= 8.0f) jT[jTI++ & 3] = dist();
    if (junc && !wasJunc) { qL = qR = 0; qFull = false; qFast = false; qMB = 0; qLc = lineC; qD0 = dist(); qH0 = heading(); }
    if (junc) {
      if (b0s <= 1 && bLe >= 6) qFull = true;
      // C3.15: a real hexagon Y splits SYMMETRICALLY around the line the robot was centered on. In the square cluster the
      //  bar sometimes sees two lines at both edges (corner crossed at an angle, right after a search) -> was taken for a Y
      //  ('H' inside the square cluster) -> square grid mode blocked -> random loops. Now: line centered before the junction,
      //  one branch on each side of it, at similar distances.
      if (nBlob >= 2 && bLs - b0e >= HEX_GAP && dist() - qD0 < HEX_SPLIT && fabsf(qLc) < 1.5f) {
        float c0 = 0.5f * (b0s + b0e) - 3.5f, cL = 0.5f * (bLs + bLe) - 3.5f;
        if (c0 < qLc - 1.2f && cL > qLc + 1.2f && fabsf((qLc - c0) - (cL - qLc)) < 2.0f) qFast = true;
      }   // C3.11: 2 branches SPLITTING FAST (60-degree angle) = hexagon Y-junction; 2 touching circles split very slowly
      qMB = max(qMB, nBlob);
      // C3.5: at a PERPENDICULAR T (black covers the whole bar, 1 trace) on the main heading = may be the ENTRY of a square cluster -> side chosen by pclust
      if (qFull && qMB == 1 && fabsf(wrap180(heading() - hRef)) < 30.0f && dist() - hexD > 600.0f) sqCandD = dist();
      if (qMB >= 2) sqCandD = -1e9;                    // splitting into 2 diagonal branches = Y-junction (hexagon) -> not one
      qL = max(qL, lineC - (b0s - 3.5f)); qR = max(qR, (bLe - 3.5f) - lineC);
    }
    // C3.29 CIRCLE CLUSTER = FOLLOW THE ARCS along the S-path through the touching points (user rule 28/11: the robot must stay on the
    //  line, no driving straight across). Path: top of the 1st bottom circle -> bottom of the 1st top circle -> top of the 2nd bottom circle
    //  -> ... -> exit line. Line heading swings about +-30 deg around the approach heading hRef; at each touching point two arcs split:
    //  one keeps curving away, the other curves BACK toward hRef. Rule: rebuild the followed line in world coordinates (odometry + bar
    //  position), take its heading at the bar; line heading right of hRef -> follow the LEFT edge, left of hRef -> follow the RIGHT edge
    //  (hysteresis CX_HYS). Robot heading is not used: the bar is 115 mm ahead, so the robot heading lags the line by ~1/4 wave.
    //  Exit: the line under the bar is straight and on hRef for CX_STR mm. Line lost / CX_MAX mm -> normal line following / search.
    // C3.31 ENTRY DETECTOR, built from the real 2 mm capture (C3.30 log 28/11): a SEPARATE trace appears at the outer channel (D8 or D1)
    //  with a white gap to the line, slides inward and MERGES with the line within CX_ARM mm, while driving straight = an arc of the circle
    //  cluster joining the line. A 90-degree corner / T branch starts AT the line (no gap); a hexagon Y splits (merged -> separate).
    //  Works on raw channel values: the readLine() noise filter drops a 1-channel trace, so it never becomes a 'junction' on a 16-20 mm line.
    int cxFire = 0;
    {
      const int li = constrain((int)lroundf(lineC + 3.5f), 0, 7);
      const bool straightNow = fabsf(wrap180(heading() - hRef)) < 20.0f && fabsf(gzSlow) < 40.0f;
      bool gapR = false, gapL = false, contR = true, contL = true;
      for (int j = li + 1; j < 7; j++) if (val[j] < 0.35f) gapR = true;
      for (int j = 1; j < li; j++) if (val[j] < 0.35f) gapL = true;
      int oR = -1, oL = 8; for (int j = 7; j > li; j--) if (val[j] > 0.5f) { oR = j; break; }
      for (int j = 0; j < li; j++) if (val[j] > 0.5f) { oL = j; break; }
      for (int j = li; j <= oR; j++) if (val[j] < 0.35f) contR = false;
      for (int j = oL; j <= li; j++) if (j >= 0 && val[j] < 0.35f) contL = false;
      if (oR < 0) contR = false;
      if (oL > 7) contL = false;
      if (!seen || !straightNow || inAvoid) cxArmR = cxArmL = -1e9;
      else {
        if (cxArmR < -1e8 && val[7] > 0.5f && gapR && li <= 5) cxArmR = dist();
        if (cxArmL < -1e8 && val[0] > 0.5f && gapL && li >= 2) cxArmL = dist();
        if (cxArmR > -1e8 && (dist() - cxArmR > CX_ARM || val[0] > 0.5f)) cxArmR = -1e9;
        if (cxArmL > -1e8 && (dist() - cxArmL > CX_ARM || val[7] > 0.5f)) cxArmL = -1e9;
        if (cxPend == 0 && cxArmR > -1e8 && contR && oR < 7 && dist() - cxArmR >= 8.0f) { cxPend = 1; cxSlide = dist() - cxArmR; cxPendD = cxHoldD = dist(); cxArmR = -1e9; }
        else if (cxPend == 0 && cxArmL > -1e8 && contL && oL > 0 && dist() - cxArmL >= 8.0f) { cxPend = -1; cxSlide = dist() - cxArmL; cxPendD = cxHoldD = dist(); cxArmL = -1e9; }
      }
      // C3.31: after the merge an arc TOUCHING the line stays merged with it (wider than the line on that side) for several cm; a T branch
      //  crossed at an angle is only a line width wide and is gone after ~2 cm. Confirm only when it stays merged for CX_HOLD mm.
      if (cxPend != 0) {
        bool wider = false;   // black still beyond the line on the trace side (merged or separate again)
        if (cxPend > 0) { for (int j = li + 1; j <= 7; j++) if (val[j] > 0.5f) wider = true; }
        else            { for (int j = 0; j < li; j++) if (val[j] > 0.5f) wider = true; }
        if (wider) cxHoldD = dist();
        if (!seen || !straightNow || dist() - cxHoldD > 4.0f) cxPend = 0;
        else if (dist() - cxPendD >= CX_HOLD) { cxFire = cxPend; cxPend = 0; }
      }
    }
    if (cxFire != 0 && cfg.clusterMode && !gridOn && dist() - hexW > HEX_MM) {
      cxArmR = cxArmL = -1e9;
      {
        logEv('E'); phase = 'E';
        const float hC = hRef, d0c = dist();
        int side = cxFire > 0 ? 0 : 1;          // arc joins from the right -> keep the LEFT edge (stay on the approach line until it merges)
        const float slide = cxSlide;
        float wHalf = 1.0f;                     // half line width (channels), learned on plain line
        const int NP = 32; float px[NP], py[NP], pd[NP]; int np = 0;   // followed line points, every ~3 mm
        float psiL = 0, strLen = 0, lostLen = 0, lastD = dist(), ePrev = lineC, eF = lineC, eT = lineC, maxDev = 0; int exitWhy = 0, nSw = 0; float swD = -1e9;
        uint32_t tc = millis(), tp = micros();
        const float v = CX_V;
        while (!aborted && millis() - tc < 4000) {
          update(); readLine();
          uint32_t now = micros(); float dt = max(1e-4f, (now - tp) * 1e-6f); tp = now;
          float trav = dist() - d0c, dd = dist() - lastD; lastD = dist();
          if (trav > CX_MAX) { exitWhy = 2; break; }
          if (!seen) { lostLen += dd; if (lostLen > 50.0f) { exitWhy = 3; break; } driveV(v + cfg.kp * eF, v - cfg.kp * eF); continue; }
          lostLen = 0;
          // line candidates: each trace gives the line at its left and right edge (one line if the trace is one line wide).
          // Keep the candidate nearest to the line followed so far; only when the followed line FORKS (two candidates both close)
          // pick the one on the chosen side. A far arc touching the bar end is never taken (it is not close).
          auto edgeL = [&](int i) { float o = i > 0 ? val[i - 1] : 0.0f; return (i - 1) + constrain((0.5f - o) / max(0.05f, val[i] - o), 0.0f, 1.0f); };
          auto edgeR = [&](int i) { float o = i < 7 ? val[i + 1] : 0.0f; return (i + 1) - constrain((0.5f - o) / max(0.05f, val[i] - o), 0.0f, 1.0f); };
          if (nBlob == 1 && nBlack <= 3 && edgeR(b0e) - edgeL(b0s) < 2.8f) wHalf += 0.03f * (constrain(0.5f * (edgeR(b0e) - edgeL(b0s)), 0.6f, 1.4f) - wHalf);
          float cand[6]; int nc = 0;
          auto addTrace = [&](int i0, int i1) {
            float l = edgeL(i0) + wHalf - 3.5f, r = edgeR(i1) - wHalf - 3.5f;
            if (r - l < 1.2f) cand[nc++] = 0.5f * (l + r); else { cand[nc++] = l; cand[nc++] = r; }   // < 1.2: one line (crossed at an angle it looks wider)
          };
          // own scan of the raw channels: at 12 mm spacing a 16-20 mm line often blackens ONE channel, which readLine() drops as noise
          for (int i = 0; i < 8 && nc < 3; ) {
            if (val[i] <= 0.5f) { i++; continue; }
            int j = i; while (j < 7 && val[j + 1] > 0.5f) j++;
            addTrace(i, j); i = j + 1;
          }
          if (nc == 0) addTrace(b0s, b0e);
          int bi = 0; for (int i = 1; i < nc; i++) if (fabsf(cand[i] - eT) < fabsf(cand[bi] - eT)) bi = i;
          for (int i = 0; i < nc; i++) if (i != bi && fabsf(cand[i] - eT) < CX_FORK && (side ? cand[i] > cand[bi] : cand[i] < cand[bi])) bi = i;
          float e = cand[bi]; if (fabsf(e - eT) > 0.5f) np = 0;   // jumped to another line -> rebuild its heading from scratch
          eT = e;
          // rebuild the followed line (world) and its heading at the bar
          float hr = heading() * DEG_TO_RAD, lx = odoX + cosf(hr) * cfg.sensAx + sinf(hr) * e * CX_PITCH, ly = odoY + sinf(hr) * cfg.sensAx - cosf(hr) * e * CX_PITCH;
          if (np == 0 || dist() - pd[(np - 1) % NP] >= 3.0f) { px[np % NP] = lx; py[np % NP] = ly; pd[np % NP] = dist(); np++; }
          int k = np - 1, j = k; while (j > 0 && j > np - NP && dist() - pd[(j - 1) % NP] <= CX_BASE) j--;
          float psiOld = psiL;
          if (k - j >= 4) psiL = atan2f(py[k % NP] - py[j % NP], px[k % NP] - px[j % NP]) * RAD_TO_DEG;
          float err = wrap180(psiL - hC);
          if (dist() - swD > 30.0f) {             // no flip-flop: a real side change is >= 50 mm from the next touching point
            if (err < -CX_HYS && side != 0) { side = 0; nSw++; swD = dist(); } else if (err > CX_HYS && side != 1) { side = 1; nSw++; swD = dist(); }
          }
          maxDev = max(maxDev, fabsf(err));
          // exit: straight line on hRef (not just the top of an arc) for CX_STR mm, after the arcs
          bool str = k - j >= 4 && fabsf(err) < 8.0f && fabsf(wrap180(psiL - psiOld)) < 2.0f && nBlack <= 3 && fabsf(gz) < 60.0f;
          strLen = str ? strLen + dd : 0;
          if (strLen >= CX_STR && trav > 250.0f && nSw >= 2) { exitWhy = 1; break; }
          eF = e; float de = (e - ePrev) / dt; ePrev = e;
          float u = cfg.kp * cfg.kpBoost * e + cfg.kd * de;
          driveV(v + u, v - u);
        }
        Serial.printf("CIRCLES: arcs %.0f mm (entry slide %.0f mm), %d edge switches, line heading max %.0f deg -> %s\n", dist() - d0c, slide, nSw, maxDev,
                      exitWhy == 1 ? "exit line found" : exitWhy == 3 ? "line lost, search" : "too long, normal following");
        logEv(exitWhy == 1 ? 'e' : 'X'); cxCandD = -1e9;
        hRef = hC; gridOn = false; gridCand = -1e9; hexW = -1e9; dCv = -1e9; crLastD = dist();
        lineCtl = 0; centeredD = -1e9; nR = nL = 0; phase = '-'; lastErr = 0; tPrev = micros(); spd = min(spd, (float)cfg.cornerSpd);
        cornerSide = 0; lastRD = lastLD = lastBD = -1e9; tStraight = millis(); wasJunc = false;
        lastPos = eF; smoothPos = eF;            // line search (if lost) turns toward where the line was
        continue;
      }
    }
    if (cxCandD > -1e8 && dist() - cxCandD > 220.0f) cxCandD = -1e9;
    // C3.11 HEXAGON CLUSTER: Y-junction (black trace SPLITTING into 2 slanted branches) -> enable hexagon mode for HEX_MM (each new Y-junction extends it)
    if (!junc && wasJunc && cfg.clusterMode && cfg.jMode && !inAvoid && qFast && fabsf(gzSlow) < HEX_GZ && dist() - qD0 < 90.0f) {   // large gzSlow = touching circles, not a hexagon
      if (dist() - hexW > HEX_MM) { logEv('H'); if (dist() - hexW > 3.0f * HEX_MM) hRefHex = qH0; }   // just ENTERED the cluster: lock the main heading = ROBOT heading at the first Y-junction (left the cluster briefly and re-entered -> keep the old heading)
      // C3.11b: the previous version used hRef - hRef stands still while curving (|gz| > 40) -> entering the hexagons RIGHT AFTER A BEND made the main heading = heading BEFORE the bend (wrong) -> wrong compensation -> circling
      hexW = hexD = dist(); gridOn = false;
    }
    if (junc && dist() - hexW < HEX_MM) hexW = dist();   // inside the cluster: every junction extends hexagon mode (C3.15: hexW only - hexD blocks square grid mode, set only by a real Y)
    if (!junc && wasJunc && cfg.clusterMode && cfg.jMode && !inAvoid && fabsf(gzSlow) < 70.0f && dist() - qD0 < 90.0f) {
      // C3.4: PERPENDICULAR T (black covers the whole bar, 1 trace - not splitting into 2 branches) on the main heading = ENTERING A SQUARE CLUSTER.
      //       Y-junction (black covering the bar then SPLITTING into 2 diagonal branches) = HEXAGON / honeycomb -> NOT latched in the grid for 60 cm (beyond the hexagon frame)
      if (qFull && qMB >= 2) { hexD = dist(); gridOn = false; }
      // C3.26: a square-cluster ENTRY is a pure T (no straight line after it). At the touching points of the CIRCLE cluster the black also covers
      //  the bar, but the arc continues straight on -> latching grid mode there made the robot stop-and-pivot (Q) inside the circles and loop.
      //  Now only a CANDIDATE here; confirmed 20 mm after the junction only if no line continues straight ahead.
      if (qFull && qMB == 1 && fabsf(wrap180(heading() - hRef)) < 30.0f && dist() - hexD > 600.0f) { if (gridOn) gridJ = dist(); else gridCand = dist(); }
      if (gridOn) { qPend = (qL >= 2.0f || qR >= 2.0f || qFull); qD1 = dist(); gridJ = dist(); }
    }
    if (gridCand > -1e8 && !junc && dist() - gridCand >= 20.0f) {                 // C3.26: confirm the square cluster only on a pure T
      bool straightOn = seen && fabsf(posC - qLc) < 1.3f;
      if (!straightOn) { gridOn = true; gridJ = dist(); }
      gridCand = -1e9;
    }
    if (gridOn && dist() - gridJ > 450.0f) gridOn = false;                       // 45 cm without a junction = out of the cluster
    wasJunc = junc;
    if (qPend && !junc && (!seen || dist() - qD1 >= 20.0f)) {
      qPend = false;
      bool hasS = seen && fabsf(posC - qLc) < 1.3f, hasL = qFull || qL >= 2.0f, hasR = qFull || qR >= 2.0f;
      float h0 = qH0, best = 1e9; int pick = 0;                                     // 0 straight, -1 left, +1 right
      auto cost = [&](float d) { return fabsf(wrap180(d - hRef)); };
      if (hasS) { best = cost(h0); pick = 0; }
      // left = heading increases (+90), right = heading decreases (-90); equal (+-15 deg) -> by pref
      if (hasL) { float c = cost(h0 + 90); if (c < best - 15 || (fabsf(c - best) <= 15 && clusterPref == 0 && pick != 0)) { best = c; pick = -1; } }
      if (hasR) { float c = cost(h0 - 90); if (c < best - 15 || (fabsf(c - best) <= 15 && clusterPref == 1 && pick != 0) || (pick == 0 && !hasS)) { best = c; pick = 1; } }
      if (!hasS && pick == 0) pick = 0;                                            // no clear branch -> let line-loss handling deal with it
      if (pick != 0 && seen) {                                                    // straight line still present but a side branch is chosen -> turn at the junction center
        logEv('Q'); phase = 'Q';
        float mv = 0.5f * (qD0 + qD1) + cfg.sensAx - 5 - dist();
        if (mv > 10) moveMM(mv, heading(), max(30.0f, min(spd, (float)cfg.vCluster)));
        else { hardStop(); if (mv < -10) moveMM(mv, heading(), 30); }
        phase = 'S'; sweepAvoidH = hRef; backH = 1e9;
        bool okQ = sweepFind(pick, 120, 120, 2, 1.8f, PIVOT_PCT + 4, 50, cfg.spdTurn, 40);
        if (!okQ && backH < 1e8) { turnTo(backH, PIVOT_PCT + 4, true); readLine(); okQ = turnHitLine || (seen && maxRun >= 2); }
        sweepAvoidH = 1e9; phase = '-';
        if (!okQ) { why = "LINE LOST"; break; }
        lineCtl = 0; centeredD = -1e9; nR = nL = 0;
        lastRecD = dist(); jRecD = dist(); jCom = 0; jPend = 0; lastErr = pos; tPrev = micros(); gridJ = dist();
        spd = max((float)cfg.cornerSpd * 0.8f, 30.0f); tStraight = millis(); cornerSide = 0; lastRD = lastLD = lastBD = -1e9;
        continue;
      }
      if (pick != 0 && !seen) { cornerSide = pick; lastRD = lastLD = lastBD = -1e9; if (pick > 0) lastRD = dist(); else lastLD = dist(); tCorner = millis(); dCornerBar = 0.5f * (qD0 + qD1); }
    }
    // 90-DEGREE CORNER / SHARP BEND: the black trace touches the sensor bar EDGE (D1-D2 or D7-D8) and is >= 3 channels wide -> the bar is right at the corner.
    // Record the POSITION (encoder) now + the turn side, and slow down so the robot does not overshoot the corner.
    if (seen && nBlack >= 1) {
      lastSeenD = dist();
      if (fabsf(posC) < 1.5f && !junc) tCentered = millis();
      // FIX-001: followed the CENTER of the narrow line continuously for 20 mm -> the old corner is passed.
      // Allow the next zigzag apex on the other side while gz is still large; do not unlock
      // just because the L corner apex sweeps briefly across the middle of the sensor bar while cornering.
      if (!junc && maxRun >= 2 && fabsf(posC) < 0.8f) {
        if (centeredD < -1e8) centeredD = dist();
        if (cornerSide != 0 && dist() - centeredD >= 20.0f) {
          cornerSide = 0; lastRD = lastLD = lastBD = -1e9;
          heldEdgeLogged = false; logEv('a');
        }
      } else centeredD = -1e9;
      // v2.9.1: REAL TURN BRANCH = black trace touching the OUTERMOST channel (still working) and >= 3 wide, or 1 channel in and >= 4 wide.
      // (old: only needed D2 touched + width 3 -> a 2 cm line slightly left (D2-D4) was taken as a LEFT CORNER -> at a right corner the robot turned LEFT)
      int lo = 0, hi = 7;
      while (lo < 7 && (sStuck[lo] || (cfg.maskBits >> lo & 1))) lo++;
      while (hi > 0 && (sStuck[hi] || (cfg.maskBits >> hi & 1))) hi--;
      bool edgeR = (bLe >= hi && bLe - bLs >= 2) || (bLe >= hi - 1 && bLe - bLs >= 3);
      bool edgeL = (b0s <= lo && b0e - b0s >= 2) || (b0s <= lo + 1 && b0e - b0s >= 3);
      // v2.9.6: the right edge must be seen CONTINUOUSLY for >= 3 reads to count (a single noisy read on D1/D8 is no longer taken as a corner)
      nR = edgeR ? min(nR + 1, 50) : 0; nL = edgeL ? min(nL + 1, 50) : 0;
      edgeR = nR >= 3; edgeL = nL >= 3;
      // LOCK AGAINST FALSE CATCH OF THE LOWER EDGE WHILE ARCING THROUGH A 90-DEGREE L CORNER:
      // When the robot starts rolling the arc into a corner (e.g. turning LEFT), the corner apex sweeps across to the opposite edge (RIGHT edge)
      // before the sensor bar passes the line. If lastRD/cornerSide is updated while the robot is pivoting (|gz| > 95) or
      // within the first 80 mm after the corner is committed -> the robot would reverse its turn direction (turning RIGHT at a LEFT corner -> 180-degree U-turn)!
      if (edgeR && edgeL) {
        lastBD = lastRD = lastLD = dist();
        cornerSide = jPref ? 1 : -1;
        if (millis() - tCorner > 300) logEv('K');
        tCorner = millis(); dCornerBar = dist();
      } else if (edgeR || edgeL) {
        int newSide = edgeR ? 1 : -1;
        bool sameOrFresh = (cornerSide == 0) || (cornerSide == newSide) || (dist() - dCornerBar > 85.0f && fabsf(gz) < 95.0f);
        if (sameOrFresh) {
          heldEdgeLogged = false;
          if (edgeR) lastRD = dist();
          if (edgeL) lastLD = dist();
          cornerSide = newSide;
          if (millis() - tCorner > 300) logEv('K');
          tCorner = millis(); dCornerBar = dist();
        } else if (!heldEdgeLogged) { logEv('h'); heldEdgeLogged = true; }
      }
      // v3.4 SHAPE: 90-degree corner = black trace FLARING from the middle to the edge (middle still black). Curve = trace SLIDING to the edge (middle already white).
      if ((edgeR && bLs <= 4) || (edgeL && b0e >= 3)) lastShapeD = dist();
    } else { centeredD = -1e9; nR = nL = 0; }
    // JUNCTIONS INSIDE CLUSTERS (jMode 1 = KEEP HEADING): keep the original v3.8 logic to pass THROUGH the 3-square cluster and the hexagon cluster!
    // Note: wrap180 keeps hRef correct when the accumulated turn angle exceeds 180 deg in later runs.
    // C3: hRef follows the robot heading only when driving NEARLY STRAIGHT (|gz| < 40 deg/s): on circles hRef is not dragged around
    //     -> at the joint of 2 circles the robot takes the branch that KEEPS THE HEADING, instead of looping one circle forever.
    // C3.11 HEXAGON: like the square cluster (gridH) - inside the cluster LOCK the main heading = heading when entering, do NOT let hRef drift along the hexagon edges
    //  (edges 12-17 cm long, robot drives straight > 400 ms -> the old hRef drifted along the slanted edge -> lost the left/right compensation -> circled one cell).
    // curving CONTINUOUSLY > HEX_CV mm = circle (hexagon edges are straight, the 60-degree corners are short) -> temporarily disable hexagon mode (the circle-exit rule runs again)
    if (fabsf(gzSlow) < CIRC_GZ) dCv = -1e9; else if (dCv < -1e8) { dCv = dist(); cvH0 = heading(); }
    else if (fabsf(heading() - cvH0) >= 200.0f) dCircle = dist();   // C3.15: curved continuously >= 200 deg = went round a circle
    if (gridOn && dCv > -1e8 && fabsf(heading() - cvH0) >= 70.0f) { gridOn = false; gridCand = -1e9; }   // C3.26: rolling round an arc >= 70 deg = circles, not squares (square edges are straight)
    hexG = cfg.clusterMode && cfg.jMode && dist() - hexW < HEX_MM && !(dCv > -1e8 && dist() - dCv > HEX_CV);
    if (hexG) hRef = hRefHex;
    else if (seen && !junc && millis() - tStraight > 400 && !inCluster() && fabsf(gz) < 40.0f) hRef += wrap180(heading() - hRef) * min(1.0f, dt * 5.0f);
    gzSlow += (gz - gzSlow) * min(1.0f, dt * 4.0f);
    // SQUARE cluster (straights + corners), not TOUCHING CIRCLES (robot curving continuously, large gzSlow) -> circles keep the C3 handling
    gClus = cfg.clusterMode && gridOn && fabsf(gzSlow) < 70.0f; gridG = gridOn;   // C3.4: old v3.8 style ONLY inside square clusters (no more false triggers in hexagons)
    // C3.5: inside a square cluster (or just met a perpendicular T at the cluster entry) -> 'even' junctions use pclust instead of pref
    clusterPref = (cfg.pCluster <= 1 && cfg.clusterMode && (gridOn || dist() - sqCandD < 150.0f)) ? cfg.pCluster : cfg.pref;
    if (cfg.jMode) { float dh = wrap180(heading() - hRef);
      // C3.9 CIRCLE EXIT: CURVING continuously (|gzSlow| > CIRC_GZ, outside square clusters) -> the EXIT branch is on the OUTER side of the arc
      //  (right curve -> left branch, left curve -> right branch). Old version compared with the cluster entry heading (hRef): the entry line is perpendicular to the exit line,
      //  or it had turned > 180 deg (wrap180 flips sign) -> wrongly picked the INNER side -> circled forever. The outer branch is the old path -> onOldPath/goingBack block it.
      if (CIRC_GZ > 0 && !gridOn && !hexG && fabsf(gzSlow) > CIRC_GZ) dh = gzSlow > 0 ? 90.0f : -90.0f;   // C3.11: disabled inside hexagon clusters (60-degree corners make gzSlow large -> wrong branch)
      jDh = dh; hRefG = hRef; jPref = dh > 20 ? 1 : dh < -20 ? 0 : clusterPref; } else { jPref = cfg.pref; jDh = 0; }
    if (seen && !junc && fabsf(posC) < 1.5f) {   // C3: record the driven path + detect RUNNING BACKWARD on the old path (>= 60 mm)
      crumbAdd();
      static uint8_t revFix = 0; if (d < 50) revFix = 0;
      // C3.15: right after going round a circle the old path crosses itself -> 'running backward' fired by mistake and turned
      //  the robot away from the finish (log: 2 'D' in a row, ~4.5 s lost). For CIRC_NO_REV mm after a circle it now needs
      //  120 mm (6 crumbs) on the old path instead of 60 mm. (Skipping the check entirely broke real U-turns after a circle.)
      if (nRev >= (dist() - dCircle > CIRC_NO_REV ? 3 : 6) && revFix < 4) {
        revFix++; nRev = 0; logEv('D'); phase = 'D';
        Serial.println("!! RUNNING BACKWARD on the path just driven -> turning 180");
        int sd = lastPos > 0 ? 1 : -1;
        turnTo(heading() + (sd > 0 ? -180.0f : 180.0f), max((float)cfg.spdTurn, 45.0f), false); readLine();
        if (!(seen && maxRun >= 2)) sweepFind(sd, 60, 120, 2, 2.2f, PIVOT_PCT + 4, 0, cfg.spdTurn, 0);
        lineCtl = 0; centeredD = -1e9; nR = nL = 0;
        phase = '-'; lastErr = 0; tPrev = micros(); spd = min(spd, (float)cfg.cornerSpd); cornerSide = 0; lastRD = lastLD = lastBD = -1e9; crLastD = dist();
        continue;
      }
    }
    if (seen) {
      // C2 (jnmode 1): near a JUNCTION / inside a CLUSTER (60 cm from the last junction) -> v3.8 controller: follow the branch EDGE firmly, turn decisively
      // correct branch (touching circles, squares, hexagons). Everything else (straights, bends, S-curves) -> the smooth gemini controller as before.
      float lastJh = max(max(jD[0], jD[1]), max(jD[2], jD[3]));
      bool nearJ = cfg.jnMode && (junc || inCluster() || dist() - lastJh < 600);
      if (nearJ) {
        float e = pos, de = lineCtl == 1 ? (e - lastErr) / dt : 0;
        lineCtl = 1; lastErr = e; lastPos = posC;
        // speed: straight and steady > 250 ms -> up to vmax; entering a bend / junction -> back to base
        if (fabsf(e) > 0.8f || junc) tStraight = millis();
        float target = (millis() - tStraight > 250) ? cfg.vmax : cfg.base;
        if (mapUse) {                                   // FAST: speed by the track shape ahead
          float md = mapPos(), ta = mapTurnAhead(md, cfg.mPre);
          target = ta < 12 ? cfg.mFast : ta < 45 ? (float)cfg.base : cfg.cornerSpd;
          if (mapNear(md, 'L', cfg.mPre) || mapNear(md, 'O', cfg.mPre + 150)) target = min(target, cfg.cornerSpd);   // approaching a known corner / obstacle
          if (fabsf(e) > 1.5f) target = min(target, (float)cfg.base);
        }
        if (junc) target = min(target, cfg.base * 0.8f);
        if (gridOn) target = min(target, (float)min((int)cfg.cornerSpd, (int)cfg.vCluster));   // C3.5 inside square clusters: slow (vclust), drive each edge exactly
        if (vlOn && vlOk && vlMM < cfg.obs + 150) target = min(target, max(oldPct(35), vSafeMM(vlMM) * 100.0f / max(1.0f, cfg.vfull)));   // C3.24: slow to a speed that can still stop / curve out
        if (millis() - tCorner < 300) target = min(target, cfg.cornerSpd);    // approaching / at a 90-degree corner
        // v3.2 CURVES: lateral acceleration = v x turn rate <= alat -> the sharper the bend (large gz) the lower the speed, smooth cornering
        if (cfg.alat > 0 && cfg.vfull > 100) { float w = fabsf(gz) * 0.0174533f; if (w > 0.5f) target = min(target, max((float)cfg.cornerSpd, cfg.alat / w / cfg.vfull * 100.0f)); }
        spd += (target - spd) * min(1.0f, dt * (target > spd ? 3.0f : 25.0f));   // rise slowly (~0.3 s), fall fast
        curSpd = spd;
        // v3.6: large error -> stronger kp (kpb) + stronger braking (brake); near the edge (|e| >= piv) and not a junction -> PIVOT
        //       (outer wheel forward, inner wheel backward) -> S-bends / zigzags do not run wide and lose the line.
        //       Junctions / 90-degree corners (wide black trace) are still handled by the 90-DEGREE CORNER logic as before.
        float kpE = fabsf(e) >= 1.8f ? cfg.kp * cfg.kpBoost : cfg.kp;
        float u = kpE * e + cfg.kd * de;
        bool newCtl = cfg.brakeK > 0.5f || cfg.pivE > 0;
        float base = newCtl ? spd * (1.0f - cfg.brakeK * min(1.0f, fabsf(e) / 2.5f)) : spd * (1.0f - 0.45f * min(1.0f, fabsf(e) / 3.0f));
        if (cfg.pivE > 0 && fabsf(e) >= cfg.pivE && !junc) {
          float tp = constrain(spd * 1.15f + 12.0f, 44.0f, 68.0f);
          if (e > 0) drive(tp, -tp * 0.9f); else drive(-tp * 0.9f, tp);
        } else driveV(base + u, base - u);
      } else {
        if (lineCtl != 2) { smoothPos = pos; lastErr = pos; filtDe = 0; }
        lineCtl = 2;
        smoothPos = 0.78f * pos + 0.22f * smoothPos;

        float e = smoothPos;
        float rawDe = (e - lastErr) / dt; lastErr = e; lastPos = posC;
        filtDe = 0.50f * filtDe + 0.50f * constrain(rawDe, -160.0f, 160.0f);
        float de = filtDe;

        // 1. SMART SPEED MANAGEMENT FOR HIGH SPEED (76/100):
        if (fabsf(e) > 0.42f || fabsf(de) > 7.5f || junc || fabsf(gz) > 35.0f) tStraight = millis();
        bool inWave = (millis() - tCorner < 420) || inCluster();
        float target = (millis() - tStraight > 300 && !inWave) ? cfg.vmax : (inWave ? cfg.cornerSpd : cfg.base);

        if (mapUse) {
          float md = mapPos(), ta = mapTurnAhead(md, cfg.mPre);
          target = ta < 12 ? cfg.mFast : ta < 45 ? (float)cfg.base : cfg.cornerSpd;
          if (mapNear(md, 'L', cfg.mPre) || mapNear(md, 'O', cfg.mPre + 150)) target = min(target, cfg.cornerSpd);
          if (fabsf(e) > 1.2f) target = min(target, (float)cfg.base);
        }
        if (junc || millis() - tCorner < 220) target = min(target, cfg.cornerSpd);
        if (gridOn) target = min(target, (float)min((int)cfg.cornerSpd, (int)cfg.vCluster));   // C3.5 inside square clusters: slow (vclust), drive each edge exactly
        if (vlOn && vlOk && vlMM < cfg.obs + 150) target = min(target, max(oldPct(35), vSafeMM(vlMM) * 100.0f / max(1.0f, cfg.vfull)));   // C3.24: slow to a speed that can still stop / curve out
        if (cfg.alat > 0 && cfg.vfull > 100) {
          float w = fabsf(gz) * 0.0174533f;
          if (w > 0.5f) target = min(target, max((float)cfg.cornerSpd, cfg.alat / w / cfg.vfull * 100.0f));
        }

        // Very fast early braking (52.0f) when diving from vmax 100 into a bend, and fast acceleration (6.5f) when leaving the bend
        spd += (target - spd) * min(1.0f, dt * (target > spd ? 6.5f : 52.0f));
        curSpd = spd;

        // 2. HIGH-SPEED DYNAMIC ARC:
        float absE = fabsf(e);
        float leadDe = (e * de > 0) ? (0.028f * fabsf(de)) : (-0.018f * fabsf(de));
        float lookAheadE = constrain(absE + leadDe, 0.0f, 3.5f);

        // Robot center speed: small error keeps high speed; near the edge (lookAheadE > 2.2) drop vCenter to 36% to tighten the radius R
        // helps the robot hug 90-degree corners tightly at high speed without swinging out!
        float fwdFactor = 1.0f - (cfg.brakeK * 0.68f) * min(1.0f, (lookAheadE * lookAheadE) / 8.0f);
        float vCenter = max(spd * 0.36f, spd * fwdFactor);

        float kpDynamic = cfg.kp * (1.0f + (cfg.kpBoost - 1.0f) * min(2.2f, max(0.0f, lookAheadE - 0.5f)));
        float gyroDamp = 0.035f * (cfg.gsgn * gz);
        float u = kpDynamic * e + cfg.kd * de - gyroDamp;

        // Pull to cut the corner early when the sensor bar touches a 90-degree corner (only when it is NOT a straight-through T inside a cluster)
        if (!junc && millis() - tCorner < 180 && cornerSide != 0 && absE > 1.0f) {
          float arcPull = cornerSide * (spd * 0.52f);
          if (u * arcPull > 0 && fabsf(u) < fabsf(arcPull)) u = arcPull;
        }

        float vLeft  = vCenter + u;
        float vRight = vCenter - u;

        // At high speed let the inner wheel reverse flexibly with lookAheadE (from -15% to -37% spd)
        // so the arc radius R shrinks automatically when the robot dives fast into a 90-degree corner!
        float maxOuter = constrain(spd * 1.32f + 12.0f, 48.0f, 88.0f);   // NOTE: 88 % also caps the STRAIGHT speed (vmax above 88 changes nothing)
        float innerBack = (cfg.pivE > 0 && lookAheadE >= cfg.pivE)
                          ? (0.15f + 0.22f * min(1.0f, (lookAheadE - cfg.pivE) / 1.5f))
                          : 0.06f;
        float minInner = -spd * innerBack;

        if (vLeft > maxOuter)  { vRight -= (vLeft - maxOuter) * 0.55f; vLeft = maxOuter; }
        if (vRight > maxOuter) { vLeft -= (vRight - maxOuter) * 0.55f; vRight = maxOuter; }
        vLeft  = max(vLeft, minInner);
        vRight = max(vRight, minInner);

        driveV(vLeft, vRight);
      }
    } else {
      // LINE LOSS WHEN ENTERING CORNERS AT HIGH SPEED:
      lineCtl = 0; centeredD = -1e9; nR = nL = 0;
      logEv('L');
      bool corner = cornerSide != 0 && millis() - tCorner < 600 && dist() - dCornerBar < 300;
      float recW = gClus ? 70.0f : 85.0f;   // C3.1: inside a cluster use the 70 mm window as in v3.8
      bool rRec = dist() - lastRD < recW, lRec = dist() - lastLD < recW;
      int s; char why2 = 'f';
      if (rRec != lRec) {
        why2 = 'b';
        s = rRec ? 1 : -1;
        if (!corner) { corner = true; dCornerBar = rRec ? lastRD : lastLD; }
      } else if (rRec && lRec) {
        if (dist() - lastBD < 70) { s = jPref ? 1 : -1; why2 = 'n'; }
        else { s = (cornerSide != 0 && !gClus) ? cornerSide : (lastRD > lastLD ? 1 : -1); why2 = 'b'; }
        if (!corner) { corner = true; dCornerBar = max(lastRD, lastLD); }
      } else s = corner ? cornerSide : (lastPos > 0 ? 1 : -1);

      bool mapGap = false; float hLoss = heading(), dLoss = dist() - d0, mLoss = mapPos();
      if (mapUse) {
        int j = mapFind(mLoss, "Lg", 150, hLoss);
        if (j >= 0) {
          if (mnode[j].t == 'g') mapGap = true;
          else if (mnode[j].s != 0) { s = mnode[j].s; why2 = 'm'; if (!corner) { corner = true; dCornerBar = lastSeenD; } }
          mapSync(j);
        }
      }

      // Dashed line in the middle -> hold heading and glide straight through
      if (mapGap || (why2 != 'm' && !corner && !rRec && !lRec && fabsf(lastPos) < 1.5f)) {
        logEv('i');   // FIX-001: tell straight-through dashed-line search apart from arc line search
        phase = 'g'; float g0 = dist(), hg = heading(); bool back = false; uint32_t tg = millis();
        while (!aborted && dist() - g0 < GAP_MM && millis() - tg < 400) {
          update(); readLine();
          if (seen && peakV >= 0.6f && maxRun >= 2) { back = true; break; }
          float b = min(max(spd, 30.0f), 55.0f), corr = constrain(2.0f * (hg - heading()) - 0.05f * gz, -15.0f, 15.0f);
          driveV(b - corr, b + corr);
        }
        phase = '-';
        if (back) { logEv('g'); mapNode('g', 0, dLoss, hLoss); lastErr = 0; tPrev = micros(); continue; }
      }

      if (corner) cornerSide = s;
      logEv(s > 0 ? '>' : '<'); logEv(why2);

      // HIGH-SPEED ROLLING ARC:
      // Only enable sweepAvoidH when REALLY inside a square / hexagon cluster (inCluster() == true).
      // Not enabled at 2 L 90-degree corners 60 cm apart (U-shaped lane change) so the robot can freely turn 90+90 = 180 deg into the new lane!
      bool ok = false;
      // C3: REMOVED C2's 'reach the apex then turn' (xv) because it STALLED the robot on square pulses. Every corner / line loss uses the gemini ROLLING ARC;
      //     anti U-turn using the DRIVEN PATH (goingBack) while rolling the arc + sweeping for the line.
      // C3: outside clusters DO NOT go to the corner apex (no more stalling on square pulses). C3.1: INSIDE SQUARE CLUSTERS (cluster 1) -> old v3.8 style:
      //     reach the corner apex / junction then turn to the chosen side (adjacent squares: robot exits at the center of the new branch, does not cut across into the next square)
      const bool vertexMode = gClus && (corner || fabsf(lastPos) > 2.0f);
      if (vertexMode) {
        phase = 'K';
        float mv = (corner ? dCornerBar : lastSeenD) + cfg.sensAx - 10 - dist();   // remaining distance until the wheel axle reaches the corner point
        if (mv > 15) moveMM(mv, heading(), min(spd, cfg.cornerSpd));
        else { hardStop(); if (mv < -15) moveMM(mv, heading(), 30); }
        phase = 'S';
        sweepAvoidH = inCluster() ? hRef : 1e9; backH = 1e9;
        ok = sweepFind(s, SWEEP_DEG, SWEEP_DEG, 2, 1.8f, PIVOT_PCT + 4, corner ? 50 : 0, cfg.spdTurn, corner ? 40 : 0);
        if (!ok && backH < 1e8) { turnTo(backH, PIVOT_PCT + 4, true); readLine(); ok = turnHitLine || (seen && maxRun >= 2); }
      }
      phase = 'S';
      if (!vertexMode) { sweepAvoidH = inCluster() ? hRef : 1e9; backH = 1e9; }
      float h0 = heading();
      uint32_t tArc = millis();
      float vOuter = constrain(max(spd, cfg.cornerSpd) * 1.25f, 54.0f, 74.0f);
      float vInnerRatio = (spd > 62.0f) ? -0.26f : -0.18f;

      while (!vertexMode && !aborted && millis() - tArc < 850) {
        update(); readLine();
        float turned = (h0 - heading()) * s;
        float maxP = (turned < 18.0f) ? 1.2f : 2.4f;
        if (seen && maxRun >= 2 && fabsf(posC) < maxP) {
          if ((sweepAvoidH < 1e8 && fabsf(wrap180(heading() - sweepAvoidH)) > 150.0f) || goingBack()) {
            if (backH > 1e8) { backH = heading(); nBack++; logEv('u'); }
          } else {
            ok = true;
            break;
          }
        }
        if (turned > 100.0f) break;
        float curInner = (turned > 50.0f) ? (-vOuter * 0.34f) : (vOuter * vInnerRatio);
        if (s > 0) driveV(vOuter, curInner);
        else       driveV(curInner, vOuter);
      }

      if (!ok && !vertexMode) {
        ok = sweepFind(s, SWEEP_DEG, SWEEP_DEG, 2, 2.2f, PIVOT_PCT + 8, 20, cfg.spdTurn, 0);
        // C3: only the OLD line (reversed) seen -> keep turning another 120 deg the other way to find the onward branch (narrow square pulse / short segment)
        if (!ok && backH < 1e8) { logEv('w'); ok = sweepFind(-s, 120, 0, 2, 2.2f, PIVOT_PCT + 8, 0, cfg.spdTurn, 0); }
        if (!ok && backH < 1e8) { turnTo(backH, PIVOT_PCT + 4, true); readLine(); ok = turnHitLine || (seen && maxRun >= 2); }
      }

      // WRONG-WAY GUARD:
      // sweepFind may re-catch the old line after turning close to 180 deg. Then
      // the sensor still saw black so the old logic thought it had returned to the line successfully.
      // Compare the heading at line loss (hLoss) with the heading when the line is re-caught.
      // Only checked in the bend/line-loss context, not applied to dashed lines
      // in the middle of the straight to avoid a wrong correction.
      if (ok && backH > 1e8) {   // C3: only correct when REALLY on the old path & reversed (no raw heading compare -> no more false triggers on sharp S-bends)
        float dhBack = wrap180(heading() - hLoss);
        if (goingBack(150.0f)) {
          if (wrongWayFixes < WRONG_WAY_MAX_FIX) {
            wrongWayFixes++;
            logEv('D');
            phase = 'D';
            Serial.printf("!! GRABBED LINE IN REVERSE heading %+0.1f deg -> turning 180 (%d/%d)\n",
                          dhBack, wrongWayFixes, WRONG_WAY_MAX_FIX);

            // Turn 180 in the direction opposite to the initial line search.
            // After turning, the robot faces the same direction as before the line loss.
            float hBad = heading();
            turnTo(hBad + (s > 0 ? 180.0f : -180.0f),
                   max((float)cfg.spdTurn, WRONG_WAY_FIX_PCT), false);
            readLine();

            // Try to re-catch the line right at the corrected position. If the sensor is
            // if the error is at the edge, sweep a little in the initial search direction, not 120 deg.
            bool fixedLine = seen && maxRun >= 2;
            if (!fixedLine) fixedLine = sweepFind(s, 45, 90, 2, 2.2f,
                                                   PIVOT_PCT + 4, 0,
                                                   max((float)cfg.spdTurn, 40.0f), 0);
            if (!fixedLine) {
              phase = '-';
              why = "LINE LOST AFTER FIX";
              break;
            }
            // Do not let the map record a 180-degree turn by mistake. From this point treat it as
            // the robot is back on the heading it had before the line loss and continues following the line.
            hLoss = heading();
            phase = '-';
          } else {
            Serial.println("!! WRONG LINE GRABBED REPEATEDLY -> STOP TO AVOID RUNNING BACKWARD");
            why = "REPEATED REVERSE";
            break;
          }
        }
      }
      sweepAvoidH = 1e9;
      phase = '-';
      if (!ok) { why = "LINE LOST"; break; }

      { float dh = heading() - hLoss; int real = dh < -25 ? 1 : dh > 25 ? -1 : 0;
        mapNode('L', real, dLoss, hLoss);
        if (why2 == 'm' && real != 0 && real != s && ++mapBad >= 2) { mapUse = false; logEv('x'); Serial.println("!! MAP MISMATCH -> switching to NORMAL run"); } }

      lastRecD = dist(); jRecD = dist(); jCom = 0; jPend = 0; lastErr = pos; tPrev = micros();
      // FIX-001: re-catching the line does not mean it is stable yet; do not rush into the next apex
      // at 88% of the speed before the line loss. Re-accelerates with the existing ramp.
      spd = min(spd, (float)cfg.cornerSpd); curSpd = spd; logEv('r');
      hint = 0; tStraight = millis(); cornerSide = 0; lastRD = lastLD = lastBD = -1e9;
    }
    if (millis() - tP >= 200) { tP = millis(); Serial.printf("d %5.0f pos %+5.2f black %d %s v %.0f vl %u\n", d, pos, nBlack, junc ? "JUNC" : "", spd, vlMM); }
    // real-time screen: drawn 4 times/s, each loop sends only one small packet (does not slow the line loop)
    static uint32_t tO = 0;
    if (cfg.runOled && millis() - tO >= 300) { tO = millis(); uiRun((millis() - tRun0) / 1000.0f, d, spd, junc ? "JUNCTION" : (vlOn && vlOk && vlMM < SLOW_MM) ? "OBJECT" : "FOLLOW LINE"); }
    uiRunStep();
  }
  brake(); delay(200); coast(); tracing = false; runActive = false;
  if (spinStop) { why = "SPINNING - STOPPED"; logEv('V'); }
  float t = (millis() - tRun0) / 1000.0f, D = dist() - d0;
  Serial.printf("\n=== %s | %.2f s | %.0f mm | %.2f m/s AVG | %d obstacles | %lu loops/s ===\n",
                why, t, D, D / 1000.0f / max(0.1f, t), nObs, (unsigned long)(loops / max(0.1f, t)));
  if (stuckEv) { Serial.print("!! STUCK CHANNELS (ignored while running):"); for (int i = 0; i < 8; i++) if (stuckEv >> i & 1) Serial.printf(" D%d", i + 1); Serial.println("  -> clean / resolder / adjust the height of this channel, or disable it with the mask command"); }
  Serial.println("EVENT LOG (J = junction, L = line lost/corner turn, > / < = chose turn RIGHT / LEFT (reason right after: b = branch seen on 1 side, n = T-junction by preference, f = unclear), O = obstacle met, o = line re-caught, F = finish):");
  Serial.println("FIX-001: a = 20 mm center line, release old heading lock; h = old heading lock blocks the opposite edge; i = start driving straight over dashed line; r = line re-caught, slow down.");
  for (int i = 0; i < evCount(); i++) { const Ev &e = evAt(i); if (e.c != 'J') Serial.printf("  %c %6.0f mm  %5.2f s\n", e.c, e.d, e.t); }
  bool fin = strcmp(why, "TO FINISH") == 0;
  if (recOn) {
    recOn = false;
    if (fin) { mapOk = true; mapT = t; mapLen = D; mapSave(); Serial.printf("MAP SAVED: %d points, %.1f m -> select FAST mode for real runs\n", nNode, D / 1000); }
    else { mapOk = false; nProf = nNode = 0; mapLoad(); Serial.println("DEMO DID NOT REACH FINISH -> NOT saved (old map kept if any)"); }
  }
  res.has = true; strncpy(res.why, why, sizeof(res.why) - 1); res.why[sizeof(res.why) - 1] = 0;
  res.t = t; res.d = D; res.v = D / 1000.0f / max(0.1f, t); res.obs = nObs; res.lost = 0; res.junc = 0;
  res.lost = cntL; res.junc = cntJ;
  uiAfterRun();
  if (fin && FIN_SONG) playBirthday(); else beep(fin ? 3 : 1, fin ? 70 : 400);
}
// ---------- serial ----------
// WHEEL SPEED MEASUREMENT (command wcal): the robot drives STRAIGHT on the floor at direct PWM -> vfull = speed at 100%, dband = dead band.
// The weaker wheel shows clearly (%).
void wheelCal() {
  // C3.16: TWO-POINT measurement. Above the start-up dead band the speed is a straight line of PWM (floor test 25/11:
  // 40/60/80/100 % -> 330/695/1150/1583 mm/s, nothing below ~28 %), so 50 % x 2 read ~35 % low. Now: 50 % for 1.0 s, then 80 % for 1.3 s,
  // (C3.17: 40 % instead of 50 %) average the last 0.4 s of each, fit the line -> vfull (speed at 100 %) and dband (% where the line hits 0). Needs ~2.5 m clear ahead.
  wifiStop(); uiBusy("WHEEL CAL", "PUT ROBOT ON FLOOR", "2.5 m clear ahead");
  Serial.println("\nWCAL: robot on the FLOOR, >= 2.5 m clear ahead. Starting in 2 s...");
  beep(2); delay(2000);
  // C3.17: 50 -> 40 % (the speed curve bends below ~60 %: 50/80 gave dband 33 -> too much PWM at low speed)
  const float P[2] = {40.0f, 80.0f}; const uint32_t RUN_MS[2] = {1100, 1400}, AVG_MS = 400;
  float spd[2][2]; aborted = false;
  Serial.println("  pct  t(ms)  left  right (mm/s)");
  for (int k = 0; k < 2; k++) {
    float sL = 0, sR = 0, s1 = 0, s2 = 0, pL = 0, pR = 0; int n = 0, n1 = 0, n2 = 0, pn = 0;
    uint32_t t0 = millis(), tP = 0;
    while (millis() - t0 < RUN_MS[k] && !aborted) {
      update(); drive(P[k], P[k]);
      uint32_t t = millis() - t0;
      pL += vL; pR += vR; pn++;
      if (t - tP >= 100) { Serial.printf("  %3.0f  %5lu %5.0f %6.0f\n", P[k], (unsigned long)t, pL / pn, pR / pn); pL = pR = 0; pn = 0; tP = t; }
      if (t > RUN_MS[k] - AVG_MS) {
        sL += vL; sR += vR; n++;
        if (t < RUN_MS[k] - AVG_MS / 2) { s1 += vL + vR; n1++; } else { s2 += vL + vR; n2++; }
      }
      delay(2);
    }
    if (n < 10) { brake(); delay(150); coast(); Serial.println("!! Stopped midway -> NOT saved"); return; }
    spd[k][0] = sL / n; spd[k][1] = sR / n;
    Serial.printf("At %.0f%%: LEFT %.0f mm/s, RIGHT %.0f mm/s (diff %.0f%%)\n", P[k], spd[k][0], spd[k][1],
                  100.0f * (spd[k][0] - spd[k][1]) / max(1.0f, (spd[k][0] + spd[k][1]) / 2));
    if (n1 > 5 && n2 > 5 && s2 / n2 > 1.03f * (s1 / n1)) Serial.printf("!! Speed still rising at %.0f%% -> result a bit low (more space? wheel slip?)\n", P[k]);
  }
  brake(); delay(200); coast();
  char l1[24], l2[24];
  float a = 0.5f * (spd[0][0] + spd[0][1]), b = 0.5f * (spd[1][0] + spd[1][1]);
  snprintf(l1, sizeof l1, "%.0f / %.0f mm/s", a, b);
  float slope = (b - a) / (P[1] - P[0]), db = slope > 1 ? P[0] - a / slope : -1, vf = slope * (100.0f - db);
  if (min(spd[0][0], spd[0][1]) < 50 || db < 0 || db > 50 || vf < 300) {
    Serial.printf("!! Odd result (slope %.1f mm/s per %%, dband %.1f) -> wheels not turning / encoder direction ('cprcnt', sgnL/sgnR) -> NOT saved\n", slope, db);
    uiBusy("WHEEL CAL ERROR", l1, "check encoder"); beep(1, 600); delay(1500); return;
  }
  cfg.vfull = vf; cfgSave(); dBand = db; prefs.putFloat("db", dBand);
  snprintf(l2, sizeof l2, "vfull %.0f db %.0f", cfg.vfull, dBand); uiBusy("WHEEL CAL OK", l1, l2);
  Serial.printf("=> SAVED vfull %.0f mm/s (100%%), dband %.1f %%. Wheel speed PID: %s\n", cfg.vfull, dBand, cfg.wpid ? "ON" : "OFF (type wpid 1)");
  Serial.println("   vfull changed a lot? the speed / kp / kd percentages now mean different real speeds -> scale them by old vfull / new vfull");
  beep(3); delay(1500);
}
void info() {
  Serial.printf("kp %.2f kd %.3f sp %d vmax %d | obs %.0f lat %.0f pass %.0f sens %.0f fin %.0f nobs %d side %s pref %s\n",
    cfg.kp, cfg.kd, cfg.base, cfg.vmax, cfg.obs, cfg.lat, cfg.pass, cfg.sensAx, cfg.finMin, cfg.nobs,
    cfg.side ? "RIGHT" : "LEFT", cfg.pref ? "RIGHT" : "LEFT");
  Serial.printf("aout %.0f aback %.0f | avoid %s: out %.0f par %.0f | wifi %d\n", cfg.aout, cfg.aback, cfg.avMode == 2 ? "FLOW" : cfg.avMode ? "MANUAL" : "AUTO", cfg.outMm, cfg.parMm, cfg.wifi);
  Serial.printf("cpr %.0f wheel %.1f track %.0f | calib %s black %s flip %d | enc %+d %+d gyro %+d | MPU %s VL53 %s\n",
    cfg.cpr, cfg.wheel, cfg.track, cfg.calOk ? "OK" : "NO", cfg.blackHigh ? "high" : "low", cfg.flip,
    cfg.sgnL, cfg.sgnR, cfg.gsgn, mpuOk ? "OK" : "NO", vlOk ? "OK" : "NO");
  Serial.printf("mask %d (manual off:", cfg.maskBits);
  bool any = false; for (int i = 0; i < 8; i++) if (cfg.maskBits >> i & 1) { Serial.printf(" D%d", i + 1); any = true; }
  Serial.printf("%s) | auto %d (auto-disable stuck channels while running) | jmode %d | cspd %.0f | fgo %.0f (stop mid finish box)\n", any ? "" : " no", cfg.noAuto ? 0 : 1, cfg.jMode, cfg.cornerSpd, cfg.finGo);
  Serial.printf("v3.2+: lag %.0f ms | wpid %d (%s) vfull %.0f dband %.1f wkp %.2f wki %.1f | alat %.0f | curve %d (%s)\n", cfg.sdly, cfg.wpid,
    wpidOk() ? "RUNNING" : cfg.wpid ? "NO wcal -> not active" : "off", cfg.vfull, dBand, cfg.wkp, cfg.wki, cfg.alat, cfg.curveOld ? 0 : 1, cfg.curveOld ? "curves like 90-degree corners" : "curve: straight + turn now");
  Serial.printf("v3.6 cornering: brake %.2f | kpb %.2f | piv %.1f %s | uturn %d | avscan %d | jnmode %d | cluster %d | vclust %d | pclust %d\n", cfg.brakeK, cfg.kpBoost, cfg.pivE, cfg.pivE > 0 ? "(pivot when near the edge)" : "(off)", cfg.cornerMode, cfg.avScan, cfg.jnMode, cfg.clusterMode, cfg.vCluster, cfg.pCluster);
}
void i2cScan() {
  Serial.print("I2C scan:"); int n = 0;
  for (uint8_t a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; } }
  Serial.println(n ? "" : " NOTHING FOUND");
}
bool quietCmd = false;   // loading several commands at once (split by ';') -> do not save / print each command
void doCmd(String s);
// v3.3 EXPORT PARAMETERS: print 1 line containing ALL current parameters. Copy that line:
//   - paste it back into the Serial Monitor (1 line, Enter) -> everything is reloaded
//   - send it to whoever writes the code -> paste into USER_CFG at the top -> the new code defaults = your parameters
// C3.17: the parameter line is built once, printed on Serial ('export') and shown on the web page (no USB cable on the track)
String cfgLine() {
  char b[1100]; int n = 0;
  n += snprintf(b + n, sizeof b - n, "kp %g;kd %g;sp %d;vmax %d;cspd %g;vfast %g;mpre %g;tlead %g;jmode %d;pref %d;mask %d;auto %d;",
    cfg.kp, cfg.kd, cfg.base, cfg.vmax, cfg.cornerSpd, cfg.mFast, cfg.mPre, cfg.turnLead, cfg.jMode, cfg.pref, cfg.maskBits, cfg.noAuto ? 0 : 1);
  n += snprintf(b + n, sizeof b - n, "cpr %g;wheel %g;track %g;sens %g;fin %g;fgo %g;vk %g;",
    cfg.cpr, cfg.wheel, cfg.track, cfg.sensAx, cfg.finMin, cfg.finGo, cfg.vcal);
  n += snprintf(b + n, sizeof b - n, "obs %g;lat %g;pass %g;nobs %d;side %d;avmode %d;aout %g;aback %g;out %g;par %g;vout %g;vpar %g;vret %g;vturn %g;retm %d;",
    cfg.obs, cfg.lat, cfg.pass, cfg.nobs, cfg.side, cfg.avMode, cfg.aout, cfg.aback, cfg.outMm, cfg.parMm, cfg.spdOut, cfg.spdPar, cfg.spdRet, cfg.spdTurn, cfg.retMode);
  snprintf(b + n, sizeof b - n, "brev %g;bms %g;dec %g;roled %d;wifi %d;lag %g;wpid %d;wkp %g;wki %g;alat %g;vfull %g;curve %d;brake %g;kpb %g;piv %g;uturn %d;avscan %d;jnmode %d;cluster %d;vclust %d;pclust %d;dband %g",
    cfg.brkRev, cfg.brkMs, cfg.decMm, cfg.runOled, cfg.wifi, cfg.sdly, cfg.wpid, cfg.wkp, cfg.wki, cfg.alat, cfg.vfull, cfg.curveOld ? 0 : 1, cfg.brakeK, cfg.kpBoost, cfg.pivE, cfg.cornerMode, cfg.avScan, cfg.jnMode, cfg.clusterMode, cfg.vCluster, cfg.pCluster, dBand);
  return String(b);
}
void exportCfg() {
  Serial.println("\n== CURRENT PARAMETERS (copy the WHOLE line below, paste it back here to load; or paste into USER_CFG at the top of the code) ==");
  Serial.println(cfgLine());
  Serial.println("(Sensor calibration is NOT in this line -> after changing the sensor bar always run 'cal')");
}
// 1 line, several commands "kp 14;kd 0.35;sp 80" -> run in order, save once
void runCmdLine(String s, bool verbose) {
  s.trim(); if (s.startsWith("load ")) s = s.substring(5); else if (s.startsWith("nap ")) s = s.substring(4);   // "nap" = old name
  if (s.indexOf(';') < 0) { quietCmd = !verbose; doCmd(s); quietCmd = false; return; }
  quietCmd = true; int a = 0;
  while (a <= (int)s.length()) {
    int b = s.indexOf(';', a); if (b < 0) b = s.length();
    String part = s.substring(a, b); part.trim(); if (part.length()) doCmd(part);
    a = b + 1;
  }
  quietCmd = false;
  if (verbose) { cfgSave(); Serial.println("=> LOADED + SAVED"); info(); }
}
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim();
  runCmdLine(s, true);
}
// Accept the original command names so parameter lines exported by older firmware still load.
String legacyKey(const String &k) {
  static const char *const MAP[][2] = {{"xuat", "export"}, {"ham", "brake"}, {"tre", "lag"}, {"dem", "cprcnt"}, {"cong", "curve"}, {"cum", "cluster"}, {"vcum", "vclust"}, {"pcum", "pclust"}, {"qd", "uturn"}, {"nedo", "avscan"}, {"ngare", "jnmode"}, {"nhieu", "noise"}, {"mapxoa", "mapclr"}};
  for (auto &m : MAP) if (k == m[0]) return m[1];
  return k;
}
void doCmd(String s) {
  char c[8] = {0}; float v = 0; int n = sscanf(s.c_str(), "%7s %f", c, &v);
  if (n < 1) return;
  String k = legacyKey(String(c)); bool set = n == 2;
  if      (k == "kp" && set) cfg.kp = v;
  else if (k == "kd" && set) cfg.kd = v;
  else if (k == "sp" && set) cfg.base = constrain((int)v, 10, 100);
  else if (k == "vmax" && set) cfg.vmax = constrain((int)v, 10, 100);
  else if (k == "obs" && set) cfg.obs = v;
  else if (k == "lat" && set) cfg.lat = v;
  else if (k == "pass" && set) cfg.pass = v;
  else if (k == "sens" && set) cfg.sensAx = v;
  else if (k == "fin" && set) cfg.finMin = v;
  else if (k == "nobs" && set) cfg.nobs = constrain((int)v, 0, 20);
  else if (k == "side" && set) cfg.side = v != 0;
  else if (k == "pref" && set) cfg.pref = v != 0;
  else if (k == "jmode" && set) cfg.jMode = v != 0;
  else if (k == "mask" && set) { cfg.maskBits = constrain((int)v, 0, 255); for (int i = 0; i < 8; i++) sStuck[i] = false; }
  else if (k == "mode" && set) { runMode = constrain((int)v, 0, 2); Serial.printf("MODE: %s\n", MODEN[runMode]); return; }
  else if (k == "vfast" && set) cfg.mFast = constrain(v, 20.0f, 100.0f);
  else if (k == "mpre" && set) cfg.mPre = constrain(v, 50.0f, 800.0f);
  else if (k == "map") { mapPrint(); return; }
  else if (k == "mapclr") { mapClear(); Serial.println("Map CLEARED"); return; }
  else if (k == "lag" && set) cfg.sdly = constrain(v, 0.0f, 80.0f);
  else if (k == "wpid" && set) cfg.wpid = v != 0;
  else if (k == "wkp" && set) cfg.wkp = constrain(v, 0.0f, 5.0f);
  else if (k == "wki" && set) cfg.wki = constrain(v, 0.0f, 30.0f);
  else if (k == "alat" && set) cfg.alat = constrain(v, 0.0f, 20000.0f);
  else if (k == "vfull" && set) cfg.vfull = constrain(v, 0.0f, 5000.0f);
  else if (k == "dband" && set) { dBand = constrain(v, 0.0f, 50.0f); prefs.putFloat("db", dBand); }
  else if (k == "wcal") { wheelCal(); return; }
  else if (k == "curve" && set) cfg.curveOld = v == 0;
  else if (k == "brake" && set) cfg.brakeK = constrain(v, 0.0f, 0.95f);
  else if (k == "kpb" && set) cfg.kpBoost = constrain(v, 1.0f, 2.0f);
  else if (k == "piv" && set) cfg.pivE = constrain(v, 0.0f, 3.5f);
  else if (k == "uturn" && set) cfg.cornerMode = v != 0;
  else if (k == "cluster" && set) cfg.clusterMode = v != 0;
  else if (k == "vclust" && set) cfg.vCluster = constrain((int)v, 20, 100);
  else if (k == "pclust" && set) cfg.pCluster = constrain((int)v, 0, 2);
  else if (k == "xv") Serial.println("C3: 'xv' REMOVED (pivot at corner apex). Anti U-turn: uturn 1 / uturn 0");
  else if (k == "avscan" && set) cfg.avScan = v != 0;
  else if (k == "jnmode" && set) cfg.jnMode = v != 0;
  else if (k == "fgo" && set) cfg.finGo = constrain(v, 0.0f, 600.0f);
  else if (k == "auto" && set) { cfg.noAuto = v == 0; for (int i = 0; i < 8; i++) sStuck[i] = false; }
  else if (k == "cpr" && set) cfg.cpr = v;
  else if (k == "wheel" && set) cfg.wheel = v;
  else if (k == "track" && set) cfg.track = v;
  else if (k == "aout" && set) cfg.aout = constrain(v, 30.0f, 90.0f);
  else if (k == "aback" && set) cfg.aback = constrain(v, 20.0f, 90.0f);
  else if (k == "avmode" && set) cfg.avMode = constrain((int)v, 0, 3);   // 0 auto | 1 manual | 2 manual FLOWING (v3.5) | 3 ARC, no stops (C3.19)
  else if (k == "out" && set) cfg.outMm = constrain(v, 30.0f, 800.0f);
  else if (k == "par" && set) cfg.parMm = constrain(v, 0.0f, 1000.0f);
  else if (k == "vout" && set) cfg.spdOut = constrain(v, 20.0f, 100.0f);
  else if (k == "vpar" && set) cfg.spdPar = constrain(v, 20.0f, 100.0f);
  else if (k == "vret" && set) cfg.spdRet = constrain(v, 20.0f, 100.0f);
  else if (k == "vturn" && set) cfg.spdTurn = constrain(v, 25.0f, 100.0f);
  else if (k == "brev" && set) cfg.brkRev = constrain(v, 0.0f, 100.0f);
  else if (k == "bms" && set) cfg.brkMs = constrain(v, 0.0f, 300.0f);
  else if (k == "dec" && set) cfg.decMm = constrain(v, 0.0f, 300.0f);
  else if (k == "retm" && set) cfg.retMode = v != 0;
  else if (k == "roled" && set) cfg.runOled = v != 0;
  else if (k == "cspd" && set) cfg.cornerSpd = constrain(v, 20.0f, 100.0f);
  else if (k == "tlead" && set) cfg.turnLead = constrain(v, 0.0f, 150.0f);
  else if (k == "wifi" && set) { cfg.wifi = v != 0; if (!cfg.wifi) wifiStop(); }
  else if (k == "log") { logToSerial(); return; }
  else if (k == "vk" && set) cfg.vcal = constrain(v, 0.5f, 2.0f);
  else if (k == "export") { exportCfg(); return; }
  else if (k == "vcal" && set) { float r = readVbat() / cfg.vcal; if (r > 1) cfg.vcal = v / r; }
  else if (k == "cprcnt") {          // measure CPR: type cprcnt -> turn one wheel by hand exactly 10 turns -> type cprcnt again
    static bool counting = false; static int32_t l0 = 0, r0 = 0;
    if (!counting) { counting = true; l0 = rawL; r0 = rawR;
      Serial.println("PULSE COUNT: mark the wheel, turn it by hand EXACTLY 10 TURNS (1 or both wheels), then type cprcnt"); }
    else { counting = false;
      long dl = labs((long)(rawL - l0)), dr = labs((long)(rawR - r0));
      Serial.printf("Left %ld pulses -> %.1f /turn | Right %ld pulses -> %.1f /turn\n", dl, dl / 10.0f, dr, dr / 10.0f);
      float c = (dl > 500 && dr > 500) ? (dl + dr) / 20.0f : max(dl, dr) / 10.0f;
      if (c > 50) { cfg.cpr = c; cfgSave(); Serial.printf("=> SAVED cpr %.0f\n", c); }
      else Serial.println("!! Too few pulses: encoder not counting (check wires 21/47, 48/1)");
    }
    return;
  }
  else if (k == "cal") { calibrate(); return; }
  else if (k == "go") { runCourse(); return; }
  else if (k == "scan") { i2cScan(); return; }
  else if (k == "info") { info(); return; }
  else if (k == "reset") { cfgDefault(); cfgSave(); Serial.println("Reset to defaults -> RECALIBRATE"); info(); return; }
  else if (k == "flip") { cfg.flip ^= 1; cfg.calOk = 0; cfgSave(); Serial.println("Channel order reversed -> RECALIBRATE"); return; }
  else if (k == "btn") {
    Serial.println("Button states 10 s (1 = released, 0 = pressed):"); uint32_t t0 = millis(); int last = -1;
    while (millis() - t0 < 10000) { int st = 0; for (int i = 0; i < 3; i++) st |= digitalRead(BTN[i]) << i;
      if (st != last) { last = st; Serial.printf("  START=%d SET=%d MODE=%d\n", st & 1, (st >> 1) & 1, (st >> 2) & 1); } delay(10); }
    return;
  }
  else if (k == "raw") {
    wifiStop();
    Serial.println("8 channels for 5 s:"); uint32_t t0 = millis();
    while (millis() - t0 < 5000) { readLine(); vlPoll();
      for (int i = 0; i < 8; i++) Serial.printf("%c", val[i] > 0.5f ? '#' : '.');
      Serial.printf("  pos %+5.2f black %d branches %d %s | vl %u mm | yaw %.1f\n", pos, nBlack, nBlob, junc ? "JUNC" : "", vlMM, heading());
      for (uint32_t t1 = millis(); millis() - t1 < 150;) update(); }
    return;
  }
  else if (k == "noise") {
    // SENSOR NOISE TEST: robot on WHITE floor (no line), WHEELS LIFTED. 4 s motors off, then 4 s motors at 50%.
    // A channel reporting "black" on white floor -> wrong height / sees an object / loose wire. Wrong only with motors running -> electrical noise.
    wifiStop();
    Serial.println("\n== NOISE TEST: robot on WHITE floor, LIFT BOTH WHEELS off the ground ==");
    for (int ph = 0; ph < 2; ph++) {
      uint16_t mn[8], mx[8]; uint32_t nb[8] = {0}, n = 0; double sm[8] = {0};
      for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
      if (ph) { Serial.println("-- motor RUN 50% (4 s) --"); drive(50, 50); } else Serial.println("-- motor OFF (4 s) --");
      uint32_t t0 = millis();
      while (millis() - t0 < 4000) {
        for (int i = 0; i < 8; i++) {
          uint16_t r = analogRead(LPIN[cfg.flip ? 7 - i : i]);
          mn[i] = min(mn[i], r); mx[i] = max(mx[i], r); sm[i] += r;
          float span = max(1, (int)cfg.lmax[i] - (int)cfg.lmin[i]);
          float v = constrain(((int)r - (int)cfg.lmin[i]) / span, 0.0f, 1.0f); if (!cfg.blackHigh) v = 1 - v;
          if (v > 0.5f) nb[i]++;
        }
        n++; update();
      }
      coast();
      Serial.println("  ch | mean | min - max (swing) | % BLACK | white/black cal");
      for (int i = 0; i < 8; i++) {
        float pb = 100.0f * nb[i] / max(1UL, (unsigned long)n);
        Serial.printf("  D%d  |   %4.0f     | %4u - %4u (%4u)    | %5.1f %%  | %u / %u %s\n", i + 1, sm[i] / max(1UL, (unsigned long)n), mn[i], mx[i], mx[i] - mn[i], pb,
                      cfg.lmin[i], cfg.lmax[i], pb > 2 ? (ph ? " <- WRONG with motors running: ELECTRICAL NOISE" : " <- WRONG even with motors off: HEIGHT / LIGHT / FLOOR") : "");
      }
      delay(300);
    }
    Serial.println("Conclusion: wrong only with motors running -> electrical noise (twist wires, 100nF caps, separate motor wires). Wrong even when off -> clean channels, adjust height, shield light.");
    return;
  }
  else { Serial.println("Commands: kp kd sp vmax obs lat pass sens fin nobs side pref cpr wheel track vcal aout aback avmode out par vout vpar vret vturn brev bms dec retm tlead roled cspd jmode mask auto fgo mode vfast mpre wifi | lag wpid wkp wki vfull dband alat wcal curve brake kpb piv uturn avscan jnmode cluster vclust pclust | export (print all parameters) | map mapclr | log cprcnt cal go raw noise info btn scan flip reset"); return; }
  if (!quietCmd) { cfgSave(); info(); }
}
// =====================================================================
//                         OLED USER INTERFACE 128x64
// =====================================================================
GFXcanvas1 oled(128, 64);          // draw into the RAM buffer, then send to the OLED
bool oledOk = false; uint8_t oledAddr = 0x3C;
uint8_t oledBuf[1024]; bool oledPending = false; int oledChunk = 0;
void oledCmd(uint8_t c) { Wire.beginTransmission(oledAddr); Wire.write(0x00); Wire.write(c); Wire.endTransmission(); }
void oledInit() {
  const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,   // 0x20 0x00: horizontal addressing
                          0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
  for (uint8_t c : init) oledCmd(c);
}
// convert the canvas (horizontal rows) -> SSD1306 page format, start sending
void oledPrepare() {
  const uint8_t* src = oled.getBuffer();
  memset(oledBuf, 0, sizeof oledBuf);
  for (int y = 0; y < 64; y++) {
    uint8_t bit = 1 << (y & 7); int pg = (y >> 3) << 7; const uint8_t* row = src + (y << 4);
    for (int x = 0; x < 128; x++) if (row[x >> 3] & (0x80 >> (x & 7))) oledBuf[pg + x] |= bit;
  }
  oledChunk = 0; oledPending = true;
}
// send one 16-byte packet (~0.45 ms at 400 kHz). 64 packets = 1 frame
void oledStep() {
  if (!oledPending) return;
  if (oledChunk == 0) { oledCmd(0x21); oledCmd(0); oledCmd(127); oledCmd(0x22); oledCmd(0); oledCmd(7); }
  Wire.beginTransmission(oledAddr); Wire.write(0x40); Wire.write(oledBuf + oledChunk * 16, 16); Wire.endTransmission();
  if (++oledChunk >= 64) oledPending = false;
}
enum { P_HOME, P_SPEED, P_VMAX, P_SENSOR, P_CAL, P_AVOID, P_RESULT, P_INFO, P_WIFI, NPAGE };
const char* PNAME[NPAGE] = {"READY", "CORNER SPEED", "STRAIGHT SPEED", "SENSOR", "CALIBRATE", "OBSTACLE AVOID", "RESULT", "INFO", "WIFI LOG"};
const char* PHINT[NPAGE] = {"SET:MODE", "SET: +10%", "SET: +10%", "", "SET: START", "SET: SWAP SIDE", "START: RUN", "SET: WHEEL CAL", "SET: ON/OFF"};
int page = P_HOME; bool dirty = true; uint32_t tDraw = 0, tBat = 0;
float vbat = 0;
float readVbat() {
  uint32_t t = 0; for (int i = 0; i < 8; i++) t += analogRead(PIN_VBAT);
  return t / 8.0f * 3.3f / 4095.0f * VBAT_RATIO * cfg.vcal;
}
bool batLow() { return vbat > 3.0f && vbat < VBAT_LOW; }
void oledShow() {                       // draw the whole frame (used when the robot is still, ~30 ms)
  if (!oledOk) return;
  oledPrepare(); while (oledPending) oledStep();
}
void txt(int x, int y, int sz, const char* s) { oled.setTextSize(sz); oled.setCursor(x, y); oled.print(s); }
void txtC(int y, int sz, const char* s) { txt((128 - (int)strlen(s) * 6 * sz) / 2, y, sz, s); }
void txtR(int xr, int y, int sz, const char* s) { txt(xr - (int)strlen(s) * 6 * sz, y, sz, s); }
// status box: pass = white background black text, fail = border + diagonal stripe
void chip(int x, int y, int w, const char* s, bool ok) {
  if (ok) { oled.fillRoundRect(x, y, w, 11, 2, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
  else    { oled.drawRoundRect(x, y, w, 11, 2, SSD1306_WHITE); oled.drawLine(x + 2, y + 9, x + w - 3, y + 1, SSD1306_WHITE); }
  txt(x + (w - (int)strlen(s) * 6) / 2 + 1, y + 2, 1, s);
  oled.setTextColor(SSD1306_WHITE);
}
void battIcon(int x, int y) {
  oled.drawRect(x, y, 13, 7, SSD1306_BLACK); oled.fillRect(x + 13, y + 2, 2, 3, SSD1306_BLACK);
  float f = constrain((vbat - 6.6f) / (8.4f - 6.6f), 0.0f, 1.0f);
  int w = (int)(f * 9 + 0.5f);
  if (w > 0) oled.fillRect(x + 2, y + 2, w, 3, SSD1306_BLACK);
}
void header() {
  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  txt(2, 2, 1, PNAME[page]);
  char b[8];
  if (vbat > 3.0f) snprintf(b, sizeof b, "%.1fV", vbat); else snprintf(b, sizeof b, "--V");
  txtR(110, 2, 1, b);
  battIcon(112, 2);
  oled.setTextColor(SSD1306_WHITE);
}
void footer() {
  oled.drawFastHLine(0, 53, 128, SSD1306_WHITE);
  if (page == P_SENSOR) { char b[16]; snprintf(b, sizeof b, "D %.0fmm", dist()); txt(0, 56, 1, b); }   // push the robot 1 m to check cpr
  else txt(0, 56, 1, PHINT[page]);
  for (int i = 0; i < NPAGE; i++) {        // page dots
    int x = 128 - NPAGE * 5 + i * 5 + 1;
    if (i == page) oled.fillRect(x, 57, 3, 5, SSD1306_WHITE); else oled.drawPixel(x + 1, 59, SSD1306_WHITE);
  }
}
void bigValue(int v, int vmin, int vmax, const char* note) {
  char b[8]; snprintf(b, sizeof b, "%d%%", v);
  txtC(15, 3, b);
  (void)note;
  oled.drawRoundRect(4, 41, 120, 8, 3, SSD1306_WHITE);
  int w = (int)(116.0f * (v - vmin) / max(1, vmax - vmin));
  if (w > 0) oled.fillRoundRect(6, 43, w, 4, 1, SSD1306_WHITE);
}
void pageHome() {
  if (!cfg.calOk) {
    oled.fillRoundRect(4, 14, 120, 16, 3, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK);
    txtC(18, 1, "NOT CALIBRATED"); oled.setTextColor(SSD1306_WHITE);
  } else if (batLow() && (millis() / 500) % 2) {
    oled.fillRoundRect(4, 13, 120, 18, 3, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK);
    txtC(15, 2, "BATT LOW!"); oled.setTextColor(SSD1306_WHITE);
  } else txtC(15, 2, "READY");
  char b[26];
  if (runMode == 0) snprintf(b, sizeof b, "NORMAL %d/%d%%", cfg.base, cfg.vmax);
  else if (runMode == 1) snprintf(b, sizeof b, "DEMO: RECORD MAP");
  else if (mapOk) snprintf(b, sizeof b, "FAST %.1fm %dd", mapLen / 1000, nNode);
  else snprintf(b, sizeof b, "FAST: NO MAP");
  if (runMode) { oled.fillRect(0, 31, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
  txtC(32, 1, b); oled.setTextColor(SSD1306_WHITE);
  chip(1, 41, 30, "MPU", mpuOk); chip(33, 41, 30, "VL53", vlOk); chip(65, 41, 30, "LINE", cfg.calOk);
  chip(97, 41, 30, "BAT", vbat > 3.0f && !batLow());
}
void pageSensor() {
  const int top = 13, h = 22;
  for (int i = 0; i < 8; i++) {
    int x = 3 + i * 16;
    oled.drawRect(x, top, 13, h, SSD1306_WHITE);
    int f = (int)(val[i] * (h - 2) + 0.5f);
    if (f > 0) oled.fillRect(x + 1, top + h - 1 - f, 11, f, SSD1306_WHITE);
  }
  // belongs to the line position
  oled.drawFastHLine(3, 39, 125, SSD1306_WHITE);
  oled.drawFastVLine(65, 37, 5, SSD1306_WHITE);
  if (seen) { int x = 65 + (int)(pos / 3.5f * 58); oled.fillTriangle(x - 3, 43, x + 3, 43, x, 38, SSD1306_WHITE); }
  char b[28];
  if (seen) snprintf(b, sizeof b, "P%+.1f", pos); else snprintf(b, sizeof b, "P --");
  txt(0, 45, 1, b);
  if (vlOk) snprintf(b, sizeof b, "VL%4u", vlMM > 2000 ? 0 : vlMM); else snprintf(b, sizeof b, "VL --");
  txt(40, 45, 1, b);
  if (junc) chip(98, 44, 30, "JUNC", true);
  else { snprintf(b, sizeof b, "Y%+4.0f", heading()); txt(86, 45, 1, b); }
}
void pageCal() {
  txtC(14, 1, "Sensor bar on LINE");
  txtC(24, 1, "auto turn ~5 s");
  char b[32];
  if (calGood < 0) snprintf(b, sizeof b, "%s", cfg.calOk ? "Calibrated (saved)" : "Not calibrated");
  else snprintf(b, sizeof b, "Result: %d/8 channels %s", calGood, cfg.calOk ? "OK" : "ERROR");
  if (calGood >= 0 && !cfg.calOk) { oled.fillRect(0, 34, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
  txtC(35, 1, b); oled.setTextColor(SSD1306_WHITE);
  snprintf(b, sizeof b, "enc %+d %+d  gyro %+d", cfg.sgnL, cfg.sgnR, cfg.gsgn);
  txtC(45, 1, b);
}
void pageAvoid() {
  txt(2, 15, 1, "Avoid side:");
  txtR(126, 13, 2, cfg.side ? "RIGHT" : "LEFT");
  char b[24];
  if (cfg.avMode) {
    snprintf(b, sizeof b, "MN out%3.0f par%3.0f", cfg.outMm, cfg.parMm);   txt(2, 33, 1, b);
    snprintf(b, sizeof b, "obs %3.0f ang %.0f/%.0f", cfg.obs, cfg.aout, cfg.aback); txt(2, 43, 1, b);
  } else {
    snprintf(b, sizeof b, "AUTO lat %3.0f", cfg.lat);   txt(2, 33, 1, b);
    snprintf(b, sizeof b, "obs %3.0f ang %.0f/%.0f", cfg.obs, cfg.aout, cfg.aback); txt(2, 43, 1, b);
  }
}
void pageResult() {
  if (!res.has) { txtC(26, 1, "No run yet"); return; }
  bool fin = strcmp(res.why, "TO FINISH") == 0;
  if (fin) txtC(14, 1, "TO FINISH");
  else { oled.fillRect(0, 13, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); txtC(14, 1, res.why); oled.setTextColor(SSD1306_WHITE); }
  char b[24]; snprintf(b, sizeof b, "%.2fs", res.t); txtC(25, 2, b);
  snprintf(b, sizeof b, "%.2fm %.2fm/s OB%d", res.d / 1000.0f, res.v, res.obs); txtC(43, 1, b);
}
void pageInfo() {
  char b[26];
  snprintf(b, sizeof b, "kp %.1f  kd %.2f", cfg.kp, cfg.kd);              txt(0, 13, 1, b);
  snprintf(b, sizeof b, "cpr %.0f  wheel %.0f", cfg.cpr, cfg.wheel);         txt(0, 23, 1, b);
  snprintf(b, sizeof b, "track %.0f  sens %.0f", cfg.track, cfg.sensAx);    txt(0, 33, 1, b);
  snprintf(b, sizeof b, "vfull %.0f lag %.0f %s", cfg.vfull, cfg.sdly, wpidOk() ? "PID" : "");  txt(0, 43, 1, b);
}
bool wifiOn = false;
void pageWifi() {
  if (!cfg.wifi) { txtC(20, 1, "WiFi is OFF"); txtC(34, 1, "SET to turn on"); return; }
  txt(0, 13, 1, "Net: LINE-ROBOT"); txt(0, 23, 1, "Pass: 12345678");
  txt(0, 33, 1, "Web : 192.168.4.1");
  char b[24];
  if (wifiOn) snprintf(b, sizeof b, "ON | %d clients connected", WiFi.softAPgetStationNum()); else snprintf(b, sizeof b, "starting...");
  txt(0, 43, 1, b);
}
void drawPage() {
  if (!oledOk) return;
  oled.fillScreen(0); oled.setTextColor(SSD1306_WHITE);
  header();
  switch (page) {
    case P_HOME: pageHome(); break;
    case P_SPEED: bigValue(cfg.base, 20, 80, "corner"); break;
    case P_VMAX: bigValue(cfg.vmax, 30, 100, "straight"); break;
    case P_SENSOR: pageSensor(); break;
    case P_CAL: pageCal(); break;
    case P_AVOID: pageAvoid(); break;
    case P_RESULT: pageResult(); break;
    case P_INFO: pageInfo(); break;
    case P_WIFI: pageWifi(); break;
  }
  footer();
  oledShow();
}
// "working" screen (calibration, get ready, errors)
void uiBusy(const char* title, const char* l1, const char* l2) {
  if (!oledOk) return;
  oled.fillScreen(0);
  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); txtC(2, 1, title);
  oled.setTextColor(SSD1306_WHITE);
  txtC(22, 1, l1); txtC(36, 1, l2);
  oled.drawRoundRect(0, 0, 128, 64, 3, SSD1306_WHITE);
  oledShow();
}
// screen while RUNNING: drawn once, not updated any more
void uiGo() {
  if (!oledOk) return;
  oled.fillScreen(0);
  oled.fillRect(0, 0, 128, 64, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK);
  txtC(8, 3, "GO!");
  char b[24]; snprintf(b, sizeof b, "CRN %d%%  STR %d%%", cfg.base, cfg.vmax); txtC(38, 1, b);
  txtC(50, 1, "press a button = STOP");
  oled.setTextColor(SSD1306_WHITE);
  oledShow();
}
void uiAfterRun() { page = P_RESULT; dirty = true; }
float vbatQuick() { return analogRead(PIN_VBAT) * 3.3f / 4095.0f * VBAT_RATIO * cfg.vcal; }
void uiRun(float tS, float dMm, float spd, const char* st) {
  if (!oledOk || oledPending) return;          // previous frame not fully sent -> skip
  static float vb = 0; float v = vbatQuick(); vb = vb < 0.1f ? v : vb * 0.8f + v * 0.2f;
  oled.fillScreen(0); oled.setTextColor(SSD1306_WHITE);
  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK);
  txt(2, 2, 1, st);
  char b[24]; snprintf(b, sizeof b, "%.1fV", vb); txtR(126, 2, 1, b);
  oled.setTextColor(SSD1306_WHITE);
  snprintf(b, sizeof b, "%.1fs", tS); txtC(16, 3, b);
  snprintf(b, sizeof b, "%.2fm", dMm / 1000.0f); txt(2, 44, 1, b);
  snprintf(b, sizeof b, "OB %d", nObs); txtC(44, 1, b);
  snprintf(b, sizeof b, "%3.0f%%", spd); txtR(126, 44, 1, b);
  int w = (int)(124 * constrain(spd, 0.0f, 100.0f) / 100.0f);
  oled.drawRect(0, 55, 128, 8, SSD1306_WHITE); if (w > 0) oled.fillRect(2, 57, w, 4, SSD1306_WHITE);
  oledPrepare();
}
void uiRunStep() {
  static uint32_t t = 0;
  if (oledPending && micros() - t >= 3000) { t = micros(); oledStep(); }
}
void uiAfterCal() { page = P_CAL; dirty = true; }
void splash() {
  if (!oledOk) return;
  oled.fillScreen(0); oled.setTextColor(SSD1306_WHITE);
  oled.drawRoundRect(0, 0, 128, 64, 4, SSD1306_WHITE);
  txtC(7, 2, "ROBOT");
  txtC(26, 1, "LINE ROBOT");
  oled.drawFastHLine(24, 37, 80, SSD1306_WHITE);
  chip(4, 44, 38, "MPU", mpuOk); chip(45, 44, 38, "VL53", vlOk); chip(86, 44, 38, "LINE", cfg.calOk);
  oledShow();
}
// =====================================================================
//         WIFI: read the log + edit parameters from a phone (no USB cable needed)
// =====================================================================
WebServer web(80);
void wifiStart() {
  if (wifiOn || !cfg.wifi) return;
  WiFi.mode(WIFI_AP); WiFi.softAP("LINE-ROBOT", "12345678");
  WiFi.setTxPower(WIFI_POWER_8_5dBm);           // low TX power (enough for a phone next to the robot) -> less noise on the 3V3 supply
  web.begin(); wifiOn = true; dirty = true;
  Serial.println("WiFi ON: network LINE-ROBOT, pass 12345678, open 192.168.4.1");
}
void wifiStop() {
  if (!wifiOn) return;
  web.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); wifiOn = false;
  delay(150);                                    // let WiFi release ADC2 (4 line channels D1..D4 use ADC2)
  for (int k = 0; k < 4; k++) for (int i = 0; i < 4; i++) analogRead(LPIN[i]);   // discard a few reads so ADC2 settles
}
// print the whole log (parameters + results + events + 50 ms table) through out()
template <class F> void writeLog(F out) {
  char b[160];
  snprintf(b, sizeof b, "LINE ROBOT C3.31 | battery %.2f V | MPU %s VL53 %s\n", vbat, mpuOk ? "OK" : "NO", vlOk ? "OK" : "NO"); out(b);
  snprintf(b, sizeof b, "kp %.2f kd %.3f sp %d vmax %d | cpr %.0f wheel %.1f track %.0f sens %.0f fin %.0f\n",
           cfg.kp, cfg.kd, cfg.base, cfg.vmax, cfg.cpr, cfg.wheel, cfg.track, cfg.sensAx, cfg.finMin); out(b);
  snprintf(b, sizeof b, "obs %.0f lat %.0f pass %.0f aout %.0f aback %.0f nobs %d side %s | enc %+d %+d gyro %+d\n",
           cfg.obs, cfg.lat, cfg.pass, cfg.aout, cfg.aback, cfg.nobs, cfg.side ? "RIGHT" : "LEFT", cfg.sgnL, cfg.sgnR, cfg.gsgn); out(b);
  snprintf(b, sizeof b, "lag %.0f ms | wpid %d vfull %.0f (%s) | curve %d | alat %.0f\n", cfg.sdly, cfg.wpid, cfg.vfull, wpidOk() ? "PID active" : "PID off", cfg.curveOld ? 0 : 1, cfg.alat); out(b);
  snprintf(b, sizeof b, "90-degree cspd %.0f | junction jmode %d pref %s | mask %d | fgo %.0f | stuck channels last run:", cfg.cornerSpd, cfg.jMode, cfg.pref ? "RIGHT" : "LEFT", cfg.maskBits, cfg.finGo); out(b);
  if (!stuckEv) out(" no\n"); else { for (int i = 0; i < 8; i++) if (stuckEv >> i & 1) { snprintf(b, sizeof b, " D%d", i + 1); out(b); } out("  <- clean/resolder/adjust height\n"); }
  snprintf(b, sizeof b, "last run mode: %s | map: %s %d points %.1f m (demo %.2f s) | vfast %.0f mpre %.0f\n", MODEN[runMode], mapOk ? "YES" : "NO", nNode, mapLen / 1000, mapT, cfg.mFast, cfg.mPre); out(b);
  snprintf(b, sizeof b, "ADC2 read errors (=0) in the last run: %lu times %s\n", (unsigned long)adc2Zero, adc2Zero > 20 ? "<- WiFi/ADC2 problem" : "(OK)"); out(b);
  snprintf(b, sizeof b, "calibration (white-black diff per channel, < 300 = WEAK channel): %u %u %u %u %u %u %u %u\n",
           cfg.lmax[0] - cfg.lmin[0], cfg.lmax[1] - cfg.lmin[1], cfg.lmax[2] - cfg.lmin[2], cfg.lmax[3] - cfg.lmin[3],
           cfg.lmax[4] - cfg.lmin[4], cfg.lmax[5] - cfg.lmin[5], cfg.lmax[6] - cfg.lmin[6], cfg.lmax[7] - cfg.lmin[7]); out(b);
  if (!res.has) { out("No run yet.\n"); return; }
  snprintf(b, sizeof b, "VL53 last run: %lu reads, NEAREST object %u mm (obs %.0f -> %s)\n", (unsigned long)vlN, vlMinRun, cfg.obs,
           !vlOk ? "VL53 NOT STARTED" : vlN == 0 ? "NOTHING READ - check wiring / I2C" : vlMinRun >= 8190 ? "NOTHING SEEN - VL53 covered / pointing up / down at floor?" : vlMinRun < cfg.obs ? "object already within avoid range" : "object not yet inside obs range"); out(b);
  snprintf(b, sizeof b, "LAST AVOID (%s): VL53 saw object at %.0f mm | out angle %.0f | diagonal %.0f (lateral %.0f) | parallel %.0f",
           cfg.avMode == 2 ? "FLOW" : cfg.avMode ? "MANUAL" : "AUTO", avGap, avAng, avOut, avLat, avPass); out(b);
  snprintf(b, sizeof b, " | SLIP after straight: diagonal out %.0f, parallel %.0f mm", avOverOut, avOverPar); out(b);
  snprintf(b, sizeof b, " | WRONG TURN ANGLE: out %+.0f, parallel %+.0f, back %+.0f deg | back to line: %s", avE1, avE3, avE5, cfg.retMode ? "STOP-ALIGN-TURN" : "FOLLOW DIRECTLY"); out(b);
  if (!cfg.avMode) { snprintf(b, sizeof b, " | brick measured width ~%.0f (%s)", 2 * avHalfW, avLen < 150 ? "ACROSS" : "READ"); out(b); }
  out("\n");
  snprintf(b, sizeof b, "RESULT: %s | %.2f s | %.0f mm | %.2f m/s | %d obstacles | %d line losses\n", res.why, res.t, res.d, res.v, res.obs, res.lost); out(b);
  out("EVENTS (E circle cluster: follow the arcs, e circle exit line found, K 90-degree corner seen, L line lost, O obstacle met, P start: drive straight out of the black box, H hexagon cluster: main heading locked, 1/4/5/7 avoid step started (7 = arc to find line), T line seen while avoiding, M follow line while moving, G line seen mid-way -> follow now, C line caught (old way: stop-center-turn), X avoid saw no line, Z failed to catch line, R back up to find line (overshot), V spinning stopped, o avoid done, F finish, u ignore U-TURN line in cluster, q line lost on curve -> straight + turn now, W avoid: crossed the line -> drive onto it then turn, D avoid: following backward -> turn 180, B avoid: too close to brick -> back up, m turn by MAP, x map mismatch -> off, g line lost in the middle -> go straight to re-catch (dashed line), > < chose turn RIGHT/LEFT, reason: b branch seen on 1 side, n T-junction by preference, f unclear):\n");
  snprintf(b, sizeof b, "  (%d junctions J, %d line losses L; important events below, J omitted)\n", cntJ, cntL); out(b);
  for (int i = 0; i < evCount(); i++) { const Ev &e = evAt(i); if (e.c == 'J') continue; snprintf(b, sizeof b, "  %c %6.0f mm %6.2f s  heading %+7.1f  black %u\n", e.c, e.d, e.t, e.h, e.nb); out(b); }
  out("TABLE 50ms: t(s) d(mm) pos(-3.5..3.5, -- = lost) spd(%) step(P start,L line,J junction,K back to corner,S search,1-7 avoid,M follow line,G grab now,C catch line) black vl(mm) heading(deg) 8ch(D1..D8, # = black)\n");
  int first = nTr > NTR ? nTr - NTR : 0;
  if (first) { snprintf(b, sizeof b, "  (only the last %d rows kept = last 35 s of the run)\n", NTR); out(b); }
  for (int i = first; i < nTr; i++) {
    const Tr &r = tr[i % NTR]; char ps[8];
    if (r.p == 127) snprintf(ps, sizeof ps, "  -- "); else snprintf(ps, sizeof ps, "%+5.1f", r.p / 10.0f);
    char mk[9]; for (int k = 0; k < 8; k++) mk[k] = (r.m >> k) & 1 ? '#' : '.'; mk[8] = 0;
    snprintf(b, sizeof b, "%6.2f %6ld %s %3u %c %u %4u %+7.1f %s\n", r.t / 100.0f, (long)r.d, ps, r.v, r.ph, r.nb, r.vl > 2000 ? 0 : r.vl, r.h / 10.0f, mk); out(b);
  }
  if (nCap > 0) {
    snprintf(b, sizeof b, "CAPTURE 2mm (%s): d(mm) heading(deg) D1..D8 analog 0-F (F = black)\n", capTrig >= 0 ? "100 samples before the first wide trace, 400 after" : "no wide trace seen, last 500 samples"); out(b);
    int c0 = nCap > NCAP ? nCap - NCAP : 0;
    for (int i = c0; i < nCap; i++) {
      const Cap &c = cap[i % NCAP]; char hx[9];
      for (int k = 0; k < 8; k++) hx[k] = "0123456789ABCDEF"[(c.v >> (4 * k)) & 15];
      hx[8] = 0;
      snprintf(b, sizeof b, "%s%5d %+7.1f %s\n", i == capTrig ? ">" : " ", c.d, c.h / 10.0f, hx); out(b);
    }
  }
}
struct WebPar { const char* k; float* f; int* i; };
void webSet() {
  WebPar ps[] = {{"kp", &cfg.kp, 0}, {"kd", &cfg.kd, 0}, {"sp", 0, &cfg.base}, {"vmax", 0, &cfg.vmax}, {"obs", &cfg.obs, 0},
                 {"lat", &cfg.lat, 0}, {"pass", &cfg.pass, 0}, {"aout", &cfg.aout, 0}, {"aback", &cfg.aback, 0},
                 {"sens", &cfg.sensAx, 0}, {"fin", &cfg.finMin, 0}, {"cpr", &cfg.cpr, 0}, {"wheel", &cfg.wheel, 0}, {"track", &cfg.track, 0}};
  if (web.hasArg("nobs") && web.arg("nobs").length()) cfg.nobs = constrain((int)web.arg("nobs").toFloat(), 0, 20);
  if (web.hasArg("side") && web.arg("side").length()) cfg.side = web.arg("side").toFloat() != 0;
  if (web.hasArg("avmode") && web.arg("avmode").length()) cfg.avMode = constrain((int)web.arg("avmode").toFloat(), 0, 3);   // C3.4: fixed the web bug that turned avmode 2 into 1
  if (web.hasArg("out") && web.arg("out").length()) cfg.outMm = constrain(web.arg("out").toFloat(), 30.0f, 800.0f);
  if (web.hasArg("par") && web.arg("par").length()) cfg.parMm = constrain(web.arg("par").toFloat(), 0.0f, 1000.0f);
  struct { const char* k; float* p; float lo, hi; } fp[] = {{"vout", &cfg.spdOut, 20, 100}, {"vpar", &cfg.spdPar, 20, 100}, {"vret", &cfg.spdRet, 20, 100},
    {"vturn", &cfg.spdTurn, 25, 100}, {"brev", &cfg.brkRev, 0, 100}, {"bms", &cfg.brkMs, 0, 300}, {"dec", &cfg.decMm, 0, 300}};
  for (auto &f : fp) if (web.hasArg(f.k) && web.arg(f.k).length()) *f.p = constrain(web.arg(f.k).toFloat(), f.lo, f.hi);
  if (web.hasArg("retm") && web.arg("retm").length()) cfg.retMode = web.arg("retm").toFloat() != 0;
  if (web.hasArg("roled") && web.arg("roled").length()) cfg.runOled = web.arg("roled").toFloat() != 0;
  if (web.hasArg("jmode") && web.arg("jmode").length()) cfg.jMode = web.arg("jmode").toFloat() != 0;
  if (web.hasArg("pref") && web.arg("pref").length()) cfg.pref = web.arg("pref").toFloat() != 0;
  if (web.hasArg("mask") && web.arg("mask").length()) { cfg.maskBits = constrain((int)web.arg("mask").toFloat(), 0, 255); for (int i = 0; i < 8; i++) sStuck[i] = false; }
  if (web.hasArg("mode") && web.arg("mode").length()) runMode = constrain((int)web.arg("mode").toFloat(), 0, 2);
  if (web.hasArg("vfast") && web.arg("vfast").length()) cfg.mFast = constrain(web.arg("vfast").toFloat(), 20.0f, 100.0f);
  if (web.hasArg("mpre") && web.arg("mpre").length()) cfg.mPre = constrain(web.arg("mpre").toFloat(), 50.0f, 800.0f);
  if (web.hasArg("fgo") && web.arg("fgo").length()) cfg.finGo = constrain(web.arg("fgo").toFloat(), 0.0f, 600.0f);
  if (web.hasArg("auto") && web.arg("auto").length()) cfg.noAuto = web.arg("auto").toFloat() == 0;
  if (web.hasArg("cspd") && web.arg("cspd").length()) cfg.cornerSpd = constrain(web.arg("cspd").toFloat(), 20.0f, 100.0f);
  if (web.hasArg("tlead") && web.arg("tlead").length()) cfg.turnLead = constrain(web.arg("tlead").toFloat(), 0.0f, 150.0f);
  for (auto &p : ps) if (web.hasArg(p.k) && web.arg(p.k).length()) {
    float v = web.arg(p.k).toFloat();
    if (p.f) *p.f = v; else *p.i = constrain((int)v, 10, 100);
  }
  {
    cfg.aout = constrain(cfg.aout, 30.0f, 90.0f); cfg.aback = constrain(cfg.aback, 20.0f, 90.0f);
  }
  cfgSave(); dirty = true;
  web.sendHeader("Location", "/"); web.send(303, "text/plain", "");
}
String webMsg;   // C3.17: result of the last web command, shown on the page
void webRoot() {
  String h; h.reserve(9000);
  h += F("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>LINE ROBOT</title>"
         "<style>body{font-family:sans-serif;margin:12px;background:#111;color:#eee}h2{color:#7cf;margin:14px 0 6px}"
         "a.b,button{display:inline-block;background:#2a6;color:#fff;padding:10px 14px;border-radius:8px;text-decoration:none;border:0;margin:4px 4px 4px 0;font-size:15px}"
         "table{border-collapse:collapse;width:100%}td{padding:6px;border-bottom:1px solid #333;vertical-align:top}.g{color:#9ab;font-size:13px}.n{font-weight:bold}tr.h td{color:#7cf;font-weight:bold;padding-top:14px;border:0}input{width:70px;padding:5px;background:#222;color:#fff;border:1px solid #555;border-radius:4px}"
         ".k{color:#aaa}</style><h1>LINE ROBOT</h1>");
  char b[260];
  snprintf(b, sizeof b, "<p>Battery <b>%.2f V</b> &middot; MPU %s &middot; VL53 %s &middot; calibration %s</p>", vbat, mpuOk ? "OK" : "ERROR", vlOk ? "OK" : "ERROR", cfg.calOk ? "OK" : "NO"); h += b;
  h += F("<h2>Last run</h2>");
  if (res.has) {
    snprintf(b, sizeof b, "<p><b>%s</b> &middot; %.2f s &middot; %.2f m &middot; %.2f m/s &middot; %d obstacles &middot; %d line losses</p>",
             res.why, res.t, res.d / 1000, res.v, res.obs, res.lost); h += b;
  } else h += F("<p>No run yet.</p>");
  h += F("<a class=b href=/log.txt>Full log</a><a class=b href=/log.txt?dl=1>Download log.txt</a><a class=b href=/>Refresh</a>");
  h += F("<h2>Command</h2><p class=k>Any parameter, same text as Serial, several at once with ';' (e.g. <b>vmax 80;alat 4500</b>). Saved at once.</p>"
         "<form action=/cmd><input name=c style='width:100%;font-size:16px;padding:8px' autocapitalize=off autocomplete=off><button>Apply</button></form>");
  if (webMsg.length()) { h += F("<p><b>"); h += webMsg; h += F("</b></p>"); }
  h += F("<p class=k>All parameters now (long-press to copy, send it with the log):</p><pre style='white-space:pre-wrap;word-break:break-all;background:#222;padding:8px'>");
  h += cfgLine(); h += F("</pre>");
  h += F("<h2>Parameters</h2><p class=k>Each row: name + explanation + when to raise/lower it. Change one value at a time and test.</p><form action=/set><table>");
  struct Row { const char* k; const char* n; const char* g; float v; };
  // group = row k wide
  const Row rows[] = {
    {"", "RUN MODE & TRACK MAP", "", 0},
    {"mode", "Mode (0 = NORMAL, 1 = DEMO record map, 2 = FAST follow map)", "1 DEMO: drive safely once, at the finish SAVE the map (track shape + correct turn side at each corner). 2 FAST: straights at speed 'vfast', slow down BEFORE corners, on line loss turn immediately to the right side. Also selectable on the READY page (SET button). Power off returns to 0, the map is kept.", (float)runMode},
    {"vfast", "FAST: straight speed (%)", "Only used in FAST mode, when the map says the next 25 cm is straight. Robot wobbles / slides on straights → LOWER.", cfg.mFast},
    {"mpre", "FAST: look-ahead to slow down (mm)", "How far before a corner / curve / obstacle to start slowing down. Robot overshoots corners → RAISE (300–400). Slows down too early → LOWER.", cfg.mPre},
    {"", "SPEED", "", 0},
    {"sp", "Corner entry speed (%)", "Speed while cornering / passing junctions. If the robot wobbles or leaves the line on curves, LOWER. Suggested 40–60.", (float)cfg.base},
    {"vmax", "Straight speed (%)", "Max speed when the line has been straight and steady for > 0.25 s. Must be ≥ corner speed. Suggested 60–90.", (float)cfg.vmax},
    {"cspd", "90-degree corner entry speed (%)", "When the line touches the sensor bar edge (90° corner / sharp curve ahead) the robot drops to this speed immediately. Overshoots corners → LOWER (35–40). Too slow into corners → RAISE.", cfg.cornerSpd},
    {"", "JUNCTIONS & SENSOR CHANNELS", "", 0},
    {"jmode", "Junctions inside CLUSTERS (1 = KEEP HEADING, 0 = always preferred side)", "1 = inside square / hexagon clusters the robot takes the branch that brings it back to the previous straight heading (alternating left-right, passing THROUGH the cluster). 0 = always follow the preferred side at every junction (may circle around one hexagon).", (float)cfg.jMode},
    {"pref", "Preferred side at junctions (0 = LEFT, 1 = RIGHT)", "When the robot is on the right heading and meets a T-junction it follows this side's branch.", (float)cfg.pref},
    {"mask", "Disable faulty channels (bit mask)", "0 = use all 8 channels. Disable D1 = 1, D2 = 2, D3 = 4, D4 = 8, D5 = 16, D6 = 32, D7 = 64, D8 = 128 (add them to disable several). Example: D8 reads wrong → 128. The robot also AUTO-ignores stuck channels while running (log shows 'STUCK CH').", (float)cfg.maskBits},
    {"auto", "Auto-disable stuck channels while running (1 = ON, 0 = OFF)", "1 = while running, any channel giving false black > 25% (of the time) is ignored (for that run only). 0 = no auto-disable, only the 'Disable faulty channels' field is used. The Sensor page always shows the real values.", (float)
(cfg.noAuto ? 0 : 1)},
    {"", "LINE FOLLOWING (PD)", "", 0},
    {"kp", "kp – steering gain", "Steering correction proportional to the line error. Robot slides out on curves → RAISE. Robot keeps wobbling → LOWER.", cfg.kp},
    {"kd", "kd – wobble damping", "Damps steering wobble. Robot wobbles/zigzags → RAISE slightly (0.05 at a time). Robot jerks, vibrates → LOWER.", cfg.kd},
    {"", "OBSTACLE AVOIDANCE", "", 0},
    {"obs", "Distance to start avoiding (mm)", "If the VL53 sees an object closer than this, brake and avoid. Robot nose hits the brick while turning out → RAISE. Avoids too early/wrongly → LOWER. Suggested 100–150.", cfg.obs},
    {"side", "Avoid side (0 = LEFT, 1 = RIGHT)", "Contest track: avoid LEFT (inside of the bend, there is free space).", (float)cfg.side},
    {"avmode", "Avoid mode (3 = ARC, 2 = FLOW, 1 = MANUAL, 0 = AUTO)", "3 = ARC: no stops, one smooth curve around the brick (uses aout, lat, pass, aback). 2 = FLOW: pivot out, then steer through. 1 = drives EXACTLY the distances below. 0 = measures the brick with the VL53.", (float)cfg.avMode},
    {"aout", "① Turn-out angle (deg)", "Robot turn angle when starting to avoid (30–90). 45 = diagonal avoid. 90 = square avoid (sideways first, then forward).", cfg.aout},
    {"out", "② Diagonal-out distance (mm)", "MANUAL: distance driven after turning out. Lateral offset = distance × sin(out angle) (45° → ×0.7). Goes out TOO FAR → LOWER. Scrapes the brick side → RAISE. Brick ACROSS the line (205 wide): ~260. Brick ALONG (95 wide): ~190.", cfg.outMm},
    {"par", "③ Parallel distance (mm)", "MANUAL: straight distance parallel to the line to pass the brick. Goes TOO FAR → LOWER. Robot tail scrapes the brick when turning back → RAISE. Brick ACROSS: ~280. Brick ALONG (205 long along the line): ~380.", cfg.parMm},
    {"aback", "④ Return angle to line (deg)", "Diagonal return angle to the line (20–90). Return CROSSES the line too far → LOWER (30–35) to meet the line at a shallow angle, smoother. Return is slow → RAISE.", cfg.aback},
    {"retm", "Return to line (1 = STOP-ALIGN-TURN, 0 = FOLLOW DIRECTLY)", "1 = on touching the line brake, center the line, move the wheel axle onto the line then turn to the line heading (reliable, as v2.3). 0 = no stop, follow an arc onto the line (faster but needs an accurate sens).", (float)cfg.retMode},
    {"tlead", "Turn inertia compensation (ms)", "The robot stops turning slightly EARLY based on turn rate so it does not OVERSHOOT. See log 'WRONG TURN ANGLE': error > 5° needs tuning. Overshoots → RAISE (40–60). Undershoots → LOWER.", cfg.turnLead},
    {"", "OBSTACLE AVOIDANCE – SPEED & BRAKING", "", 0},
    {"vout", "Diagonal-out speed (%)", "Segment ② . Too fast and it slides past the set distance.", cfg.spdOut},
    {"vpar", "Parallel speed (%)", "Segment ③ . On a curve the line passes the sensor bar very quickly → use 40–50 so the line is detected in time.", cfg.spdPar},
    {"vret", "Diagonal-back speed (%)", "Return segment. Near the line the robot slows down to ≤ 38%. Crosses the line widely → LOWER.", cfg.spdRet},
    {"vturn", "Turn speed while avoiding (%)", "Turns ① ④ . Overshoots the angle → LOWER.", cfg.spdTurn},
    {"brev", "Brake force (%)", "When stopping at the end of each segment, reverse the motors with this force to cancel inertia. 0 = short-circuit brake only. Robot still slides far → RAISE (40–60). Robot jerks backward → LOWER.", cfg.brkRev},
    {"bms", "Brake time (ms)", "Max reverse time (stops early once the robot halts). Suggested 40–80.", cfg.brkMs},
    {"dec", "Slow down before stop point (mm)", "In this last stretch the robot ramps down to 22% before braking → stops at the right distance. Still slides → RAISE (60–80). Avoid too slow → LOWER.", cfg.decMm},
    {"lat", "AUTO: minimum lateral offset (mm)", "Only used when mode = 0. The robot swings wider if it measures a large brick.", cfg.lat},
    {"pass", "AUTO: clearance after the brick (mm)", "Only used when mode = 0. Drive this far past the back of the brick before turning back. Default 80.", cfg.pass},
    {"", "ROBOT DIMENSIONS (measure with a ruler)", "", 0},
    {"sens", "Sensor bar → wheel axle (mm)", "Distance from the line sensor row to the wheel axle. Used at 90° corners and when re-acquiring the line.", cfg.sensAx},
    {"track", "Wheel track (mm)", "From the center of the left wheel to the center of the right wheel. Used to compute turns without the gyro.", cfg.track},
    {"wheel", "Wheel diameter (mm)", "Include the tire. Used with pulses/turn to compute distance.", cfg.wheel},
    {"cpr", "Encoder pulses / 1 wheel turn", "Check: push the robot 60 cm, the Sensor page must show D ≈ 600. If not: new = old × D ÷ 600.", cfg.cpr},
    {"", "FINISHED", "", 0},
    {"nobs", "Max number of obstacles (0 = unlimited)", "After this many avoids the VL53 is turned off so the track border is not mistaken for an obstacle. Contest: 2 (2 obstacles per lane). Multi-lap test loop: set 0.", (float)cfg.nobs},
    {"roled", "Screen while running (1 = on, 0 = off)", "Off keeps the line loop more regular (no OLED data over I2C while running). Use 0 in the contest.", (float)cfg.runOled},
    {"fgo", "Stop mid FINISH box: extra distance after the box edge (mm)", "Sensor bar touches the finish box edge → robot drives STRAIGHT (holding heading) this much further then brakes. Formula: half the box length + sens − (wheel axle → robot center). Stops before the middle → RAISE; overshoots → LOWER.", cfg.finGo},
    {"fin", "Minimum distance before accepting the finish (mm)", "The black FINISH box is accepted only after this distance (to skip the START box). Robot stops wrongly mid-track → RAISE.", cfg.finMin}};
  for (auto &r : rows) {
    if (!r.k[0]) { h += "<tr class=h><td colspan=2>"; h += r.n; h += "</td></tr>"; continue; }
    h += "<tr><td><div class=n>"; h += r.n; h += "</div><div class=g>"; h += r.g; h += "</div></td><td>";
    snprintf(b, sizeof b, "<input name=%s value='%g' inputmode=decimal>", r.k, r.v); h += b; h += "</td></tr>";
  }
  h += F("</table><button>Save parameters</button></form><p class=k>Takes effect on the next run, no need to re-upload the code. "
         "WiFi turns off while running / calibrating / on the Sensor page (WiFi shares ADC2 with 4 line channels) – reconnect after the run.</p>");
  web.send(200, "text/html", h);
}
void webLog() {
  if (web.hasArg("dl")) web.sendHeader("Content-Disposition", "attachment; filename=robot_log.txt");
  web.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web.send(200, "text/plain; charset=utf-8", "");
  String chunk; chunk.reserve(1500);
  writeLog([&](const char* l) { chunk += l; if (chunk.length() > 1200) { web.sendContent(chunk); chunk = ""; } });
  if (chunk.length()) web.sendContent(chunk);
  web.sendContent("");
}
void logToSerial() { writeLog([](const char* l) { Serial.print(l); }); }
// C3.17 WEB COMMAND BOX: set ANY parameter from the phone, same text as on Serial: "vmax 80;alat 4500;dband 28".
// Only "name value" pairs are accepted; driving commands (cal, wcal, go...) stay on the buttons (cal = CAL page + SET, wcal = INFO page + SET).
void webCmd() {
  String c = web.arg("c"); c.replace("\n", ";"); c.replace("\r", ""); c.trim();
  bool ok = c.length() > 0; int a = 0;
  String known = ";" + cfgLine() + ";";   // only names that appear in the parameter line (so 'wcal 1' / 'go 1' cannot start a run)
  while (ok && a <= (int)c.length()) {
    int e = c.indexOf(';', a); if (e < 0) e = c.length();
    String part = c.substring(a, e); part.trim(); a = e + 1;
    if (!part.length()) continue;
    char k[16] = {0}; float v;
    ok = sscanf(part.c_str(), "%15s %f", k, &v) == 2;
    ok = ok && known.indexOf(";" + String(k) + " ") >= 0;
  }
  if (ok) { runCmdLine(c, false); cfgSave(); webMsg = "OK, saved: " + c; Serial.printf("WEB: %s\n", c.c_str()); }
  else webMsg = "Refused: only parameter name + value, names as in the line below, e.g. vmax 80;alat 4500 (cal / wcal: use the buttons)";
  dirty = true;
  web.sendHeader("Location", "/"); web.send(303, "text/plain", "");
}
void webInit() { web.on("/", webRoot); web.on("/log.txt", webLog); web.on("/set", webSet); web.on("/cmd", webCmd); }
void uiLoop() {
  uint32_t now = millis();
  if (now - tBat >= 500) {                 // read the battery 2 times/s, light filtering
    tBat = now; float v = readVbat();
    if (fabsf(v - vbat) > 0.05f) dirty = true;
    vbat = vbat < 0.1f ? v : vbat * 0.7f + v * 0.3f;
  }
  if (page == P_SENSOR) {                  // sensor page: read continuously, drawn ~8 times/s (WiFi off because it holds ADC2)
    wifiStop(); update(); readLine();
    if (now - tDraw >= 120) dirty = true;
  }
  if (page == P_HOME && batLow() && now - tDraw >= 500) dirty = true;   // blink the battery warning
  if (page == P_WIFI && now - tDraw >= 1000) dirty = true;
  if (page != P_SENSOR) wifiStart();
  if (wifiOn) web.handleClient();
  if (dirty) { dirty = false; tDraw = now; drawPage(); }
}
void uiButton(int b) {
  if (b < 0) return;
  if (b == 0) { runCourse(); dirty = true; return; }           // START: run from any page
  if (b == 2) { page = (page + 1) % NPAGE; dirty = true; digitalWrite(PIN_BUZ, HIGH); delay(8); digitalWrite(PIN_BUZ, LOW); return; }
  // SET: action for the current page
  switch (page) {
    case P_SPEED: cfg.base = cfg.base >= 80 ? 20 : min(80, cfg.base + 10); cfgSave(); break;
    case P_VMAX:  cfg.vmax = cfg.vmax >= 100 ? 30 : min(100, cfg.vmax + 10); cfgSave(); break;   // C3.18: 73 -> 83 -> 93 -> 100 (was 103)
    case P_CAL:   calibrate(); break;
    case P_AVOID: cfg.side ^= 1; cfgSave(); break;
    case P_WIFI:  cfg.wifi ^= 1; cfgSave(); if (!cfg.wifi) wifiStop(); break;
    case P_HOME: runMode = (runMode + 1) % 3; break;              // SET on the home page: switch mode NORMAL -> DEMO -> FAST
    case P_RESULT: page = P_SENSOR; break;
    case P_INFO: wheelCal(); break;               // v3.2: SET on the INFO page = measure wheel speed (wcal)
    default: break;
  }
  beep(1, 25); dirty = true;
}
void setup() {
  Serial.setRxBufferSize(1024);   // the 'export' line is ~600 characters -> pasting it back at once loses nothing
  Serial.begin(115200);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  coast();
  for (int i = 0; i < 3; i++) pinMode(BTN[i], INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP); pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  prevL = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  prevR = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  for (int i = 0; i < 8; i++) analogSetPinAttenuation(LPIN[i], ADC_11db);
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(I2C_HZ); Wire.setTimeOut(5);
  delay(800);
  cfgLoad(); mapLoad(); dBand = prefs.getFloat("db", 25.0f);
  for (int i = 0; i < 3 && !mpuOk; i++) { mpuOk = mpuInit(); if (!mpuOk) delay(200); }
  for (int i = 0; i < 3 && !vlOk; i++) { vlOk = vl.begin(0x29, false, &Wire); if (!vlOk) delay(200); }
  if (vlOk) {
    if (VL_LONG) vl.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);   // v3.8: longer range (sees the brick earlier)
    else vl.setMeasurementTimingBudgetMicroSeconds(20000);
    vlOk = vl.startRangeContinuous(VL_LONG ? 33 : 20);
  }
  if (mpuOk) calibrateGyro();
  // OLED: only enabled if it answers on I2C (the robot still runs without OLED)
  for (uint8_t a : {0x3C, 0x3D}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { oledAddr = a; oledInit(); oledOk = true; break; }
  }
  vbat = readVbat();
  webInit();
  Serial.printf("\n=== LINE ROBOT v3.8 ===  OLED %s | MODE = page | SET = action | START = run\n", oledOk ? "OK" : "NONE");
  info();
  splash();
  beep(cfg.calOk ? 1 : 2, 60);
  delay(1200);
  dirty = true;
}
void loop() {
  uiButton(pressed());
  uiLoop();
  handleSerial();
}