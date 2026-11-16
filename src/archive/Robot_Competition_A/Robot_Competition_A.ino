/*
  LINE ROBOT - CONTEST FIRMWARE (official contest track)                   v3.5.1 (version A)
  ---------------------------------------------------------------------------------
  2 cm line on white + junctions (square clusters, hexagon clusters) + 90-degree corners/zigzags + OBSTACLE AVOIDANCE 10x15 cm.

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
        command 'wcal' OR press SET on the INFO page (page 8) -> the robot drives straight ~40 cm, saves 'vfull'.
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
      enters an arc R = sens*cot(angle/2) -> the wheel axle lands on the line just as the robot becomes parallel -> PD line following right away.
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
struct Tr { uint16_t t; int16_t d; int8_t p; uint8_t v; char ph; uint8_t nb; uint16_t vl; int16_t h; uint8_t m; };

// ================= YOUR PARAMETERS =================
// Loaded automatically when: the code is flashed for the first time (no parameters yet), or after 'reset'. New code still KEEPS the parameters stored in the robot.
// UPDATE: type 'export' in the Serial Monitor -> copy the whole line -> paste between the 2 double quotes below (replace the old line).
const char* USER_CFG = "kp 16;kd 0.3;sp 65;vmax 100;cspd 45;vfast 100;mpre 250;tlead 20;jmode 1;pref 0;mask 0;auto 1;cpr 816.9;wheel 48;track 132;sens 115;fin 500;fgo 180;vk 1;obs 330;lat 165;pass 80;nobs 2;side 0;avmode 2;aout 45;aback 45;out 320;par 400;vout 70;vpar 70;vret 60;vturn 70;retm 1;brev 35;bms 50;dec 50;roled 0;wifi 1;lag 8;wpid 1;wkp 1.5;wki 6;alat 0;vfull 1157.63;curve 0";   // 29/10: robot gets a new layer + photoresistor bar 10 cm, height 6-7 mm
// ================= EDIT HERE =================
bool REV_L = true;     // LEFT motor reversed (confirmed)
bool REV_R = false;
const float PIVOT_PCT = 34;     // line search / corner turn speed
const float AV_PCT = 60;        // straight speed while avoiding obstacles
const float AV_TURN = 60;       // max turn speed while avoiding
const float ROBOT_HALF_W = 80;  // half the robot width (robot ~157 mm wide)
const float REAR_MM = 35;       // robot tail behind the wheel axle
const float OB_LONG = 205, OB_SHORT = 95;   // OBSTACLE = BRICK 205 x 95 x 95 mm
const float CATCH_PCT = 38;     // speed right after touching the line on return
const float TURN_MIN = 20, TURN_TOL = 3.0f, KP_T = 1.4f, KD_T = 0.07f;   // gyro-controlled turning
const float SWEEP_DEG = 120;    // line search turn of at most 120 deg each way (anti-spin)
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
const uint32_t MAGIC = 0xF1A1D013, MAGIC_V34 = 0xF1A1D012, MAGIC_V32 = 0xF1A1D011, MAGIC_V31 = 0xF1A1D010, MAGIC_V296 = 0xF1A1D00F, MAGIC_V295 = 0xF1A1D00E, MAGIC_V28 = 0xF1A1D00D, MAGIC_V27 = 0xF1A1D00C, MAGIC_V24 = 0xF1A1D00B, MAGIC_V22 = 0xF1A1D00A, MAGIC_V21 = 0xF1A1D009, MAGIC_V20 = 0xF1A1D008, MAGIC_V19 = 0xF1A1D007, MAGIC_V13 = 0xF1A1D006, MAGIC_V12 = 0xF1A1D005;
Cfg cfg; Preferences prefs;
void runCmdLine(String s, bool verbose);
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
  cfg.sdly = 15; cfg.wpid = 1; cfg.vfull = 0; cfg.wkp = 1.5f; cfg.wki = 6; cfg.alat = 0; cfg.curveOld = 1;
  runCmdLine(USER_CFG, false);   // YOUR parameters (top of the file) override the defaults
}
void cfgLoad() {
  prefs.begin("thidau", false);   // NVS name kept from the first firmware so data already stored on the robot survives updates
  Cfg t; CfgV12 o; CfgV21 q; CfgV22 w; CfgV24 y; CfgV27 z; CfgV28 x8; CfgV295 x9; CfgV296 xa; CfgV31 xb; CfgV32 xc;
  if (prefs.getBytes("c", &t, sizeof(t)) == sizeof(t) && t.magic == MAGIC) { cfg = t; return; }
  if (prefs.getBytes("c", &t, sizeof(t)) == sizeof(t) && t.magic == MAGIC_V34) {   // v3.4 -> v3.4.1: same structure, new curve handling turned OFF
    cfg = t; cfg.magic = MAGIC; cfg.curveOld = 1;
    Serial.println("Migrated from v3.4: curves back to the OLD way (curve 0) because the new way circled in the hexagons");
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
void drive(float l, float r) { motorOne(0, l); motorOne(1, r); }
// v3.2 PER-WHEEL SPEED PID: command % -> target speed (mm/s) = % x vfull. PWM = % (as before) + correction from the speed error
// measured by the encoders -> both wheels turn at the RIGHT speed: a weak wheel / sagging battery / different friction does not pull the robot aside, faster acceleration.
float vL = 0, vR = 0;                    // measured wheel speed (mm/s), updated in update()
float wiL = 0, wiR = 0; uint32_t tDV = 0;
bool wpidOk() { return cfg.wpid && cfg.vfull > 100; }
float wheelPid(float pct, float v, float &integ, float dt) {
  if (pct * integ < 0 || fabsf(pct) < 1) integ = 0;            // direction change / stop -> clear the integral
  float err = (pct * cfg.vfull / 100.0f - v) * 100.0f / cfg.vfull;   // speed mismatch as %
  integ = constrain(integ + cfg.wki * err * dt, -25.0f, 25.0f);
  return pct + cfg.wkp * err + integ;
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
      spinAcc += fabsf(hdg - hPrev);
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
uint8_t jPref = 0;   // side followed at the current junction (0 left, 1 right): = cfg.pref, or chosen automatically when jMode = 1
uint8_t lineMask = 0; float pos = 0, posC = 0, peakV = 0; bool seen = false, junc = false;
void readLine() {
  float sw = 0, s = 0, peak = 0; nBlack = 0; nBlob = 0; lineMask = 0;
  bool prev = false;
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
  if (!junc) { pos = posC; lineC = posC; return; }
  // v2.9.3: 1 WIDE BLACK TRACE flaring to only ONE side of the previous line center = CORNER (only 1 branch) -> follow that edge,
  // NOT by the preferred side. (old: always followed the LEFT edge -> left corners smooth, right corners pulled straight then turned wrong)
  int side = jPref;
  if (nBlob == 1) {
    float extL = lineC - (b0s - 3.5f), extR = (b0e - 3.5f) - lineC;
    if (extR > extL + 2.5f) side = 1; else if (extL > extR + 2.5f) side = 0;
  }
  // real junction (2 sides / 2 traces): follow the edge on the preferred side (the 2 outermost channels of that branch)
  if (side == 0) { int a = b0s, b = min(b0s + 1, b0e); pos = (a + b) * 0.5f - 3.5f; }
  else               { int b = bLe, a = max(bLe - 1, bLs);  pos = (a + b) * 0.5f - 3.5f; }
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
char phase = '-';     // '-' line following | 'S' line search | '1'..'6' avoid steps | 'C' line re-caught
const int NTR = 700; Tr tr[NTR]; int nTr = 0; float curSpd = 0; uint32_t tTr = 0;
void traceTick() {
  if (!tracing || millis() - tTr < 50) return;
  tTr = millis();
  Tr &r = tr[nTr % NTR]; nTr++;              // ring buffer: keeps the LAST 35 s (where the robot runs into trouble)
  r.t = (millis() - tRun0) / 10; r.d = (int16_t)constrain(dist() - d0, -32000.0f, 32000.0f);
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
      if (peakV >= 0.6f && maxRun >= 2) { if (++hits >= 3) { turnHitLine = true; logEv('T'); break; } } else hits = 0;   // >= 2 ADJACENT channels make a line
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
bool straightMM(float mm, float h, float pct, bool stopLine, float minMm = 0, bool stopOnLine = true, bool needClear = false, bool flow = false) {
  float s0 = dist(); uint32_t t0 = millis(), tmax = 500 + (uint32_t)(mm * 10);
  int hits = 0; bool cleared = !needClear; float clr0 = s0;
  while (!aborted) {
    update();
    float done = dist() - s0;
    if (done >= mm || millis() - t0 > tmax) break;
    // REAL line seen: NEW read + 3 in a row, at least once >= 2 black channels -> BRAKE NOW (do not slide past the line)
    if (stopLine && done >= minMm) {
      readLine();
      if (!cleared) { if (maxRun >= 1 || peakV >= 0.5f) clr0 = dist(); else if (dist() - clr0 >= 40) cleared = true; }
      if (cleared && peakV >= 0.6f && nBlack >= 1) {
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
float sweepAvoidH = 1e9, backH = 1e9; int nBack = 0;   // backH: heading when the U-TURN line was seen (to turn back if no other line exists)
float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
bool sweepFind(int dir, float a1, float a2, int minNb = 1, float maxPos = 2.0f, float pivPct = PIVOT_PCT, float fastDeg = 0, float fastPct = 60, float minTurn = 0) {
  float h0 = heading();
  for (int k = 0; k < 2; k++) {
    int d = k ? -dir : dir; float lim = k ? a2 : a1; uint32_t t0 = millis();
    while (!aborted && millis() - t0 < 2500) {
      update(); readLine();
      bool early = k == 0 && (h0 - heading()) * d < minTurn;   // early: only accept a line RIGHT IN THE MIDDLE (no off-center trace at the edge)
      if (seen && fabsf(posC) < (early ? 0.8f : maxPos) && maxRun >= minNb) {
        if (sweepAvoidH < 1e8 && fabsf(wrap180(heading() - sweepAvoidH)) > 150) { if (backH > 1e8) { backH = heading(); nBack++; logEv('u'); } }
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
  t0 = millis();
  while (!aborted && millis() - t0 < 1500) {
    update(); readLine();
    float e = hRef - heading();
    if (seen && maxRun >= 2 && fabsf(posC) < 1.6f && (useRef ? fabsf(e) < 0.6f * fabsf(e0) + 5 : fabsf(e0 - e) > 4)) { brake(); waitMs(30); return true; }
    if (e * dir <= 0) break;                    // reached / passed the old heading
    float u = constrain(KP_T * fabsf(e), TURN_MIN, AV_TURN);
    drive(-dir * u, dir * u);
  }
  brake(); waitMs(40); readLine();
  if (seen && nBlack >= 1 && fabsf(posC) < 2.5f) return true;      // curved line: the sensor bar is slightly off but PD can still follow
  // 4. Not seen yet (e.g. curved line): small sweep both ways, angle limited, needs >= 2 black channels
  logEv('Z');
  return sweepFind(s > 0 ? 1 : -1, 40, 80, 2, 2.5f);
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
  const float v = constrain(min(cfg.spdRet, CATCH_PCT + 5), 25.0f, 45.0f);
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
    drive(v - corr, v + corr);
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
    if (dir > 0) drive(inner, outer); else drive(outer, inner);
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
bool grabLine() {
  phase = 'G'; logEv('G');
  const float base = 32;
  float lastE = posC; uint32_t t0 = millis(), tPrev = micros(), tLost = 0, tOk = 0;
  while (!aborted && millis() - t0 < 1200) {
    update(); readLine();
    uint32_t now = micros(); float dt = max(1e-4f, (now - tPrev) * 1e-6f); tPrev = now;
    if (!(seen && nBlack >= 1)) {                   // the line just passed -> turn toward the side it was seen
      if (!tLost) tLost = millis();
      if (millis() - tLost > 500) { brake(); waitMs(30); return false; }
      int sg = lastE > 0 ? 1 : -1;                   // +1 = line on the RIGHT
      drive(sg * PIVOT_PCT, -sg * PIVOT_PCT);
      continue;
    }
    tLost = 0;
    float e = posC, de = (e - lastE) / dt; lastE = e;
    float u = cfg.kp * e + cfg.kd * de;
    drive(base + u, base - u);
    if (fabsf(e) < 0.8f) { if (!tOk) tOk = millis(); if (millis() - tOk > 150) return true; } else tOk = 0;
  }
  return seen;
}

// ---------- obstacle avoidance ----------
int nObs = 0;
// Result of the latest avoid (written to the log)
float avAng = 0, avHalfW = 0, avLat = 0, avPass = 0, avGap = 0, avLen = 0, avOut = 0, avOverOut = 0, avOverPar = 0, avE1 = 0, avE3 = 0, avE5 = 0;
bool avoidObstacle() {
  nObs++; logEv('O');
  const float gap = vlMM;                          // distance from the robot nose to the obstacle face at detection
  Serial.printf("[OBSTACLE #%d] %u mm\n", nObs, vlMM);
  brake(); waitMs(50);
  const int s = cfg.side ? -1 : 1;                 // +1 = avoid on the LEFT, -1 = avoid on the RIGHT
  const float h = heading();
  const float nose = cfg.sensAx + 10;              // robot nose distance from the wheel axle (VL53 at the nose)
  const float faceX = nose + gap;                  // obstacle face distance from the wheel axle
  const float ab = constrain(cfg.aback, 20.0f, 90.0f), sinB = sinf(ab * DEG_TO_RAD);
  float ang, lat, outDist, passMm, halfW = 0, lenAlong = 0;
  phase = '1'; logEv('1');
  const bool flowAvoid = cfg.avMode == 2;   // v3.5 FLOWING AVOID: after the first turn-out, NO more stop-and-turn: the robot STEERS continuously into the parallel and return segments (~1 s faster)
  if (cfg.avMode >= 1) {
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
  Serial.printf("  %s | out angle %.0f | diagonal %.0f (lateral %.0f) | parallel %.0f\n", cfg.avMode == 2 ? "FLOW" : cfg.avMode ? "MANUAL" : "AUTO", ang, outDist, lat, passMm);
  avE1 = cfg.avMode >= 1 ? lastTurnErr : 0;
  // FROM HERE ON READ THE LINE CONTINUOUSLY: line seen (e.g. a line curving back close) -> FOLLOW NOW, do not finish the avoid sequence
  auto grabDone = [&]() -> bool { bool ok = grabLine(); if (!ok) ok = sweepFind(s, 40, 80, 2, 2.5f); phase = '-'; if (ok) { logEv('o'); return true; } return false; };
  phase = '2'; bool hit = straightMM(outDist, h + ang * s, cfg.spdOut, true, 0, false, true, flowAvoid); avOverOut = flowAvoid ? 0 : lastOver;
  if (hit) return grabDone();
  if (!flowAvoid) { phase = '3'; turnTo(h, cfg.spdTurn, true); avE3 = lastTurnErr;
    if (turnHitLine) return grabDone(); }
  // 4. parallel. Only reads the line in the SECOND HALF (wheel axle past the obstacle front) - to catch the line when the obstacle is on a bend
  //    a curved line crossing the path. Needs >= 2 black channels, 3 reads in a row -> no false catch on the obstacle edge / dirt.
  // only read the line once the sensor bar is level with the obstacle front (before that it only sees the obstacle edge)
  phase = '4'; logEv('4'); bool found = straightMM(passMm, h, cfg.spdPar, true, 0, false, true, flowAvoid); avOverPar = flowAvoid ? 0 : lastOver;
  if (found) return grabDone();
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
  bool arc = false, ok = false;
  if (!found) { phase = '7'; logEv('7'); found = arcFind(s, 150, 900); arc = true; }
  if (found && !arc && cfg.retMode == 0) ok = mergeLine(h);
  if (found && !ok) ok = catchLine(s, h, !arc);
  if (!ok) { logEv('X'); ok = sweepFind(s, 40, 80, 2, 2.5f); }
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
  runActive = true; adc2Zero = 0; stuckEv = 0; nBack = 0; for (int i = 0; i < 8; i++) sStuck[i] = false; nEv = 0; cntL = 0; cntJ = 0; nObs = 0; vlOn = true; spinStop = false; spinAcc = 0; tSpin = millis(); tRun0 = millis(); d0 = dist(); nTr = 0; tracing = true; phase = '-'; vlMinRun = 8190; vlN = 0;
  float lastErr = 0, lastPos = 0, spd = cfg.base, finStart = -1, hFin = 0;
  uint32_t tPrev = micros(), tStraight = millis(), tHint = 0, tP = 0, loops = 0;
  int hint = 0; bool wasJunc = false; const char* why = "STOP BUTTON";
  float hRef = heading(); jPref = cfg.pref; lastProfH = heading();
  float jD[4] = {-1e9, -1e9, -1e9, -1e9}; int jI = 0;   // v3.1: positions of the last 4 junctions
  auto inCluster = [&]() { int n = 0; for (float x : jD) if (dist() - x < 450) n++; return n >= 2; };   // >= 2 junctions within 45 cm = inside a CLUSTER
  float lastRD = -1e9, lastLD = -1e9, lastBD = -1e9, lastShapeD = -1e9, lastRecD = -1e9;   // lastBD: both edges AT ONCE (real T-junction)   // v2.9.1: position (encoder) when a branch last touched the RIGHT / LEFT edge of the sensor bar
  int cornerSide = 0; uint32_t tCorner = 0, tCentered = millis(); float dCornerBar = 0, lastSeenD = dist();   // 90-degree corner: side, when seen, sensor bar position at the corner

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

    // VAT CAN
    if (vlOn && vlOk && seen && !junc && vlMM > 20 && vlMM < cfg.obs) {
      inAvoid = true; jPref = cfg.pref;            // while avoiding: follow the line on a fixed preferred side, as in v2.8
      if (mapUse) mapSync(mapFind(mapPos(), "O", 350));
      mapNode('O', 0);
      bool okA = avoidObstacle();
      mapNode('o', 0); if (mapUse) mapSync(mapFind(mapPos(), "o", 400));
      inAvoid = false; hRef = heading(); tStraight = millis(); cornerSide = 0; lastRD = lastLD = lastBD = -1e9;
      if (!okA) { why = "OBSTACLE AVOID FAILED"; break; }
      if (cfg.nobs && nObs >= cfg.nobs) vlOn = false;   // nobs 0 = unlimited
      lastErr = 0; tPrev = micros(); spd = min(cfg.base * 0.7f, 45.0f); continue;
    }

    // remember the branch/corner side for use when the line is lost
    if (junc) {
      static uint32_t tJ = 0;
      if (!wasJunc && millis() - tJ > 300) { logEv('J'); jD[jI++ & 3] = dist(); }
      tJ = millis();
    }
    wasJunc = junc;
    // 90-DEGREE CORNER / SHARP BEND: the black trace touches the sensor bar EDGE (D1-D2 or D7-D8) and is >= 3 channels wide -> the bar is right at the corner.
    // Record the POSITION (encoder) now + the turn side, and slow down so the robot does not overshoot the corner.
    if (seen && nBlack >= 1) {
      lastSeenD = dist();
      if (fabsf(posC) < 1.5f && !junc) tCentered = millis();
      // v2.9.1: REAL TURN BRANCH = black trace touching the OUTERMOST channel (still working) and >= 3 wide, or 1 channel in and >= 4 wide.
      // (old: only needed D2 touched + width 3 -> a 2 cm line slightly left (D2-D4) was taken as a LEFT CORNER -> at a right corner the robot turned LEFT)
      int lo = 0, hi = 7;
      while (lo < 7 && (sStuck[lo] || (cfg.maskBits >> lo & 1))) lo++;
      while (hi > 0 && (sStuck[hi] || (cfg.maskBits >> hi & 1))) hi--;
      bool edgeR = (bLe >= hi && bLe - bLs >= 2) || (bLe >= hi - 1 && bLe - bLs >= 3);
      bool edgeL = (b0s <= lo && b0e - b0s >= 2) || (b0s <= lo + 1 && b0e - b0s >= 3);
      // v2.9.6: the right edge must be seen CONTINUOUSLY for >= 3 reads to count (a single noisy read on D1/D8 is no longer taken as a corner)
      static uint8_t nR = 0, nL = 0;
      nR = edgeR ? min(nR + 1, 50) : 0; nL = edgeL ? min(nL + 1, 50) : 0;
      edgeR = nR >= 3; edgeL = nL >= 3;
      if (edgeR) lastRD = dist();
      if (edgeL) lastLD = dist();
      if (edgeR && edgeL) lastBD = dist();
      // v3.4 SHAPE: 90-degree corner = black trace FLARING from the middle to the edge (middle still black). Curve = trace SLIDING to the edge (middle already white).
      if ((edgeR && bLs <= 4) || (edgeL && b0e >= 3)) lastShapeD = dist();
      // only counted as a CORNER when the robot was following the line straight just before (avoids a false trigger right after catching the line at an angle)
      if ((edgeR || edgeL) && millis() - tCentered < 200) {
        cornerSide = (edgeR && edgeL) ? (jPref ? 1 : -1) : (edgeR ? 1 : -1);
        if (millis() - tCorner > 400) logEv('K');
        tCorner = millis(); dCornerBar = dist();
      }
    }

    // JUNCTIONS INSIDE CLUSTERS (jMode 1 = KEEP HEADING): remember the heading of the last straight (hRef). At a junction, pick the branch
    // pull the robot back to hRef: drifted LEFT > 20 deg -> follow RIGHT, drifted RIGHT -> follow LEFT, nearly on heading -> by 'pref'.
    // => hexagon / square clusters: alternate left-right turns, pass THROUGH the cluster, never circle one cell.
    if (seen && !junc && millis() - tStraight > 400 && !inCluster()) hRef += (heading() - hRef) * min(1.0f, dt * 5);   // inside a cluster: KEEP the cluster entry heading
    // dh in +-180 deg (relative to the cluster entry heading)
    if (cfg.jMode) { float dh = wrap180(heading() - hRef); jPref = dh > 20 ? 1 : dh < -20 ? 0 : cfg.pref; } else jPref = cfg.pref;

    if (seen) {
      float e = pos, de = (e - lastErr) / dt; lastErr = e; lastPos = posC;
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
      if (vlOn && vlOk && vlMM < SLOW_MM) target = min(target, 35.0f);
      if (millis() - tCorner < 300) target = min(target, cfg.cornerSpd);    // approaching / at a 90-degree corner
      // v3.2 CURVES: lateral acceleration = v x turn rate <= alat -> the sharper the bend (large gz) the lower the speed, smooth cornering
      if (cfg.alat > 0 && cfg.vfull > 100) { float w = fabsf(gz) * 0.0174533f; if (w > 0.5f) target = min(target, max((float)cfg.cornerSpd, cfg.alat / w / cfg.vfull * 100.0f)); }
      spd += (target - spd) * min(1.0f, dt * (target > spd ? 3.0f : 25.0f));   // rise slowly (~0.3 s), fall fast
      curSpd = spd;
      float u = cfg.kp * e + cfg.kd * de;
      float base = spd * (1.0f - 0.45f * min(1.0f, fabsf(e) / 3.0f));
      driveV(base + u, base - u);
    } else {
      // LINE LOST.
      //  - Just past a 90-DEGREE CORNER / sharp bend: BRAKE HARD, then move forward (or BACK if it slid past) so the WHEEL AXLE returns to
      //    corner point (sensor bar position at the corner + sens), then pivot toward the turn side -> sensor bar lands on the new branch.
      //  - Normal line loss (gentle curve): turn right away toward the side just lost.
      logEv('L');
      bool corner = cornerSide != 0 && millis() - tCorner < 600 && dist() - dCornerBar < 300;
      // v2.9.1 TURN SIDE by EVIDENCE: within the last 70 mm, which edge did the branch touch on the sensor bar?
      //  RIGHT only -> turn right, LEFT only -> turn left (NOT depending on 'pref'), both -> T-junction (by jPref), none -> the side the line was lost
      bool rRec = dist() - lastRD < 70, lRec = dist() - lastLD < 70;
      int s; char why2 = 'f';
      if (rRec != lRec) {
        why2 = 'b';
        s = rRec ? 1 : -1;
        if (!corner) { corner = true; dCornerBar = rRec ? lastRD : lastLD; }   // real corner even if the robot has not centered on the line yet
      } else if (rRec && lRec) {
        if (dist() - lastBD < 70) { s = jPref ? 1 : -1; why2 = 'n'; }   // both branches AT ONCE = real T-junction -> by jPref
        else { s = lastRD > lastLD ? 1 : -1; why2 = 'b'; }                    // seen left then right (noise/wobble) -> trust the side seen LAST
        if (!corner) { corner = true; dCornerBar = max(lastRD, lastLD); }
      }
      else s = corner ? cornerSide : (lastPos > 0 ? 1 : -1);    // +1 = line on the RIGHT side
      // v3.0 FAST: line lost at a place known from the map -> use the ACTUAL side from the demo run (no guessing)
      bool mapGap = false; float hLoss = heading(), dLoss = dist() - d0, mLoss = mapPos();
      if (mapUse) {
        int j = mapFind(mLoss, "Lg", 150, hLoss);
        if (j >= 0) {
          if (mnode[j].t == 'g') mapGap = true;
          // the demo run knew the real side FOR SURE (after the search turn) -> trust the MAP (even if the sensor says otherwise, e.g. the robot cut the corner)
          else if (mnode[j].s != 0) { s = mnode[j].s; why2 = 'm'; if (!corner) { corner = true; dCornerBar = lastSeenD; } }
          mapSync(j);
        }
      }
      // v2.9.6 DASHED LINE / FLICKERING CHANNEL: no corner sign, the line was in the MIDDLE when lost -> NO stop, NO turn:
      // hold heading straight for up to GAP_MM to find the line again. Found -> keep following as if nothing happened.
      if (mapGap || (why2 != 'm' && !corner && !rRec && !lRec && fabsf(lastPos) < 1.5f)) {
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
      logEv(s > 0 ? '>' : '<'); logEv(why2);   // log: side the robot chose to turn + REASON (b = branch seen on 1 side, n = T-junction -> by preference, f = unclear -> side the line was just lost)
      // v3.4 CURVES / S-BENDS: line lost at the edge and the last 90 mm did NOT show a '90-degree corner' shape (black trace flaring from the MIDDLE to the edge),
      // only seeing the line SLIDE to the edge = CURVE. The old version treated it as a corner: drove ~sens mm to the 'apex' before turning
      // -> the robot ran off the line. New: BRAKE NOW + turn toward the side the line was lost. Inside a CLUSTER / by MAP the old way is kept.
      // v3.4.1: ONLY when diving FAST from a long straight into a bend (speed > cspd + 15, >= 500 mm without junctions / turns).
      // (hexagon / square: robot drives slowly + many corners in a row -> keep the 90-DEGREE CORNER handling as in v3.3, no circling)
      float lastJ = max(max(jD[0], jD[1]), max(jD[2], jD[3]));
      bool curveLoss = !cfg.curveOld && !inCluster() && why2 != 'm' && !mapGap && dist() - lastShapeD > 90 && (corner || fabsf(lastPos) > 2.0f)
                       && spd > cfg.cornerSpd + 15 && dist() - lastJ > 500 && dist() - lastRecD > 500;
      if (curveLoss) logEv('q');
      bool sharp = (corner || fabsf(lastPos) > 2.0f) && !curveLoss;
      bool ok;
      if (sharp) {
        phase = 'K';
        float ref = corner ? dCornerBar : lastSeenD;
        float mv = ref + cfg.sensAx - 10 - dist();           // remaining distance until the wheel axle reaches the corner point
        if (mv > 15) moveMM(mv, heading(), min(spd, cfg.cornerSpd));   // not reached yet: keep driving (no stop) then brake at the right point
        else { hardStop(); if (mv < -15) moveMM(mv, heading(), 30); }  // slid past: brake then BACK UP to the corner point
        phase = 'S';
        // 90-degree corner: turn FAST for the first 50 deg (the new branch cannot reach the sensor bar yet), then slow down to catch the line accurately
        sweepAvoidH = inCluster() ? hRef : 1e9; backH = 1e9;
        ok = sweepFind(s, SWEEP_DEG, SWEEP_DEG, 2, 1.8f, PIVOT_PCT + 4, corner ? 50 : 0, cfg.spdTurn, corner ? 40 : 0);
        if (!ok && backH < 1e8) { turnTo(backH, PIVOT_PCT + 4, true); readLine(); ok = turnHitLine || (seen && maxRun >= 2); }   // only the U-TURN line remains -> turn back onto it, still accept it
        sweepAvoidH = 1e9;
      } else {
        hardStop();
        phase = 'S';
        sweepAvoidH = inCluster() ? hRef : 1e9; backH = 1e9;
        ok = sweepFind(s, SWEEP_DEG, SWEEP_DEG, 2);   // angle limit: no spinning around
        if (!ok && backH < 1e8) { turnTo(backH, PIVOT_PCT + 4, true); readLine(); ok = turnHitLine || (seen && maxRun >= 2); }
        sweepAvoidH = 1e9;
      }
      phase = '-';
      if (!ok) { why = "LINE LOST"; break; }
      { float dh = heading() - hLoss; int real = dh < -25 ? 1 : dh > 25 ? -1 : 0;
        mapNode('L', real, dLoss, hLoss);
        // FAST: map wrong about the side 2 times (position offset) -> map OFF, run as NORMAL for safety
        if (why2 == 'm' && real != 0 && real != s && ++mapBad >= 2) { mapUse = false; logEv('x'); Serial.println("!! MAP MISMATCH -> switching to NORMAL run"); } }   // record the ACTUAL side turned (heading decreasing = right turn)
      lastRecD = dist(); lastErr = 0; tPrev = micros(); spd = min(cfg.base * 0.8f, cfg.cornerSpd); hint = 0; tStraight = millis(); cornerSide = 0; lastRD = lastLD = lastBD = -1e9;
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
  beep(fin ? 3 : 1, fin ? 70 : 400);
}

// ---------- serial ----------
// v3.2 WHEEL SPEED MEASUREMENT (command wcal): the robot drives STRAIGHT at 60% (direct PWM) ~0.8 s on the floor (needs ~50 cm clear),
// measure both wheel speeds once steady -> vfull = speed at 100%. The weaker wheel shows clearly (%).
void wheelCal() {
  wifiStop(); uiBusy("WHEEL CAL", "PUT ROBOT ON FLOOR", "50 cm clear ahead");
  Serial.println("\nWCAL: robot on the FLOOR, >= 50 cm clear ahead. Starting in 2 s...");
  beep(2); delay(2000);
  float sL = 0, sR = 0; int n = 0; uint32_t t0 = millis(); aborted = false;
  while (millis() - t0 < 800 && !aborted) {
    update(); drive(60, 60);
    if (millis() - t0 > 350) { sL += vL; sR += vR; n++; }
    delay(2);
  }
  brake(); delay(150); coast();
  if (n < 10) { Serial.println("!! Stopped midway"); return; }
  float a = sL / n, b = sR / n;
  Serial.printf("At 60%%: LEFT wheel %.0f mm/s, RIGHT %.0f mm/s (diff %.0f%%)\n", a, b, 100.0f * (a - b) / max(1.0f, (a + b) / 2));
  char l1[24], l2[24]; snprintf(l1, sizeof l1, "L %.0f  R %.0f mm/s", a, b);
  if (a < 50 || b < 50) { Serial.println("!! Wheels not turning / encoder wrong direction (check 'cprcnt', sgnL/sgnR) -> NOT saved"); uiBusy("WHEEL CAL ERROR", l1, "check encoder"); beep(1, 600); delay(1500); return; }
  cfg.vfull = (a + b) / 2 / 0.6f; cfgSave();
  snprintf(l2, sizeof l2, "vfull %.0f  SAVED", cfg.vfull); uiBusy("WHEEL CAL OK", l1, l2);
  Serial.printf("=> SAVED vfull %.0f mm/s (100%%). Wheel speed PID: %s\n", cfg.vfull, cfg.wpid ? "ON" : "OFF (type wpid 1)");
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
  Serial.printf("v3.2+: lag %.0f ms | wpid %d (%s) vfull %.0f wkp %.2f wki %.1f | alat %.0f | curve %d (%s)\n", cfg.sdly, cfg.wpid,
    wpidOk() ? "RUNNING" : cfg.wpid ? "NO wcal -> not active" : "off", cfg.vfull, cfg.wkp, cfg.wki, cfg.alat, cfg.curveOld ? 0 : 1, cfg.curveOld ? "curves like 90-degree corners" : "curve: straight + turn now");
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
void exportCfg() {
  Serial.println("\n== CURRENT PARAMETERS (copy the WHOLE line below, paste it back here to load; or paste into USER_CFG at the top of the code) ==");
  Serial.printf("kp %g;kd %g;sp %d;vmax %d;cspd %g;vfast %g;mpre %g;tlead %g;jmode %d;pref %d;mask %d;auto %d;",
    cfg.kp, cfg.kd, cfg.base, cfg.vmax, cfg.cornerSpd, cfg.mFast, cfg.mPre, cfg.turnLead, cfg.jMode, cfg.pref, cfg.maskBits, cfg.noAuto ? 0 : 1);
  Serial.printf("cpr %g;wheel %g;track %g;sens %g;fin %g;fgo %g;vk %g;",
    cfg.cpr, cfg.wheel, cfg.track, cfg.sensAx, cfg.finMin, cfg.finGo, cfg.vcal);
  Serial.printf("obs %g;lat %g;pass %g;nobs %d;side %d;avmode %d;aout %g;aback %g;out %g;par %g;vout %g;vpar %g;vret %g;vturn %g;retm %d;",
    cfg.obs, cfg.lat, cfg.pass, cfg.nobs, cfg.side, cfg.avMode, cfg.aout, cfg.aback, cfg.outMm, cfg.parMm, cfg.spdOut, cfg.spdPar, cfg.spdRet, cfg.spdTurn, cfg.retMode);
  Serial.printf("brev %g;bms %g;dec %g;roled %d;wifi %d;lag %g;wpid %d;wkp %g;wki %g;alat %g;vfull %g;curve %d\n",
    cfg.brkRev, cfg.brkMs, cfg.decMm, cfg.runOled, cfg.wifi, cfg.sdly, cfg.wpid, cfg.wkp, cfg.wki, cfg.alat, cfg.vfull, cfg.curveOld ? 0 : 1);
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
  static const char *const MAP[][2] = {{"xuat", "export"}, {"tre", "lag"}, {"dem", "cprcnt"}, {"cong", "curve"}, {"nhieu", "noise"}, {"mapxoa", "mapclr"}};
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
  else if (k == "wcal") { wheelCal(); return; }
  else if (k == "curve" && set) cfg.curveOld = v == 0;
  else if (k == "fgo" && set) cfg.finGo = constrain(v, 0.0f, 600.0f);
  else if (k == "auto" && set) { cfg.noAuto = v == 0; for (int i = 0; i < 8; i++) sStuck[i] = false; }
  else if (k == "cpr" && set) cfg.cpr = v;
  else if (k == "wheel" && set) cfg.wheel = v;
  else if (k == "track" && set) cfg.track = v;
  else if (k == "aout" && set) cfg.aout = constrain(v, 30.0f, 90.0f);
  else if (k == "aback" && set) cfg.aback = constrain(v, 20.0f, 90.0f);
  else if (k == "avmode" && set) cfg.avMode = constrain((int)v, 0, 2);   // 0 auto | 1 manual | 2 manual FLOWING (v3.5)
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
  else { Serial.println("Commands: kp kd sp vmax obs lat pass sens fin nobs side pref cpr wheel track vcal aout aback avmode out par vout vpar vret vturn brev bms dec retm tlead roled cspd jmode mask auto fgo mode vfast mpre wifi | lag wpid wkp wki vfull alat wcal curve | export (print all parameters) | map mapclr | log cprcnt cal go raw noise info btn scan flip reset"); return; }
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
  snprintf(b, sizeof b, "LINE ROBOT v3.5 | battery %.2f V | MPU %s VL53 %s\n", vbat, mpuOk ? "OK" : "NO", vlOk ? "OK" : "NO"); out(b);
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
  out("EVENTS (K 90-degree corner seen, L line lost, O obstacle met, 1/4/5/7 avoid step started (7 = arc to find line), T line seen while avoiding, M follow line while moving, G line seen mid-way -> follow now, C line caught (old way: stop-center-turn), X avoid saw no line, Z failed to catch line, R back up to find line (overshot), V spinning stopped, o avoid done, F finish, u ignore U-TURN line in cluster, q line lost on curve -> straight + turn now, m turn by MAP, x map mismatch -> off, g line lost in the middle -> go straight to re-catch (dashed line), > < chose turn RIGHT/LEFT, reason: b branch seen on 1 side, n T-junction by preference, f unclear):\n");
  snprintf(b, sizeof b, "  (%d junctions J, %d line losses L; important events below, J omitted)\n", cntJ, cntL); out(b);
  for (int i = 0; i < evCount(); i++) { const Ev &e = evAt(i); if (e.c == 'J') continue; snprintf(b, sizeof b, "  %c %6.0f mm %6.2f s  heading %+7.1f  black %u\n", e.c, e.d, e.t, e.h, e.nb); out(b); }
  out("TABLE 50ms: t(s) d(mm) pos(-3.5..3.5, -- = lost) spd(%) step(L line,J junction,K back to corner,S search,1-7 avoid,M follow line,G grab now,C catch line) black vl(mm) heading(deg) 8ch(D1..D8, # = black)\n");
  int first = nTr > NTR ? nTr - NTR : 0;
  if (first) { snprintf(b, sizeof b, "  (only the last %d rows kept = last 35 s of the run)\n", NTR); out(b); }
  for (int i = first; i < nTr; i++) {
    const Tr &r = tr[i % NTR]; char ps[8];
    if (r.p == 127) snprintf(ps, sizeof ps, "  -- "); else snprintf(ps, sizeof ps, "%+5.1f", r.p / 10.0f);
    char mk[9]; for (int k = 0; k < 8; k++) mk[k] = (r.m >> k) & 1 ? '#' : '.'; mk[8] = 0;
    snprintf(b, sizeof b, "%6.2f %6d %s %3u %c %u %4u %+7.1f %s\n", r.t / 100.0f, r.d, ps, r.v, r.ph, r.nb, r.vl > 2000 ? 0 : r.vl, r.h / 10.0f, mk); out(b);
  }
}
struct WebPar { const char* k; float* f; int* i; };
void webSet() {
  WebPar ps[] = {{"kp", &cfg.kp, 0}, {"kd", &cfg.kd, 0}, {"sp", 0, &cfg.base}, {"vmax", 0, &cfg.vmax}, {"obs", &cfg.obs, 0},
                 {"lat", &cfg.lat, 0}, {"pass", &cfg.pass, 0}, {"aout", &cfg.aout, 0}, {"aback", &cfg.aback, 0},
                 {"sens", &cfg.sensAx, 0}, {"fin", &cfg.finMin, 0}, {"cpr", &cfg.cpr, 0}, {"wheel", &cfg.wheel, 0}, {"track", &cfg.track, 0}};
  if (web.hasArg("nobs") && web.arg("nobs").length()) cfg.nobs = constrain((int)web.arg("nobs").toFloat(), 0, 20);
  if (web.hasArg("side") && web.arg("side").length()) cfg.side = web.arg("side").toFloat() != 0;
  if (web.hasArg("avmode") && web.arg("avmode").length()) cfg.avMode = web.arg("avmode").toFloat() != 0;
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
    {"auto", "Auto-disable stuck channels while running (1 = ON, 0 = OFF)", "1 = while running, any channel giving false black > 25% (of the time) is ignored (for that run only). 0 = no auto-disable, only the 'Disable faulty channels' field is used. The Sensor page always shows the real values.", (float)(cfg.noAuto ? 0 : 1)},
    {"", "LINE FOLLOWING (PD)", "", 0},
    {"kp", "kp – steering gain", "Steering correction proportional to the line error. Robot slides out on curves → RAISE. Robot keeps wobbling → LOWER.", cfg.kp},
    {"kd", "kd – wobble damping", "Damps steering wobble. Robot wobbles/zigzags → RAISE slightly (0.05 at a time). Robot jerks, vibrates → LOWER.", cfg.kd},
    {"", "OBSTACLE AVOIDANCE", "", 0},
    {"obs", "Distance to start avoiding (mm)", "If the VL53 sees an object closer than this, brake and avoid. Robot nose hits the brick while turning out → RAISE. Avoids too early/wrongly → LOWER. Suggested 100–150.", cfg.obs},
    {"side", "Avoid side (0 = LEFT, 1 = RIGHT)", "Contest track: avoid LEFT (inside of the bend, there is free space).", (float)cfg.side},
    {"avmode", "Avoid mode (1 = MANUAL, 0 = AUTO)", "1 = the robot drives EXACTLY the distances set right below (easy to tune, recommended). 0 = the robot measures the brick with the VL53 and computes the distances itself.", (float)cfg.avMode},
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
void webInit() { web.on("/", webRoot); web.on("/log.txt", webLog); web.on("/set", webSet); }

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
    case P_SPEED: cfg.base = cfg.base >= 80 ? 20 : cfg.base + 10; cfgSave(); break;
    case P_VMAX:  cfg.vmax = cfg.vmax >= 100 ? 30 : cfg.vmax + 10; cfgSave(); break;
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
  cfgLoad(); mapLoad();
  for (int i = 0; i < 3 && !mpuOk; i++) { mpuOk = mpuInit(); if (!mpuOk) delay(200); }
  for (int i = 0; i < 3 && !vlOk; i++) { vlOk = vl.begin(0x29, false, &Wire); if (!vlOk) delay(200); }
  if (vlOk) { vl.setMeasurementTimingBudgetMicroSeconds(20000); vlOk = vl.startRangeContinuous(20); }
  if (mpuOk) calibrateGyro();
  // OLED: only enabled if it answers on I2C (the robot still runs without OLED)
  for (uint8_t a : {0x3C, 0x3D}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { oledAddr = a; oledInit(); oledOk = true; break; }
  }
  vbat = readVbat();
  webInit();
  Serial.printf("\n=== LINE ROBOT v3.5 ===  OLED %s | MODE = page | SET = action | START = run\n", oledOk ? "OK" : "NONE");
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
