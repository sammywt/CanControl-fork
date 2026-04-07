#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <math.h>
#include <Servo.h>
#include "DFRobotDFPlayerMini.h"

// PIN ASSIGNMENTS (Arduino Mega)
// Serial1 (19 RX) = Controller input from Nano [Purple]
// Serial2 (16 TX, 17 RX) = DFPlayer Mini [Green, Yellow]
// SPI (50 MISO, 51 MOSI, 52 SCK, 53 SS) = MCP2515 CAN [Gray]
// 2  = CAN INT [Blue]
// 3  = Red LED (PWM)
// 4  = Green LED (PWM)
// 5  = Blue LED (PWM)
// 6  = DRV8871 IN2 propeller speed (PWM) [Green]
// 7  = DRV8871 IN1 propeller direction [White]
// 8  = Hopper servo [Orange]

MCP2515 mcp2515(53);
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;
Servo hopperServo;

// RGB LED pins (Common Cathode)
const int redPin   = 3;
const int greenPin = 4;
const int bluePin  = 5;

// motor driver pins (propeller hat)
const int motorIN1 = 7; // Direction
const int motorIN2 = 6; // Speed

// Hopper servo pin
const int servoPin = 8;

// Servo positions
const int HOPPER_CLOSED = 75;
const int HOPPER_OPEN   = 170;
bool hopper_open = false;


// SD CARD FOLDER LAYOUT
//   01 - startup sound
//   02 - d pad tracks
//   03 - annoying sounds (Y button)
//   04 - honks           (X button)
//   05 - DJGoose Intros
//   06 - NarrationToggle (L Stick Click)
//
// /01/001.mp3                    = startup sound
// /02/001.mp3  Nirvana
// /02/002.mp3  Manchild
// /02/003.mp3  Tainted Love
// /02/004.mp3  Umbrella
// /02/005.mp3  Rasputin
// /02/006.mp3  Mission Impossible
// /02/007.mp3  James Bond
// /02/008.mp3  Sweet Dreams

// /03/001.mp3  Apple Pay         /03/006.mp3  Bell
// /03/002.mp3  Lego              /03/007.mp3  Windows
// /03/003.mp3  Smoke Alarm       /03/008.mp3  Bonk
// /03/004.mp3  Pan               /03/009.mp3  Snake
// /03/005.mp3  Fnaf squeak       /03/010.mp3  Dialup

// /04/001.mp3 to 007.mp3          = honk sounds

// /05/001.mp3  Nirvana intro
// /05/002.mp3  Manchild intro
// /05/003.mp3  Tainted Love intro
// /05/004.mp3  Umbrella intro
// /05/005.mp3  Rasputin intro
// /05/008.mp3  Sweet Dreams intro

// /06/001.mp3  "Welcome to DJ Goose"
// /06/002.mp3  "DJ Goose Signing Off"

// folder number (can be up to 99)
const int FOLDER_STARTUP  = 1;
const int FOLDER_MUSIC    = 2;
const int FOLDER_RANDOM   = 3;
const int FOLDER_HONK     = 4;
const int FOLDER_DJ_INTRO = 5;
const int FOLDER_DJ_SFX   = 6;
// track counts (can be up to 255)
const int STARTUP_FILE        = 1;
const int MUSIC_TRACK_COUNT   = 8;
const int RANDOM_TRACK_COUNT  = 10;
const int HONK_TRACK_COUNT    = 7;
// Narration On/Off toggle
const int DJ_ON_FILE          = 1;  // "Welcome to DJ Goose"
const int DJ_OFF_FILE         = 2;  // "DJ Goose Signing Off"

// Tracks that have DJ intros in folder 05
// if a track number is in this list, its intro will play before the song
const int DJ_INTRO_TRACKS[] = {1, 2, 3, 4, 5, 8};
const int DJ_INTRO_COUNT = 6;
const unsigned long DJ_INTRO_DURATION = 4800; // flat wait (in ms) for all intros


// LED COLOR (common cathode)
void setColor(int r, int g, int b) {
  analogWrite(redPin,   r);
  analogWrite(greenPin, g);
  analogWrite(bluePin,  b);
}


// CAN CONTROL MODES (Spark MAX API IDs)
enum control_mode {
  Duty_Cycle_Set     = 0x2050080,
  Speed_Set          = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set       = 0x2050C80,
  Voltage_Set        = 0x2051080,
  Current_Set        = 0x20510C0,
  Smart_Motion_Set   = 0x2051480
};

// CAN STATUS FRAME IDs (Spark MAX telemetry)
enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

// CAN HEARTBEAT (keeps Spark MAXes alive)
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t  HEARTBEAT_DATA[8] = {255,255,255,255,255,255,255,255};

struct can_frame heartbeat_frame;
struct can_frame control_frame;
const uint8_t CONTROL_SIZE = 8;


// DRIVING CONSTANTS
const float MAX_DUTY        = 0.3f;   // max motor power (30%)
const float DEADZONE        = 0.15f;  // stick deadzone threshold
const float RESPONSE        = 2.0f;   // exponential curve power
const float RAMP_RATE_ACCEL = 0.002f; // how fast motors ramp up
const float RAMP_RATE_DECEL = 0.002f; // how fast motors ramp down

// MOTOR STATE
float duty_left_target   = 0.0f;
float duty_right_target  = 0.0f;
float duty_left_current  = 0.0f; // actual value sent to motor (ramped)
float duty_right_current = 0.0f;

// SENSOR STATE (from Spark MAX status frames)
float   velocity_left  = 0.0f;
float   velocity_right = 0.0f;
float   position_left  = 0.0f;
float   position_right = 0.0f;
float   current_left   = 0.0f;  // motor current in amps
float   current_right  = 0.0f;
uint8_t temp_left      = 0;     // motor temp in celsius
uint8_t temp_right     = 0;
float   voltage        = 0.0f;  // bus voltage (rolling average)
float   battery_pct    = 0.0f;  // estimated battery percentage

// Voltage rolling average (smooths out noisy readings)
static constexpr int VOLTAGE_SAMPLES = 20;
float voltage_buf[VOLTAGE_SAMPLES] = {};
int   voltage_idx      = 0;
bool  voltage_buf_full = false;

// CONTROLLER DATA (parsed from Nano CSV)
struct ControllerData {
  int lx, ly, rx, ry;       // stick axes (-511 to 512)
  int brake, throttle;       // triggers (0 to 1023)
  int buttons, dpad;         // bitmasks
};
ControllerData controller;

char buffer[64];     // serial receive buffer
uint8_t bufIdx = 0;  // current position in buffer

// BUTTON BIT MASKS (Bluepad32 format)
const int BTN_A  = (1 << 0);
const int BTN_B  = (1 << 1);
const int BTN_X  = (1 << 2);
const int BTN_Y  = (1 << 3);
const int BTN_LB = (1 << 4);
const int BTN_RB = (1 << 5);
const int BTN_LS = (1 << 8);  // left stick click
const int BTN_RS = (1 << 9);  // right stick click

// DPAD BITMASK VALUES (Bluepad32 format)
const int DPAD_UP    = 0x01;
const int DPAD_DOWN  = 0x02;
const int DPAD_RIGHT = 0x04;
const int DPAD_LEFT  = 0x08;

// LED STATE
int led_r = 0, led_g = 0, led_b = 0;
bool rainbow_mode = false;
bool leds_off = true;  // starts off, wakes up during startup
float rainbow_hue = 0.0f;

// LED BREATHING (sine wave oscillation)
float breathPhase = 0.0f;
const float BREATH_SPEED = 0.04f;

// SOUND STATE
int current_track = 1;     // currently selected track in folder 02
bool sound_playing = false;

// DJ MODE STATE
bool dj_mode = false;
bool dj_intro_playing = false;  // true while a DJ intro is playing
int dj_pending_track = 0;       // track to play after intro finishes
unsigned long dj_intro_start = 0;

// SERIAL RX COUNTER (for debugging connection)
unsigned long rxByteCount = 0;
unsigned long rxParseCount = 0;

// DEBUG
static bool DEBUG_RAW_BYTES = false;


// DECLARATIONS
float addVoltageSample(float v);
float batteryPercent(float v);
void hsvToRgb(float h, float s, float v, int &r, int &g, int &b);
void handleLEDButtons(int buttons);
void updateRainbow();
void updateBreathing();
void speakerSetup();
void playMusicTrack(int track);
void playRandomSound();
void playRandomHonk();
void stopSound();
void handleSoundDpad(int dpad);
void handleRandomSoundButton(int buttons);
void handleHonkButton(int buttons);
void handleDJModeToggle(int buttons);
void updateDJMode();
bool trackHasDJIntro(int track);
void propellerSetup();
void updatePropeller(int triggerValue);
void hopperSetup();
void hopperToggle();
void handleHopperButton(int buttons);
void startupSequence();
bool parseCSV(char* line, ControllerData& c);
float applyDeadzone(float v);
float applyCurve(float v);
void computeMotorFromStick();
void updateMotorRamp();
void packData(can_frame &frame, const uint8_t *data, const int size);
void createData(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size);
void sendControlFrame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint);
void drainCANStatus();
void printRawBytes(const char* label, const struct can_frame& frame);
void printStatus();


// checks if a track has a matching DJ intro file in folder 05
bool trackHasDJIntro(int track) {
  for (int i = 0; i < DJ_INTRO_COUNT; i++) {
    if (DJ_INTRO_TRACKS[i] == track) return true;
  }
  return false;
}


// STARTUP: plays startup sound, then fades eyes from black to white
void startupSequence() {
  if (dfPlayerReady) {
    dfPlayer.playFolder(FOLDER_STARTUP, STARTUP_FILE);
    Serial.println("Playing startup sound");
  }
  delay(200); // let audio start before eyes begin
  // fade eyes from off to full white (~4 seconds)
  for (int i = 0; i <= 255; i++) {
    setColor(i, i, i);
    delay(16); // 256 steps * 16ms = ~4 seconds
  }
  delay(500); // hold full white briefly before breathing starts
  led_r = 255;
  led_g = 255;
  led_b = 255;
  leds_off = false;
}


// smooths voltage readings over 20 samples
float addVoltageSample(float v) {
  voltage_buf[voltage_idx] = v;
  voltage_idx = (voltage_idx + 1) % VOLTAGE_SAMPLES;
  if (voltage_idx == 0) voltage_buf_full = true;
  int count = voltage_buf_full ? VOLTAGE_SAMPLES : voltage_idx;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += voltage_buf[i];
  return sum / count;
}

// converts voltage to battery percentage (3S LiPo: 10.5V empty, 12.6V full)
float batteryPercent(float v) {
  const float max_v = 12.6f;
  const float min_v = 10.5f;
  float pct = (v - min_v) / (max_v - min_v) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}


// converts hue/saturation/value to RGB (0-255 per channel)
void hsvToRgb(float h, float s, float v, int &r, int &g, int &b) {
  float c = v * s;
  float x = c * (1.0f - fabs(fmod(h / 60.0f, 2.0f) - 1.0f));
  float m = v - c;
  float r1, g1, b1;
  if      (h <  60) { r1 = c; g1 = x; b1 = 0; }
  else if (h < 120) { r1 = x; g1 = c; b1 = 0; }
  else if (h < 180) { r1 = 0; g1 = c; b1 = x; }
  else if (h < 240) { r1 = 0; g1 = x; b1 = c; }
  else if (h < 300) { r1 = x; g1 = 0; b1 = c; }
  else              { r1 = c; g1 = 0; b1 = x; }
  r = (int)((r1 + m) * 255);
  g = (int)((g1 + m) * 255);
  b = (int)((b1 + m) * 255);
}


// breathing effect: sine wave with hold at peak brightness
// squared brightness makes the dim phase much darker for a dramatic pulse
void updateBreathing() {
  breathPhase += BREATH_SPEED;
  if (breathPhase >= 2.0f * PI) breathPhase -= 2.0f * PI;
  float raw = sin(breathPhase);
  if (raw > 0.8f) raw = 1.0f; // clamp top of sine = hold at peak
  float brightness = (raw + 1.0f) * 0.5f; // remap -1..1 to 0..1
  brightness = brightness * brightness;    // square for deeper dimming
  int r = (int)(led_r * brightness);
  int g = (int)(led_g * brightness);
  int b = (int)(led_b * brightness);
  setColor(r, g, b);
}


// sets LED color based on button press
// A = green, B = red, RS = white
// (X, Y, LS are used for sounds/DJ mode instead of LEDs)
void handleLEDButtons(int buttons) {
  if      (buttons & BTN_A)  { rainbow_mode = false; leds_off = false; led_r =   0; led_g = 255; led_b =   0; } // green
  else if (buttons & BTN_B)  { rainbow_mode = false; leds_off = false; led_r = 255; led_g =   0; led_b =   0; } // red
  else if (buttons & BTN_RS) { rainbow_mode = false; leds_off = false; led_r = 255; led_g = 255; led_b = 255; } // white
}

// cycles through colors smoothly (perceptually even, no blue lingering)
// uses direct sine waves on each channel offset by 120 degrees
void updateRainbow() {
  rainbow_hue += 2.0f; // degrees per tick (smooth cycle)
  if (rainbow_hue >= 360.0f) rainbow_hue -= 360.0f;
  float rad = rainbow_hue * PI / 180.0f;
  // three sine waves 120 degrees apart = smooth RGB cycling
  float r = sin(rad)           * 0.5f + 0.5f;
  float g = sin(rad + 2.094f)  * 0.5f + 0.5f; // +120 degrees
  float b = sin(rad + 4.189f)  * 0.5f + 0.5f; // +240 degrees
  setColor((int)(r * 255), (int)(g * 255), (int)(b * 255));
}


// initializes DFPlayer on hardware Serial2
void speakerSetup() {
  Serial2.begin(9600);
  Serial.println("Initializing DFPlayer...");
  delay(2000); // DFPlayer needs ~2 seconds to boot
  if (!dfPlayer.begin(Serial2, false, false)) {
    Serial.println("DFPlayer ERROR: check wiring and SD card");
    dfPlayerReady = false;
    return;
  }
  Serial.println("DFPlayer ready");
  dfPlayer.volume(30); // max volume (0-30)
  delay(500); // extra settle time so startup sound doesn't get missed
  dfPlayerReady = true;
}

// plays a music track from folder 02
// if DJ mode is on and this track has an intro, plays intro first then waits 5 seconds
void playMusicTrack(int track) {
  if (!dfPlayerReady) return;

  if (dj_mode && trackHasDJIntro(track)) {
    // this track has a DJ intro, play it and queue the song
    Serial.print("DJ intro for track ");
    Serial.println(track);
    dfPlayer.playFolder(FOLDER_DJ_INTRO, track);
    dj_intro_playing = true;
    dj_pending_track = track;
    dj_intro_start = millis();
    sound_playing = true;
  } else {
    // no DJ mode or no intro, just play the song directly
    Serial.print("Playing track ");
    Serial.println(track);
    dfPlayer.playFolder(FOLDER_MUSIC, track);
    dj_intro_playing = false;
    sound_playing = true;
  }
}

// plays a random annoying sound from folder 03 (Y button)
void playRandomSound() {
  if (!dfPlayerReady) return;
  int track = 1 + random(RANDOM_TRACK_COUNT); // random between 1 and count
  Serial.print("Random sound: ");
  Serial.println(track);
  dfPlayer.playFolder(FOLDER_RANDOM, track);
  dj_intro_playing = false; // cancels any pending DJ intro
  sound_playing = true;
}

// plays a random honk from folder 04 (X button)
void playRandomHonk() {
  if (!dfPlayerReady) return;
  int track = 1 + random(HONK_TRACK_COUNT);
  Serial.print("Honk: ");
  Serial.println(track);
  dfPlayer.playFolder(FOLDER_HONK, track);
  dj_intro_playing = false;
  sound_playing = true;
}

// pauses whatever is currently playing
void stopSound() {
  if (!dfPlayerReady) return;
  Serial.println("Stopping audio");
  dfPlayer.pause();
  dj_intro_playing = false;
  sound_playing = false;
}

// d-pad cycles through music tracks in folder 02
// right/left = next/prev track and auto-play
// up = replay current track, down = stop
void handleSoundDpad(int dpad) {
  static int prev_dpad = 0;
  if (dpad == prev_dpad) return; // edge detection: only trigger on change
  prev_dpad = dpad;

  if (dpad & DPAD_RIGHT) {
    current_track++;
    if (current_track > MUSIC_TRACK_COUNT) current_track = 1; // wrap around
    Serial.print("Selected track: ");
    Serial.println(current_track);
    playMusicTrack(current_track);
  }
  else if (dpad & DPAD_LEFT) {
    current_track--;
    if (current_track < 1) current_track = MUSIC_TRACK_COUNT; // wrap around
    Serial.print("Selected track: ");
    Serial.println(current_track);
    playMusicTrack(current_track);
  }
  else if (dpad & DPAD_UP) {
    playMusicTrack(current_track); // replay current
  }
  else if (dpad & DPAD_DOWN) {
    stopSound();
  }
}

// Y button = random annoying sound (edge detection so it only fires once per press)
void handleRandomSoundButton(int buttons) {
  static bool prev_y = false;
  bool y_now = buttons & BTN_Y;
  if (y_now && !prev_y) playRandomSound();
  prev_y = y_now;
}

// X button = random honk (edge detection)
void handleHonkButton(int buttons) {
  static bool prev_x = false;
  bool x_now = buttons & BTN_X;
  if (x_now && !prev_x) playRandomHonk();
  prev_x = x_now;
}

// left stick click = toggle DJ mode on/off (edge detection)
// plays narration from folder 06 when toggling
// enables rainbow LEDs when DJ mode turns on
void handleDJModeToggle(int buttons) {
  static bool prev_ls = false;
  bool ls_now = buttons & BTN_LS;

  if (ls_now && !prev_ls) {
    dj_mode = !dj_mode; // toggle bool
    Serial.print("DJ Mode: ");
    Serial.println(dj_mode ? "ON" : "OFF"); // bool ? true : false

    // play the "Welcome to DJ Goose" or "Signing Off" narration
    if (dfPlayerReady) {
      dfPlayer.playFolder(FOLDER_DJ_SFX, dj_mode ? DJ_ON_FILE : DJ_OFF_FILE);
    }

    // turn on rainbow mode when DJ mode activates
    if (dj_mode) {
      rainbow_mode = true;
      leds_off = false;
    }
  }
  prev_ls = ls_now;
}

// waits 5 seconds for DJ intro to finish, then plays the queued music track
void updateDJMode() {
  if (!dj_intro_playing) return;

  if (millis() - dj_intro_start >= DJ_INTRO_DURATION) {
    Serial.print("DJ intro done, playing track ");
    Serial.println(dj_pending_track);
    dfPlayer.playFolder(FOLDER_MUSIC, dj_pending_track);
    dj_intro_playing = false;
  }
}


// sets up DRV8871 motor driver pins for propeller hat
void propellerSetup() {
  pinMode(motorIN1, OUTPUT); // direction (held LOW)
  pinMode(motorIN2, OUTPUT); // speed (PWM)
  digitalWrite(motorIN1, LOW);
  analogWrite(motorIN2, 0);
  Serial.println("Propeller motor ready");
}

// maps left trigger value to PWM (capped at 50%)
void updatePropeller(int triggerValue) {
  int speed = map(triggerValue, 0, 1023, 0, 128); // 128 = 50% of 255
  if (speed < 10) speed = 0; // trigger deadzone to prevent motor creep
  digitalWrite(motorIN1, LOW);
  analogWrite(motorIN2, speed);
}


// attaches hopper servo and sets to closed position
void hopperSetup() {
  hopperServo.attach(servoPin);
  hopperServo.write(HOPPER_CLOSED);
  hopper_open = false;
  Serial.println("Hopper servo ready");
}

// flips hopper between open and closed
void hopperToggle() {
  hopper_open = !hopper_open; // toggle bool
  hopperServo.write(hopper_open ? HOPPER_OPEN : HOPPER_CLOSED); // bool ? true : false
  Serial.print("Hopper ");
  Serial.println(hopper_open ? "OPEN" : "CLOSED"); // bool ? true : false
}

// LB button toggles hopper (has edge detection so it doesn't spam)
void handleHopperButton(int buttons) {
  static bool prev_lb = false;
  bool lb_now = buttons & BTN_LB;
  if (lb_now && !prev_lb) hopperToggle(); // only on press, not hold
  prev_lb = lb_now;
}


// parses CSV from Nano: "lx,ly,rx,ry,brake,throttle,buttons,dpad\n"
bool parseCSV(char* line, ControllerData& c) {
  int fields[8];
  int count = 0;
  char* token = strtok(line, ",");
  while (token != nullptr && count < 8) {
    fields[count++] = atoi(token); // convert each field to int
    token = strtok(nullptr, ",");
  }
  if (count != 8) return false; // reject if we didn't get all 8 fields
  c.lx       = fields[0];
  c.ly       = fields[1];
  c.rx       = fields[2];
  c.ry       = fields[3];
  c.brake    = fields[4]; // LT analog (0-1023)
  c.throttle = fields[5]; // RT analog (0-1023)
  c.buttons  = fields[6]; // bitmask of face buttons
  c.dpad     = fields[7]; // bitmask of dpad directions
  return true;
}


// add deadzones, rescales remaining range to 0..1
float applyDeadzone(float v) {
  if (fabs(v) < DEADZONE) return 0;
  if (v > 0) return (v - DEADZONE) / (1.0f - DEADZONE);
  else       return (v + DEADZONE) / (1.0f - DEADZONE);
}

// makes small stick movements less sensitive, full stick still = full power
float applyCurve(float v) {
  float sign = (v >= 0) ? 1.0f : -1.0f; // preserve direction
  return pow(fabs(v), RESPONSE) * sign;  // RESPONSE=2.0 = quadratic curve
}

// converts stick positions to left/right motor duty targets
// left stick Y = forward/back, right stick X = steering
void computeMotorFromStick() {
  // convert from -511..512 to -1.0..1.0
  float throttle =  controller.rx / 512.0f;
  float steering = -controller.ly / 512.0f;
  // apply deadzone and exponential curve
  throttle = applyCurve(applyDeadzone(throttle));
  steering = applyCurve(applyDeadzone(steering));
  // tank drive mixing: left = throttle + steering, right = throttle - steering
  float left  = throttle + steering;
  float right = throttle - steering;
  // normalize so neither motor exceeds 1.0
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) { left /= maxVal; right /= maxVal; }
  // scale to MAX_DUTY (0.3 = 30% max power)
  duty_left_target  = left  * MAX_DUTY;
  duty_right_target = right * MAX_DUTY;
}

// smoothly ramps current motor duty toward target (prevents jerky starts/stops)
void updateMotorRamp() {
  auto rampAxis = [](float current, float target) -> float {
    bool decelerating = fabs(target) < fabs(current);
    float rate = decelerating ? RAMP_RATE_DECEL : RAMP_RATE_ACCEL;
    if      (current < target) { current += rate; if (current > target) current = target; }
    else if (current > target) { current -= rate; if (current < target) current = target; }
    return current;
  };
  duty_left_current  = rampAxis(duty_left_current,  duty_left_target);
  duty_right_current = rampAxis(duty_right_current, duty_right_target);
}


// copies raw bytes into a CAN frame
void packData(can_frame &frame, const uint8_t *data, const int size) {
  for (int i = 0; i < size; i++) frame.data[i] = data[i];
}

// converts a value (like a float) to bytes, pads rest with zeros
void createData(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size) {
  const byte* src = static_cast<const byte*>(data);
  for (int i = 0; i < data_size; i++)  frame_data[i] = src[i];
  for (int i = data_size; i < total_size; i++) frame_data[i] = 0;
}

// sends a motor control command over CAN to a Spark MAX
void sendControlFrame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint) {
  control_frame.can_id = (mode + device_id) | CAN_EFF_FLAG; // extended frame ID
  byte data[CONTROL_SIZE];
  createData(&setpoint, data, 4, CONTROL_SIZE); // float = 4 bytes
  packData(control_frame, data, CONTROL_SIZE);
  mcp2515.sendMessage(txb, &control_frame);
}

// reads all pending status frames from Spark MAX controllers
// extracts velocity, temp, current, voltage from each motor encoder
void drainCANStatus() {
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    uint32_t id = rx.can_id & ~CAN_EFF_FLAG; // strip extended flag

    if (id == (status_1 + 1)) {
      // left motor status: velocity (4 bytes), temp, voltage, current
      memcpy(&velocity_left, rx.data, 4);
      temp_left = rx.data[4];
      float raw_i = rx.data[6] * 0.125f;
      if (raw_i < 40.0f) current_left = raw_i; // filter out garbage readings
      float new_v = rx.data[5] * 0.0658f;
      if (new_v >= 10.0f) {
        voltage = addVoltageSample(new_v);
        // only update battery % when motors are idle (voltage sags under load)
        bool idle = fabs(duty_left_current) < 0.05f && fabs(duty_right_current) < 0.05f;
        if (idle) battery_pct = batteryPercent(voltage);
      }
      if (DEBUG_RAW_BYTES) printRawBytes("S1_L", rx);
    }
    else if (id == (status_1 + 2)) {
      // right motor status
      memcpy(&velocity_right, rx.data, 4);
      temp_right = rx.data[4];
      float raw_i = rx.data[6] * 0.125f;
      if (raw_i < 40.0f) current_right = raw_i;
      if (DEBUG_RAW_BYTES) printRawBytes("S1_R", rx);
    }
    else if (id == (status_2 + 1)) {
      memcpy(&position_left, rx.data, 4); // encoder position left
      if (DEBUG_RAW_BYTES) printRawBytes("S2_L", rx);
    }
    else if (id == (status_2 + 2)) {
      memcpy(&position_right, rx.data, 4); // encoder position right
      if (DEBUG_RAW_BYTES) printRawBytes("S2_R", rx);
    }
  }
}

// prints raw CAN frame bytes for debugging
void printRawBytes(const char* label, const struct can_frame& frame) {
  Serial.print(label);
  Serial.print(" RAW [hex]: ");
  for (int i = 0; i < 8; i++) {
    if (frame.data[i] < 0x10) Serial.print("0");
    Serial.print(frame.data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("| [dec]: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(frame.data[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// prints all telemetry to serial monitor every 100ms
void printStatus() {
  Serial.print("LY=");       Serial.print(controller.ly);
  Serial.print(" RX=");      Serial.print(controller.rx);
  Serial.print(" | Target L="); Serial.print(duty_left_target);
  Serial.print(" R=");       Serial.print(duty_right_target);
  Serial.print(" | RPM L="); Serial.print(velocity_left);
  Serial.print(" R=");       Serial.print(velocity_right);
  Serial.print(" | Temp L="); Serial.print(temp_left);
  Serial.print("C R=");      Serial.print(temp_right);
  Serial.print("C | Volts="); Serial.print(voltage);
  Serial.print(" | Batt=");  Serial.print(battery_pct);
  Serial.print("% | Track="); Serial.print(current_track);
  Serial.print(" | DJ=");    Serial.print(dj_mode ? "ON" : "OFF"); // bool ? true : false
  Serial.print(" | Prop=");  Serial.print(controller.brake);
  Serial.print(" | Hopper="); Serial.print(hopper_open ? "OPEN" : "CLOSED"); // bool ? true : false
  Serial.print(" | RX=");    Serial.print(rxByteCount);
  Serial.print("/");          Serial.println(rxParseCount);
}


// SETUP
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0)); // seed RNG from floating analog pin

  // LED pins
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);
  setColor(0, 0, 0); // start off

  // CAN bus init (Mega SPI CS = pin 53)
  heartbeat_frame.can_id  = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = 8;
  packData(heartbeat_frame, HEARTBEAT_DATA, 8);
  control_frame.can_dlc = 8;
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  speakerSetup();    // DFPlayer on Serial2
  propellerSetup();  // DRV8871 motor driver
  hopperSetup();     // candy dispenser servo
  Serial1.begin(9600); // controller input from Nano

  startupSequence(); // play sound + fade in eyes

  Serial.println("ROBOGOOSE MEGA READY");
}


// MAIN LOOP
void loop() {

  drainCANStatus(); // read motor telemetry
  updateDJMode();   // check if DJ intro timer expired

  // read controller CSV data from Nano on Serial1
  while (Serial1.available()) {
    char c = Serial1.read();
    rxByteCount++;
    if (c == '\n') {
      buffer[bufIdx] = '\0';
      if (bufIdx > 0 && parseCSV(buffer, controller)) {
        rxParseCount++;

        computeMotorFromStick(); // convert sticks to motor targets

        // LED colors: A = green, B = red, RS = white
        handleLEDButtons(controller.buttons);

        // RT (right trigger) = purple LED
        if (controller.throttle > 10) {
          rainbow_mode = false;
          leds_off = false;
          led_r = 148; led_g = 0; led_b = 211;
        }

        // RB = rainbow mode (no breathing, full brightness cycling)
        if (controller.buttons & BTN_RB) { rainbow_mode = true; leds_off = false; }

        handleHopperButton(controller.buttons);       // LB = hopper toggle
        handleDJModeToggle(controller.buttons);       // LS = DJ mode toggle
        handleRandomSoundButton(controller.buttons);  // Y = random annoying sound
        handleHonkButton(controller.buttons);         // X = random honk
        handleSoundDpad(controller.dpad);             // D-pad = music track selection
        updatePropeller(controller.brake);            // LT = propeller speed
      }
      bufIdx = 0;
    } else if (c != '\r' && bufIdx < sizeof(buffer) - 1) {
      buffer[bufIdx++] = c;
    } else {
      bufIdx = 0; // buffer overflow or carriage return, reset
    }
  }

  unsigned long now = millis();

  // LED update (every 20ms = 50Hz)
  static unsigned long led_last = 0;
  if (now - led_last >= 20) {
    if (rainbow_mode) {
      updateRainbow(); // full brightness color cycling
    } else if (!leds_off && (led_r > 0 || led_g > 0 || led_b > 0)) {
      updateBreathing(); // breathing pulse on solid colors
    }
    led_last = now;
  }

  // motor ramp (every 1ms for smooth acceleration)
  static unsigned long ramp_last = 0;
  if (now - ramp_last >= 1) {
    updateMotorRamp();
    ramp_last = now;
  }

  // CAN heartbeat (every 10ms, keeps Spark MAXes from timing out)
  static unsigned long hb_last = 0;
  if (now - hb_last >= 10) {
    mcp2515.sendMessage(MCP2515::TXB0, &heartbeat_frame);
    hb_last = now;
  }

  // left motor command (every 10ms)
  static unsigned long ctrl_left_last = 5;
  if (now - ctrl_left_last >= 10) {
    sendControlFrame(MCP2515::TXB1, 1, Duty_Cycle_Set, duty_left_current);
    ctrl_left_last = now;
  }

  // right motor command (every 10ms, offset 5ms from left to spread CAN traffic)
  static unsigned long ctrl_right_last = 10;
  if (now - ctrl_right_last >= 10) {
    sendControlFrame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right_current);
    ctrl_right_last = now;
  }

  // serial telemetry print (every 100ms)
  static unsigned long print_last = 0;
  if (now - print_last >= 100) {
    printStatus();
    print_last = now;
  }
}
