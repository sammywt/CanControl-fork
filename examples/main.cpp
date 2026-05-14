#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "DFRobotDFPlayerMini.h"

// PIN ASSIGNMENTS (Arduino Mega)
// Serial1 (19 RX) = Controller input from Nano [Purple]
// Serial2 (16 TX, 17 RX) = DFPlayer Mini [Green, Yellow]
// Serial3 (14 TX, 15 RX) = Soundboard Nano (web server) [Green, Yellow]
// SPI (50 MISO, 51 MOSI, 52 SCK, 53 CS) = MCP2515 CAN [White, White, Purple, Gray]
// I2C (20 SDA, 21 SCL) = 16x2 LCD with I2C adapter [Blue, Purple]
// 2  = CAN INT [Purple]
// 3  = Red LED (PWM)
// 4  = Green LED (PWM)
// 5  = Blue LED (PWM)
// 6  = DRV8871 IN2 propeller speed (PWM) [Green] (unused)
// 7  = DRV8871 IN1 propeller direction [White] (unused)
// 8  = Hopper servo [Orange]

MCP2515 mcp2515(53);
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;
Servo hopperServo;

// LCD (I2C address 0x27 is most common, try 0x3F if it doesn't work)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// BNO055 IMU on I2C (shares bus with LCD, different address)
// used for anti-tip protection during hard braking
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imuReady = false;
float pitch = 0.0f; // current forward/back tilt angle in degrees

// RGB LED pins (Common Cathode)
const int redPin   = 3;
const int greenPin = 4;
const int bluePin  = 5;

// motor driver pins (propeller hat)
const int motorIN1 = 7;
const int motorIN2 = 6;

// Hopper servo pin
const int servoPin = 8;

// Servo positions
const int HOPPER_CLOSED = 75;
const int HOPPER_OPEN   = 170;
bool hopper_open = false;


// SERIAL DEBUG TOGGLE                                          //CHANGE ME TO ENABLE/DISABLE SERIAL OUTPUT
// flip to false when running on battery to speed up the loop
// serial prints block the loop for a few ms each, adds up fast
// DEBUG_RAW_BYTES dumps the raw CAN frame bytes, only useful when decoding new status frames
static bool SERIAL_ENABLED = false;       //makes stuff run slower
static bool DEBUG_RAW_BYTES = false;      //makes stuff run A LOT slower


// SD CARD FOLDER LAYOUT
//   01 - startup sounds
//   02 - d pad music tracks
//   03 - random annoying sounds (Y button)
//   04 - honks (X button)
//   05 - DJGoose Intros
//   06 - DJ on/off toggle sounds
//   07 - death sounds (when goose tips over)
//   08 - secret sounds - LT+Y plays 08/001, LT+X plays 08/002 (SECRET)
// too lazy to list them all, check the SD card

// folder number (can be up to 99)
const int FOLDER_STARTUP  = 1;
const int FOLDER_MUSIC    = 2;
const int FOLDER_RANDOM   = 3;
const int FOLDER_HONK     = 4;
const int FOLDER_DJ_INTRO = 5;
const int FOLDER_DJ_SFX   = 6;
const int FOLDER_DEATH    = 7;
const int FOLDER_SECRET   = 8;
// track counts (can be up to 255)                              //CHANGE ME WHEN UPDATING TRACK COUNTS
const int STARTUP_TRACK_COUNT = 4;
const int MUSIC_TRACK_COUNT   = 29;
const int RANDOM_TRACK_COUNT  = 17;
const int HONK_TRACK_COUNT    = 17;
const int DEATH_TRACK_COUNT   = 4;
// Specific tracks for LT+button combos
const int LT_A_SOUND = 1;  // 03/001 - LT+A
const int LT_B_SOUND = 4;  // 03/004 - LT+B
const int LT_Y_SOUND = 1;  // 08/001 - LT+Y (secret)
const int LT_X_SOUND = 2;  // 08/002 - LT+X (secret)
// Narration On/Off toggle
const int DJ_ON_FILE          = 1;
const int DJ_OFF_FILE         = 2;

// Tracks that have DJ intros in folder 05
const int DJ_INTRO_TRACKS[] = {1, 2, 3, 4, 5, 8};
const int DJ_INTRO_COUNT = 6;
const unsigned long DJ_INTRO_DURATION = 5000;

// VOLUME CONTROL                                              //CHANGE ME TO TUNE VOLUME
// DFPlayer volume range is 0-30, we start at max and let LT+DPadUp/Down adjust it
const int VOLUME_MAX = 30;
const int VOLUME_MIN = 0;
const int VOLUME_STEP = 2;  // how much each press changes the volume
int current_volume = VOLUME_MAX;


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


// DRIVING CONSTANTS                                        //CHANGE ME TO CHANGE SPEEDS
const float MAX_DUTY        = 0.50f;  // cap on motor power, 1.0 = full send, keep capped for safety
const float DEADZONE        = 0.15f;  // ignore tiny stick movements so the goose doesn't creep
const float RESPONSE        = 2.0f;   // 1.0=linear stick, 2.0=quadratic (more precision near center)

// RAMP RATES                                                    //CHANGE ME TO CHANGE RAMP FEEL
// the ramp logic applies to throttle and steering targets, not the final motor duty
// because of that, changing steering ramp affects driving feel when turning while moving
// and changing throttle ramp affects how steering feels when braking mid-turn
// this is kind of unavoidable with tank drive mixing, just something to keep in mind while tuning
// accel = how fast the value ramps up when pushing the stick
// decel = how fast it ramps back down when letting go or flicking the opposite way
const float THROTTLE_RAMP_ACCEL = 0.0007f; // forward/back accel - lower = smoother starts
const float THROTTLE_RAMP_DECEL = 0.0012f; // forward/back decel - higher = faster braking
const float STEERING_RAMP_ACCEL = 0.001f;  // turning accel - lower = more gradual turns
const float STEERING_RAMP_DECEL = 0.005f;  // turning decel - how fast turning eases out

// ANTI-TIP SETTINGS (BNO055 IMU)                                //CHANGE ME TO TUNE ANTI-TIP
// two-stage protection against tipping forward during hard braking, plus death mode for crashes:
//
// STAGE 1 - brake softening: when forward tilt crosses TILT_THRESHOLD, the decel rate
// gets scaled down proportionally. at TILT_MAX the brake is at 10% of normal strength.
// this is passive - just eases off the brake so the goose settles back down
//
// STAGE 2 - active catch boost: when forward tilt exceeds TILT_MAX (about to tip),
// both motors briefly drive forward to slide the wheels under the falling goose.
// this is aggressive - the bigger CATCH_GAIN, the harder the save. too aggressive
// and it can drive you into stuff. only triggers if the user is actually pushing the stick
// to prevent runaway when the IMU glitches
//
// STAGE 3 - death (failsafe): if pitch goes below DEATH_PITCH (forward tip) OR above
// DEATH_PITCH_BACK (backward tip from crashing into something), the goose is "dead".
// motors cut out completely and a random death sound plays. stays dead until the goose
// is righted back to a reasonable angle (between DEATH_RECOVERY_PITCH and DEATH_RECOVERY_BACK).
// also kicks in if the IMU returns garbage data (out of IMU_PITCH range).
//
// forward tilt is (TILT_RESTING - current_pitch), so bigger = more tipping forward
const float TILT_RESTING   = 16.0f;  // measured with the goose sitting level
const float TILT_THRESHOLD = 3.0f;   // when brake softening starts kicking in
const float TILT_MAX       = 7.0f;   // when brake is at 10%, and catch boost begins
const float CATCH_GAIN     = 0.03f;  // duty cycle added per degree of tilt past TILT_MAX
const float CATCH_MAX      = 0.30f;  // safety cap - never add more than this much boost
const float DEATH_PITCH         = -20.0f; // pitch this low or lower = forward tip = death
const float DEATH_PITCH_BACK    = 60.0f;  // pitch this high or higher = backward tip = death (crashing into stuff)
const float DEATH_RECOVERY_PITCH = 0.0f;  // forward tip recovery: pitch must rise above this
const float DEATH_RECOVERY_BACK  = 25.0f; // backward tip recovery: pitch must drop below this
// IMU sanity range - if reading is outside this, ignore it (probably garbage from impact)
const float IMU_PITCH_MIN = -90.0f;
const float IMU_PITCH_MAX = 90.0f;


// MOTOR STATE
float duty_left_current  = 0.0f;
float duty_right_current = 0.0f;

// RAMPED STICK STATE
float throttle_target  = 0.0f;
float steering_target  = 0.0f;
float throttle_current = 0.0f;
float steering_current = 0.0f;

// SENSOR STATE (from Spark MAX status frames)
float   velocity_left  = 0.0f;
float   velocity_right = 0.0f;
float   position_left  = 0.0f;
float   position_right = 0.0f;
uint8_t temp_left      = 0;
uint8_t temp_right     = 0;
float   voltage        = 0.0f;
float   battery_pct    = 0.0f;

// Voltage rolling average (smooths out noisy readings)
static constexpr int VOLTAGE_SAMPLES = 20;
float voltage_buf[VOLTAGE_SAMPLES] = {};
int   voltage_idx      = 0;
bool  voltage_buf_full = false;

// CONTROLLER DATA (parsed from Nano CSV)
struct ControllerData {
  int lx, ly, rx, ry;
  int brake, throttle;
  int buttons, dpad;
};
ControllerData controller;

char buffer[64];
uint8_t bufIdx = 0;

// SOUNDBOARD NANO BUFFER (commands from web server)
char    sbBuffer[64];
uint8_t sbBufIdx = 0;

// BUTTON BIT MASKS (Bluepad32 format)
const int BTN_A  = (1 << 0);
const int BTN_B  = (1 << 1);
const int BTN_X  = (1 << 2);
const int BTN_Y  = (1 << 3);
const int BTN_LB = (1 << 4);
const int BTN_RB = (1 << 5);
const int BTN_LS = (1 << 8);
const int BTN_RS = (1 << 9);

// DPAD BITMASK VALUES (Bluepad32 format)
const int DPAD_UP    = 0x01;
const int DPAD_DOWN  = 0x02;
const int DPAD_RIGHT = 0x04;
const int DPAD_LEFT  = 0x08;

// LT MODIFIER STATE
// LT (controller.brake, analog 0-1023) acts as a modifier button now
// when held above LT_MOD_THRESHOLD, other buttons do "modifier" actions instead of normal actions
// for example: A normally = green eyes, but LT+A = play sound 03/001
const int LT_MOD_THRESHOLD = 100; // analog threshold for "LT is held"
bool lt_held = false; // updated each loop

// LED STATE
int led_r = 0, led_g = 0, led_b = 0;
bool rainbow_mode = false;
bool leds_off = true;
float rainbow_hue = 0.0f;

// LED BREATHING ANIMATION                                      //CHANGE ME TO CHANGE EYES
const unsigned long BLINK_HOLD_BRIGHT = 2000;
const unsigned long BLINK_CLOSE_TIME  = 600;
const unsigned long BLINK_HOLD_DIM    = 300;
const unsigned long BLINK_OPEN_TIME   = 600;
const float BLINK_DIM_LEVEL = 0.05f;
int blinkState = 0;
unsigned long blinkTimer = 0;

// RAINBOW SPEED
const float RAINBOW_SPEED = 8.0f;

// SOUND STATE
int current_track = 1;
bool sound_playing = false;

// DJ MODE STATE
bool dj_mode = false;
bool dj_intro_playing = false;
int dj_pending_track = 0;
unsigned long dj_intro_start = 0;

// DEATH STATE (triggered when goose tips over past DEATH_PITCH)
// when true, motors are disabled and a death sound plays once
// only resets when the goose is picked up and righted (pitch rises above DEATH_RECOVERY_PITCH)
bool is_dead = false;

// LCD DISPLAY MODES (cycled with LT+DPadLeft/Right)
int lcd_mode = 0;
const int LCD_MODE_COUNT = 5; // RPM, Temp, Battery, Track, IMU Tilt

// SERIAL RX COUNTER
unsigned long rxByteCount = 0;
unsigned long rxParseCount = 0;


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
void playSpecificSound(int folder, int track);
void playDeathSound();
void stopSound();
void changeVolume(int delta);
void handleSoundDpad(int dpad);
void handleRandomSoundButton(int buttons);
void handleHonkButton(int buttons);
void handleDJModeToggle(int buttons);
void handleLTModifierButtons(int buttons, int dpad);
void handleRTPurpleEyes(int triggerValue);
void updateDJMode();
bool trackHasDJIntro(int track);
void hopperSetup();
void hopperToggle();
void handleHopperButton(int buttons);
void lcdSetup();
void updateLCD();
void imuSetup();
void readIMU();
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
// Soundboard Nano comms
void notifySoundboardTrack(int trackId);
void notifySoundboardStopped();
void notifySoundboardPaused();
void sendStatsToNano();
void processSoundboardCommand(const char* line);
void handleSoundboardSerial();


// checks if a track has a matching DJ intro file in folder 05
bool trackHasDJIntro(int track) {
  for (int i = 0; i < DJ_INTRO_COUNT; i++) {
    if (DJ_INTRO_TRACKS[i] == track) return true;
  }
  return false;
}


// initializes the I2C LCD and shows a boot message
// address 0x27 is standard for most I2C LCD adapters (change to 0x3F if needed)
void lcdSetup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ROBOGOOSE");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
}

// updates the LCD display based on the current mode, cycled by LT+DPad Left/Right
// modes: 0=RPM, 1=Temp, 2=Battery, 3=Track, 4=IMU tilt (for anti-tip tuning)
// builds each line in a char buffer then pushes to the LCD in one I2C transaction (faster)
// pads lines to 16 chars with spaces so switching modes cleanly overwrites old text
void updateLCD() {
  char line0[17];
  char line1[17];

  switch (lcd_mode) {
    case 0: // RPM mode
      snprintf(line0, 17, "L:%-5d RPM  ", (int)velocity_left);
      snprintf(line1, 17, "R:%-5d RPM  ", (int)velocity_right);
      break;

    case 1: // Temperature mode
      snprintf(line0, 17, "L:%dC  R:%dC    ", temp_left, temp_right);
      snprintf(line1, 17, "Motor Temps     ");
      break;

    case 2: // Battery mode - also shows volume so you can see it while adjusting
      {
        int vWhole = (int)voltage;
        int vFrac  = (int)((voltage - vWhole) * 10);
        snprintf(line0, 17, "%d.%dV  %d%%      ", vWhole, vFrac, (int)battery_pct);
        snprintf(line1, 17, "Vol:%-2d Battery  ", current_volume);
      }
      break;

    case 3: // Track info mode
      snprintf(line0, 17, "Track:%d/%-3d     ", current_track, MUSIC_TRACK_COUNT);
      snprintf(line1, 17, "DJ:%-3s Vol:%-2d   ", dj_mode ? "ON" : "OFF", current_volume);
      break;

    case 4: // IMU tilt mode (for anti-tip tuning)
      if (imuReady) {
        if (is_dead) {
          // override with big DEAD banner when tipped over
          snprintf(line0, 17, "** GOOSE DEAD **");
          snprintf(line1, 17, "Pitch:%d.%d      ", (int)pitch, (int)(fabs(pitch - (int)pitch) * 10));
        } else {
          // normal tilt display - raw pitch on line 0, forward tilt from resting on line 1
          int pitchWhole = (int)pitch;
          int pitchFrac  = (int)(fabs(pitch - pitchWhole) * 10);
          float fwdTilt = TILT_RESTING - pitch;
          int fwdWhole = (int)fwdTilt;
          int fwdFrac  = (int)(fabs(fwdTilt - fwdWhole) * 10);
          snprintf(line0, 17, "Pitch:%d.%d    ", pitchWhole, pitchFrac);
          snprintf(line1, 17, "Fwd:%d.%d   ", fwdWhole, fwdFrac);
        }
      } else {
        snprintf(line0, 17, "IMU NOT FOUND   ");
        snprintf(line1, 17, "Check wiring    ");
      }
      break;
  }

  // ensure lines are exactly 16 chars (pad with spaces if snprintf was shorter)
  for (int i = 0; i < 16; i++) {
    if (line0[i] == '\0') { memset(&line0[i], ' ', 16 - i); break; }
  }
  for (int i = 0; i < 16; i++) {
    if (line1[i] == '\0') { memset(&line1[i], ' ', 16 - i); break; }
  }
  line0[16] = '\0';
  line1[16] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}


// starts up the BNO055 IMU on the I2C bus
// the IMU shares SDA/SCL with the LCD but uses a different address (0x28 vs 0x27)
// if it fails to init we set imuReady=false so the rest of the code knows to skip anti-tip
void imuSetup() {
  if (SERIAL_ENABLED) Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    if (SERIAL_ENABLED) Serial.println("BNO055 ERROR: check wiring (SDA=20, SCL=21)");
    imuReady = false;
    return;
  }
  delay(500); // let the IMU settle after power on
  bno.setExtCrystalUse(true); // use external crystal for more stable readings
  imuReady = true;
  if (SERIAL_ENABLED) Serial.println("BNO055 ready");
}

// reads pitch angle from the IMU, called every 20ms from the main loop
// pitch is the forward/back tilt. which axis corresponds to pitch depends on how the
// IMU is physically mounted - if values look wrong, swap to orientation.x or .z
// bails out fast if no IMU connected so anti-tip just never kicks in
void readIMU() {
  if (!imuReady) return;
  sensors_event_t event;
  bno.getEvent(&event);
  pitch = event.orientation.y;
}


// STARTUP: plays a random startup sound, then fades eyes from black to white
void startupSequence() {
  if (dfPlayerReady) {
    int startupTrack = 1 + random(STARTUP_TRACK_COUNT);
    dfPlayer.playFolder(FOLDER_STARTUP, startupTrack);
    if (SERIAL_ENABLED) { Serial.print("Playing startup sound: "); Serial.println(startupTrack); }
  }
  delay(200);
  for (int i = 0; i <= 255; i++) {
    setColor(i, i, i);
    delay(16);
  }
  delay(500);
  led_r = 255;
  led_g = 255;
  led_b = 255;
  leds_off = false;
  blinkState = 0;
  blinkTimer = millis();
  lcd.clear();
  updateLCD();
}


// smooths voltage readings over samples
float addVoltageSample(float v) {
  voltage_buf[voltage_idx] = v;
  voltage_idx = (voltage_idx + 1) % VOLTAGE_SAMPLES;
  if (voltage_idx == 0) voltage_buf_full = true;
  int count = voltage_buf_full ? VOLTAGE_SAMPLES : voltage_idx;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += voltage_buf[i];
  return sum / count;
}

// converts voltage to battery percentage (3S LiPo: 9.6V empty, 12.6V full)
float batteryPercent(float v) {
  const float max_v = 12.6f;
  const float min_v = 9.6f;
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


// blink animation
void updateBreathing() {
  unsigned long elapsed = millis() - blinkTimer;
  float brightness = 1.0f;

  switch (blinkState) {
    case 0:
      brightness = 1.0f;
      if (elapsed >= BLINK_HOLD_BRIGHT) { blinkState = 1; blinkTimer = millis(); }
      break;
    case 1:
      brightness = 1.0f - (1.0f - BLINK_DIM_LEVEL) * ((float)elapsed / BLINK_CLOSE_TIME);
      if (elapsed >= BLINK_CLOSE_TIME) { brightness = BLINK_DIM_LEVEL; blinkState = 2; blinkTimer = millis(); }
      break;
    case 2:
      brightness = BLINK_DIM_LEVEL;
      if (elapsed >= BLINK_HOLD_DIM) { blinkState = 3; blinkTimer = millis(); }
      break;
    case 3:
      brightness = BLINK_DIM_LEVEL + (1.0f - BLINK_DIM_LEVEL) * ((float)elapsed / BLINK_OPEN_TIME);
      if (elapsed >= BLINK_OPEN_TIME) { brightness = 1.0f; blinkState = 0; blinkTimer = millis(); }
      break;
  }

  if (brightness > 1.0f) brightness = 1.0f;
  if (brightness < 0.0f) brightness = 0.0f;
  setColor((int)(led_r * brightness), (int)(led_g * brightness), (int)(led_b * brightness));
}


// LED buttons (only respond when LT is NOT held - LT modifies these)
// A = green, B = red, RS = white
void handleLEDButtons(int buttons) {
  if      (buttons & BTN_A)  { rainbow_mode = false; leds_off = false; led_r =   0; led_g = 255; led_b =   0; }
  else if (buttons & BTN_B)  { rainbow_mode = false; leds_off = false; led_r = 255; led_g =   0; led_b =   0; }
  else if (buttons & BTN_RS) { rainbow_mode = false; leds_off = false; led_r = 255; led_g = 255; led_b = 255; }
}

// RT analog trigger sets eyes to blue (held = blue, released = restores previous)
// since RT is now an analog trigger we use a threshold to detect "held"
void handleRTPurpleEyes(int triggerValue) {
  static bool prev_rt_held = false;
  static int saved_r = 0, saved_g = 0, saved_b = 0;
  static bool saved_rainbow = false;
  bool rt_held = (triggerValue > 100);

  if (rt_held && !prev_rt_held) {
    // RT just pressed - save current eye color and switch to blue
    saved_r = led_r;
    saved_g = led_g;
    saved_b = led_b;
    saved_rainbow = rainbow_mode;
    rainbow_mode = false;
    leds_off = false;
    led_r = 0;
    led_g = 0;
    led_b = 255;
  }
  else if (!rt_held && prev_rt_held) {
    // RT just released - restore previous color
    led_r = saved_r;
    led_g = saved_g;
    led_b = saved_b;
    rainbow_mode = saved_rainbow;
  }

  prev_rt_held = rt_held;
}

// rainbow cycle
void updateRainbow() {
  rainbow_hue += RAINBOW_SPEED;
  if (rainbow_hue >= 360.0f) rainbow_hue -= 360.0f;
  float rad = rainbow_hue * PI / 180.0f;
  float r = sin(rad)           * 0.5f + 0.5f;
  float g = sin(rad + 2.094f)  * 0.5f + 0.5f;
  float b = sin(rad + 4.189f)  * 0.5f + 0.5f;
  setColor((int)(r * 255), (int)(g * 255), (int)(b * 255));
}


// initializes DFPlayer on hardware Serial2
// starts at max volume - LT+DPadUp/Down adjusts it during use
void speakerSetup() {
  Serial2.begin(9600);
  if (SERIAL_ENABLED) Serial.println("Initializing DFPlayer...");
  delay(2000);
  if (!dfPlayer.begin(Serial2, false, false)) {
    if (SERIAL_ENABLED) Serial.println("DFPlayer ERROR: check wiring and SD card");
    dfPlayerReady = false;
    return;
  }
  if (SERIAL_ENABLED) Serial.println("DFPlayer ready");
  dfPlayer.volume(VOLUME_MAX);
  current_volume = VOLUME_MAX;
  delay(500);
  dfPlayerReady = true;
}

// changes the DFPlayer volume by delta (positive = louder, negative = quieter)
// clamps to VOLUME_MIN..VOLUME_MAX so it can't go out of range
// called by the LT+DPadUp/Down handler
void changeVolume(int delta) {
  if (!dfPlayerReady) return;
  current_volume += delta;
  current_volume = constrain(current_volume, VOLUME_MIN, VOLUME_MAX);
  dfPlayer.volume(current_volume);
  if (SERIAL_ENABLED) { Serial.print("Volume: "); Serial.println(current_volume); }
}

// plays a music track from folder 02
// also notifies the soundboard Nano so the web UI's Now Playing stays in sync
void playMusicTrack(int track) {
  if (!dfPlayerReady) return;
  if (dj_mode && trackHasDJIntro(track)) {
    if (SERIAL_ENABLED) { Serial.print("DJ intro for track "); Serial.println(track); }
    dfPlayer.playFolder(FOLDER_DJ_INTRO, track);
    dj_intro_playing = true;
    dj_pending_track = track;
    dj_intro_start = millis();
    sound_playing = true;
  } else {
    if (SERIAL_ENABLED) { Serial.print("Playing track "); Serial.println(track); }
    dfPlayer.playFolder(FOLDER_MUSIC, track);
    dj_intro_playing = false;
    sound_playing = true;
  }
  notifySoundboardTrack(track);
}

void playRandomSound() {
  if (!dfPlayerReady) return;
  int track = 1 + random(RANDOM_TRACK_COUNT);
  if (SERIAL_ENABLED) { Serial.print("Random sound: "); Serial.println(track); }
  dfPlayer.playFolder(FOLDER_RANDOM, track);
  dj_intro_playing = false;
  sound_playing = true;
}

void playRandomHonk() {
  if (!dfPlayerReady) return;
  int track = 1 + random(HONK_TRACK_COUNT);
  if (SERIAL_ENABLED) { Serial.print("Honk: "); Serial.println(track); }
  dfPlayer.playFolder(FOLDER_HONK, track);
  dj_intro_playing = false;
  sound_playing = true;
}

// plays a specific track from a specific folder
// used for LT+A and LT+B which play exact files (03/001 and 03/004)
void playSpecificSound(int folder, int track) {
  if (!dfPlayerReady) return;
  if (SERIAL_ENABLED) { Serial.print("Specific sound: "); Serial.print(folder); Serial.print("/"); Serial.println(track); }
  dfPlayer.playFolder(folder, track);
  dj_intro_playing = false;
  sound_playing = true;
}

// plays a random death sound from folder 07 when the goose tips over
// called once when is_dead transitions from false to true
void playDeathSound() {
  if (!dfPlayerReady) return;
  int track = 1 + random(DEATH_TRACK_COUNT);
  if (SERIAL_ENABLED) { Serial.print("DEATH: "); Serial.println(track); }
  dfPlayer.playFolder(FOLDER_DEATH, track);
  dj_intro_playing = false;
  sound_playing = true;
}

// stops playback and notifies the soundboard Nano so the web UI updates
void stopSound() {
  if (!dfPlayerReady) return;
  if (SERIAL_ENABLED) Serial.println("Stopping audio");
  dfPlayer.pause();
  dj_intro_playing = false;
  sound_playing = false;
  notifySoundboardStopped();
}

// d-pad WITHOUT LT held: right/left = next/prev music track, up = replay, down = stop
// d-pad WITH LT held: handled by handleLTModifierButtons() instead (volume + LCD)
void handleSoundDpad(int dpad) {
  static int prev_dpad = 0;
  if (dpad == prev_dpad) return;
  prev_dpad = dpad;

  if (dpad & DPAD_RIGHT) {
    current_track++;
    if (current_track > MUSIC_TRACK_COUNT) current_track = 1;
    playMusicTrack(current_track);
  }
  else if (dpad & DPAD_LEFT) {
    current_track--;
    if (current_track < 1) current_track = MUSIC_TRACK_COUNT;
    playMusicTrack(current_track);
  }
  else if (dpad & DPAD_UP) { playMusicTrack(current_track); }
  else if (dpad & DPAD_DOWN) { stopSound(); }
}

// Y = random sound (edge detection, only when LT not held)
void handleRandomSoundButton(int buttons) {
  static bool prev_y = false;
  bool y_now = buttons & BTN_Y;
  if (y_now && !prev_y) playRandomSound();
  prev_y = y_now;
}

// X = random honk (edge detection, only when LT not held)
void handleHonkButton(int buttons) {
  static bool prev_x = false;
  bool x_now = buttons & BTN_X;
  if (x_now && !prev_x) playRandomHonk();
  prev_x = x_now;
}

// LS = DJ mode toggle (edge detection)
void handleDJModeToggle(int buttons) {
  static bool prev_ls = false;
  bool ls_now = buttons & BTN_LS;
  if (ls_now && !prev_ls) {
    dj_mode = !dj_mode;
    if (SERIAL_ENABLED) { Serial.print("DJ Mode: "); Serial.println(dj_mode ? "ON" : "OFF"); }
    if (dfPlayerReady) { dfPlayer.playFolder(FOLDER_DJ_SFX, dj_mode ? DJ_ON_FILE : DJ_OFF_FILE); }
    if (dj_mode) { rainbow_mode = true; leds_off = false; }
  }
  prev_ls = ls_now;
}

// handles all LT-modifier combinations
// only called when lt_held is true, so we don't need to check LT inside again
// LT+A    = play sound 03/001 (specific track)
// LT+B    = play sound 03/004 (specific track)
// LT+Y    = play sound 08/001 (secret)
// LT+X    = play sound 08/002 (secret)
// LT+DPadUp    = volume up
// LT+DPadDown  = volume down
// LT+DPadLeft  = previous LCD mode
// LT+DPadRight = next LCD mode
// edge-detected so each button press triggers once
void handleLTModifierButtons(int buttons, int dpad) {
  // LT+A
  static bool prev_lt_a = false;
  bool lt_a_now = buttons & BTN_A;
  if (lt_a_now && !prev_lt_a) {
    playSpecificSound(FOLDER_RANDOM, LT_A_SOUND);
  }
  prev_lt_a = lt_a_now;

  // LT+B
  static bool prev_lt_b = false;
  bool lt_b_now = buttons & BTN_B;
  if (lt_b_now && !prev_lt_b) {
    playSpecificSound(FOLDER_RANDOM, LT_B_SOUND);
  }
  prev_lt_b = lt_b_now;

  // LT+Y (secret 1)
  static bool prev_lt_y = false;
  bool lt_y_now = buttons & BTN_Y;
  if (lt_y_now && !prev_lt_y) {
    playSpecificSound(FOLDER_SECRET, LT_Y_SOUND);
  }
  prev_lt_y = lt_y_now;

  // LT+X (secret 2)
  static bool prev_lt_x = false;
  bool lt_x_now = buttons & BTN_X;
  if (lt_x_now && !prev_lt_x) {
    playSpecificSound(FOLDER_SECRET, LT_X_SOUND);
  }
  prev_lt_x = lt_x_now;

  // LT+DPad - all 4 directions edge detected from a single dpad value
  static int prev_lt_dpad = 0;
  if (dpad != prev_lt_dpad) {
    if      (dpad & DPAD_UP)    { changeVolume(VOLUME_STEP); }
    else if (dpad & DPAD_DOWN)  { changeVolume(-VOLUME_STEP); }
    else if (dpad & DPAD_RIGHT) { lcd_mode = (lcd_mode + 1) % LCD_MODE_COUNT; }
    else if (dpad & DPAD_LEFT)  { lcd_mode = (lcd_mode - 1 + LCD_MODE_COUNT) % LCD_MODE_COUNT; }
    prev_lt_dpad = dpad;
  }
}

// DJ intro timer
void updateDJMode() {
  if (!dj_intro_playing) return;
  if (millis() - dj_intro_start >= DJ_INTRO_DURATION) {
    if (SERIAL_ENABLED) { Serial.print("DJ intro done, playing track "); Serial.println(dj_pending_track); }
    dfPlayer.playFolder(FOLDER_MUSIC, dj_pending_track);
    dj_intro_playing = false;
  }
}

// propeller motor (UNUSED FOR NOW, add it in, it was used with the analog trigger before modifying it)
void propellerSetup() {
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  digitalWrite(motorIN1, LOW);
  analogWrite(motorIN2, 0);
}

void updatePropeller(int triggerValue) {
  int speed = map(triggerValue, 0, 1023, 0, 128);
  if (speed < 10) speed = 0;
  digitalWrite(motorIN1, LOW);
  analogWrite(motorIN2, speed);
}

// hopper servo
void hopperSetup() {
  hopperServo.attach(servoPin);
  hopperServo.write(HOPPER_CLOSED);
  hopper_open = false;
}

void hopperToggle() {
  hopper_open = !hopper_open;
  hopperServo.write(hopper_open ? HOPPER_OPEN : HOPPER_CLOSED);
  if (SERIAL_ENABLED) { Serial.print("Hopper "); Serial.println(hopper_open ? "OPEN" : "CLOSED"); }
}

// LB = hopper toggle (edge detection)
void handleHopperButton(int buttons) {
  static bool prev_lb = false;
  bool lb_now = buttons & BTN_LB;
  if (lb_now && !prev_lb) hopperToggle();
  prev_lb = lb_now;
}


// 
// SOUNDBOARD NANO COMMS (Serial3)
// 
// Protocol:
//   Nano -> Mega:
//     MUSIC:NN      play music track from folder 02
//     HONK:NN       play specific honk from folder 04
//     HONK:RND      play random honk
//     SOUND:NN      play specific sound from folder 03
//     SOUND:RND     play random sound
//     NEXT / PREV   skip music tracks
//     PAUSE / RESUME / STOP
//     VOL:NN | VOL:UP | VOL:DOWN
//
//   Mega -> Nano:
//     TRACK:NN      music track started
//     STOPPED       playback stopped
//     PAUSED        playback paused
//     STATS:vL,vR,tL,tR,V,B,vol,trk,dj,hop,pitch,dead,playing
//                   periodic telemetry dump (every 500 ms)

void notifySoundboardTrack(int trackId) {
  char buf[16];
  snprintf(buf, sizeof(buf), "TRACK:%02d", trackId);
  Serial3.println(buf);
}

void notifySoundboardStopped() {
  Serial3.println("STOPPED");
}

void notifySoundboardPaused() {
  Serial3.println("PAUSED");
}

// CSV: velL,velR,tempL,tempR,voltage,batt%,vol,track,dj,hop,pitch,dead,playing
void sendStatsToNano() {
  Serial3.print("STATS:");
  Serial3.print((int)velocity_left);  Serial3.print(',');
  Serial3.print((int)velocity_right); Serial3.print(',');
  Serial3.print(temp_left);           Serial3.print(',');
  Serial3.print(temp_right);          Serial3.print(',');
  Serial3.print(voltage, 1);          Serial3.print(',');
  Serial3.print((int)battery_pct);    Serial3.print(',');
  Serial3.print(current_volume);      Serial3.print(',');
  Serial3.print(current_track);       Serial3.print(',');
  Serial3.print(dj_mode ? 1 : 0);     Serial3.print(',');
  Serial3.print(hopper_open ? 1 : 0); Serial3.print(',');
  Serial3.print(pitch, 1);            Serial3.print(',');
  Serial3.print(is_dead ? 1 : 0);     Serial3.print(',');
  Serial3.println(sound_playing ? 1 : 0);
}

void processSoundboardCommand(const char* line) {
  // ── MUSIC:NN ──────────────────────────────────────────
  if (strncmp(line, "MUSIC:", 6) == 0) {
    int t = atoi(line + 6);
    if (t >= 1 && t <= MUSIC_TRACK_COUNT) {
      current_track = t;
      playMusicTrack(t);
    }
  }
  // ── HONK:NN or HONK:RND ───────────────────────────────
  else if (strncmp(line, "HONK:", 5) == 0) {
    if (line[5] == 'R') {
      playRandomHonk();
    } else {
      int t = atoi(line + 5);
      if (t >= 1 && t <= HONK_TRACK_COUNT) {
        playSpecificSound(FOLDER_HONK, t);
      }
    }
  }
  // ── SOUND:NN or SOUND:RND ─────────────────────────────
  else if (strncmp(line, "SOUND:", 6) == 0) {
    if (line[6] == 'R') {
      playRandomSound();
    } else {
      int t = atoi(line + 6);
      if (t >= 1 && t <= RANDOM_TRACK_COUNT) {
        playSpecificSound(FOLDER_RANDOM, t);
      }
    }
  }
  // ── Transport ─────────────────────────────────────────
  else if (strcmp(line, "NEXT") == 0) {
    current_track++;
    if (current_track > MUSIC_TRACK_COUNT) current_track = 1;
    playMusicTrack(current_track);
  }
  else if (strcmp(line, "PREV") == 0) {
    current_track--;
    if (current_track < 1) current_track = MUSIC_TRACK_COUNT;
    playMusicTrack(current_track);
  }
  else if (strcmp(line, "PAUSE") == 0) {
    if (dfPlayerReady) {
      dfPlayer.pause();
      sound_playing = false;
      notifySoundboardPaused();
    }
  }
  else if (strcmp(line, "RESUME") == 0) {
    if (dfPlayerReady) {
      dfPlayer.start();
      sound_playing = true;
      notifySoundboardTrack(current_track);
    }
  }
  else if (strcmp(line, "STOP") == 0) {
    stopSound();
  }
  // ── Volume ────────────────────────────────────────────
  else if (strncmp(line, "VOL:", 4) == 0) {
    if (strcmp(line + 4, "UP") == 0)        changeVolume(VOLUME_STEP);
    else if (strcmp(line + 4, "DOWN") == 0) changeVolume(-VOLUME_STEP);
    else {
      int v = atoi(line + 4);
      v = constrain(v, VOLUME_MIN, VOLUME_MAX);
      current_volume = v;
      if (dfPlayerReady) dfPlayer.volume(v);
    }
  }
}

// reads serial bytes from the soundboard Nano and dispatches complete lines
void handleSoundboardSerial() {
  while (Serial3.available()) {
    char c = (char)Serial3.read();
    if (c == '\n') {
      sbBuffer[sbBufIdx] = '\0';
      if (sbBufIdx > 0) processSoundboardCommand(sbBuffer);
      sbBufIdx = 0;
    } else if (c == '\r') {
      // ignore carriage return, don't reset
    } else if (sbBufIdx < sizeof(sbBuffer) - 1) {
      sbBuffer[sbBufIdx++] = c;
    } else {
      sbBufIdx = 0;  // overflow protection
    }
  }
}


// parses the comma-separated controller data sent from the Nano ESP32 over Serial1
// format: "lx,ly,rx,ry,brake,throttle,buttons,dpad\n"
// walks the string character by character to build up each integer value
// faster than strtok() which has function call overhead and modifies the input string
// returns false if we didn't get exactly 8 fields (bad data, drop it)
bool parseCSV(char* line, ControllerData& c) {
  int fields[8];
  int count = 0;
  int val = 0;
  bool neg = false;
  char ch;

  for (int i = 0; count < 8; i++) {
    ch = line[i];
    if (ch == ',' || ch == '\0') {
      // hit a separator or end of string - save the current value and reset
      fields[count++] = neg ? -val : val;
      val = 0;
      neg = false;
      if (ch == '\0') break;
    } else if (ch == '-') {
      neg = true;
    } else {
      // accumulate digits: val = val * 10 + digit
      val = val * 10 + (ch - '0');
    }
  }

  if (count != 8) return false; // malformed line, reject
  c.lx       = fields[0];
  c.ly       = fields[1];
  c.rx       = fields[2];
  c.ry       = fields[3];
  c.brake    = fields[4];
  c.throttle = fields[5];
  c.buttons  = fields[6];
  c.dpad     = fields[7];
  return true;
}


// deadzones
float applyDeadzone(float v) {
  if (fabs(v) < DEADZONE) return 0;
  if (v > 0) return (v - DEADZONE) / (1.0f - DEADZONE);
  else       return (v + DEADZONE) / (1.0f - DEADZONE);
}

// exponential curve - makes small stick movements gentle, full stick still hits full power
// RESPONSE=1 is linear, RESPONSE=2 is quadratic (smoother near center, same peak)
float applyCurve(float v) {
  float sign = (v >= 0) ? 1.0f : -1.0f;
  return pow(fabs(v), RESPONSE) * sign;
}

// converts stick positions into throttle and steering target values (-1.0 to 1.0 range)
// this just sets targets, the actual ramping happens in updateMotorRamp
// left stick Y = throttle (forward/back), negated so pushing up = forward
// right stick X = steering (turning), positive = right turn
void computeMotorFromStick() {
  float rawThrottle = -controller.ly / 512.0f;
  float rawSteering =  controller.rx / 512.0f;
  throttle_target = applyCurve(applyDeadzone(rawThrottle));
  steering_target = applyCurve(applyDeadzone(rawSteering));
}

// ramps throttle and steering toward their targets, then mixes into motor duty cycles
// runs every 1ms from main loop for smooth feel
//
// three-stage anti-tip when BNO055 sees forward lean:
//   stage 1 (below TILT_MAX): brake softening - reduces decel rate proportionally
//   stage 2 (above TILT_MAX): active catch boost - drives motors forward to catch the fall
//   stage 3 (pitch < DEATH_PITCH): death mode - cuts motors entirely, plays death sound
// steering doesn't get anti-tip because turning doesn't cause forward weight transfer
void updateMotorRamp() {
  // DEATH CHECK - if the goose has tipped over, cut motors completely and bail out
  // death mode triggers in 3 ways:
  //   1. pitch <= DEATH_PITCH (forward tip - face down)
  //   2. pitch >= DEATH_PITCH_BACK (backward tip - happens when crashing into walls)
  //   3. pitch outside IMU sanity range (sensor returning garbage from impact)
  // stays dead until pitch returns to a reasonable upright range
  if (imuReady) {
    bool imu_garbage = (pitch < IMU_PITCH_MIN || pitch > IMU_PITCH_MAX);
    bool tipped_forward = (pitch <= DEATH_PITCH);
    bool tipped_backward = (pitch >= DEATH_PITCH_BACK);

    if (!is_dead && (tipped_forward || tipped_backward || imu_garbage)) {
      // just died - play the death sound once and enter death mode
      is_dead = true;
      playDeathSound();
      if (SERIAL_ENABLED) {
        Serial.print("GOOSE HAS FALLEN (pitch=");
        Serial.print(pitch);
        if (tipped_forward)  Serial.print(" forward tip)");
        if (tipped_backward) Serial.print(" backward tip)");
        if (imu_garbage)     Serial.print(" IMU garbage)");
        Serial.println();
      }
    }
    else if (is_dead && !imu_garbage && pitch > DEATH_RECOVERY_PITCH && pitch < DEATH_RECOVERY_BACK) {
      // goose is back to a reasonable upright angle and IMU is sane - wake up
      is_dead = false;
      if (SERIAL_ENABLED) Serial.println("GOOSE REVIVED");
    }
  }

  // while dead, zero out all motor output and skip the rest of the ramp logic
  // also zero the ramped state so we don't jump to full speed on revive
  if (is_dead) {
    throttle_target = 0.0f;
    steering_target = 0.0f;
    throttle_current = 0.0f;
    steering_current = 0.0f;
    duty_left_current  = 0.0f;
    duty_right_current = 0.0f;
    return;
  }

  // default to normal braking, only softens if tilt detected
  float effectiveThrottleDecel = THROTTLE_RAMP_DECEL;
  // active catch boost - added to both motors after mixing if tilt is extreme
  float catchBoost = 0.0f;

  if (imuReady) {
    // how much forward lean relative to the resting pitch
    // positive = leaning forward, negative/zero = level or leaning back
    float forwardTilt = TILT_RESTING - pitch;

    if (forwardTilt > TILT_THRESHOLD) {
      // STAGE 1: linear brake softening between TILT_THRESHOLD and TILT_MAX
      // 0.0 at threshold (normal brake), 1.0 at max (softest brake)
      float tiltFactor = (forwardTilt - TILT_THRESHOLD) / (TILT_MAX - TILT_THRESHOLD);
      tiltFactor = constrain(tiltFactor, 0.0f, 1.0f);
      // reduce brake force: at max tilt we still brake at 10% so it doesn't just coast forever
      // change the 0.9f to 1.0f if you want brakes to fully pause at max tilt
      effectiveThrottleDecel = THROTTLE_RAMP_DECEL * (1.0f - tiltFactor * 0.9f);
    }

    if (forwardTilt > TILT_MAX) {
      // STAGE 2: past the tipping zone - actively drive forward to catch the fall
      // this is proportional to how far past TILT_MAX we are
      // capped at CATCH_MAX so it can't go crazy if the IMU glitches or reads extreme values
      // works regardless of stick input so you can demo by kicking the goose
      catchBoost = (forwardTilt - TILT_MAX) * CATCH_GAIN;
      catchBoost = constrain(catchBoost, 0.0f, CATCH_MAX);
    }
  }

  // lambda that handles the actual ramping math
  // "reversing" means stick flipped to opposite direction - treat as decel first so it
  // brakes to zero before accelerating the other way (prevents coasting on stick flip)
  auto ramp = [](float current, float target, float accel, float decel) -> float {
    bool reversing = (current > 0.0f && target < 0.0f) || (current < 0.0f && target > 0.0f);
    bool decelerating = reversing || (fabs(target) < fabs(current));
    float rate = decelerating ? decel : accel;
    if      (current < target) { current += rate; if (current > target) current = target; }
    else if (current > target) { current -= rate; if (current < target) current = target; }
    return current;
  };

  // throttle gets the adjusted decel (for stage 1 anti-tip), steering uses its own rates
  throttle_current = ramp(throttle_current, throttle_target, THROTTLE_RAMP_ACCEL, effectiveThrottleDecel);
  steering_current = ramp(steering_current, steering_target, STEERING_RAMP_ACCEL, STEERING_RAMP_DECEL);

  // tank drive mixing: adding steering to one motor and subtracting from the other turns
  // normalization prevents one side from exceeding 1.0 (which would clip)
  float left  = throttle_current + steering_current;
  float right = throttle_current - steering_current;
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) { left /= maxVal; right /= maxVal; }
  duty_left_current  = left  * MAX_DUTY;
  duty_right_current = right * MAX_DUTY;

  // apply stage 2 catch boost equally to both motors (keeps driving straight while catching)
  // applied AFTER mixing so steering still works normally, and AFTER MAX_DUTY scale so the
  // save move can exceed MAX_DUTY if needed to prevent falling
  if (catchBoost > 0.0f) {
    duty_left_current  += catchBoost;
    duty_right_current += catchBoost;
    // hard cap so we never send more than 100% duty
    duty_left_current  = constrain(duty_left_current,  -1.0f, 1.0f);
    duty_right_current = constrain(duty_right_current, -1.0f, 1.0f);
  }
}


// CAN helpers
void packData(can_frame &frame, const uint8_t *data, const int size) {
  for (int i = 0; i < size; i++) frame.data[i] = data[i];
}

void createData(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size) {
  const byte* src = static_cast<const byte*>(data);
  for (int i = 0; i < data_size; i++)  frame_data[i] = src[i];
  for (int i = data_size; i < total_size; i++) frame_data[i] = 0;
}

void sendControlFrame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint) {
  control_frame.can_id = (mode + device_id) | CAN_EFF_FLAG;
  byte data[CONTROL_SIZE];
  createData(&setpoint, data, 4, CONTROL_SIZE);
  packData(control_frame, data, CONTROL_SIZE);
  mcp2515.sendMessage(txb, &control_frame);
}

// reads CAN status frames (only processes what we need)
void drainCANStatus() {
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    uint32_t id = rx.can_id & ~CAN_EFF_FLAG;

    if (id == (status_1 + 1)) {
      memcpy(&velocity_left, rx.data, 4);
      temp_left = rx.data[4];
      uint16_t raw_voltage = ((uint16_t)rx.data[6] << 8) | rx.data[5];
      float new_v = raw_voltage * 0.00782f;
      if (new_v >= 8.0f && new_v <= 14.0f) {
        voltage = addVoltageSample(new_v);
        bool idle = fabs(duty_left_current) < 0.05f && fabs(duty_right_current) < 0.05f;
        if (idle) battery_pct = batteryPercent(voltage);
      }
      if (DEBUG_RAW_BYTES) printRawBytes("S1_L", rx);
    }
    else if (id == (status_1 + 2)) {
      memcpy(&velocity_right, rx.data, 4);
      temp_right = rx.data[4];
      if (DEBUG_RAW_BYTES) printRawBytes("S1_R", rx);
    }
    // skip status_2 frames unless DEBUG_RAW_BYTES is on (saves time)
    else if (DEBUG_RAW_BYTES) {
      if (id == (status_2 + 1)) {
        memcpy(&position_left, rx.data, 4);
        printRawBytes("S2_L", rx);
      }
      else if (id == (status_2 + 2)) {
        memcpy(&position_right, rx.data, 4);
        printRawBytes("S2_R", rx);
      }
    }
  }
}

// debug: raw CAN bytes
void printRawBytes(const char* label, const struct can_frame& frame) {
  if (!SERIAL_ENABLED) return;
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

// serial telemetry (only runs if SERIAL_ENABLED)
void printStatus() {
  if (!SERIAL_ENABLED) return;
  Serial.print("LY=");       Serial.print(controller.ly);
  Serial.print(" RX=");      Serial.print(controller.rx);
  Serial.print(" | LT=");    Serial.print(controller.brake);
  Serial.print(" RT=");      Serial.print(controller.throttle);
  Serial.print(" | Duty L="); Serial.print(duty_left_current, 3);
  Serial.print(" R=");       Serial.print(duty_right_current, 3);
  Serial.print(" | RPM L="); Serial.print(velocity_left);
  Serial.print(" R=");       Serial.print(velocity_right);
  Serial.print(" | Temp L="); Serial.print(temp_left);
  Serial.print("C R=");      Serial.print(temp_right);
  Serial.print("C | Volts="); Serial.print(voltage);
  Serial.print(" | Batt=");  Serial.print(battery_pct);
  Serial.print("% | Vol=");  Serial.print(current_volume);
  Serial.print(" | Track="); Serial.print(current_track);
  Serial.print(" | DJ=");    Serial.print(dj_mode ? "ON" : "OFF");
  Serial.print(" | Hopper="); Serial.print(hopper_open ? "OPEN" : "CLOSED");
  Serial.print(" | Btn=");   Serial.print(controller.buttons);
  Serial.print(" | LCD=");   Serial.print(lcd_mode);
  if (imuReady) {
    Serial.print(" | Tilt="); Serial.print(pitch, 1);
    Serial.print(" Fwd="); Serial.print(TILT_RESTING - pitch, 1);
  }
  Serial.print(" | RX=");    Serial.print(rxByteCount);
  Serial.print("/");          Serial.println(rxParseCount);
}


// SETUP
void setup() {
  if (SERIAL_ENABLED) Serial.begin(115200);

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);
  setColor(0, 0, 0);

  heartbeat_frame.can_id  = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = 8;
  packData(heartbeat_frame, HEARTBEAT_DATA, 8);
  control_frame.can_dlc = 8;
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  lcdSetup();
  imuSetup();
  speakerSetup();
  hopperSetup();
  Serial1.begin(9600); // controller Nano
  Serial3.begin(9600); // soundboard Nano (web server)
  randomSeed(analogRead(A0));

  startupSequence();

  if (SERIAL_ENABLED) Serial.println("ROBOGOOSE MEGA READY");
}


// MAIN LOOP
void loop() {

  // DEBUG: dump anything coming in on Serial3
  //while (Serial3.available()) {
    //char c = Serial3.read();
    //if (SERIAL_ENABLED) Serial.print(c);
  //}
  //return; // bail out so nothing else runs

  drainCANStatus();

  // only check DJ timer if an intro is actually playing
  if (dj_intro_playing) updateDJMode();

  // read controller data — process all available bytes without yielding
  while (Serial1.available()) {
    char c = Serial1.read();
    rxByteCount++;
    if (c == '\n') {
      buffer[bufIdx] = '\0';
      if (bufIdx > 0 && parseCSV(buffer, controller)) {
        rxParseCount++;
        computeMotorFromStick();

        // check if LT is held - this gates whether buttons do normal or modifier actions
        // controller.brake is the LT analog trigger value (0-1023)
        lt_held = (controller.brake > LT_MOD_THRESHOLD);

        // RT (controller.throttle) sets eyes purple when held - works regardless of LT
        handleRTPurpleEyes(controller.throttle);

        if (lt_held) {
          // LT held - run modifier handlers only
          // these handle: LT+A, LT+B, LT+DPad combos
          handleLTModifierButtons(controller.buttons, controller.dpad);
        } else {
          // LT not held - run all the normal handlers
          handleLEDButtons(controller.buttons);
          if (controller.buttons & BTN_RB) { rainbow_mode = true; leds_off = false; }
          handleHopperButton(controller.buttons);
          handleDJModeToggle(controller.buttons);
          handleRandomSoundButton(controller.buttons);
          handleHonkButton(controller.buttons);
          handleSoundDpad(controller.dpad);
        }
      }
      bufIdx = 0;
    } else if (c != '\r' && bufIdx < sizeof(buffer) - 1) {
      buffer[bufIdx++] = c;
    } else {
      bufIdx = 0;
    }
  }

  // process incoming commands from soundboard Nano
  handleSoundboardSerial();

  unsigned long now = millis();

  // LED + IMU update (every 20ms = 50Hz)
  static unsigned long led_last = 0;
  if (now - led_last >= 20) {
    readIMU(); // read pitch angle for anti-tip (fast, ~1ms I2C read)
    if (rainbow_mode) updateRainbow();
    else if (!leds_off && (led_r > 0 || led_g > 0 || led_b > 0)) updateBreathing();
    led_last = now;
  }

  // motor ramp (every 1ms)
  static unsigned long ramp_last = 0;
  if (now - ramp_last >= 1) {
    updateMotorRamp();
    ramp_last = now;
  }

  // CAN heartbeat (every 10ms)
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

  // right motor command (every 10ms, offset 5ms)
  static unsigned long ctrl_right_last = 10;
  if (now - ctrl_right_last >= 10) {
    sendControlFrame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right_current);
    ctrl_right_last = now;
  }

  // LCD update (every 500ms normally, 150ms when showing IMU for live tuning [makes driving less responsive])
  static unsigned long lcd_last = 0;
  unsigned long lcd_interval = (lcd_mode == 4) ? 150 : 500;
  if (now - lcd_last >= lcd_interval) {
    updateLCD();
    lcd_last = now;
  }

  // serial telemetry (every 250ms)
  static unsigned long print_last = 0;
  if (now - print_last >= 250) {
    printStatus();
    print_last = now;
  }

  // soundboard Nano stats dump (every 500ms)
  static unsigned long stats_last = 0;
  if (now - stats_last >= 500) {
    sendStatsToNano();
    stats_last = now;
  }
}