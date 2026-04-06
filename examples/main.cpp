#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <math.h>
#include <Servo.h>
#include "DFRobotDFPlayerMini.h"

// PIN ASSIGNMENTS (Arduino Mega)
// Serial1 (18 TX, 19 RX) = Controller input from Nano
// Serial2 (16 TX, 17 RX) = DFPlayer Mini
// SPI (50 MISO, 51 MOSI, 52 SCK, 53 SS) = MCP2515 CAN
// 2  = CAN INT
// 3  = Red LED (PWM)
// 4  = Green LED (PWM)
// 5  = Blue LED (PWM)
// 6  = DRV8871 IN2 propeller speed (PWM)
// 7  = DRV8871 IN1 propeller direction
// 8  = Hopper servo

MCP2515 mcp2515(53);
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;
Servo hopperServo;

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
const int HOPPER_CLOSED = 75;   //
const int HOPPER_OPEN   = 170;  //

// Hopper state
bool hopper_open = false;


// LED COLOR (common cathode, all PWM)
void setColor(int r, int g, int b) {
  analogWrite(redPin,   r);
  analogWrite(greenPin, g);
  analogWrite(bluePin,  b);
}


// CAN CONTROL MODES
enum control_mode {
  Duty_Cycle_Set     = 0x2050080,
  Speed_Set          = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set       = 0x2050C80,
  Voltage_Set        = 0x2051080,
  Current_Set        = 0x20510C0,
  Smart_Motion_Set   = 0x2051480
};

// CAN STATUS FRAME IDs
enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

// CAN HEARTBEAT
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t  HEARTBEAT_DATA[8] = {255,255,255,255,255,255,255,255};

struct can_frame heartbeat_frame;
struct can_frame control_frame;
const uint8_t CONTROL_SIZE = 8;


// DRIVING CONSTANTS
const float MAX_DUTY        = 0.3f;
const float DEADZONE        = 0.15f;
const float RESPONSE        = 2.0f;
const float RAMP_RATE_ACCEL = 0.002f;
const float RAMP_RATE_DECEL = 0.002f;

// MOTOR STATE (drive motors)
float duty_left_target   = 0.0f;
float duty_right_target  = 0.0f;
float duty_left_current  = 0.0f;
float duty_right_current = 0.0f;

// SENSOR STATE
float   velocity_left  = 0.0f;
float   velocity_right = 0.0f;
float   position_left  = 0.0f;
float   position_right = 0.0f;
float   current_left   = 0.0f;
float   current_right  = 0.0f;
uint8_t temp_left      = 0;
uint8_t temp_right     = 0;
float   voltage        = 0.0f;
float   battery_pct    = 0.0f;

// Voltage rolling average
static constexpr int VOLTAGE_SAMPLES = 20;
float voltage_buf[VOLTAGE_SAMPLES] = {};
int   voltage_idx      = 0;
bool  voltage_buf_full = false;

// CONTROLLER DATA
struct ControllerData {
  int lx, ly, rx, ry;
  int brake, throttle;
  int buttons, dpad;
};
ControllerData controller;

char buffer[64];
uint8_t bufIdx = 0;

// BUTTON BIT MASKS (Bluepad32)
const int BTN_A  = (1 << 0);
const int BTN_B  = (1 << 1);
const int BTN_X  = (1 << 2);
const int BTN_Y  = (1 << 3);
const int BTN_LB = (1 << 4);
const int BTN_RB = (1 << 5);
const int BTN_LS = (1 << 8);
const int BTN_RS = (1 << 9);

// DPAD BITMASK VALUES (Bluepad32)
const int DPAD_UP    = 0x01;
const int DPAD_DOWN  = 0x02;
const int DPAD_RIGHT = 0x04;
const int DPAD_LEFT  = 0x08;

// LED STATE
int led_r = 0, led_g = 0, led_b = 0;
bool rainbow_mode = false;
float rainbow_hue = 0.0f;

// SOUND STATE
int current_track  = 1;
const int MAX_TRACKS = 9;
bool sound_playing = false;

// SERIAL RX COUNTER (for debugging)
unsigned long rxByteCount = 0;
unsigned long rxParseCount = 0;

// DEBUG
static bool DEBUG_RAW_BYTES = false;


// FORWARD DECLARATIONS
float addVoltageSample(float v);
float batteryPercent(float v);
void hsvToRgb(float h, float s, float v, int &r, int &g, int &b);
void handleLEDButtons(int buttons);
void updateRainbow();
void speakerSetup();
void playTrack(int track);
void stopSound();
void handleSoundDpad(int dpad);
void propellerSetup();
void updatePropeller(int triggerValue);
void hopperSetup();
void hopperToggle();
void handleHopperButton(int buttons);
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


// VOLTAGE AVERAGING
float addVoltageSample(float v) {
  voltage_buf[voltage_idx] = v;
  voltage_idx = (voltage_idx + 1) % VOLTAGE_SAMPLES;
  if (voltage_idx == 0) voltage_buf_full = true;
  int count = voltage_buf_full ? VOLTAGE_SAMPLES : voltage_idx;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += voltage_buf[i];
  return sum / count;
}

// BATTERY PERCENTAGE (3S LiPo)
float batteryPercent(float v) {
  const float max_v = 12.6f;
  const float min_v = 10.5f;
  float pct = (v - min_v) / (max_v - min_v) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}


// HSV TO RGB
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


// LED: set color from controller buttons
// A = green, B = red, X = blue, Y = yellow
// LS (left stick click) = purple, RS (right stick click) = white
void handleLEDButtons(int buttons) {
  if      (buttons & BTN_A)  { rainbow_mode = false; led_r =   0; led_g = 255; led_b =   0; } // green
  else if (buttons & BTN_B)  { rainbow_mode = false; led_r = 255; led_g =   0; led_b =   0; } // red
  else if (buttons & BTN_X)  { rainbow_mode = false; led_r =   0; led_g =   0; led_b = 255; } // blue
  else if (buttons & BTN_Y)  { rainbow_mode = false; led_r = 255; led_g = 180; led_b =   0; } // yellow
  else if (buttons & BTN_LS) { rainbow_mode = false; led_r = 148; led_g =   0; led_b = 211; } // purple
  else if (buttons & BTN_RS) { rainbow_mode = false; led_r = 255; led_g = 255; led_b = 255; } // white

  if (!rainbow_mode) setColor(led_r, led_g, led_b);
}

// LED: advance rainbow cycle (call periodically)
void updateRainbow() {
  rainbow_hue += 4.0f;
  if (rainbow_hue >= 360.0f) rainbow_hue -= 360.0f;
  int r, g, b;
  hsvToRgb(rainbow_hue, 1.0f, 1.0f, r, g, b);
  setColor(r, g, b);
}


// SPEAKER: initialize DFPlayer on Serial2 (hardware serial)
void speakerSetup() {
  Serial2.begin(9600);
  Serial.println("Initializing DFPlayer...");
  delay(2000);
  if (!dfPlayer.begin(Serial2, false, false)) {
    Serial.println("DFPlayer ERROR: check wiring and SD card");
    dfPlayerReady = false;
    return;
  }
  Serial.println("DFPlayer ready");
  dfPlayer.volume(30);
  dfPlayerReady = true;
}

// SPEAKER: play a specific track number
void playTrack(int track) {
  if (!dfPlayerReady) return;
  Serial.print("Playing track: ");
  Serial.println(track);
  dfPlayer.play(track);
  sound_playing = true;
}

// SPEAKER: stop playback
void stopSound() {
  if (!dfPlayerReady) return;
  Serial.println("Stopping audio");
  dfPlayer.pause();
  sound_playing = false;
}

// SPEAKER: handle dpad input for sound selection
void handleSoundDpad(int dpad) {
  static int prev_dpad = 0;

  if (dpad == prev_dpad) return;
  prev_dpad = dpad;

  if (dpad & DPAD_RIGHT) {
    current_track++;
    if (current_track > MAX_TRACKS) current_track = 1;
    Serial.print("Selected track: ");
    Serial.println(current_track);
    playTrack(current_track);
  }
  else if (dpad & DPAD_LEFT) {
    current_track--;
    if (current_track < 1) current_track = MAX_TRACKS;
    Serial.print("Selected track: ");
    Serial.println(current_track);
    playTrack(current_track);
  }
  else if (dpad & DPAD_UP) {
    playTrack(current_track);
  }
  else if (dpad & DPAD_DOWN) {
    stopSound();
  }
}


// PROPELLER: initialize DRV8871 pins
void propellerSetup() {
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  digitalWrite(motorIN1, LOW);
  analogWrite(motorIN2, 0);
  Serial.println("Propeller motor ready");
}

// PROPELLER: set speed from left trigger (0-1023 analog input)
// Capped at 50% PWM for 12V battery safety
void updatePropeller(int triggerValue) {
  int speed = map(triggerValue, 0, 1023, 0, 128);
  if (speed < 10) speed = 0;
  digitalWrite(motorIN1, LOW);
  analogWrite(motorIN2, speed);
}


// HOPPER: initialize servo
void hopperSetup() {
  hopperServo.attach(servoPin);
  hopperServo.write(HOPPER_CLOSED);
  hopper_open = false;
  Serial.println("Hopper servo ready (closed)");
}

// HOPPER: toggle open/closed
void hopperToggle() {
  hopper_open = !hopper_open;
  hopperServo.write(hopper_open ? HOPPER_OPEN : HOPPER_CLOSED);
  Serial.print("Hopper ");
  Serial.println(hopper_open ? "OPEN" : "CLOSED");
}

// HOPPER: check LB button and toggle (edge detection)
void handleHopperButton(int buttons) {
  static bool prev_lb = false;
  bool lb_now = buttons & BTN_LB;

  if (lb_now && !prev_lb) {
    hopperToggle();
  }
  prev_lb = lb_now;
}


// CSV PARSER: "lx,ly,rx,ry,brake,throttle,buttons,dpad"
bool parseCSV(char* line, ControllerData& c) {
  int fields[8];
  int count = 0;
  char* token = strtok(line, ",");
  while (token != nullptr && count < 8) {
    fields[count++] = atoi(token);
    token = strtok(nullptr, ",");
  }
  if (count != 8) return false;
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


// DRIVING: apply deadzone to a -1..1 value
float applyDeadzone(float v) {
  if (fabs(v) < DEADZONE) return 0;
  if (v > 0) return (v - DEADZONE) / (1.0f - DEADZONE);
  else       return (v + DEADZONE) / (1.0f - DEADZONE);
}

// DRIVING: apply exponential response curve
float applyCurve(float v) {
  float sign = (v >= 0) ? 1.0f : -1.0f;
  return pow(fabs(v), RESPONSE) * sign;
}

// DRIVING: compute left/right duty from stick input
void computeMotorFromStick() {
  float throttle =  controller.rx / 512.0f;
  float steering = -controller.ly / 512.0f;
  throttle = applyCurve(applyDeadzone(throttle));
  steering = applyCurve(applyDeadzone(steering));
  float left  = throttle + steering;
  float right = throttle - steering;
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) { left /= maxVal; right /= maxVal; }
  duty_left_target  = left  * MAX_DUTY;
  duty_right_target = right * MAX_DUTY;
}

// DRIVING: smooth ramp toward target duty
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


// CAN: pack bytes into a frame
void packData(can_frame &frame, const uint8_t *data, const int size) {
  for (int i = 0; i < size; i++) frame.data[i] = data[i];
}

// CAN: create data array from a value
void createData(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size) {
  const byte* src = static_cast<const byte*>(data);
  for (int i = 0; i < data_size; i++)  frame_data[i] = src[i];
  for (int i = data_size; i < total_size; i++) frame_data[i] = 0;
}

// CAN: send a motor control frame
void sendControlFrame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint) {
  control_frame.can_id = (mode + device_id) | CAN_EFF_FLAG;
  byte data[CONTROL_SIZE];
  createData(&setpoint, data, 4, CONTROL_SIZE);
  packData(control_frame, data, CONTROL_SIZE);
  mcp2515.sendMessage(txb, &control_frame);
}

// CAN: read all pending status frames
void drainCANStatus() {
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    uint32_t id = rx.can_id & ~CAN_EFF_FLAG;

    if (id == (status_1 + 1)) {
      memcpy(&velocity_left, rx.data, 4);
      temp_left = rx.data[4];
      float raw_i = rx.data[6] * 0.125f;
      if (raw_i < 40.0f) current_left = raw_i;
      float new_v = rx.data[5] * 0.0658f;
      if (new_v >= 10.0f) {
        voltage = addVoltageSample(new_v);
        bool idle = fabs(duty_left_current) < 0.05f && fabs(duty_right_current) < 0.05f;
        if (idle) battery_pct = batteryPercent(voltage);
      }
      if (DEBUG_RAW_BYTES) printRawBytes("S1_L", rx);
    }
    else if (id == (status_1 + 2)) {
      memcpy(&velocity_right, rx.data, 4);
      temp_right = rx.data[4];
      float raw_i = rx.data[6] * 0.125f;
      if (raw_i < 40.0f) current_right = raw_i;
      if (DEBUG_RAW_BYTES) printRawBytes("S1_R", rx);
    }
    else if (id == (status_2 + 1)) {
      memcpy(&position_left, rx.data, 4);
      if (DEBUG_RAW_BYTES) printRawBytes("S2_L", rx);
    }
    else if (id == (status_2 + 2)) {
      memcpy(&position_right, rx.data, 4);
      if (DEBUG_RAW_BYTES) printRawBytes("S2_R", rx);
    }
  }
}

// DEBUG: print raw CAN frame bytes
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

// DEBUG: print telemetry
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
  Serial.print(" | Prop=");  Serial.print(controller.brake);
  Serial.print(" | Hopper="); Serial.print(hopper_open ? "OPEN" : "CLOSED");
  Serial.print(" | RX=");    Serial.print(rxByteCount);
  Serial.print("/");          Serial.println(rxParseCount);
}


// SETUP
void setup() {
  Serial.begin(115200);

  // LED
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);
  setColor(0, 0, 0);

  // CAN bus (Mega SPI CS = pin 53)
  heartbeat_frame.can_id  = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = 8;
  packData(heartbeat_frame, HEARTBEAT_DATA, 8);
  control_frame.can_dlc = 8;
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Speaker on Serial2
  speakerSetup();

  // Propeller motor
  propellerSetup();

  // Hopper servo
  hopperSetup();

  // Controller input on Serial1
  Serial1.begin(9600);

  Serial.println("ROBOGOOSE MEGA READY");
}


// MAIN LOOP
void loop() {

  drainCANStatus();

  // Read controller data from Nano on Serial1
  while (Serial1.available()) {
    char c = Serial1.read();
    rxByteCount++;
    if (c == '\n') {
      buffer[bufIdx] = '\0';
      if (bufIdx > 0 && parseCSV(buffer, controller)) {
        rxParseCount++;

        computeMotorFromStick();

        handleLEDButtons(controller.buttons);

        // LB = hopper toggle
        handleHopperButton(controller.buttons);

        // RB = rainbow mode
        if (controller.buttons & BTN_RB) { rainbow_mode = true; }

        // RT (throttle) = LEDs off
        if (controller.throttle > 10) { rainbow_mode = false; led_r = 0; led_g = 0; led_b = 0; setColor(0, 0, 0); }

        handleSoundDpad(controller.dpad);

        updatePropeller(controller.brake);
      }
      bufIdx = 0;
    } else if (c != '\r' && bufIdx < sizeof(buffer) - 1) {
      buffer[bufIdx++] = c;
    } else {
      bufIdx = 0;
    }
  }

  unsigned long now = millis();

  // Rainbow LED update (every 20ms)
  static unsigned long rainbow_last = 0;
  if (rainbow_mode && now - rainbow_last >= 20) {
    updateRainbow();
    rainbow_last = now;
  }

  // Motor ramp (every 1ms)
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

  // Left motor command (every 10ms)
  static unsigned long ctrl_left_last = 5;
  if (now - ctrl_left_last >= 10) {
    sendControlFrame(MCP2515::TXB1, 1, Duty_Cycle_Set, duty_left_current);
    ctrl_left_last = now;
  }

  // Right motor command (every 10ms, offset 5ms)
  static unsigned long ctrl_right_last = 10;
  if (now - ctrl_right_last >= 10) {
    sendControlFrame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right_current);
    ctrl_right_last = now;
  }

  // Serial telemetry (every 100ms)
  static unsigned long print_last = 0;
  if (now - print_last >= 100) {
    printStatus();
    print_last = now;
  }
}
