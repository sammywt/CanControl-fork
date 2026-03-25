#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial Serial1(3, -1);  // RX=Pin3

MCP2515 mcp2515(10);

// RGB LED PINS (Common Anode)
const int redPin   = 6;
const int greenPin = 5;
const int bluePin  = 4;

void setColor(int r, int g, int b) {
  analogWrite(redPin,   255 - r);
  analogWrite(greenPin, 255 - g);
  analogWrite(bluePin,  255 - b);
}

// CONTROL MODES
enum control_mode {
  Duty_Cycle_Set     = 0x2050080,
  Speed_Set          = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set       = 0x2050C80,
  Voltage_Set        = 0x2051080,
  Current_Set        = 0x20510C0,
  Smart_Motion_Set   = 0x2051480
};

// STATUS FRAME IDs
enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

// HEARTBEAT
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t  HEARTBEAT_DATA[8] = {255,255,255,255,255,255,255,255};

struct can_frame heartbeat_frame;
struct can_frame control_frame;

const uint8_t CONTROL_SIZE = 8;

// DRIVING CONSTANTS
const float MAX_DUTY        = 0.3f;
const float DEADZONE        = 0.15f;
const float RESPONSE        = 2.0f;
<<<<<<< HEAD
const float RAMP_RATE_ACCEL = 0.002f;
const float RAMP_RATE_DECEL = 0.002f;
=======
const float RAMP_RATE_ACCEL = 0.001f;   // increased from 0.001f
const float RAMP_RATE_DECEL = 0.001f;   // increased from 0.001f, faster for safety
>>>>>>> a0a2a1571452a2c4e2775822ffc1f5f74719fc68

// MOTOR STATE
float duty_left_target  = 0.0f;
float duty_right_target = 0.0f;
float duty_left_current  = 0.0f;
float duty_right_current = 0.0f;

// ENCODER / SENSOR STATE
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

// Rolling average for voltage
static constexpr int VOLTAGE_SAMPLES = 20;
float voltage_buf[VOLTAGE_SAMPLES] = {};
int   voltage_idx                  = 0;
bool  voltage_buf_full             = false;

float addVoltageSample(float v) {
  voltage_buf[voltage_idx] = v;
  voltage_idx = (voltage_idx + 1) % VOLTAGE_SAMPLES;
  if (voltage_idx == 0) voltage_buf_full = true;
  int count = voltage_buf_full ? VOLTAGE_SAMPLES : voltage_idx;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += voltage_buf[i];
  return sum / count;
}

static bool DEBUG_RAW_BYTES = false;

// SERIAL PARSER
struct ControllerData {
  int lx, ly, rx, ry;
  int brake;
  int throttle;
  int buttons;
  int dpad;
};

ControllerData controller;

char buffer[64];
uint8_t bufIdx = 0;

// BUTTON BIT MASKS
const int BTN_A    = (1 << 0);
const int BTN_B    = (1 << 1);
const int BTN_X    = (1 << 2);
const int BTN_Y    = (1 << 3);
const int BTN_LB   = (1 << 4);
const int BTN_RB   = (1 << 5);
const int BTN_LS   = (1 << 8);
const int BTN_RS   = (1 << 9);

// LED STATE
int led_r = 0, led_g = 0, led_b = 0;

// RAINBOW STATE
bool rainbow_mode = false;
float rainbow_hue = 0.0f;

// HSV to RGB helper
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

void handleLEDButtons(int buttons) {
  if      (buttons & BTN_A)  { rainbow_mode = false; led_r =   0; led_g = 255; led_b =   0; }
  else if (buttons & BTN_B)  { rainbow_mode = false; led_r = 255; led_g =   0; led_b =   0; }
  else if (buttons & BTN_Y)  { rainbow_mode = false; led_r = 255; led_g = 180; led_b =   0; }
  else if (buttons & BTN_X)  { rainbow_mode = false; led_r =   0; led_g =   0; led_b = 255; }
  else if (buttons & BTN_LB) { rainbow_mode = false; led_r = 148; led_g =   0; led_b = 211; }
  else if (buttons & BTN_RB) { rainbow_mode = false; led_r = 255; led_g = 255; led_b = 255; }

  if (!rainbow_mode) setColor(led_r, led_g, led_b);
}

// BATTERY PERCENTAGE
float batteryPercent(float v) {
  const float max_v = 12.6f;
  const float min_v = 10.5f;
  float pct = (v - min_v) / (max_v - min_v) * 100.0f;
  if (pct > 100.0f) pct = 100.0f;
  if (pct <   0.0f) pct =   0.0f;
  return pct;
}

// CSV PARSER
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

// DEADZONE
float applyDeadzone(float v) {
  if (fabs(v) < DEADZONE) return 0;
  if (v > 0) return (v - DEADZONE) / (1.0f - DEADZONE);
  else       return (v + DEADZONE) / (1.0f - DEADZONE);
}

// RESPONSE CURVE
float applyCurve(float v) {
  float sign = (v >= 0) ? 1.0f : -1.0f;
  return pow(fabs(v), RESPONSE) * sign;
}

// MOTOR DRIVE
// Left stick up/down (ly):     up = forward, down = backward
// Right stick left/right (rx): right = turn right, left = turn left
void computeMotorFromStick() {
  float y =  controller.rx / 512.0f;   // throttle
  float x = -controller.ly / 512.0f;   // steering
  x = applyDeadzone(x);
  y = applyDeadzone(y);
  x = applyCurve(x);
  y = applyCurve(y);
  float left  = y + x;
  float right = y - x;
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) {
    left  /= maxVal;
    right /= maxVal;
  }
  duty_left_target  = left  * MAX_DUTY;
  duty_right_target = right * MAX_DUTY;
}

// MOTOR RAMPING
void updateMotorRamp() {
  auto rampAxis = [](float current, float target) -> float {
    bool decelerating = fabs(target) < fabs(current);
    float rate = decelerating ? RAMP_RATE_DECEL : RAMP_RATE_ACCEL;
    if (current < target) {
      current += rate;
      if (current > target) current = target;
    } else if (current > target) {
      current -= rate;
      if (current < target) current = target;
    }
    return current;
  };
  duty_left_current  = rampAxis(duty_left_current,  duty_left_target);
  duty_right_current = rampAxis(duty_right_current, duty_right_target);
}

// SERIAL PRINT
void printStatus() {
  Serial.print("LY=");       Serial.print(controller.ly);
  Serial.print(" RX=");      Serial.print(controller.rx);
  Serial.print(" | Target L="); Serial.print(duty_left_target);
  Serial.print(" R=");       Serial.print(duty_right_target);
  Serial.print(" | RPM L="); Serial.print(velocity_left);
  Serial.print(" R=");       Serial.print(velocity_right);
  Serial.print(" | Pos L="); Serial.print(position_left);
  Serial.print(" R=");       Serial.print(position_right);
  Serial.print(" | Temp L="); Serial.print(temp_left);
  Serial.print("C R=");      Serial.print(temp_right);
  Serial.print("C | mA L="); Serial.print(current_left);
  Serial.print(" R=");       Serial.print(current_right);
  Serial.print(" | Volts="); Serial.print(voltage);
  Serial.print(" | Batt=");  Serial.print(battery_pct);
  Serial.println("%");
}

// CAN HELPERS
void pack_data(can_frame &frame, const uint8_t *data, const int size) {
  for (int i = 0; i < size; i++)
    frame.data[i] = data[i];
}

void create_data(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size) {
  const byte* data_arr = static_cast<const byte*>(data);
  for (int i = 0; i < data_size; i++)
    frame_data[i] = data_arr[i];
  for (int i = data_size; i < total_size; i++)
    frame_data[i] = 0;
}

void send_control_frame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint) {
  control_frame.can_id = (mode + device_id) | CAN_EFF_FLAG;
  byte control_data[CONTROL_SIZE];
  create_data(&setpoint, control_data, 4, CONTROL_SIZE);
  pack_data(control_frame, control_data, CONTROL_SIZE);
  mcp2515.sendMessage(txb, &control_frame);
}

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

// SETUP
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("CAN CONTROLLER READY");

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);
  setColor(0, 0, 0);

  heartbeat_frame.can_id  = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = 8;
  pack_data(heartbeat_frame, HEARTBEAT_DATA, 8);
  control_frame.can_dlc = 8;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

// LOOP
void loop() {

  // DRAIN RX FIRST — prevents status frame backlog from delaying control sends
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {

    uint32_t id = rx.can_id & ~CAN_EFF_FLAG;

    if (id == (status_1 + 1)) {
      memcpy(&velocity_left, rx.data, 4);
      temp_left = rx.data[4];
      float raw_current_l = rx.data[6] * 0.125f;
      current_left = (raw_current_l < 40.0f) ? raw_current_l : current_left;
      float new_voltage = rx.data[5] * 0.0658f;
      if (new_voltage >= 10.0f) {
        voltage = addVoltageSample(new_voltage);
        bool motors_idle = fabs(duty_left_current) < 0.05f && fabs(duty_right_current) < 0.05f;
        if (motors_idle) battery_pct = batteryPercent(voltage);
      }
      if (DEBUG_RAW_BYTES) printRawBytes("S1_L", rx);
    }
    else if (id == (status_1 + 2)) {
      memcpy(&velocity_right, rx.data, 4);
      temp_right = rx.data[4];
      float raw_current_r = rx.data[6] * 0.125f;
      current_right = (raw_current_r < 40.0f) ? raw_current_r : current_right;
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

  // READ CONTROLLER DATA
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buffer[bufIdx] = '\0';
      if (bufIdx > 0 && parseCSV(buffer, controller)) {
        computeMotorFromStick();
        handleLEDButtons(controller.buttons);
        if      (controller.brake    > 10) { rainbow_mode = false; led_r = 0; led_g = 0; led_b = 0; setColor(0, 0, 0); }
        else if (controller.throttle > 10) { rainbow_mode = true; }
      }
      bufIdx = 0;
    } else if (c != '\r' && bufIdx < sizeof(buffer) - 1) {
      buffer[bufIdx++] = c;
    } else {
      bufIdx = 0;
    }
  }

  unsigned long now = millis();

  // RAINBOW UPDATE every 20ms
  static unsigned long rainbow_last = 0;
  if (rainbow_mode && now - rainbow_last >= 20) {
    rainbow_hue += 4.0f;
    if (rainbow_hue >= 360.0f) rainbow_hue -= 360.0f;
    int r, g, b;
    hsvToRgb(rainbow_hue, 1.0f, 1.0f, r, g, b);
    setColor(r, g, b);
    rainbow_last = now;
  }

  // RAMP MOTORS every 1ms
  static unsigned long ramp_last = 0;
  if (now - ramp_last >= 1) {
    updateMotorRamp();
    ramp_last = now;
  }

  // HEARTBEAT every 10ms
  static unsigned long hb_last = 0;
  if (now - hb_last >= 10) {
    mcp2515.sendMessage(MCP2515::TXB0, &heartbeat_frame);
    hb_last = now;
  }

  // LEFT MOTOR every 10ms
  static unsigned long ctrl_left_last = 5;
  if (now - ctrl_left_last >= 10) {
    send_control_frame(MCP2515::TXB1, 1, Duty_Cycle_Set, duty_left_current);
    ctrl_left_last = now;
  }

  // RIGHT MOTOR every 10ms, offset by 5ms from left
  static unsigned long ctrl_right_last = 10;
  if (now - ctrl_right_last >= 10) {
    send_control_frame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right_current);
    ctrl_right_last = now;
  }

  // SERIAL PRINT every 100ms
  static unsigned long print_last = 0;
  if (now - print_last >= 100) {
    printStatus();
    print_last = now;
  }
}