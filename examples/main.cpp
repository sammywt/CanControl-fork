#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial Serial1(3, -1);  // RX=Pin3

MCP2515 mcp2515(10);

// -------- CONTROL MODES --------
enum control_mode {
  Duty_Cycle_Set    = 0x2050080,
  Speed_Set         = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set      = 0x2050C80,
  Voltage_Set       = 0x2051080,
  Current_Set       = 0x20510C0,
  Smart_Motion_Set  = 0x2051480
};

// -------- STATUS FRAME IDs --------
enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

// -------- HEARTBEAT --------
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t  HEARTBEAT_DATA[8] = {255,255,255,255,255,255,255,255};

struct can_frame heartbeat_frame;
struct can_frame control_frame;

const uint8_t CONTROL_SIZE = 8;

// DRIVING CONSTANTS
const float MAX_DUTY      = 0.35f;  // max motor power (forward and reverse)
const float DEADZONE      = 0.10f;  // joystick deadzone
const float RESPONSE      = 2.0f;   // response curve exponent (lower = more linear feel)
const float RAMP_RATE     = 0.0001f; // acceleration ramp per loop
const float TURN_BLEND    = 0.65f;  // how much turning mixes in (lower = gentler turns)
const float RAMP_RATE_ACCEL = 0.0001f;  // how fast it speeds up
const float RAMP_RATE_DECEL = 0.0001f;  // how fast it slows down (gentler)

// -------- MOTOR STATE --------
float duty_left_target  = 0.0f;
float duty_right_target = 0.0f;

float duty_left_current  = 0.0f;
float duty_right_current = 0.0f;

// -------- SERIAL PARSER --------
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

// ------------------------------------------------
// CSV PARSER
// ------------------------------------------------

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

// ------------------------------------------------
// DEADZONE
// ------------------------------------------------

float applyDeadzone(float v) {

  if (fabs(v) < DEADZONE) return 0;

  if (v > 0)
    return (v - DEADZONE) / (1.0f - DEADZONE);
  else
    return (v + DEADZONE) / (1.0f - DEADZONE);
}

// ------------------------------------------------
// RESPONSE CURVE
// ------------------------------------------------

float applyCurve(float v) {

  float sign = (v >= 0) ? 1.0f : -1.0f;

  v = pow(fabs(v), RESPONSE);

  return v * sign;
}

// ------------------------------------------------
// MOTOR DRIVE
// Left stick Y = forward/back, Right stick X = turn
// ------------------------------------------------

void computeMotorFromStick() {

  float y = -controller.ly / 512.0f;  // left stick forward/back
  float x =  controller.rx / 512.0f;  // right stick turn

  x = applyDeadzone(x);
  y = applyDeadzone(y);

  x = applyCurve(x);
  y = applyCurve(y);

  float left  = y + x;
  float right = y - x;

  // Normalize instead of clamp — preserves ratio so one motor
  // is always faster than the other when combining forward + turn
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) {
    left  /= maxVal;
    right /= maxVal;
  }

  duty_left_target  = left  * MAX_DUTY;
  duty_right_target = right * MAX_DUTY;
}

// ------------------------------------------------
// MOTOR RAMPING
// Accelerates at RAMP_RATE_ACCEL, decelerates slower
// ------------------------------------------------

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

// ------------------------------------------------
// CAN HELPERS
// ------------------------------------------------

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

// ------------------------------------------------
// SETUP
// ------------------------------------------------

void setup() {

  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println("CAN CONTROLLER READY");

  heartbeat_frame.can_id  = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = 8;

  pack_data(heartbeat_frame, HEARTBEAT_DATA, 8);

  control_frame.can_dlc = 8;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

// ------------------------------------------------
// LOOP
// ------------------------------------------------

void loop() {

  // ---- READ CONTROLLER DATA ----
  while (Serial1.available()) {

    char c = Serial1.read();

    if (c == '\n') {

      buffer[bufIdx] = '\0';

      if (bufIdx > 0 && parseCSV(buffer, controller)) {

        computeMotorFromStick();

        Serial.print("LY=");
        Serial.print(controller.ly);

        Serial.print(" RX=");
        Serial.print(controller.rx);

        Serial.print(" | Target L=");
        Serial.print(duty_left_target);

        Serial.print(" R=");
        Serial.println(duty_right_target);
      }

      bufIdx = 0;
    }

    else if (c != '\r' && bufIdx < sizeof(buffer) - 1)
      buffer[bufIdx++] = c;

    else
      bufIdx = 0;
  }

  // ---- RAMP MOTORS ----
  updateMotorRamp();

  // ---- CLEAR CAN RX ----
  struct can_frame rx;

  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {}

  unsigned long now = millis();

  // ---- HEARTBEAT ----
  static unsigned long hb_last = 0;

  if (now - hb_last >= 10) {

    mcp2515.sendMessage(MCP2515::TXB0, &heartbeat_frame);

    hb_last = now;
  }

  // ---- MOTOR COMMANDS ----
  static unsigned long ctrl_last = 5;

  if (now - ctrl_last >= 10) {

    send_control_frame(MCP2515::TXB1, 1, Duty_Cycle_Set, duty_left_current);
    send_control_frame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right_current);

    ctrl_last = now;
  }
}