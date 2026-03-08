#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial Serial1(3, -1);  // RX=Pin3

MCP2515 mcp2515(10);

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
const float MAX_DUTY        = 1.0f;
const float DEADZONE        = 0.15f;
const float RESPONSE        = 2.0f;
// Ramp rates are now per-millisecond (loop calls ramp every ms via timer)
// 0.001 per ms = full speed in ~350ms, tune to taste
const float RAMP_RATE_ACCEL = 0.001f;
const float RAMP_RATE_DECEL = 0.001f;

// MOTOR STATE
float duty_left_target  = 0.0f;
float duty_right_target = 0.0f;
float duty_left_current  = 0.0f;
float duty_right_current = 0.0f;

// ENCODER / SENSOR STATE
float   velocity_left  = 0.0f;  // RPM
float   velocity_right = 0.0f;  // RPM
float   position_left  = 0.0f;  // rotations
float   position_right = 0.0f;  // rotations
float   current_left   = 0.0f;  // amps
float   current_right  = 0.0f;  // amps
uint8_t temp_left      = 0;     // °C
uint8_t temp_right     = 0;     // °C
float   voltage        = 0.0f;  // volts (smoothed)
float   battery_pct    = 0.0f;  // 0-100%

// Rolling average for voltage — larger N = smoother but slower to respond
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

// RAW BYTE DEBUG — set true to print raw status bytes to verify scaling
// Keep false during normal use — floods serial and causes dropped characters
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

// ─────────────────────────────────────────────
// BATTERY PERCENTAGE
// Adjust min_v / max_v for your LiPo cell count:
//   3S: max=12.6  min=10.5
//   4S: max=16.8  min=13.2
//   5S: max=21.0  min=16.5
//   6S: max=25.2  min=19.8
// ─────────────────────────────────────────────
float batteryPercent(float v) {
  const float max_v = 12.6f;  // 3S fully charged
  const float min_v = 10.5f;  // 3S cutoff (stop using below this)
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
void computeMotorFromStick() {
  float y = -controller.ly / 512.0f;
  float x =  controller.rx / 512.0f;
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
  Serial.print("LY=");      Serial.print(controller.ly);
  Serial.print(" RX=");     Serial.print(controller.rx);
  Serial.print(" | Target L="); Serial.print(duty_left_target);
  Serial.print(" R=");      Serial.print(duty_right_target);
  Serial.print(" | RPM L="); Serial.print(velocity_left);
  Serial.print(" R=");      Serial.print(velocity_right);
  Serial.print(" | Pos L="); Serial.print(position_left);
  Serial.print(" R=");      Serial.print(position_right);
  Serial.print(" | Temp L="); Serial.print(temp_left);
  Serial.print("C R=");     Serial.print(temp_right);
  Serial.print("C | mA L="); Serial.print(current_left);
  Serial.print(" R=");      Serial.print(current_right);
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

// ─────────────────────────────────────────────
// RAW BYTE DEBUG PRINTER
// Prints all 8 bytes of a status frame as hex and decimal
// so you can figure out the real scaling for current/voltage
// ─────────────────────────────────────────────
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

  // READ CONTROLLER DATA
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buffer[bufIdx] = '\0';
      if (bufIdx > 0 && parseCSV(buffer, controller))
        computeMotorFromStick();
      bufIdx = 0;
    } else if (c != '\r' && bufIdx < sizeof(buffer) - 1) {
      buffer[bufIdx++] = c;
    } else {
      bufIdx = 0;
    }
  }

  unsigned long now = millis();

  // RAMP MOTORS — gated to run once per ms so rate is time-based not loop-speed-based
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

  // MOTOR COMMANDS every 10ms, offset by 5ms
  static unsigned long ctrl_last = 5;
  if (now - ctrl_last >= 10) {
    send_control_frame(MCP2515::TXB1, 1, Duty_Cycle_Set, duty_left_current);
    send_control_frame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right_current);
    ctrl_last = now;
  }

  // SERIAL PRINT every 100ms
  static unsigned long print_last = 0;
  if (now - print_last >= 100) {
    printStatus();
    print_last = now;
  }

  // READ AND PARSE CAN RX
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {

    uint32_t id = rx.can_id & ~CAN_EFF_FLAG;

    // ── STATUS 1 ──────────────────────────────────────────────────────────
    // Byte layout (per SparkMax CAN protocol):
    //   [0-3] velocity  → float, RPM
    //   [4]   temp      → uint8, °C  ✓ confirmed
    //   [5]   voltage   → uint8, *0.1 = volts (e.g. 175 → 17.5V) ✓ confirmed
    //   [6]   current   → uint8, *0.125 = amps ✓ confirmed (5 → 0.625A at idle)
    //   [7]   unknown   → almost always 0, likely fault flags
    // ─────────────────────────────────────────────────────────────────────
    if (id == (status_1 + 1)) {
      memcpy(&velocity_left, rx.data, 4);
      temp_left    = rx.data[4];
      // Clamp current — spikes above ~40A are corrupt frames, not real current
      float raw_current_l = rx.data[6] * 0.125f;
      current_left = (raw_current_l < 40.0f) ? raw_current_l : current_left;
      // rx.data[7] appears to be fault flags, ignored for now
      float new_voltage = rx.data[5] * 0.0658f;   // calibrated against REV Hardware Client (avg 11.00V)
      // Ignore readings below 10V — SparkMax reports junk when battery is absent
      if (new_voltage >= 10.0f) {
        voltage = addVoltageSample(new_voltage);
        // Only update battery % when motors are near idle — avoids voltage sag skewing the reading
        // duty threshold of 0.05 gives a small buffer above true zero
        bool motors_idle = fabs(duty_left_current) < 0.05f && fabs(duty_right_current) < 0.05f;
        if (motors_idle) {
          battery_pct = batteryPercent(voltage);
        }
      }

      if (DEBUG_RAW_BYTES) printRawBytes("S1_L", rx);
    }
    else if (id == (status_1 + 2)) {
      memcpy(&velocity_right, rx.data, 4);
      temp_right    = rx.data[4];
      float raw_current_r = rx.data[6] * 0.125f;
      current_right = (raw_current_r < 40.0f) ? raw_current_r : current_right;

      if (DEBUG_RAW_BYTES) printRawBytes("S1_R", rx);
    }

    // ── STATUS 2 ──────────────────────────────────────────────────────────
    // Byte layout:
    //   [0-3] position → float, rotations
    //   [4-7] unknown  → printing raw to figure out
    // ─────────────────────────────────────────────────────────────────────
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
