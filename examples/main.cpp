#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>
SoftwareSerial Serial1(3, -1);  // RX=Pin3, TX=Pin-1 

MCP2515 mcp2515(10);

enum control_mode {
  Duty_Cycle_Set = 0x2050080,
  Speed_Set = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set = 0x2050C80,
  Voltage_Set = 0x2051080,
  Current_Set = 0x20510C0,
  Smart_Motion_Set = 0x2051480
};

enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
struct can_frame heartbeat_frame;
const uint8_t HEARTBEAT_SIZE = 8;
struct can_frame control_frame;
const uint8_t CONTROL_SIZE = 8;
struct can_frame status_frame;
const uint8_t STATUS_SIZE = 2;

void pack_data(can_frame &frame, const uint8_t *data, const int size);
void create_data(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size);
void send_control_frame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint);

static void spi_bit_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  digitalWrite(10, LOW);
  SPI.transfer(0x05);
  SPI.transfer(reg);
  SPI.transfer(mask);
  SPI.transfer(val);
  digitalWrite(10, HIGH);
}

// Motor duties
static float duty_left  = 0.0;
static float duty_right = 0.0;

// Serial1 CSV parser
struct ControllerData {
  int lx, ly, rx, ry;
  int brake;      // Left trigger  0-1023 → left motor
  int throttle;   // Right trigger 0-1023 → right motor
  int buttons;
  int dpad;
};

ControllerData controller;
char buffer[64];
uint8_t bufIdx = 0;

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

// Map 0-1023 trigger to 0.0-0.5 duty
float triggerToDuty(int trigger) {
  float duty = (trigger / 1023.0f) * 0.5f;
  if (duty >  0.5f) duty =  0.5f;
  if (duty <  0.0f) duty =  0.0f;
  return duty;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);  // Uno R3 Serial1: RX=Pin19, TX=Pin18

  Serial.println("CAN CONTROLLER");

  heartbeat_frame.can_id  = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = HEARTBEAT_SIZE;
  pack_data(heartbeat_frame, HEARTBEAT_DATA, HEARTBEAT_SIZE);

  control_frame.can_dlc = CONTROL_SIZE;
  status_frame.can_dlc  = STATUS_SIZE;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  // Read incoming CSV from Nano ESP32
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buffer[bufIdx] = '\0';
      if (bufIdx > 0 && parseCSV(buffer, controller)) {
        duty_left  = triggerToDuty(controller.brake);
        duty_right = triggerToDuty(controller.throttle);

        Serial.print("L="); Serial.print(duty_left);
        Serial.print(" R="); Serial.println(duty_right);
      }
      bufIdx = 0;
    } else if (c != '\r' && bufIdx < sizeof(buffer) - 1) {
      buffer[bufIdx++] = c;
    } else if (bufIdx >= sizeof(buffer) - 1) {
      bufIdx = 0;  // overflow, reset
    }
  }

  // Drain incoming CAN RX frames
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {}

  unsigned long now = millis();

  // Heartbeat every 10ms
  static unsigned long hb_last = 0;
  if (now - hb_last >= 10) {
    mcp2515.sendMessage(MCP2515::TXB0, &heartbeat_frame);
    hb_last = now;
  }

  // Motor control every 10ms, offset by 5ms
  static unsigned long ctrl_last = 5;
  if (now - ctrl_last >= 10) {
    send_control_frame(MCP2515::TXB1, 1, Duty_Cycle_Set, duty_left);
    send_control_frame(MCP2515::TXB2, 2, Duty_Cycle_Set, duty_right);
    ctrl_last = now;
  }
}

void pack_data(can_frame &frame, const uint8_t *data, const int size) {
  for (int i = 0; i < size; i++) {
    frame.data[i] = data[i];
  }
}

void create_data(const void* data, byte *frame_data, const uint8_t data_size, const uint8_t total_size) {
  const byte* data_arr = static_cast<const byte*>(data);
  for (int i = 0; i < data_size; i++) {
    frame_data[i] = data_arr[i];
  }
  for (int i = data_size; i < total_size; i++) {
    frame_data[i] = 0;
  }
}

void send_control_frame(MCP2515::TXBn txb, const uint32_t device_id, const control_mode mode, const float setpoint) {
  control_frame.can_id = (mode + device_id) | CAN_EFF_FLAG;
  byte control_data[CONTROL_SIZE];
  create_data(&setpoint, control_data, 4, CONTROL_SIZE);
  pack_data(control_frame, control_data, CONTROL_SIZE);
  mcp2515.sendMessage(txb, &control_frame);
}