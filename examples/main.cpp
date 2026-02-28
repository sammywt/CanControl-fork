#include <SPI.h>
#include <mcp2515.h>

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

// Heartbeat Frame
const uint32_t HEARTBEAT_ID = 0x2052C80;  // 0x2052C80
const uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
struct can_frame heartbeat_frame;
const uint8_t HEARTBEAT_SIZE = 8;

// Control Frame
struct can_frame control_frame;
const uint8_t CONTROL_SIZE = 8;

// Status Frame
struct can_frame status_frame;
const uint8_t STATUS_SIZE = 2;

// Function Prototypes
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

static float duty_left = 0.3;
static float duty_right = 0.3;
static char inBuf[16];
static uint8_t inPos = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("CAN CONTROLLER");
  Serial.println("Commands: L0.5 (left), R0.5 (right), B0.5 (both), 0.5 (both)");
  Serial.print("Current duty L=");
  Serial.print(duty_left);
  Serial.print(" R=");
  Serial.println(duty_right);

  heartbeat_frame.can_id = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = HEARTBEAT_SIZE;
  pack_data(heartbeat_frame, HEARTBEAT_DATA, HEARTBEAT_SIZE);

  control_frame.can_dlc = CONTROL_SIZE;
  status_frame.can_dlc = STATUS_SIZE;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

  mcp2515.setNormalMode();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inPos > 0) {
        inBuf[inPos] = '\0';
        char target = 'B';
        const char* numStart = inBuf;
        if (inBuf[0] == 'L' || inBuf[0] == 'l' || inBuf[0] == 'R' || inBuf[0] == 'r' || inBuf[0] == 'B' || inBuf[0] == 'b') {
          target = inBuf[0] & 0xDF; // uppercase
          numStart = inBuf + 1;
        }
        float val = atof(numStart);
        if (val > 1.0f) val = 1.0f;
        if (val < -1.0f) val = -1.0f;
        if (target == 'L' || target == 'B') duty_left = val;
        if (target == 'R' || target == 'B') duty_right = val;
        Serial.print("Duty L=");
        Serial.print(duty_left);
        Serial.print(" R=");
        Serial.println(duty_right);
        inPos = 0;
      }
    } else if (inPos < sizeof(inBuf) - 1) {
      inBuf[inPos++] = c;
    }
  }

  // Drain incoming RX frames to prevent buffer overflow
  struct can_frame rx;
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {}

  unsigned long now = millis();

  // Send heartbeat on TXB0 every 10ms
  static unsigned long hb_last = 0;
  if (now - hb_last >= 10) {
    mcp2515.sendMessage(MCP2515::TXB0, &heartbeat_frame);
    hb_last = now;
  }

  // Send control to both motors every 10ms, offset by 5ms
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
  // Copy data to frame_data
  const byte* data_arr = static_cast<const byte*>(data);
  for (int i = 0; i < data_size; i++) {
    frame_data[i] = data_arr[i];
  }

  // Fill remaining space with zeros
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
