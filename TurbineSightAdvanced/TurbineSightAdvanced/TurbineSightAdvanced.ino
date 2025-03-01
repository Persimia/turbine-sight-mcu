#include <MAVLink.h>
#include <string>
#include <unordered_map>
#include <EEPROM.h>

// Pinout
const int solenoid_control_I1_pin = 2;
const int solenoid_control_I2_pin = 3;
const int pump_control_I1_pin = 4;
const int pump_control_I2_pin = 5;

const int AnalogPressurePin1 = 14;
const int AnalogPressurePin2 = 15;

const uint8_t numStrips = 2;
const int LED_R[numStrips] = {9, 19};
const int LED_G[numStrips] = {10, 18};
const int LED_B[numStrips] = {11, 22};

// Pressure related values
float Patm = 970.0f;
float attachPressureThreshold = 500.0f;
float detachPressureThreshold = 850.0f;
const int pressureDisconnected = 10; // int because we are directly checking analogRead
bool P1_disconnected = false;
bool P2_disconnected = false;
const uint8_t EEPROM_ADDR_BASE = 0;  // Base address in EEPROM

const float alpha = 0.1;
float P1 = Patm;
float P2 = Patm;
bool pressureReporting = false;
bool _color_test_enabled = false;

// LED Color table
uint8_t colors[][3] = {
    {255, 255, 255},  // White (Default) 0
    {255, 0, 0},      // Red (Lass) 1
    {255, 128, 0},    // Orange (LeadUp) 2
    {255, 255, 0},    // Yellow (CoastIn) 3
    {0, 255, 255},    // Cyan (WindDown) 4
    {0, 0, 255},      // Blue (Vegetable) 5
    {128, 0, 255},    // Violet (WindUp) 6
    {255, 0, 255},    // Magenta (CoastOut) 7
    {255, 0, 128},    // Rose (Recover) 8
    {0, 0, 0},        // Off (all LEDs off) 9
    {0, 255, 0},      // Green (CoastIn Suction) 10
};

enum class PressureEEPROMIdx : uint8_t {
  Patm = 0,
  attachPressureThreshold = 1,
  detachPressureThreshold = 2
};

enum class CupName : uint8_t {
  Left = 0,
  Right = 1
};

enum class StateName : uint8_t {
    Default = 0,
    Lass, // 1
    LeadUp, // 2
    CoastIn, // 3
    WindDown, // 4
    Vegetable, // 5
    WindUp, // 6
    CoastOut, // 7
    Recover, // 8
    Exited, // 9
    CoastInSuctionOn //10
};

StateName _lass_state_name = StateName::Exited;
StateName _color_test_state_name = StateName::Exited;

// MAVLink variables
const uint8_t MAVLINK_SYS_ID = 1;
const uint8_t MAVLINK_COMP_ID = 2;
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
IntervalTimer heartbeatTimer;             // Create an interval timer
unsigned long heartbeatTimer_ms = 1000;   // Heartbeat period in milliseconds
volatile bool attachState = false; // Store the current switch state

// State handlers
volatile bool need_to_send_heartbeat = false;
const float EPSILON = 1e-6;

// Function to save a float at a specific index (0, 1, or 2)
void saveFloatToEEPROM(PressureEEPROMIdx index_enum, float value) {
  uint8_t index = static_cast<uint8_t>(index_enum);
  if (index < 0 || index > 2) return;  // Prevent out-of-bounds access
  int address = EEPROM_ADDR_BASE + index * sizeof(float);
  EEPROM.put(address, value);
}

// Function to read a float from a specific index (0, 1, or 2)
void readFloatFromEEPROM(PressureEEPROMIdx index_enum, float &current_value) {
  uint8_t index = static_cast<uint8_t>(index_enum);
  if (index < 0 || index > 2) return;  // Return default value if out of bounds
  int address = EEPROM_ADDR_BASE + index * sizeof(float);
  float value;
  EEPROM.get(address, value);
  if (!isnan(value)){
    current_value = value;
  }
}

void setColor(CupName cup, StateName state) {
  uint8_t color = static_cast<uint8_t>(state);
  uint8_t strip = static_cast<uint8_t>(cup);
  analogWrite(LED_R[strip], colors[color][0]);
  analogWrite(LED_G[strip], colors[color][1]);
  analogWrite(LED_B[strip], colors[color][2]);
}

void handleAttach(float &value_f) {
  uint8_t value = (uint8_t)value_f;
  switch (value){
    case 0:
      initDetach();
      break;
    case 1:
      initAttach();
      break;
    default:
      break;
  }
}

void handleLass(float &value_f) {
  uint8_t value = (uint8_t)value_f;
  _lass_state_name = static_cast<StateName>(value);
  setColor(CupName::Right, _lass_state_name);
  setColor(CupName::Left, _lass_state_name);
}

void handleTest(float &value_f) {
  uint8_t value = (uint8_t)value_f;
  switch (value){
    case 0: // disable test mode
      pumpOff();
      solenoidOff();
      pressureReporting = false;
      _color_test_enabled = false;
      setColor(CupName::Left, _lass_state_name);
      setColor(CupName::Right, _lass_state_name);
      break;
    case 1: // comms check
      pumpOn();
      solenoidOn();
      pressureReporting = true;
      _color_test_enabled = true;
      _color_test_state_name = StateName::Exited;
      break;

      break;
    default:
      break;
  }
}

void handleMotor(float &value_f) {
  uint8_t value = (uint8_t)value_f;
  switch (value){
    case 0:
      pumpOff();
      break;
    case 1: 
      pumpOn();
      break;
    default:
      break;
  }
}

void handleSolenoid(float &value_f) {
  uint8_t value = (uint8_t)value_f;
  switch (value){
    case 0: 
      solenoidOff();
      break;
    case 1:
      solenoidOn();
      break;
    default:
      break;
  }
}

void handlePressure(float &value_f) {
  uint8_t value = (uint8_t)value_f;
  switch (value){
    case 0:
      pressureReporting = false;
      break;
    case 1:
      pressureReporting = true;
      break;
    default:
      break;
  }
}

void handleSet_p_atm(float &value_f) {
  Patm = value_f;
  saveFloatToEEPROM(PressureEEPROMIdx::Patm, Patm);
}

void handleSet_p_att(float &value_f) {
  attachPressureThreshold = value_f;
  saveFloatToEEPROM(PressureEEPROMIdx::attachPressureThreshold, attachPressureThreshold);
}

void handleSet_p_det(float &value_f) {
  detachPressureThreshold = value_f;
  saveFloatToEEPROM(PressureEEPROMIdx::detachPressureThreshold, detachPressureThreshold);
}

void handleGet_p_thresh(float &value_f) {
  reportPressureThresholds();
}

std::unordered_map<std::string, void(*)(float &)> commandHandlers = {
    {"attach", handleAttach},
    {"lass", handleLass},
    {"test", handleTest},
    {"motor", handleMotor},
    {"solenoid", handleSolenoid},
    {"pressure", handlePressure},
    {"set_p_atm", handleSet_p_atm},
    {"set_p_att", handleSet_p_att},
    {"set_p_det", handleSet_p_det},
    {"get_p_lims", handleGet_p_thresh}
};

void initAttach() {
  solenoidOff();
  pumpOn();
}

void initDetach() {
  solenoidOn();
  pumpOff();
}

void solenoidOn() { // releases vacuum
  digitalWrite(solenoid_control_I1_pin, HIGH);
}

void solenoidOff() { // seals vacuum
  digitalWrite(solenoid_control_I1_pin, LOW);
}

void pumpOn() {
  digitalWrite(pump_control_I1_pin, HIGH);
}

void pumpOff() {
  digitalWrite(pump_control_I1_pin, LOW);
}

void parseMavlinkInput() {
  mavlink_message_t received_msg;
  mavlink_status_t status;
  // Check for incoming MAVLink messages
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    // Try to parse a message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status)) {
      // Handle message by checking its ID
      switch (received_msg.msgid) {
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
          // Handle attach or detach commands (can be expanded)
          mavlink_named_value_float_t msg_struct;
          mavlink_msg_named_value_float_decode(&received_msg, &msg_struct);
          float value = msg_struct.value;
          auto it = commandHandlers.find(msg_struct.name);
          if (it != commandHandlers.end()) {it->second(value); sendNVF("ACK", value);} 
          else {} // unknown command
        }
        default:
          break;
      }
    }
  }
}

// Function to send MAVLink heartbeat and info
void send_mavlink_heartbeat() {
  need_to_send_heartbeat = true;
}

void sendHeartbeat() {
  if(need_to_send_heartbeat) {
    mavlink_msg_heartbeat_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    send_attached_msg();  // Send att_st message via MAVLink
    if (pressureReporting) {
      reportPressure();
    }
    if (_color_test_enabled) {
      if (static_cast<uint8_t>(_color_test_state_name) + 1 > static_cast<uint8_t>(StateName::CoastInSuctionOn)) {
          _color_test_state_name = StateName::Default; // Wrap around
      } 
      else {
        _color_test_state_name = static_cast<StateName>(static_cast<uint8_t>(_color_test_state_name) + 1);
      }
      setColor(CupName::Right, _color_test_state_name);
      setColor(CupName::Left, _color_test_state_name);
    }
    if (P1_disconnected) {
      sendNVF("P1_dis", 1.0f);
    }
    if (P2_disconnected) {
      sendNVF("P2_dis", 2.0f);
    }

    need_to_send_heartbeat = false;
  }
}

void sendNVF(const char* name, float value){
  char name_bytes[10];
  nullTerminateNVF(name, name_bytes, sizeof(name_bytes));
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), name_bytes, value);
  Serial2.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void nullTerminateNVF(const char* name, char* name_bytes, size_t size) {
  memset(name_bytes, 0, size);  // Ensure all elements are initialized to '\0'
  strncpy(name_bytes, name, size-1);  // Copy the string with null-termination
}

void send_attached_msg() {
  sendNVF("att_st", (float)attachState);
}

void updatePressure(){
  if (analogRead(AnalogPressurePin1) < pressureDisconnected) {
    P1_disconnected = true;
  } else {
    P1 = alpha * (float)analogRead(AnalogPressurePin1) + (1.0-alpha)*P1;
    P1_disconnected = false;
  }

  if (analogRead(AnalogPressurePin2) < pressureDisconnected) {
    P2_disconnected = true;
  } else {
    P2 = alpha * (float)analogRead(AnalogPressurePin2) + (1.0-alpha)*P2;
    P2_disconnected = false;
  }

  if (P1 < attachPressureThreshold && P2 < attachPressureThreshold) {
    if (!attachState) {
      attachState = true;
      send_attached_msg();
    } 
  }
  if (P1 > detachPressureThreshold && P2 > detachPressureThreshold) {
    if (attachState) {
      attachState = false;
      send_attached_msg();
    }
  }

  // Handle coastin LED colors
  if ( _lass_state_name != StateName::CoastIn) {return;}
  if (P1 < attachPressureThreshold) {
    setColor(CupName::Right, StateName::CoastInSuctionOn);
  }
  else {
    setColor(CupName::Right, StateName::CoastIn);
  }
  if (P2 < attachPressureThreshold) {
    setColor(CupName::Left, StateName::CoastInSuctionOn);
  }
  else {
    setColor(CupName::Left, StateName::CoastIn);
  }
  
}

void reportPressure(){
  sendNVF("suction1", P1);
  sendNVF("suction2", P2);
}

void reportPressureThresholds(){
  sendNVF("p_atm", Patm);
  sendNVF("p_att", attachPressureThreshold);
  sendNVF("p_det", detachPressureThreshold);
}

void ping() {
  sendNVF("ping", 69.0f);
}


//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
void setup() {
  readFloatFromEEPROM(PressureEEPROMIdx::Patm, Patm);
  readFloatFromEEPROM(PressureEEPROMIdx::attachPressureThreshold, attachPressureThreshold);
  readFloatFromEEPROM(PressureEEPROMIdx::detachPressureThreshold, detachPressureThreshold);

  // Setup switch input pin
  pinMode(solenoid_control_I1_pin,OUTPUT);
  pinMode(pump_control_I1_pin,OUTPUT);
  for (int i = 0; i < numStrips; i++) {
    pinMode(LED_R[i], OUTPUT);
    pinMode(LED_G[i], OUTPUT);
    pinMode(LED_B[i], OUTPUT);
    setColor(static_cast<CupName>(i), _lass_state_name);
  }

  digitalWrite(solenoid_control_I2_pin, LOW);
  digitalWrite(pump_control_I2_pin, LOW);

  pumpOff();
  solenoidOff();

  // Set builtin LED outputs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial2.begin(115200); // Initialize serial communication for MAVLink
  heartbeatTimer.begin(send_mavlink_heartbeat, heartbeatTimer_ms * 1000); // Setup heartbeat timer
}

void loop() {
  parseMavlinkInput();
  sendHeartbeat();
  updatePressure();
}

