#include <MAVLink.h>
#include <string>
#include <unordered_map>

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
const float Patm = 970.0;
const float atmosphereThreshold = 930.0;
const float attachPressureThreshold = 400.0;
const float alpha = 0.1;
float P1 = Patm;
float P2 = Patm;
bool pressureReporting = false;

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
void handleAttach(uint8_t &value) {
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

void setColor(uint8_t strip, StateName state) {
  uint8_t color = static_cast<uint8_t>(state);
  analogWrite(LED_R[strip], colors[color][0]);
  analogWrite(LED_G[strip], colors[color][1]);
  analogWrite(LED_B[strip], colors[color][2]);
}

void handleLass(uint8_t &value) {
  _lass_state_name = static_cast<StateName>(value);
  switch (_lass_state_name){
    case StateName::Default: // Default
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::Lass: // Lass
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::LeadUp: // LeadUp
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::CoastIn: // CoastIn
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::WindDown: // WindDown
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::Vegetable: // Vegetable
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::WindUp: // WindUp
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::CoastOut: // CoastOut
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::Recover: // Recover
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    case StateName::Exited: // Exited
      setColor(0, _lass_state_name);
      setColor(1, _lass_state_name);
      break;
    default:
      break;
  }
}

void handleTest(uint8_t &value) {
  switch (value){
    case 0: // disable test mode
      pumpOff();
      solenoidOff();
      pressureReporting = false;
      break;
    case 1: // comms check
      ping();
      break;
    case 2: // test motor
      pumpOn();
      break;
    case 3: // test solenoid
      solenoidOn();
      break;
    case 4: // test pressure sensors
      pressureReporting = true;
      break;
    case 5: // test LEDs

      break;
    default:
      break;
  }
}
std::unordered_map<std::string, void(*)(uint8_t &)> commandHandlers = {
    {"attach", handleAttach},
    {"lass", handleLass},
    {"test", handleTest}
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
          uint8_t value = (uint8_t)msg_struct.value;
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
    // send_attached_msg();  // Send detach message via MAVLink
    if (pressureReporting) {
      reportPressure();
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
  attachState = false;
  P1 = alpha * analogRead(AnalogPressurePin1) + (1.0-alpha)*P1;
  P2 = alpha * analogRead(AnalogPressurePin2) + (1.0-alpha)*P2;

  // Handle solenoid off after atmosphere reached
  if (P1 > atmosphereThreshold && P2 > atmosphereThreshold) {
    solenoidOff();
  }

  // Handle coastin LED colors
  if ( _lass_state_name != StateName::CoastIn) {return;}
  if (P1 < attachPressureThreshold) {
    setColor(0, StateName::CoastInSuctionOn);
  }
  else {
    setColor(0, StateName::CoastIn);
  }
  if (P2 < attachPressureThreshold) {
    setColor(0, StateName::CoastInSuctionOn);
  }
  else {
    setColor(0, StateName::CoastIn);
  }
  if (P1 < attachPressureThreshold && P2 < attachPressureThreshold) {
    attachState = true;
    send_attached_msg();
  }
  else {
    attachState = false;
    send_attached_msg();
  }
}

void reportPressure(){
  sendNVF("suction1", P1);
  sendNVF("suction2", P2);
}

void ping() {
  sendNVF("ping", 69.0f);
}


//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
void setup() {
  // Setup switch input pin
  pinMode(solenoid_control_I1_pin,OUTPUT);
  pinMode(pump_control_I1_pin,OUTPUT);
  for (int i = 0; i < numStrips; i++) {
    pinMode(LED_R[i], OUTPUT);
    pinMode(LED_G[i], OUTPUT);
    pinMode(LED_B[i], OUTPUT);
    setColor(i, _lass_state_name);
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

