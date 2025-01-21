#include <MAVLink.h>
#include <string>
#include <unordered_map>

// Pinout
const int solenoid_control_H1_pin = 2;
const int solenoid_control_H2_pin = 3;
const int pump_control_H1_pin = 4;
const int pump_control_H2_pin = 5; // need to check this
const int AnalogPressurePin1 = 14;
const int AnalogPressurePin2 = 15;

// Pressure related values
const float Patm = 930.0;
const float Pmin = 350.0;
const float attachPressureThreshold = 400.0;
const float alpha = 0.1;
float P1 = Patm;
float P2 = Patm;
bool pressureReporting = false;

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

void handleLass(uint8_t &value) {

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
  digitalWrite(solenoid_control_H1_pin, HIGH);
  digitalWrite(solenoid_control_H2_pin, LOW);
}

void solenoidOff() { // seals vacuum
  digitalWrite(solenoid_control_H1_pin, LOW);
  digitalWrite(solenoid_control_H2_pin, LOW);
}

void pumpOn() {
  digitalWrite(pump_control_H1_pin, HIGH);
  digitalWrite(pump_control_H2_pin, LOW);
}

void pumpOff() {
  digitalWrite(pump_control_H1_pin, LOW);
  digitalWrite(pump_control_H2_pin, LOW);
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
          if (it != commandHandlers.end()) {it->second(value);} 
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
    if (attachState) {
      send_attached_msg();  // Send attach message via MAVLink
    } else {
      send_detached_msg();  // Send detach message via MAVLink
    }
    if (pressureReporting) {
      reportPressure();
    }
    need_to_send_heartbeat = false;
  }
}

void send_attached_msg() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", 1.0f);
  Serial2.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void send_detached_msg() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", 0.0f);
  Serial2.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void updatePressure(){
  attachState = false;
  P1 = alpha * analogRead(AnalogPressurePin1) + (1.0-alpha)*P1;
  P2 = alpha * analogRead(AnalogPressurePin2) + (1.0-alpha)*P2;
}

void reportPressure(){
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "suction1", P1);
  Serial2.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "suction2", P2);
  Serial2.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void ping() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "ping", 69.0f);
  Serial2.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}


//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
void setup() {
  // Setup switch input pin
  pinMode(solenoid_control_H1_pin,OUTPUT);
  pinMode(solenoid_control_H2_pin, OUTPUT);
  pinMode(pump_control_H1_pin,OUTPUT);
  pinMode(pump_control_H2_pin, OUTPUT);

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

