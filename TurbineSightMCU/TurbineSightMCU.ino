#include <MAVLink.h>
#include <Servo.h>

// Pinout
const int solenoid_control_H1_pin = 2;
const int solenoid_control_H2_pin = 3;
const int pump_control_pin = 4;
const int AnalogPressurePin1 = 14;
const int AnalogPressurePin2 = 15;

// Pressure related values
const float Patm = 930.0;
const float Pmin = 350.0;
const float attachPressureThreshold = 400.0;
const float alpha = 0.1;
float P1;
float P2;
const int pump_on_us = 1100;
const int pump_off_us = 1500;
Servo pump_controller;

// MAVLink variables
const uint8_t MAVLINK_SYS_ID = 1;
const uint8_t MAVLINK_COMP_ID = 2;
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
IntervalTimer heartbeatTimer;             // Create an interval timer
unsigned long heartbeatTimer_ms = 1000;   // Heartbeat period in milliseconds
volatile bool attachState = false; // Store the current switch state

void setup() {
  // Setup switch input pin
  pinMode(solenoid_control_H1_pin,OUTPUT);
  pinMode(solenoid_control_H2_pin, OUTPUT);

  // Set LED outputs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial2.begin(115200); // Initialize serial communication for MAVLink
  heartbeatTimer.begin(send_mavlink_heartbeat, heartbeatTimer_ms * 1000); // Setup heartbeat timer

  pump_controller.attach(pump_control_pin);
  pump_controller.write(pump_off_us);
  
  digitalWrite(solenoid_control_H2_pin,LOW);
  P1 = Patm;
  P2 = Patm;
}

void loop() {
  mavlink_message_t received_msg;
  mavlink_status_t status;
  // Check for incoming MAVLink messages
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    // Try to parse a message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status)) {
      // Handle message by checking its ID
      switch (received_msg.msgid) {
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
          // Handle attach or detach commands (can be expanded)
          mavlink_named_value_float_t msg_struct;
          mavlink_msg_named_value_float_decode(&received_msg, &msg_struct);
          if (strcmp(msg_struct.name, "attach") == 0) {
              if (abs(msg_struct.value)>0.01){ // attach is nonzero, we want to attach
                digitalWrite(solenoid_control_H1_pin, LOW);
                pump_controller.write(pump_on_us);
              }
              else { // attach is zero, we want to detach
                digitalWrite(solenoid_control_H1_pin, HIGH);
                pump_controller.write(pump_off_us);
              }
          } else if (strcmp(msg_struct.name, "lass") == 0) {
            // TODO: Analog LEDs
          }
          break;
        default:
          break;
      }
    }
  }
  updatePressure();
}

// Function to send MAVLink heartbeat and info
void send_mavlink_heartbeat() {
  mavlink_msg_heartbeat_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
  if (attachState) {
    send_attached_msg();  // Send attach message via MAVLink
  } else {
    send_detached_msg();  // Send detach message via MAVLink
  }
}

void send_attached_msg() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", 1.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void send_detached_msg() {
  mavlink_msg_named_value_float_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg, millis(), "att_st", 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void updatePressure(){
  attachState = false;
  P1 = alpha * map(analogRead(AnalogPressurePin1),Pmin,Patm,40.0,0.0) + (1.0-alpha)*P1;
  P2 = alpha * map(analogRead(AnalogPressurePin2),Pmin,Patm,40.0,0.0) + (1.0-alpha)*P2;
  if (P1 < attachPressureThreshold && P2 < attachPressureThreshold) {
    attachState = true;
    send_attached_msg();
  }
}

void reportPressure(){ // TODO: report pressure

  // Serial1.print(P1);
  // Serial1.print(", ");
  // Serial1.println(P2);
}