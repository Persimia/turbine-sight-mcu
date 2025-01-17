#include <Servo.h>
#include <sbus.h>

const int suctionChannel = 5;
const int releaseCmd = 300;
const int attachCmd = 1800;

const int pump_control_pin = 4;
const int solenoid_control_H1_pin = 2;
const int solenoid_control_H2_pin = 3;

const int pump_on_us = 1100;
const int pump_off_us = 1500;

const int AnalogPressurePin1 = 14;
const int AnalogPressurePin2 = 15;
const float Patm = 930.0;
const float Pmin = 350.0;

bfs::SbusRx sbus_rx(&Serial2);
bfs::SbusData data;
Servo pump_controller;

int count = 0;
float P1;
float P2;
float alpha = 0.1;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(57600);
  sbus_rx.Begin();
  pump_controller.attach(pump_control_pin);
  pump_controller.write(pump_off_us);
  pinMode(solenoid_control_H1_pin,OUTPUT);
  pinMode(solenoid_control_H2_pin, OUTPUT);
  digitalWrite(solenoid_control_H2_pin,LOW);
  P1 = Patm;
  P2 = Patm;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sbus_rx.Read()){
    data = sbus_rx.data();
    if(data.ch[suctionChannel-1]<releaseCmd){
      digitalWrite(solenoid_control_H1_pin, HIGH);
      pump_controller.write(pump_off_us);
    }
    else if(data.ch[suctionChannel-1]>attachCmd){
      digitalWrite(solenoid_control_H1_pin, LOW);
      pump_controller.write(pump_on_us);
    }
    else{
      digitalWrite(solenoid_control_H1_pin, LOW);
      pump_controller.write(pump_off_us);
    }
  }
  updatePressure();
  if (count++ >= 200){
    reportPressure();
    count = 0;
  }
}

void updatePressure(){
    P1 = alpha * map(analogRead(AnalogPressurePin1),Pmin,Patm,40.0,0.0) + (1.0-alpha)*P1;
    P2 = alpha * map(analogRead(AnalogPressurePin2),Pmin,Patm,40.0,0.0) + (1.0-alpha)*P2;
}

void reportPressure(){

  Serial1.print(P1);
  Serial1.print(", ");
  Serial1.println(P2);
}